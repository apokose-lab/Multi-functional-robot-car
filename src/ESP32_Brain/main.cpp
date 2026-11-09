#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <RobotProtocol.h>

// Thông tin Wi-Fi do chính ESP32-S3 phát ra (Access Point)
const char* AP_SSID = "Mecanum_Robot";
const char* AP_PASS = "12345678";
const uint16_t UDP_PORT = 4210;

WiFiUDP udp;
HardwareSerial MotionSerial(1); // Dùng UART1 trên ESP32-S3

// Chân UART nối sang Motion 2350 Pro
const int UART_RX_PIN = 17; // Nối vào GP16 (TX) của Motion 2350
const int UART_TX_PIN = 18; // Nối vào GP17 (RX) của Motion 2350

int16_t vx = 0, vy = 0, omega = 0;
uint8_t buzzer = 0;
unsigned long lastInputTime = 0;

// Gói tin UDP nhận từ script Python (tay cầm PS4 trên Laptop)
#pragma pack(push, 1)
struct JoyPacket {
    int16_t vx;      // Tiến / Lùi (-255 đến 255)
    int16_t vy;      // Trượt ngang Trái / Phải (-255 đến 255)
    int16_t omega;   // Xoay Trái / Phải (-255 đến 255)
    uint8_t buzzer;  // Nút bấm còi
};
#pragma pack(pop)
// Khai báo thêm mảng lưu trạng thái PWM toàn cục (đặt phía trên hàm sendToMotion2350)
int16_t current_pwm[4] = {0, 0, 0, 0};

void sendToMotion2350(int16_t vx, int16_t vy, int16_t rot, uint8_t buzz) {
    // Công thức trộn động học ngược cho 4 bánh Mecanum
    current_pwm[0] = constrain(vx - vy - rot, -255, 255); // M1: Front-Left
    current_pwm[1] = constrain(vx + vy + rot, -255, 255); // M2: Front-Right
    current_pwm[2] = constrain(vx + vy - rot, -255, 255); // M3: Rear-Left
    current_pwm[3] = constrain(vx - vy + rot, -255, 255); // M4: Rear-Right

    ControlPacket pkt;
    for (int i = 0; i < 4; i++) {
        pkt.motor_pwm[i] = current_pwm[i];
    }
    pkt.buzzer_cmd = buzz;
    pkt.checksum   = calculateChecksum(pkt);

    MotionSerial.write(reinterpret_cast<const uint8_t*>(&pkt), sizeof(ControlPacket));
}

void setup() {
    Serial.begin(115200);
    MotionSerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    // Khởi tạo Wi-Fi Access Point
    WiFi.softAP(AP_SSID, AP_PASS);
    udp.begin(UDP_PORT);

    Serial.println("\n=== KHOI DONG BO NAO MECANUM ESP32-S3 ===");
    Serial.print("Ket noi Laptop vao Wi-Fi: "); Serial.println(AP_SSID);
    Serial.print("IP cua xe: "); Serial.println(WiFi.softAPIP()); // Mặc định là 192.168.4.1
    Serial.println("Go W/S (Tien/Lui), A/D (Truot ngang), Q/E (Xoay), X (Dung) de test nhanh!");
}

void loop() {
    // 1A. Nhận dữ liệu tay cầm PS4 trực tiếp qua cáp USB (nếu bắt đầu bằng byte 0xFE)
    if (Serial.available() >= (int)sizeof(JoyPacket) + 1 && Serial.peek() == 0xFE) {
        Serial.read(); // Bỏ qua byte đánh dấu 0xFE
        JoyPacket joy;
        Serial.readBytes(reinterpret_cast<uint8_t*>(&joy), sizeof(JoyPacket));
        vx = joy.vx;
        vy = joy.vy;
        omega = joy.omega;
        buzzer = joy.buzzer;
        lastInputTime = millis();
    }

    // 1B. Hoặc nhận qua Wi-Fi UDP (khi rút cáp USB chạy không dây)
    int packetSize = udp.parsePacket();
    if (packetSize == sizeof(JoyPacket)) {
        JoyPacket joy;
        udp.read(reinterpret_cast<uint8_t*>(&joy), sizeof(JoyPacket));
        vx = joy.vx;
        vy = joy.vy;
        omega = joy.omega;
        buzzer = joy.buzzer;
        lastInputTime = millis();
    }

    // 2. Hỗ trợ test nhanh qua Serial Monitor bằng bàn phím (W, A, S, D, Q, E, X)
    if (Serial.available()) {
        char c = toupper(Serial.read());
        int16_t testSpeed = 150;
        lastInputTime = millis();
        if (c == 'W')      { vx =  testSpeed; vy = 0; omega = 0; }
        else if (c == 'S') { vx = -testSpeed; vy = 0; omega = 0; }
        else if (c == 'A') { vx = 0; vy =  testSpeed; omega = 0; }
        else if (c == 'D') { vx = 0; vy = -testSpeed; omega = 0; }
        else if (c == 'Q') { vx = 0; vy = 0; omega =  testSpeed; }
        else if (c == 'E') { vx = 0; vy = 0; omega = -testSpeed; }
        else if (c == 'X') { vx = 0; vy = 0; omega = 0; }
    }

    // Tự động đưa tốc độ về 0 nếu không nhận lệnh mới sau 250ms
    if (millis() - lastInputTime > 250) {
        vx = 0; vy = 0; omega = 0; buzzer = 0;
    }

    // 3. Gửi gói tin UART xuống Motion 2350 Pro ở tần số 50Hz (mỗi 20ms)
    static unsigned long lastSend = 0;
    if (millis() - lastSend >= 20) {
        lastSend = millis();
        sendToMotion2350(vx, vy, omega, buzzer);
    }
    // 4. In chi tiết PWM từng bánh mỗi 200ms lên Serial Monitor
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 200) {
        lastPrint = millis();
        Serial.printf("In[Vx:%4d Vy:%4d W:%4d] -> PWM[ M1_FL:%4d | M2_FR:%4d | M3_RL:%4d | M4_RR:%4d ]\n",
                      vx, vy, omega,
                      current_pwm[0], current_pwm[1], current_pwm[2], current_pwm[3]);
    }
}