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

void sendToMotion2350(int16_t vx, int16_t vy, int16_t rot, uint8_t buzz) {
    // Công thức trộn động học ngược (Inverse Kinematics) cho 4 bánh Mecanum
    int16_t fl = vx - vy - rot; // Bánh Trái Trước (M1)
    int16_t fr = vx + vy + rot; // Bánh Phải Trước (M2)
    int16_t rl = vx + vy - rot; // Bánh Trái Sau   (M3)
    int16_t rr = vx - vy + rot; // Bánh Phải Sau   (M4)

    ControlPacket pkt;
    pkt.motor_pwm[0] = constrain(fl, -255, 255);
    pkt.motor_pwm[1] = constrain(fr, -255, 255);
    pkt.motor_pwm[2] = constrain(rl, -255, 255);
    pkt.motor_pwm[3] = constrain(rr, -255, 255);
    pkt.buzzer_cmd   = buzz;
    pkt.checksum     = calculateChecksum(pkt);

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
        vy = joy.vy;z
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
    // Thêm vào cuối hàm loop() của src/esp32_brain/main.cpp
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
      lastPrint = millis();
      Serial.printf("[ESP32-S3] Dang chay! Vx=%d, Vy=%d, Omega=%d\n", vx, vy, omega);
  }
}