#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <RobotProtocol.h>

const char* AP_SSID = "Mecanum_Robot";
const char* AP_PASS = "12345678";
const uint16_t UDP_PORT = 4210;

WiFiUDP udp;
HardwareSerial MotionSerial(1);

// Giữ đúng cặp chân UART bạn đã đấu nối thành công trên xe (44/43 hoặc 17/18)
const int UART_RX_PIN = 44; // Nối với GP16 (TX) của Motion 2350 Pro
const int UART_TX_PIN = 43; // Nối với GP17 (RX) của Motion 2350 Pro

int16_t vx = 0, vy = 0, omega = 0;
uint8_t buzzer = 0;
unsigned long lastInputTime = 0;

// Biến lưu địa chỉ Laptop để gửi phản hồi không dây
IPAddress laptopIP;
uint16_t laptopPort = 0;
bool hasLaptopClient = false;

// Biến lưu dữ liệu phản hồi thực tế từ Motion 2350 Pro
MotionFeedbackPacket latestMotionFB;
unsigned long lastMotionFBTime = 0;

#pragma pack(push, 1)
struct JoyPacket {
    int16_t vx;
    int16_t vy;
    int16_t omega;
    uint8_t buzzer;
};
#pragma pack(pop)

void sendToMotion2350(int16_t vx, int16_t vy, int16_t rot, uint8_t buzz) {
    ControlPacket pkt;
    pkt.motor_pwm[0] = constrain(vx - vy - rot, -255, 255); // M1: Front-Left
    pkt.motor_pwm[1] = constrain(vx + vy + rot, -255, 255); // M2: Front-Right
    pkt.motor_pwm[2] = constrain(vx + vy - rot, -255, 255); // M3: Rear-Left
    pkt.motor_pwm[3] = constrain(vx - vy + rot, -255, 255); // M4: Rear-Right
    pkt.buzzer_cmd   = buzz;
    pkt.checksum     = calculateChecksum(pkt);

    MotionSerial.write(reinterpret_cast<const uint8_t*>(&pkt), sizeof(ControlPacket));
}

void setup() {
    Serial.begin(115200);
    MotionSerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS, 6, 0, 2);
    WiFi.setSleep(false); // Tắt chế độ ngủ Wi-Fi để truyền nhận 2 chiều mượt mà
    udp.begin(UDP_PORT);

    memset(&latestMotionFB, 0, sizeof(latestMotionFB));
}

void loop() {
    // 1. Nhận lệnh điều khiển từ Laptop qua Wi-Fi UDP
    int packetSize = udp.parsePacket();
    if (packetSize == sizeof(JoyPacket)) {
        laptopIP = udp.remoteIP();
        laptopPort = udp.remotePort();
        hasLaptopClient = true;

        JoyPacket joy;
        udp.read(reinterpret_cast<uint8_t*>(&joy), sizeof(JoyPacket));
        vx = joy.vx;
        vy = joy.vy;
        omega = joy.omega;
        buzzer = joy.buzzer;
        lastInputTime = millis();
    }

    // Dự phòng nhận qua cáp USB nếu đang cắm dây
    if (Serial.available() >= (int)sizeof(JoyPacket) + 1 && Serial.peek() == 0xFE) {
        Serial.read();
        JoyPacket joy;
        Serial.readBytes(reinterpret_cast<uint8_t*>(&joy), sizeof(JoyPacket));
        vx = joy.vx;
        vy = joy.vy;
        omega = joy.omega;
        buzzer = joy.buzzer;
        lastInputTime = millis();
    }

    // Tự ngắt an toàn nếu quá 250ms không có lệnh mới
    if (millis() - lastInputTime > 250) {
        vx = 0; vy = 0; omega = 0; buzzer = 0;
    }

    // 2. Đọc gói tin phản hồi từ Motion 2350 Pro gửi lên qua UART
    while (MotionSerial.available() >= (int)sizeof(MotionFeedbackPacket)) {
        if (MotionSerial.peek() != 0xBB) {
            MotionSerial.read();
            continue;
        }
        MotionFeedbackPacket fb;
        MotionSerial.readBytes(reinterpret_cast<uint8_t*>(&fb), sizeof(MotionFeedbackPacket));
        if (fb.header1 == 0xBB && fb.header2 == 0x66 && fb.checksum == calculateChecksum(fb)) {
            latestMotionFB = fb;
            lastMotionFBTime = millis();
        }
    }

    // 3. Gửi lệnh điều khiển xuống Motion 2350 Pro mỗi 20ms (50Hz)
    static unsigned long lastSend = 0;
    if (millis() - lastSend >= 20) {
        lastSend = millis();
        sendToMotion2350(vx, vy, omega, buzzer);
        buzzer = 0;
    }

    // 4. Truyền gói tin Telemetry không dây ngược về Laptop mỗi 40ms (25Hz)
    static unsigned long lastTelemetry = 0;
    if (hasLaptopClient && (millis() - lastTelemetry >= 40)) {
        lastTelemetry = millis();

        WifiTelemetryPacket tele;
        tele.vx = vx;
        tele.vy = vy;
        tele.omega = omega;
        
        bool motionAlive = (millis() - lastMotionFBTime < 300);
        tele.motion_alive = motionAlive ? 1 : 0;

        for (int i = 0; i < 4; i++) {
            // Nếu nhận được phản hồi từ Motion 2350 thì lấy số thực tế, nếu chưa thì hiển thị số tính toán
            tele.actual_pwm[i] = motionAlive ? latestMotionFB.actual_pwm[i] : 0;
            tele.actual_rpm[i] = motionAlive ? latestMotionFB.actual_rpm[i] : 0;
        }
        tele.wifi_rssi = WiFi.RSSI();
        tele.uptime_ms = millis();
        tele.checksum  = calculateChecksum(tele);

        udp.beginPacket(laptopIP, laptopPort);
        udp.write(reinterpret_cast<const uint8_t*>(&tele), sizeof(WifiTelemetryPacket));
        udp.endPacket();
    }
}