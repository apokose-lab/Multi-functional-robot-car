#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <RobotProtocol.h>

const char* AP_SSID = "Mecanum_Robot";
const char* AP_PASS = "12345678";
const uint16_t UDP_PORT = 4210;

WiFiUDP udp;
HardwareSerial MotionSerial(1);

// Giữ đúng cặp chân UART đang hoạt động trên xe của bạn (44/43 hoặc 17/18)
const int UART_RX_PIN = 44; // Nối GP16 (TX) của Motion 2350 Pro
const int UART_TX_PIN = 43; // Nối GP17 (RX) của Motion 2350 Pro

// Cấu hình chân 3 cảm biến siêu âm SR-04 (Trước, Trái, Phải)
const uint8_t TRIG_PINS[3] = {4, 6, 10}; // A0, A2, A4
const uint8_t ECHO_PINS[3] = {5, 8, 11}; // A1, A3, A5

volatile uint32_t echoStart[3] = {0, 0, 0};
volatile uint16_t distanceCm[3] = {200, 200, 200};

void IRAM_ATTR echoISR0() {
    if (digitalRead(ECHO_PINS[0])) echoStart[0] = micros();
    else {
        uint32_t dur = micros() - echoStart[0];
        if (dur < 25000) distanceCm[0] = dur * 0.0343f / 2.0f;
    }
}
void IRAM_ATTR echoISR1() {
    if (digitalRead(ECHO_PINS[1])) echoStart[1] = micros();
    else {
        uint32_t dur = micros() - echoStart[1];
        if (dur < 25000) distanceCm[1] = dur * 0.0343f / 2.0f;
    }
}
void IRAM_ATTR echoISR2() {
    if (digitalRead(ECHO_PINS[2])) echoStart[2] = micros();
    else {
        uint32_t dur = micros() - echoStart[2];
        if (dur < 25000) distanceCm[2] = dur * 0.0343f / 2.0f;
    }
}

int16_t vx = 0, vy = 0, omega = 0;
uint8_t buzzer = 0, pidEnable = 0;
int16_t currentTarget[4] = {0, 0, 0, 0};
unsigned long lastInputTime = 0;

IPAddress laptopIP;
uint16_t laptopPort = 0;
bool hasWifiClient = false;

MotionFeedbackPacket latestMotionFB;
unsigned long lastMotionFBTime = 0;

// Tốc độ tối đa của động cơ FIT0450 (khoảng 140 RPM ở 12V)
const float MAX_TARGET_RPM = 130.0f;

void sendToMotion2350() {
    int16_t raw_fl = constrain(vx - vy - omega, -255, 255);
    int16_t raw_fr = constrain(vx + vy + omega, -255, 255);
    int16_t raw_rl = constrain(vx + vy - omega, -255, 255);
    int16_t raw_rr = constrain(vx - vy + omega, -255, 255);
    int16_t raw[4] = {raw_fl, raw_fr, raw_rl, raw_rr};

    ControlPacket pkt;
    pkt.pid_enable = pidEnable;
    for (int i = 0; i < 4; i++) {
        if (pidEnable == 1) {
            // Quy đổi thang -255..255 sang vận tốc mục tiêu RPM (-130..+130 RPM)
            currentTarget[i] = (int16_t)((raw[i] / 255.0f) * MAX_TARGET_RPM);
        } else {
            currentTarget[i] = raw[i];
        }
        pkt.target_val[i] = currentTarget[i];
    }
    pkt.buzzer_cmd = buzzer;
    pkt.checksum   = calculateChecksum(pkt);

    MotionSerial.write(reinterpret_cast<const uint8_t*>(&pkt), sizeof(ControlPacket));
}

void setup() {
    Serial.begin(115200);
    MotionSerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    // Khởi tạo 3 cảm biến SR-04 với ngắt ngoài
    for (int i = 0; i < 3; i++) {
        pinMode(TRIG_PINS[i], OUTPUT);
        digitalWrite(TRIG_PINS[i], LOW);
        pinMode(ECHO_PINS[i], INPUT);
    }
    attachInterrupt(digitalPinToInterrupt(ECHO_PINS[0]), echoISR0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ECHO_PINS[1]), echoISR1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ECHO_PINS[2]), echoISR2, CHANGE);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS, 6, 0, 2);
    WiFi.setSleep(false);
    udp.begin(UDP_PORT);

    memset(&latestMotionFB, 0, sizeof(latestMotionFB));
}

void loop() {
    // 1A. Nhận lệnh từ Laptop qua Wi-Fi UDP
    int packetSize = udp.parsePacket();
    if (packetSize == sizeof(JoyPacket)) {
        laptopIP = udp.remoteIP();
        laptopPort = udp.remotePort();
        hasWifiClient = true;

        JoyPacket joy;
        udp.read(reinterpret_cast<uint8_t*>(&joy), sizeof(JoyPacket));
        vx = joy.vx; vy = joy.vy; omega = joy.omega;
        buzzer = joy.buzzer; pidEnable = joy.pid_enable;
        lastInputTime = millis();
    }

    // 1B. Nhận lệnh từ Laptop qua cáp USB (nếu đang cắm cáp)
    while (Serial.available() >= (int)sizeof(JoyPacket) + 1) {
        if (Serial.peek() != 0xFE) {
            Serial.read();
            continue;
        }
        Serial.read(); // Bỏ byte 0xFE
        JoyPacket joy;
        Serial.readBytes(reinterpret_cast<uint8_t*>(&joy), sizeof(JoyPacket));
        vx = joy.vx; vy = joy.vy; omega = joy.omega;
        buzzer = joy.buzzer; pidEnable = joy.pid_enable;
        lastInputTime = millis();
    }

    if (millis() - lastInputTime > 250) {
        vx = 0; vy = 0; omega = 0; buzzer = 0;
    }

    // 2. Kích xung xoay vòng 3 cảm biến siêu âm mỗi 30ms (không chặn CPU)
    static unsigned long lastPing = 0;
    static uint8_t currentSensor = 0;
    if (millis() - lastPing >= 30) {
        lastPing = millis();
        uint8_t pin = TRIG_PINS[currentSensor];
        digitalWrite(pin, HIGH);
        delayMicroseconds(10);
        digitalWrite(pin, LOW);
        currentSensor = (currentSensor + 1) % 3;
    }

    // 3. Nhận gói phản hồi từ Motion 2350 Pro
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

    // 4. Gửi lệnh xuống Motion 2350 Pro mỗi 20ms (50Hz)
    static unsigned long lastSend = 0;
    if (millis() - lastSend >= 20) {
        lastSend = millis();
        sendToMotion2350();
        buzzer = 0;
    }

    // 5. Gửi gói Telemetry về Laptop qua cả Wi-Fi UDP và USB Serial mỗi 40ms
    static unsigned long lastTele = 0;
    if (millis() - lastTele >= 40) {
        lastTele = millis();

        WifiTelemetryPacket tele;
        tele.pid_mode = pidEnable;
        tele.vx = vx; tele.vy = vy; tele.omega = omega;

        bool motionAlive = (millis() - lastMotionFBTime < 300);
        tele.motion_alive = motionAlive ? 1 : 0;

        for (int i = 0; i < 4; i++) {
            tele.target_val[i] = currentTarget[i];
            tele.actual_pwm[i] = motionAlive ? latestMotionFB.actual_pwm[i] : (pidEnable ? 0 : currentTarget[i]);
            tele.actual_rpm[i] = motionAlive ? latestMotionFB.actual_rpm[i] : 0;
        }
        for (int i = 0; i < 3; i++) {
            tele.us_cm[i] = distanceCm[i];
        }
        tele.uptime_ms = millis();
        tele.checksum  = calculateChecksum(tele);

        // Bắn qua Wi-Fi UDP nếu Laptop đang kết nối Wi-Fi xe
        if (hasWifiClient) {
            udp.beginPacket(laptopIP, laptopPort);
            udp.write(reinterpret_cast<const uint8_t*>(&tele), sizeof(WifiTelemetryPacket));
            udp.endPacket();
        }

        // Bắn song song qua cáp USB để cắm dây vẫn lên Dashboard đầy đủ
        Serial.write(reinterpret_cast<const uint8_t*>(&tele), sizeof(WifiTelemetryPacket));
    }
}