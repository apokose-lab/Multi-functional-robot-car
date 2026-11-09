#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <RobotProtocol.h>

const char* AP_SSID = "Mecanum_Robot";
const char* AP_PASS = "12345678";
const uint16_t UDP_PORT = 4210;

WiFiUDP udp;
HardwareSerial MotionSerial(1);

// =========================================================================
// 1. KHAI BÁO CHÂN UART KẾT NỐI VỚI MOTION 2350 PRO (CHÂN 16, 17 ĐỂ CHO LIDAR)
// =========================================================================
const int UART_RX_PIN = 44; // Chân RX (GPIO 44) của ESP32-S3 <- nối vào GP16 (TX) của Motion 2350 Pro
const int UART_TX_PIN = 43; // Chân TX (GPIO 43) của ESP32-S3 -> nối vào GP17 (RX) của Motion 2350 Pro

// =========================================================================
// 2. KHAI BÁO CHÂN 3 CẢM BIẾN SIÊU ÂM SR-04 (TRƯỚC, TRÁI, PHẢI)
// =========================================================================
const uint8_t TRIG_PINS[3] = {4, 6, 10};
const uint8_t ECHO_PINS[3] = {5, 8, 11};

volatile uint32_t echoStart[3] = {0, 0, 0};
volatile uint16_t distanceCm[3] = {200, 200, 200};

void IRAM_ATTR handleEchoISR(uint8_t idx) {
    uint32_t now = micros();
    if (digitalRead(ECHO_PINS[idx])) {
        echoStart[idx] = now;
    } else {
        uint32_t dur = now - echoStart[idx];
        if (dur > 100 && dur < 24000) {
            distanceCm[idx] = (uint16_t)(dur * 0.01715f);
        }
    }
}
void IRAM_ATTR echoISR0() { handleEchoISR(0); }
void IRAM_ATTR echoISR1() { handleEchoISR(1); }
void IRAM_ATTR echoISR2() { handleEchoISR(2); }

int16_t vx = 0, vy = 0, omega = 0;
uint8_t buzzer = 0, pidEnable = 0, testWheel = 0;
int16_t currentTarget[4] = {0, 0, 0, 0};
unsigned long lastInputTime = 0;

IPAddress laptopIP;
uint16_t laptopPort = 0;
bool hasWifiClient = false;

MotionFeedbackPacket latestMotionFB;
unsigned long lastMotionFBTime = 0;

uint8_t fbBuf[sizeof(MotionFeedbackPacket)];
uint8_t fbIdx = 0;

const float MAX_TARGET_RPM = 120.0f;

void sendToMotion2350() {
    ControlPacket pkt;

    // Chế độ bấm phím 1, 2, 3, 4 trên bàn phím để kiểm tra từng bánh
    if (testWheel >= 1 && testWheel <= 4) {
        pkt.pid_enable = 0;
        for (int i = 0; i < 4; i++) {
            currentTarget[i] = (i == (testWheel - 1)) ? 150 : 0;
            pkt.target_val[i] = currentTarget[i];
        }
    } else {
        int16_t raw_fl = constrain(vx - vy - omega, -255, 255); // M1: Front-Left
        int16_t raw_fr = constrain(vx + vy + omega, -255, 255); // M2: Front-Right
        int16_t raw_rl = constrain(vx + vy - omega, -255, 255); // M3: Rear-Left
        int16_t raw_rr = constrain(vx - vy + omega, -255, 255); // M4: Rear-Right
        int16_t raw[4] = {raw_fl, raw_fr, raw_rl, raw_rr};

        pkt.pid_enable = pidEnable;
        for (int i = 0; i < 4; i++) {
            if (pidEnable == 1) {
                currentTarget[i] = (int16_t)((raw[i] / 255.0f) * MAX_TARGET_RPM);
            } else {
                currentTarget[i] = raw[i];
            }
            pkt.target_val[i] = currentTarget[i];
        }
    }

    pkt.buzzer_cmd = buzzer;
    pkt.checksum   = calculateChecksum(pkt);
    MotionSerial.write(reinterpret_cast<const uint8_t*>(&pkt), sizeof(ControlPacket));
}

void setup() {
    // QUAN TRỌNG: Giải phóng UART0 mặc định để chân 43 và 44 không bị xung đột
    Serial.end();

    MotionSerial.setRxBufferSize(512);
    MotionSerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    for (int i = 0; i < 3; i++) {
        pinMode(TRIG_PINS[i], OUTPUT);
        digitalWrite(TRIG_PINS[i], LOW);
        pinMode(ECHO_PINS[i], INPUT_PULLDOWN);
    }
    attachInterrupt(digitalPinToInterrupt(ECHO_PINS[0]), echoISR0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ECHO_PINS[1]), echoISR1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ECHO_PINS[2]), echoISR2, CHANGE);

    WiFi.persistent(false);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS, 6, 0, 2);
    WiFi.setSleep(false);
    WiFi.setTxPower(WIFI_POWER_15dBm);
    udp.begin(UDP_PORT);

    memset(&latestMotionFB, 0, sizeof(latestMotionFB));
}

void loop() {
    // 1. Nhận lệnh điều khiển không dây từ Laptop qua Wi-Fi UDP
    int packetSize = udp.parsePacket();
    if (packetSize == sizeof(JoyPacket)) {
        laptopIP = udp.remoteIP();
        laptopPort = udp.remotePort();
        hasWifiClient = true;

        JoyPacket joy;
        udp.read(reinterpret_cast<uint8_t*>(&joy), sizeof(JoyPacket));
        vx = joy.vx; vy = joy.vy; omega = joy.omega;
        buzzer = joy.buzzer; pidEnable = joy.pid_enable; testWheel = joy.test_wheel;
        lastInputTime = millis();
    } else if (packetSize > 0) {
        udp.flush();
    }

    if (millis() - lastInputTime > 300) {
        vx = 0; vy = 0; omega = 0; buzzer = 0; testWheel = 0;
    }

    // 2. Kích xung xoay vòng 3 cảm biến SR-04 mỗi 35ms
    static unsigned long lastPing = 0;
    static uint8_t currentSensor = 0;
    if (millis() - lastPing >= 35) {
        lastPing = millis();
        uint8_t pin = TRIG_PINS[currentSensor];
        digitalWrite(pin, HIGH);
        delayMicroseconds(10);
        digitalWrite(pin, LOW);
        currentSensor = (currentSensor + 1) % 3;
    }

    // 3. Đọc gói phản hồi từ Motion 2350 Pro qua chân GPIO 44 (RX)
    while (MotionSerial.available() > 0) {
        uint8_t b = MotionSerial.read();
        if (fbIdx == 0) {
            if (b == 0xBB) fbBuf[fbIdx++] = b;
        } else if (fbIdx == 1) {
            if (b == 0x66) fbBuf[fbIdx++] = b;
            else if (b == 0xBB) fbIdx = 1;
            else fbIdx = 0;
        } else {
            fbBuf[fbIdx++] = b;
            if (fbIdx == sizeof(MotionFeedbackPacket)) {
                fbIdx = 0;
                MotionFeedbackPacket fb;
                memcpy(&fb, fbBuf, sizeof(MotionFeedbackPacket));
                if (fb.checksum == calculateChecksum(fb)) {
                    latestMotionFB = fb;
                    lastMotionFBTime = millis();
                }
            }
        }
    }

    // 4. Gửi lệnh xuống Motion 2350 Pro qua chân GPIO 43 (TX) mỗi 20ms (50Hz)
    static unsigned long lastSend = 0;
    if (millis() - lastSend >= 20) {
        lastSend = millis();
        sendToMotion2350();
        buzzer = 0;
    }

    // 5. Gửi Telemetry không dây về Laptop mỗi 50ms (20Hz)
    static unsigned long lastTele = 0;
    if (hasWifiClient && (millis() - lastTele >= 50)) {
        lastTele = millis();

        WifiTelemetryPacket tele;
        tele.pid_mode = pidEnable;
        tele.vx = vx; tele.vy = vy; tele.omega = omega;

        bool motionAlive = (millis() - lastMotionFBTime < 400);
        tele.motion_alive = motionAlive ? 1 : 0;

        for (int i = 0; i < 4; i++) {
            tele.target_val[i] = currentTarget[i];
            tele.actual_pwm[i] = motionAlive ? latestMotionFB.actual_pwm[i] : 0;
            tele.actual_rpm[i] = motionAlive ? latestMotionFB.actual_rpm[i] : 0;
        }
        for (int i = 0; i < 3; i++) {
            tele.us_cm[i] = distanceCm[i];
        }
        tele.uptime_ms = millis();
        tele.checksum  = calculateChecksum(tele);

        udp.beginPacket(laptopIP, laptopPort);
        udp.write(reinterpret_cast<const uint8_t*>(&tele), sizeof(WifiTelemetryPacket));
        udp.endPacket();
    }
}