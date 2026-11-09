#include <Arduino.h>
#include <RobotProtocol.h>

const uint8_t M_PINS[4][2] = {
    {8, 9},   // Motor 1: Front-Left  (M1A = GP8,  M1B = GP9)
    {10, 11}, // Motor 2: Front-Right (M2A = GP10, M2B = GP11)
    {12, 13}, // Motor 3: Rear-Left   (M3A = GP12, M3B = GP13)
    {14, 15}  // Motor 4: Rear-Right  (M4A = GP14, M4B = GP15)
};

const uint8_t BUZZER_PIN = 22;
unsigned long lastPacketTime = 0;
int16_t applied_pwm[4] = {0, 0, 0, 0};

void setMotor(uint8_t index, int16_t pwm) {
    pwm = constrain(pwm, -255, 255);
    applied_pwm[index] = pwm; // Lưu lại giá trị thực tế đang cấp ra cầu H
    if (pwm > 0) {
        analogWrite(M_PINS[index][0], pwm);
        analogWrite(M_PINS[index][1], 0);
    } else if (pwm < 0) {
        analogWrite(M_PINS[index][0], 0);
        analogWrite(M_PINS[index][1], -pwm);
    } else {
        analogWrite(M_PINS[index][0], 0);
        analogWrite(M_PINS[index][1], 0);
    }
}

void stopAllMotors() {
    for (int i = 0; i < 4; i++) setMotor(i, 0);
}

void setup() {
    Serial.begin(115200);

    analogWriteFreq(20000);
    analogWriteRange(255);
    for (int i = 0; i < 4; i++) {
        pinMode(M_PINS[i][0], OUTPUT);
        pinMode(M_PINS[i][1], OUTPUT);
    }
    stopAllMotors();

    pinMode(BUZZER_PIN, OUTPUT);
    tone(BUZZER_PIN, 2000, 100);
    delay(150);
    tone(BUZZER_PIN, 2500, 100);

    // UART0: GP16 (TX) nối sang RX của ESP32-S3 | GP17 (RX) nối sang TX của ESP32-S3
    Serial1.setTX(16);
    Serial1.setRX(17);
    Serial1.begin(115200);
}

void loop() {
    // 1. Nhận lệnh điều khiển từ ESP32-S3
    while (Serial1.available() >= (int)sizeof(ControlPacket)) {
        if (Serial1.peek() != 0xAA) {
            Serial1.read();
            continue;
        }

        ControlPacket pkt;
        Serial1.readBytes(reinterpret_cast<uint8_t*>(&pkt), sizeof(ControlPacket));

        if (pkt.header1 == 0xAA && pkt.header2 == 0x55 && pkt.checksum == calculateChecksum(pkt)) {
            lastPacketTime = millis();
            for (int i = 0; i < 4; i++) {
                setMotor(i, pkt.motor_pwm[i]);
            }
            if (pkt.buzzer_cmd == 1) {
                tone(BUZZER_PIN, 1800, 80);
            }
        }
    }

    // 2. Watchdog an toàn: Tự ngắt nếu mất tín hiệu từ ESP32-S3 quá 300ms
    if (millis() - lastPacketTime > 300) {
        stopAllMotors();
    }

    // 3. Gửi gói tin phản hồi (Feedback) ngược lên ESP32-S3 mỗi 40ms (25Hz)
    static unsigned long lastFeedback = 0;
    if (millis() - lastFeedback >= 40) {
        lastFeedback = millis();
        MotionFeedbackPacket fb;
        for (int i = 0; i < 4; i++) {
            fb.actual_pwm[i] = applied_pwm[i];
            fb.actual_rpm[i] = 0; // Sẽ gán giá trị từ PIO Encoder ở bước lắp PID
        }
        fb.uptime_ms = millis();
        fb.checksum  = calculateChecksum(fb);

        Serial1.write(reinterpret_cast<const uint8_t*>(&fb), sizeof(MotionFeedbackPacket));
    }
}