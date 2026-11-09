#include <Arduino.h>
#include <RobotProtocol.h>

// =========================================================================
// 1. KHAI BÁO CHÂN GIAO TIẾP UART VỚI ESP32-S3 TRÊN MOTION 2350 PRO
// =========================================================================
const uint8_t MOTION_UART_TX = 16; // Chân GP16 (TX) -> nối sang GPIO 44 (RX) của ESP32-S3
const uint8_t MOTION_UART_RX = 17; // Chân GP17 (RX) -> nối sang GPIO 43 (TX) của ESP32-S3

// =========================================================================
// 2. KHAI BÁO CHÂN ĐỘNG CƠ, ENCODER & BẢNG ĐẢO CHIỀU
// =========================================================================
const uint8_t M_PINS[4][2] = {
    {8, 9},   // M1: Front-Left  (Trái Trước)
    {10, 11}, // M2: Front-Right (Phải Trước)
    {12, 13}, // M3: Rear-Left   (Trái Sau)
    {14, 15}  // M4: Rear-Right  (Phải Sau)
};

const uint8_t ENC_A[4] = {4, 6, 18, 20};
const uint8_t ENC_B[4] = {5, 7, 19, 21};

// Chỉnh chiều quay Động cơ & Encoder (1 hoặc -1):
// Nếu giữ phím 1, 2, 3, 4 trên Dashboard mà bánh nào quay LÙI -> đổi dấu ở MOTOR_DIR
const int8_t MOTOR_DIR[4] = {1, 1, 1, 1};
// Nếu bánh quay TIẾN mà Encoder RPM hiện SỐ ÂM -> đổi dấu ở ENC_DIR
const int8_t ENC_DIR[4]   = {1, 1, 1, 1};

volatile long encCount[4] = {0, 0, 0, 0};

void isrEnc0() { encCount[0] += (digitalReadFast(ENC_A[0]) == digitalReadFast(ENC_B[0])) ? 1 : -1; }
void isrEnc1() { encCount[1] += (digitalReadFast(ENC_A[1]) == digitalReadFast(ENC_B[1])) ? 1 : -1; }
void isrEnc2() { encCount[2] += (digitalReadFast(ENC_A[2]) == digitalReadFast(ENC_B[2])) ? 1 : -1; }
void isrEnc3() { encCount[3] += (digitalReadFast(ENC_A[3]) == digitalReadFast(ENC_B[3])) ? 1 : -1; }

const float PULSES_PER_REV = 680.0f;
const float KP  = 1.2f;
const float KI  = 2.5f;
const float KD  = 0.02f;
const float KFF = 1.6f; // Hệ số bù trước giúp PID phản hồi ngay không bị trễ

const uint8_t BUZZER_PIN = 22;
unsigned long lastPacketTime = 0;

uint8_t pidMode = 0;
int16_t targetVal[4]   = {0, 0, 0, 0};
int16_t appliedPWM[4]  = {0, 0, 0, 0};
int16_t measuredRPM[4] = {0, 0, 0, 0};
float filteredRPM[4]   = {0, 0, 0, 0};

long prevEncCount[4]   = {0, 0, 0, 0};
float integralErr[4]   = {0, 0, 0, 0};
float prevErr[4]       = {0, 0, 0, 0};

uint8_t rxBuf[sizeof(ControlPacket)];
uint8_t rxIdx = 0;

void setMotorHardware(uint8_t i, int16_t pwm) {
    pwm = constrain(pwm, -255, 255);
    appliedPWM[i] = pwm;

    int16_t hw_pwm = pwm * MOTOR_DIR[i];
    if (hw_pwm > 0) {
        analogWrite(M_PINS[i][0], hw_pwm);
        analogWrite(M_PINS[i][1], 0);
    } else if (hw_pwm < 0) {
        analogWrite(M_PINS[i][0], 0);
        analogWrite(M_PINS[i][1], -hw_pwm);
    } else {
        analogWrite(M_PINS[i][0], 0);
        analogWrite(M_PINS[i][1], 0);
    }
}

void stopAllMotors() {
    for (int i = 0; i < 4; i++) {
        integralErr[i] = 0;
        prevErr[i] = 0;
        targetVal[i] = 0;
        setMotorHardware(i, 0);
    }
}

void setup() {
    analogWriteFreq(20000);
    analogWriteRange(255);

    for (int i = 0; i < 4; i++) {
        pinMode(M_PINS[i][0], OUTPUT);
        pinMode(M_PINS[i][1], OUTPUT);
        pinMode(ENC_A[i], INPUT_PULLUP);
        pinMode(ENC_B[i], INPUT_PULLUP);
    }

    attachInterrupt(digitalPinToInterrupt(ENC_A[0]), isrEnc0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_A[1]), isrEnc1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_A[2]), isrEnc2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_A[3]), isrEnc3, CHANGE);

    stopAllMotors();

    pinMode(BUZZER_PIN, OUTPUT);
    tone(BUZZER_PIN, 2000, 100);
    delay(150);
    tone(BUZZER_PIN, 2500, 100);

    // Khởi tạo UART0 tại chân GP16 (TX) và GP17 (RX) nối sang ESP32-S3
    Serial1.setTX(MOTION_UART_TX);
    Serial1.setRX(MOTION_UART_RX);
    Serial1.begin(115200);
}

void loop() {
    // 1. Đọc gói lệnh từ ESP32-S3 qua GP17 (RX)
    while (Serial1.available() > 0) {
        uint8_t b = Serial1.read();
        if (rxIdx == 0) {
            if (b == 0xAA) rxBuf[rxIdx++] = b;
        } else if (rxIdx == 1) {
            if (b == 0x55) rxBuf[rxIdx++] = b;
            else if (b == 0xAA) rxIdx = 1;
            else rxIdx = 0;
        } else {
            rxBuf[rxIdx++] = b;
            if (rxIdx == sizeof(ControlPacket)) {
                rxIdx = 0;
                ControlPacket pkt;
                memcpy(&pkt, rxBuf, sizeof(ControlPacket));
                if (pkt.checksum == calculateChecksum(pkt)) {
                    lastPacketTime = millis();
                    if (pkt.pid_enable != pidMode) {
                        for (int i = 0; i < 4; i++) { integralErr[i] = 0; prevErr[i] = 0; }
                    }
                    pidMode = pkt.pid_enable;
                    for (int i = 0; i < 4; i++) targetVal[i] = pkt.target_val[i];
                    if (pkt.buzzer_cmd == 1) tone(BUZZER_PIN, 1800, 80);
                }
            }
        }
    }

    // 2. Watchdog an toàn (350ms)
    if (millis() - lastPacketTime > 350) {
        stopAllMotors();
    }

    // 3. Vòng lặp đọc Encoder & Tính PID mỗi 20ms (50Hz)
    static unsigned long lastPIDTime = 0;
    unsigned long now = millis();
    if (now - lastPIDTime >= 20) {
        float dt = (now - lastPIDTime) / 1000.0f;
        lastPIDTime = now;

        for (int i = 0; i < 4; i++) {
            noInterrupts();
            long count = encCount[i] * ENC_DIR[i];
            interrupts();

            long delta = count - prevEncCount[i];
            prevEncCount[i] = count;

            float rawRPM = ((float)delta / PULSES_PER_REV) * (60.0f / dt);
            filteredRPM[i] = 0.6f * filteredRPM[i] + 0.4f * rawRPM;
            measuredRPM[i] = (int16_t)filteredRPM[i];

            if (pidMode == 1) {
                float targetRPM = (float)targetVal[i];
                if (abs(targetRPM) < 3.0f) {
                    integralErr[i] = 0;
                    prevErr[i] = 0;
                    setMotorHardware(i, 0);
                } else {
                    float err = targetRPM - filteredRPM[i];
                    integralErr[i] = constrain(integralErr[i] + err * dt, -80.0f, 80.0f);
                    float deriv = (err - prevErr[i]) / dt;
                    prevErr[i] = err;

                    float outPWM = (KFF * targetRPM) + (KP * err) + (KI * integralErr[i]) + (KD * deriv);
                    setMotorHardware(i, (int16_t)constrain(outPWM, -255.0f, 255.0f));
                }
            } else {
                integralErr[i] = 0;
                prevErr[i] = 0;
                setMotorHardware(i, targetVal[i]);
            }
        }
    }

    // 4. Gửi gói phản hồi về ESP32-S3 qua GP16 (TX) mỗi 40ms
    static unsigned long lastFB = 0;
    if (now - lastFB >= 40) {
        lastFB = now;
        MotionFeedbackPacket fb;
        for (int i = 0; i < 4; i++) {
            fb.actual_pwm[i] = appliedPWM[i];
            fb.actual_rpm[i] = measuredRPM[i];
        }
        fb.uptime_ms = now;
        fb.checksum  = calculateChecksum(fb);
        Serial1.write(reinterpret_cast<const uint8_t*>(&fb), sizeof(MotionFeedbackPacket));
    }
}