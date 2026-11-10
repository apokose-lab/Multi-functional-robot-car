#include <Arduino.h>
#include <RobotProtocol.h>

// 1. Chân UART kết nối với ESP32-S3 (43/44)
const uint8_t MOTION_UART_TX = 16; // GP16 (TX) -> GPIO 44 (RX) của ESP32-S3
const uint8_t MOTION_UART_RX = 17; // GP17 (RX) -> GPIO 43 (TX) của ESP32-S3

// 2. Chân 4 Cầu H Động cơ
const uint8_t M_PINS[4][2] = {
    {8, 9},   // M1: Front-Left
    {10, 11}, // M2: Front-Right
    {12, 13}, // M3: Rear-Left
    {14, 15}  // M4: Rear-Right
};

// 3. Chân đọc 4 bộ Encoder (M4 đổi sang GP26, GP27 vì GP20, GP21 là nút nhấn trên mạch)
const uint8_t ENC_A[4] = {4, 6, 18, 26}; // Pha A của M1, M2, M3, M4
const uint8_t ENC_B[4] = {5, 7, 19, 27}; // Pha B của M1, M2, M3, M4

// Bảng đảo chiều Động cơ và Encoder (1 hoặc -1)
const int8_t MOTOR_DIR[4] = {1, 1, 1, 1};
const int8_t ENC_DIR[4]   = {1, 1, 1, 1};

// Danh sách 12 chân GPIO để quét xung trực tiếp gửi lên Dashboard
const uint8_t SCAN_PINS[12] = {4, 5, 6, 7, 18, 19, 20, 21, 26, 27, 28, 29};
uint8_t prevScanState[12];
volatile uint16_t gpioPulseMask = 0;

// Biến đếm xung độc lập cho cả Pha A và Pha B (Chống triệt tiêu xung khi lỏng 1 dây)
volatile long encCount[4] = {0, 0, 0, 0};
volatile uint32_t pulseA[4] = {0, 0, 0, 0};
volatile uint32_t pulseB[4] = {0, 0, 0, 0};

void handleEncA(uint8_t i) {
    pulseA[i]++;
    encCount[i] += digitalReadFast(ENC_B[i]) ? -1 : 1;
}
void handleEncB(uint8_t i) {
    pulseB[i]++;
}

void isrA0() { handleEncA(0); } void isrB0() { handleEncB(0); }
void isrA1() { handleEncA(1); } void isrB1() { handleEncB(1); }
void isrA2() { handleEncA(2); } void isrB2() { handleEncB(2); }
void isrA3() { handleEncA(3); } void isrB3() { handleEncB(3); }

// Đếm 1 sườn RISING trên kênh A -> khoảng 340 xung/vòng trục ra của FIT0450
const float PULSES_PER_REV = 340.0f;
const float KP  = 1.2f;
const float KI  = 2.5f;
const float KD  = 0.02f;
const float KFF = 1.6f;

const uint8_t BUZZER_PIN = 22;
unsigned long lastPacketTime = 0;

uint8_t pidMode = 0;
int16_t targetVal[4]    = {0, 0, 0, 0};
int16_t appliedPWM[4]   = {0, 0, 0, 0};
int16_t measuredRPM[4]  = {0, 0, 0, 0};
uint8_t encHealth[4]    = {0, 0, 0, 0};
float filteredRPM[4]    = {0, 0, 0, 0};

long prevEncCount[4]    = {0, 0, 0, 0};
uint32_t prevPulseA[4]  = {0, 0, 0, 0};
uint32_t prevPulseB[4]  = {0, 0, 0, 0};
float integralErr[4]    = {0, 0, 0, 0};
float prevErr[4]        = {0, 0, 0, 0};

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

    for (int i = 0; i < 12; i++) {
        pinMode(SCAN_PINS[i], INPUT_PULLUP);
        prevScanState[i] = digitalReadFast(SCAN_PINS[i]);
    }

    for (int i = 0; i < 4; i++) {
        pinMode(M_PINS[i][0], OUTPUT);
        pinMode(M_PINS[i][1], OUTPUT);
        pinMode(ENC_A[i], INPUT_PULLUP);
        pinMode(ENC_B[i], INPUT_PULLUP);
    }

    // Dùng RISING thay vì CHANGE để tránh tự triệt tiêu +1/-1 nếu lỏng 1 dây
    attachInterrupt(digitalPinToInterrupt(ENC_A[0]), isrA0, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_B[0]), isrB0, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_A[1]), isrA1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_B[1]), isrB1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_A[2]), isrA2, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_B[2]), isrB2, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_A[3]), isrA3, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_B[3]), isrB3, RISING);

    stopAllMotors();

    pinMode(BUZZER_PIN, OUTPUT);
    tone(BUZZER_PIN, 2000, 100);
    delay(150);
    tone(BUZZER_PIN, 2500, 100);

    Serial1.setTX(MOTION_UART_TX);
    Serial1.setRX(MOTION_UART_RX);
    Serial1.begin(115200);
}

void loop() {
    // 0. Quét liên tục 12 chân GPIO để phát hiện chân nào có xung Encoder gửi lên Dashboard
    for (int k = 0; k < 12; k++) {
        uint8_t s = digitalReadFast(SCAN_PINS[k]);
        if (s != prevScanState[k]) {
            prevScanState[k] = s;
            gpioPulseMask |= (1 << k);
        }
    }

    // 1. Đọc gói lệnh từ ESP32-S3
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

    if (millis() - lastPacketTime > 350) {
        stopAllMotors();
    }

    // 2. Vòng lặp tính tốc độ Encoder & PID mỗi 20ms (50Hz)
    static unsigned long lastPIDTime = 0;
    unsigned long now = millis();
    if (now - lastPIDTime >= 20) {
        float dt = (now - lastPIDTime) / 1000.0f;
        lastPIDTime = now;

        for (int i = 0; i < 4; i++) {
            noInterrupts();
            long count = encCount[i];
            uint32_t pA = pulseA[i];
            uint32_t pB = pulseB[i];
            interrupts();

            long delta = (count - prevEncCount[i]) * ENC_DIR[i];
            uint32_t dA = pA - prevPulseA[i];
            uint32_t dB = pB - prevPulseB[i];
            prevEncCount[i] = count;
            prevPulseA[i]   = pA;
            prevPulseB[i]   = pB;

            // Kiểm tra sức khỏe từng dây Pha A và Pha B
            uint8_t health = 0;
            if (dA > 0) health |= 0x01; // Bit 0: Dây A có xung
            if (dB > 0) health |= 0x02; // Bit 1: Dây B có xung
            encHealth[i] = health;

            // Dự phòng: Nếu dây A bị đứt/lỏng nhưng dây B vẫn sống, tự lấy xung dây B tính tốc độ!
            float pulses = (dA > 0) ? (float)delta : ((float)dB * ENC_DIR[i]);
            float rawRPM = (pulses / PULSES_PER_REV) * (60.0f / dt);
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

    // 3. Gửi gói phản hồi về ESP32-S3 mỗi 40ms
    static unsigned long lastFB = 0;
    if (now - lastFB >= 40) {
        lastFB = now;
        MotionFeedbackPacket fb;
        for (int i = 0; i < 4; i++) {
            fb.actual_pwm[i] = appliedPWM[i];
            fb.actual_rpm[i] = measuredRPM[i];
            fb.enc_health[i] = encHealth[i];
        }
        fb.active_gpios = gpioPulseMask;
        gpioPulseMask = 0; // Reset mặt nạ quét sau mỗi lần gửi
        fb.uptime_ms = now;
        fb.checksum  = calculateChecksum(fb);
        Serial1.write(reinterpret_cast<const uint8_t*>(&fb), sizeof(MotionFeedbackPacket));
    }
}