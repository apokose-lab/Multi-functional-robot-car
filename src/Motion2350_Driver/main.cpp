#include <Arduino.h>
#include <RobotProtocol.h>

// 1. Chân cầu H cho 4 động cơ trên Cytron Motion 2350 Pro
const uint8_t M_PINS[4][2] = {
    {8, 9},   // M1: Front-Left  (GP8, GP9)
    {10, 11}, // M2: Front-Right (GP10, GP11)
    {12, 13}, // M3: Rear-Left   (GP12, GP13)
    {14, 15}  // M4: Rear-Right  (GP14, GP15)
};

// 2. Chân đọc 4 bộ Encoder (Pha A, Pha B) trên Motion 2350 Pro
const uint8_t ENC_A[4] = {4, 6, 18, 26}; // Dây Xanh lá (Pha A) của M1, M2, M3, M4
const uint8_t ENC_B[4] = {5, 7, 19, 27}; // Dây Vàng    (Pha B) của M1, M2, M3, M4

// Đảo dấu thành -1 nếu bánh nào quay tiến mà RPM lại hiện số âm trên Dashboard
const int8_t ENC_DIR[4] = {1, 1, 1, 1};

// Biến đếm xung trực tiếp bằng ngắt phần cứng (Không cần thư viện ngoài)
volatile long encCount[4] = {0, 0, 0, 0};

void isrEnc0() {
    bool a = digitalReadFast(ENC_A[0]);
    bool b = digitalReadFast(ENC_B[0]);
    encCount[0] += (a == b) ? 1 : -1;
}
void isrEnc1() {
    bool a = digitalReadFast(ENC_A[1]);
    bool b = digitalReadFast(ENC_B[1]);
    encCount[1] += (a == b) ? 1 : -1;
}
void isrEnc2() {
    bool a = digitalReadFast(ENC_A[2]);
    bool b = digitalReadFast(ENC_B[2]);
    encCount[2] += (a == b) ? 1 : -1;
}
void isrEnc3() {
    bool a = digitalReadFast(ENC_A[3]);
    bool b = digitalReadFast(ENC_B[3]);
    encCount[3] += (a == b) ? 1 : -1;
}

// 3. Thông số hộp số FIT0450 và hệ số PID
// (Đếm 2 sườn xung CHANGE trên kênh A -> khoảng 680 xung/vòng trục ra)
const float PULSES_PER_REV = 680.0f;
const float KP = 1.4f;
const float KI = 3.5f;
const float KD = 0.03f;

const uint8_t BUZZER_PIN = 22;
unsigned long lastPacketTime = 0;

uint8_t pidMode = 0;
int16_t targetVal[4]   = {0, 0, 0, 0};
int16_t appliedPWM[4]  = {0, 0, 0, 0};
int16_t measuredRPM[4] = {0, 0, 0, 0};

long prevEncCount[4]   = {0, 0, 0, 0};
float integralErr[4]   = {0, 0, 0, 0};
float prevErr[4]       = {0, 0, 0, 0};

void setMotorHardware(uint8_t i, int16_t pwm) {
    pwm = constrain(pwm, -255, 255);
    appliedPWM[i] = pwm;
    if (pwm > 0) {
        analogWrite(M_PINS[i][0], pwm);
        analogWrite(M_PINS[i][1], 0);
    } else if (pwm < 0) {
        analogWrite(M_PINS[i][0], 0);
        analogWrite(M_PINS[i][1], -pwm);
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
    Serial.begin(115200);

    analogWriteFreq(20000);
    analogWriteRange(255);
    for (int i = 0; i < 4; i++) {
        pinMode(M_PINS[i][0], OUTPUT);
        pinMode(M_PINS[i][1], OUTPUT);

        // Bật điện trở kéo lên nội bộ (INPUT_PULLUP) giúp tín hiệu Encoder chống nhiễu
        pinMode(ENC_A[i], INPUT_PULLUP);
        pinMode(ENC_B[i], INPUT_PULLUP);
    }

    // Gắn ngắt phần cứng cho 4 bộ Encoder
    attachInterrupt(digitalPinToInterrupt(ENC_A[0]), isrEnc0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_A[1]), isrEnc1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_A[2]), isrEnc2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_A[3]), isrEnc3, CHANGE);

    stopAllMotors();

    pinMode(BUZZER_PIN, OUTPUT);
    tone(BUZZER_PIN, 2000, 100);
    delay(150);
    tone(BUZZER_PIN, 2500, 100);

    // Giao tiếp UART0 với ESP32-S3
    Serial1.setTX(16);
    Serial1.setRX(17);
    Serial1.begin(115200);
}

void loop() {
    // 1. Đọc gói lệnh từ ESP32-S3
    while (Serial1.available() >= (int)sizeof(ControlPacket)) {
        if (Serial1.peek() != 0xAA) {
            Serial1.read();
            continue;
        }
        ControlPacket pkt;
        Serial1.readBytes(reinterpret_cast<uint8_t*>(&pkt), sizeof(ControlPacket));

        if (pkt.header1 == 0xAA && pkt.header2 == 0x55 && pkt.checksum == calculateChecksum(pkt)) {
            lastPacketTime = millis();
            pidMode = pkt.pid_enable;
            for (int i = 0; i < 4; i++) {
                targetVal[i] = pkt.target_val[i];
            }
            if (pkt.buzzer_cmd == 1) {
                tone(BUZZER_PIN, 1800, 80);
            }
        }
    }

    // 2. Watchdog an toàn (300ms)
    if (millis() - lastPacketTime > 300) {
        stopAllMotors();
    }

    // 3. Vòng lặp đọc Encoder & Tính toán PID cố định mỗi 20ms (50Hz)
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

            // Tính tốc độ thực tế (RPM)
            float rpm = ((float)delta / PULSES_PER_REV) * (60.0f / dt);
            measuredRPM[i] = (int16_t)rpm;

            if (pidMode == 1) {
                // Chế độ 1: Điều khiển PID bám theo vận tốc mục tiêu (RPM)
                float targetRPM = (float)targetVal[i];
                if (abs(targetRPM) < 2.0f) {
                    integralErr[i] = 0;
                    prevErr[i] = 0;
                    setMotorHardware(i, 0);
                } else {
                    float err = targetRPM - rpm;
                    integralErr[i] = constrain(integralErr[i] + err * dt, -150.0f, 150.0f);
                    float deriv = (err - prevErr[i]) / dt;
                    prevErr[i] = err;

                    float outPWM = KP * err + KI * integralErr[i] + KD * deriv;
                    setMotorHardware(i, (int16_t)constrain(outPWM, -255.0f, 255.0f));
                }
            } else {
                // Chế độ 0: Xuất trực tiếp PWM (Open-loop)
                integralErr[i] = 0;
                prevErr[i] = 0;
                setMotorHardware(i, targetVal[i]);
            }
        }
    }

    // 4. Gửi gói phản hồi (PWM + RPM thực tế) về ESP32-S3 mỗi 40ms
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