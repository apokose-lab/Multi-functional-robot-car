#include <Arduino.h>
#include <RobotProtocol.h>

// Định nghĩa chân 4 Cầu H tích hợp trên Cytron Motion 2350 Pro
const uint8_t M_PINS[4][2] = {
    {14, 15},   // Motor 1: Front-Left  (M1A = GP8,  M1B = GP9)
    {12, 13}, // Motor 2: Front-Right (M2A = GP10, M2B = GP11)
    {10, 11}, // Motor 3: Rear-Left   (M3A = GP12, M3B = GP13)
    {8, 9}  // Motor 4: Rear-Right  (M4A = GP14, M4B = GP15)
};

const uint8_t BUZZER_PIN = 22; // Buzzer tích hợp sẵn trên Motion 2350 Pro
unsigned long lastPacketTime = 0;

// Hàm điều khiển 1 động cơ với giá trị PWM từ -255 đến +255
void setMotor(uint8_t index, int16_t pwm) {
    pwm = constrain(pwm, -255, 255);
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

    // Cấu hình chân PWM cho 4 động cơ
    analogWriteFreq(20000); // Tần số 20kHz giúp động cơ chạy êm
    analogWriteRange(255);
    for (int i = 0; i < 4; i++) {
        pinMode(M_PINS[i][0], OUTPUT);
        pinMode(M_PINS[i][1], OUTPUT);
    }
    stopAllMotors();

    // Cấu hình Buzzer báo khởi động (kêu 2 tiếng bíp ngắn)
    pinMode(BUZZER_PIN, OUTPUT);
    tone(BUZZER_PIN, 2000, 100);
    delay(150);
    tone(BUZZER_PIN, 2500, 100);

    // Cấu hình UART0 tại chân GP16 (TX) và GP17 (RX) để nhận lệnh từ ESP32-S3
    Serial1.setTX(16);
    Serial1.setRX(17);
    Serial1.begin(115200);
}

void loop() {
    // Kiểm tra và đọc gói tin UART từ ESP32-S3
    while (Serial1.available() >= (int)sizeof(ControlPacket)) {
        if (Serial1.peek() != 0xAA) {
            Serial1.read(); // Bỏ qua byte rác nếu lệch khung truyền
            continue;
        }

        ControlPacket pkt;
        Serial1.readBytes(reinterpret_cast<uint8_t*>(&pkt), sizeof(ControlPacket));

        if (pkt.header1 == 0xAA && pkt.header2 == 0x55 && pkt.checksum == calculateChecksum(pkt)) {
            lastPacketTime = millis();

            // Xuất PWM ra 4 động cơ
            for (int i = 0; i < 4; i++) {
                setMotor(i, pkt.motor_pwm[i]);
            }

            // Điều khiển Buzzer nếu có lệnh
            if (pkt.buzzer_cmd == 1) {
                tone(BUZZER_PIN, 1800, 80);
            }
        }
    }

    // Cơ chế an toàn (Watchdog): Tự dừng xe nếu mất kết nối UART quá 300ms
    if (millis() - lastPacketTime > 300) {
        stopAllMotors();
    }
}