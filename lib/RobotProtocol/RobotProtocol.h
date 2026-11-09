#pragma once
#include <Arduino.h>

#pragma pack(push, 1)

// 1. Gói tin từ Laptop -> ESP32-S3 (Qua Wi-Fi UDP hoặc USB Serial)
struct JoyPacket {
    int16_t vx;          // -255 đến 255
    int16_t vy;          // -255 đến 255
    int16_t omega;       // -255 đến 255
    uint8_t buzzer;      // 1 = Kêu còi
    uint8_t pid_enable;  // 1 = Chạy PID khoá vận tốc RPM, 0 = Chạy PWM trực tiếp
};

// 2. Gói tin điều khiển: ESP32-S3 -> Motion 2350 Pro (Qua UART)
struct ControlPacket {
    uint8_t header1 = 0xAA;
    uint8_t header2 = 0x55;
    uint8_t pid_enable = 0;  // 1 = PID Mode, 0 = Direct PWM Mode
    int16_t target_val[4];   // Nếu pid_enable=1: Target RPM | Nếu pid_enable=0: Target PWM (-255..255)
    uint8_t buzzer_cmd = 0;
    uint8_t checksum;
};

// 3. Gói tin phản hồi: Motion 2350 Pro -> ESP32-S3 (Qua UART)
struct MotionFeedbackPacket {
    uint8_t header1 = 0xBB;
    uint8_t header2 = 0x66;
    int16_t actual_pwm[4];   // Giá trị PWM thực tế đang cấp cho 4 cầu H (-255..255)
    int16_t actual_rpm[4];   // Tốc độ vòng/phút (RPM) đọc từ 4 PIO Encoder
    uint32_t uptime_ms;
    uint8_t checksum;
};

// 4. Gói tin Telemetry tổng hợp: ESP32-S3 -> Laptop (45 bytes)
struct WifiTelemetryPacket {
    uint8_t header1 = 0xCC;
    uint8_t header2 = 0x77;
    uint8_t pid_mode;        // Trạng thái PID (1 = Bật, 0 = Tắt)
    int16_t vx;
    int16_t vy;
    int16_t omega;
    int16_t target_val[4];   // Giá trị mục tiêu của 4 bánh
    int16_t actual_pwm[4];   // PWM thực tế của 4 bánh
    int16_t actual_rpm[4];   // RPM thực tế từ 4 Encoder
    uint16_t us_cm[3];       // Khoảng cách 3 cảm biến SR-04: [0]=Trước, [1]=Trái, [2]=Phải (cm)
    uint8_t motion_alive;    // 1 = UART OK, 0 = Mất kết nối UART
    uint32_t uptime_ms;
    uint8_t checksum;
};

#pragma pack(pop)

template <typename T>
inline uint8_t calculateChecksum(const T& pkt) {
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&pkt);
    uint8_t xor_sum = 0;
    for (size_t i = 0; i < sizeof(T) - 1; i++) {
        xor_sum ^= ptr[i];
    }
    return xor_sum;
}