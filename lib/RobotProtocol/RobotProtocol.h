#pragma once
#include <Arduino.h>

#pragma pack(push, 1)

// 1. Gói tin từ Laptop -> ESP32-S3 qua Wi-Fi UDP (9 bytes)
struct JoyPacket {
    int16_t vx;          // Tiến / Lùi (-255..255)
    int16_t vy;          // Trượt ngang (-255..255)
    int16_t omega;       // Xoay (-255..255)
    uint8_t buzzer;      // 1 = Còi
    uint8_t pid_enable;  // 1 = PID Mode, 0 = Direct PWM
    uint8_t test_wheel;  // D-Pad Test: 1=M1(Lên), 2=M2(Phải), 3=M3(Xuống), 4=M4(Trái)
};

// 2. Gói tin điều khiển: ESP32-S3 -> Motion 2350 Pro (13 bytes)
struct ControlPacket {
    uint8_t header1 = 0xAA;
    uint8_t header2 = 0x55;
    uint8_t pid_enable = 0;
    int16_t target_val[4];
    uint8_t buzzer_cmd = 0;
    uint8_t checksum;
};

// 3. Gói tin phản hồi: Motion 2350 Pro -> ESP32-S3 (29 bytes)
struct MotionFeedbackPacket {
    uint8_t header1 = 0xBB;
    uint8_t header2 = 0x66;
    int16_t actual_pwm[4];
    int16_t actual_rpm[4];
    uint8_t enc_health[4];   // Bit 0: Pha A có xung | Bit 1: Pha B có xung
    uint16_t active_gpios;   // Bitmask 12 chân GPIO báo chân nào đang có xung Encoder
    uint32_t uptime_ms;
    uint8_t checksum;
};

// 4. Gói tin Telemetry: ESP32-S3 -> Laptop qua Wi-Fi UDP (52 bytes)
struct WifiTelemetryPacket {
    uint8_t header1 = 0xCC;
    uint8_t header2 = 0x77;
    uint8_t pid_mode;
    int16_t vx;
    int16_t vy;
    int16_t omega;
    int16_t target_val[4];
    int16_t actual_pwm[4];
    int16_t actual_rpm[4];
    uint8_t enc_health[4];   // Trạng thái dây A/B của từng Encoder M1..M4
    uint16_t active_gpios;   // Bitmask hiển thị chân GPxx đang nhận xung
    uint16_t us_cm[3];       // Khoảng cách 3 cảm biến SR-04 (cm)
    uint8_t us_status;       // Bit 0=Front OK, Bit 1=Left OK, Bit 2=Right OK (0 = Lỗi/Chưa cắm)
    uint8_t motion_alive;    // 1 = UART 2350 OK, 0 = WAIT
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