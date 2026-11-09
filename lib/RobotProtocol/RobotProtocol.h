#pragma once
#include <Arduino.h>

#pragma pack(push, 1)

// 1. Gói tin điều khiển: ESP32-S3 -> Motion 2350 Pro (Qua UART)
struct ControlPacket {
    uint8_t header1 = 0xAA;
    uint8_t header2 = 0x55;
    int16_t motor_pwm[4];    // PWM mục tiêu cho 4 bánh: FL, FR, RL, RR (-255 đến 255)
    uint8_t buzzer_cmd = 0;  // 1 = Kêu còi, 0 = Tắt
    uint8_t checksum;
};

// 2. Gói tin phản hồi: Motion 2350 Pro -> ESP32-S3 (Qua UART)
struct MotionFeedbackPacket {
    uint8_t header1 = 0xBB;
    uint8_t header2 = 0x66;
    int16_t actual_pwm[4];   // PWM thực tế đang xuất ra 4 động cơ
    int16_t actual_rpm[4];   // Tốc độ vòng/phút từ 4 Encoder (sẵn sàng cho PID)
    uint32_t uptime_ms;      // Thời gian chạy của Motion 2350 Pro
    uint8_t checksum;
};

// 3. Gói tin Telemetry không dây: ESP32-S3 -> Laptop (Qua Wi-Fi UDP)
struct WifiTelemetryPacket {
    uint8_t header1 = 0xCC;
    uint8_t header2 = 0x77;
    int16_t vx;              // Vận tốc tiến/lùi (-255 đến 255)
    int16_t vy;              // Vận tốc trượt ngang (-255 đến 255)
    int16_t omega;           // Vận tốc xoay (-255 đến 255)
    int16_t actual_pwm[4];   // PWM thực tế xác nhận từ Motion 2350 Pro
    int16_t actual_rpm[4];   // Tốc độ RPM thực tế của 4 bánh
    int8_t  wifi_rssi;       // Cường độ sóng Wi-Fi (dBm)
    uint8_t motion_alive;    // 1 = Kết nối UART với Motion 2350 tốt, 0 = Mất kết nối
    uint32_t uptime_ms;      // Thời gian hoạt động của hệ thống (ms)
    uint8_t checksum;
};

#pragma pack(pop)

// Hàm tính Checksum XOR dùng chung cho mọi loại gói tin
template <typename T>
inline uint8_t calculateChecksum(const T& pkt) {
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&pkt);
    uint8_t xor_sum = 0;
    for (size_t i = 0; i < sizeof(T) - 1; i++) {
        xor_sum ^= ptr[i];
    }
    return xor_sum;
}