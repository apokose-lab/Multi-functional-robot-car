#pragma once
#include <Arduino.h>

#pragma pack(push, 1)
struct ControlPacket {
    uint8_t header1 = 0xAA;
    uint8_t header2 = 0x55;
    int16_t motor_pwm[4];    // Giá trị PWM từ -255 đến +255 cho 4 bánh (FL, FR, RL, RR)
    uint8_t buzzer_cmd = 0;  // 1 = Kêu còi Buzzer, 0 = Tắt
    uint8_t checksum;
};
#pragma pack(pop)

inline uint8_t calculateChecksum(const ControlPacket& pkt) {
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&pkt);
    uint8_t xor_sum = 0;
    for (size_t i = 0; i < sizeof(ControlPacket) - 1; i++) {
        xor_sum ^= ptr[i];
    }
    return xor_sum;
}