#pragma once
#include <Arduino.h>

// Force 1-byte struct alignment so binary packets match across ESP32-S3, RP2350, and Python
#pragma pack(push, 1)

/**
 * @brief 1. Command packet sent from Laptop -> ESP32-S3 over Wi-Fi UDP (9 bytes)
 */
struct JoyPacket {
    int16_t vx;          // Forward / Backward velocity command (-255..255)
    int16_t vy;          // Left / Right strafe velocity command (-255..255)
    int16_t omega;       // Rotational (yaw) velocity command (-255..255)
    uint8_t buzzer;      // 1 = Trigger horn/buzzer, 0 = Silent
    uint8_t pid_enable;  // 1 = Closed-loop PID (RPM), 0 = Open-loop Direct PWM
    uint8_t test_wheel;  // Single-wheel D-Pad test: 1=M1(Up), 2=M2(Right), 3=M3(Down), 4=M4(Left)
};

/**
 * @brief 2. Control packet sent from ESP32-S3 -> Motion 2350 Pro over UART (13 bytes)
 */
struct ControlPacket {
    uint8_t header1 = 0xAA;   // Sync byte 1
    uint8_t header2 = 0x55;   // Sync byte 2
    uint8_t pid_enable = 0;   // 1 = Target values are in RPM, 0 = Target values are raw PWM
    int16_t target_val[4];    // Target setpoints for wheels [M1(FL), M2(FR), M3(RL), M4(RR)]
    uint8_t buzzer_cmd = 0;   // 1 = Beep buzzer on Motion 2350 Pro
    uint8_t checksum;         // XOR checksum of all preceding bytes
};

/**
 * @brief 3. Feedback packet sent from Motion 2350 Pro -> ESP32-S3 over UART (29 bytes)
 */
struct MotionFeedbackPacket {
    uint8_t header1 = 0xBB;   // Sync byte 1
    uint8_t header2 = 0x66;   // Sync byte 2
    int16_t actual_pwm[4];    // Applied PWM duty cycle (-255..255) for M1..M4
    int16_t actual_rpm[4];    // Measured wheel speed in RPM for M1..M4
    uint8_t enc_health[4];    // Encoder wire health bitmask: Bit 0 = Phase A active, Bit 1 = Phase B active
    uint16_t active_gpios;    // 12-bit mask indicating which scanned GPIOs detected logic transitions
    uint32_t uptime_ms;       // Motion 2350 Pro system uptime in milliseconds
    uint8_t checksum;         // XOR checksum of all preceding bytes
};

/**
 * @brief 4. Telemetry packet sent from ESP32-S3 -> Laptop over Wi-Fi UDP (52 bytes)
 */
struct WifiTelemetryPacket {
    uint8_t header1 = 0xCC;   // Sync byte 1
    uint8_t header2 = 0x77;   // Sync byte 2
    uint8_t pid_mode;         // Current active control mode (1 = PID, 0 = Direct PWM)
    int16_t vx;               // Current linear X command
    int16_t vy;               // Current linear Y command
    int16_t omega;            // Current angular Z command
    int16_t target_val[4];    // Computed wheel targets (RPM or PWM)
    int16_t actual_pwm[4];    // Actual PWM reported by Motion 2350 Pro
    int16_t actual_rpm[4];    // Actual RPM reported by Motion 2350 Pro
    uint8_t enc_health[4];    // Phase A/B wire health status for encoders M1..M4
    uint16_t active_gpios;    // Bitmask of GPIO pins currently receiving encoder pulses
    uint16_t us_cm[3];        // Measured distance (cm) from 3 HC-SR04 ultrasonic sensors
    uint8_t us_status;        // Sensor connection mask: Bit 0=Front OK, Bit 1=Left OK, Bit 2=Right OK
    uint8_t motion_alive;     // 1 = Receiving valid UART feedback from Motion 2350 Pro, 0 = Timeout
    uint32_t uptime_ms;       // ESP32-S3 system uptime in milliseconds
    uint8_t checksum;         // XOR checksum of all preceding bytes
};

#pragma pack(pop)

/**
 * @brief Computes an 8-bit XOR checksum over all bytes of a packet except the last byte (checksum field).
 */
template <typename T>
inline uint8_t calculateChecksum(const T& pkt) {
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&pkt);
    uint8_t xor_sum = 0;
    for (size_t i = 0; i < sizeof(T) - 1; i++) {
        xor_sum ^= ptr[i];
    }
    return xor_sum;
}

/**
 * @brief Calculates the packet checksum and writes the raw binary struct to the given UART/Serial stream.
 */
template <typename T>
inline void sendPacket(Stream& serialPort, T& pkt) {
    pkt.checksum = calculateChecksum(pkt);
    serialPort.write(reinterpret_cast<const uint8_t*>(&pkt), sizeof(T));
}

/**
 * @brief Non-blocking state-machine parser for fixed-size binary UART packets with a 2-byte header.
 * @tparam T   Packet struct type (must end with a 1-byte checksum field)
 * @tparam H1  Expected first header byte
 * @tparam H2  Expected second header byte
 */
template <typename T, uint8_t H1, uint8_t H2>
class PacketParser {
public:
    /**
     * @brief Reads available bytes from serialPort and attempts to assemble a valid packet.
     * @param serialPort Stream to read incoming bytes from.
     * @param outPkt     Destination struct populated when a valid packet is received.
     * @return true if a complete packet with a matching checksum was decoded, false otherwise.
     */
    bool readFrom(Stream& serialPort, T& outPkt) {
        while (serialPort.available() > 0) {
            uint8_t b = serialPort.read();
            if (idx_ == 0) {
                // Wait for first sync header byte
                if (b == H1) buf_[idx_++] = b;
            } else if (idx_ == 1) {
                // Verify second sync header byte; handle repeated H1 byte gracefully
                if (b == H2) buf_[idx_++] = b;
                else if (b == H1) idx_ = 1;
                else idx_ = 0;
            } else {
                // Collect payload bytes until full packet size is reached
                buf_[idx_++] = b;
                if (idx_ == sizeof(T)) {
                    idx_ = 0;
                    T candidate;
                    memcpy(&candidate, buf_, sizeof(T));
                    // Validate XOR checksum before accepting the packet
                    if (candidate.checksum == calculateChecksum(candidate)) {
                        outPkt = candidate;
                        return true;
                    }
                }
            }
        }
        return false;
    }

private:
    uint8_t buf_[sizeof(T)];
    size_t idx_ = 0;
};
