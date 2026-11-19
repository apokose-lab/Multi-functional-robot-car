#pragma once
#include <Arduino.h>

/**
 * @file RobotProtocol.h
 * @brief Binary UART communication protocol shared between the ESP32-S3 Brain
 *        and the Cytron Motion 2350 Pro (RP2350) Motor Driver board.
 *
 * Packet architecture:
 *   - ControlPacket (13 bytes):        ESP32-S3 Brain  --->  Motion 2350 Pro
 *   - MotionFeedbackPacket (23 bytes): Motion 2350 Pro --->  ESP32-S3 Brain
 *
 * Every packet uses a 2-byte sync header, 1-byte packing (no struct padding),
 * and a trailing 8-bit XOR checksum to detect UART transmission errors.
 */

// Force 1-byte struct alignment so binary layouts match identically on ESP32-S3 and RP2350
#pragma pack(push, 1)

/**
 * @brief Motion command packet sent from ESP32-S3 Brain -> Motion 2350 Pro over UART (13 bytes).
 */
struct ControlPacket {
    uint8_t header1 = 0xAA;   // Sync byte 1
    uint8_t header2 = 0x55;   // Sync byte 2
    uint8_t pid_enable = 1;   // Control mode: 1 = Closed-loop PID (target_val is RPM), 0 = Open-loop raw PWM (-255..255)
    int16_t target_val[4];    // Wheel setpoints for [M1: Front-Left, M2: Front-Right, M3: Rear-Left, M4: Rear-Right]
    uint8_t buzzer_cmd = 0;   // Buzzer trigger: 1 = Emit a short beep on Motion 2350 Pro, 0 = Silent
    uint8_t checksum;         // 8-bit XOR checksum of all preceding bytes in this struct
};

/**
 * @brief Telemetry feedback packet sent from Motion 2350 Pro -> ESP32-S3 Brain over UART (23 bytes).
 */
struct MotionFeedbackPacket {
    uint8_t header1 = 0xBB;   // Sync byte 1
    uint8_t header2 = 0x66;   // Sync byte 2
    int16_t actual_pwm[4];    // Current PWM duty cycle (-255..255) applied to motors [M1..M4]
    int16_t actual_rpm[4];    // Current wheel rotational speed (RPM) measured by encoders [M1..M4]
    uint32_t uptime_ms;       // Motion 2350 Pro system uptime in milliseconds
    uint8_t checksum;         // 8-bit XOR checksum of all preceding bytes in this struct
};

#pragma pack(pop)

/**
 * @brief Computes an 8-bit XOR checksum over all bytes of a packet excluding the final checksum byte.
 * @tparam T Packet struct type whose last byte is `checksum`.
 * @param pkt Packet instance to checksum.
 * @return 8-bit XOR checksum value.
 */
template <typename T>
inline uint8_t calculateChecksum(const T& pkt) {
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&pkt);
    uint8_t xorSum = 0;
    for (size_t i = 0; i < sizeof(T) - 1; i++) {
        xorSum ^= ptr[i];
    }
    return xorSum;
}

/**
 * @brief Populates the packet's checksum field and transmits the binary struct over UART.
 * @tparam T Packet struct type whose last byte is `checksum`.
 * @param serialPort Target Arduino Stream/HardwareSerial interface.
 * @param pkt        Packet to finalize and send.
 */
template <typename T>
inline void sendPacket(Stream& serialPort, T& pkt) {
    pkt.checksum = calculateChecksum(pkt);
    serialPort.write(reinterpret_cast<const uint8_t*>(&pkt), sizeof(T));
}

/**
 * @brief Non-blocking byte-by-byte state machine parser for fixed-size binary UART packets.
 *        Synchronizes on a 2-byte header (H1, H2) and validates the XOR checksum before accepting data.
 *
 * @tparam T   Packet struct type (must end with a 1-byte `checksum` field).
 * @tparam H1  Expected first sync header byte.
 * @tparam H2  Expected second sync header byte.
 */
template <typename T, uint8_t H1, uint8_t H2>
class PacketParser {
public:
    /**
     * @brief Reads all currently available bytes from serialPort and decodes complete valid packets.
     * @param serialPort Incoming UART/Serial stream.
     * @param outPkt     Output struct populated only when a complete packet passes checksum validation.
     * @return true if a valid packet was decoded during this call; false otherwise.
     */
    bool readFrom(Stream& serialPort, T& outPkt) {
        while (serialPort.available() > 0) {
            uint8_t b = serialPort.read();
            if (idx_ == 0) {
                // Wait for the first sync byte
                if (b == H1) {
                    buf_[idx_++] = b;
                }
            } else if (idx_ == 1) {
                // Verify the second sync byte (preserve alignment if H1 repeats back-to-back)
                if (b == H2) {
                    buf_[idx_++] = b;
                } else if (b == H1) {
                    idx_ = 1;
                } else {
                    idx_ = 0;
                }
            } else {
                // Collect payload bytes until the full struct size is buffered
                buf_[idx_++] = b;
                if (idx_ == sizeof(T)) {
                    idx_ = 0;
                    T candidate;
                    memcpy(&candidate, buf_, sizeof(T));
                    // Accept packet only if the checksum matches
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
