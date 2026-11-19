#include <Arduino.h>
#include <RobotProtocol.h>
#include <MotorPID.h>

/**
 * @file src/Motion2350_Driver/main.cpp
 * @brief Firmware entry point for the Cytron Motion 2350 Pro (RP2350) Motor Driver board.
 *
 * Responsibilities:
 *   1. Receives binary ControlPackets from the ESP32-S3 Brain over UART1 (GP16/GP17 @ 115200 baud).
 *   2. Executes the 50Hz (20ms) closed-loop PID + Feed-Forward motor speed controller.
 *   3. Enforces a 350ms communication safety watchdog (stops all motors if UART signal is lost).
 *   4. Sends MotionFeedbackPackets (actual PWM & encoder RPM) back to ESP32-S3 at 25Hz (40ms).
 *
 * Note: Team members normally do NOT need to edit this file. Write all high-level robot
 *       logic and movement commands in `src/ESP32_Brain/main.cpp`.
 */

// UART1 hardware pins wired to ESP32-S3 Brain (GPIO 43/44) and onboard buzzer pin
const uint8_t MOTION_UART_TX = 16; // GP16 (TX) -> GPIO 44 (RX) on ESP32-S3
const uint8_t MOTION_UART_RX = 17; // GP17 (RX) -> GPIO 43 (TX) on ESP32-S3
const uint8_t BUZZER_PIN     = 22; // Onboard piezoelectric buzzer on Motion 2350 Pro

// Motor controller and incoming UART packet parser
MotorPID drivetrain;
PacketParser<ControlPacket, 0xAA, 0x55> controlParser;
unsigned long lastPacketTime = 0;

void setup() {
    // Initialize H-bridge PWM outputs and quadrature encoder interrupts for M1..M4
    drivetrain.begin();

    // Play a short dual-tone startup chime to confirm the motor driver board booted cleanly
    pinMode(BUZZER_PIN, OUTPUT);
    tone(BUZZER_PIN, 2000, 100);
    delay(150);
    tone(BUZZER_PIN, 2500, 100);

    // Start UART1 link to ESP32-S3 Brain
    Serial1.setTX(MOTION_UART_TX);
    Serial1.setRX(MOTION_UART_RX);
    Serial1.begin(115200);
}

void loop() {
    // 1. Decode any incoming ControlPackets from ESP32-S3 Brain
    ControlPacket pkt;
    while (controlParser.readFrom(Serial1, pkt)) {
        lastPacketTime = millis();
        drivetrain.setTargets(pkt.pid_enable, pkt.target_val);

        // Trigger a short horn tone when requested by the Brain
        if (pkt.buzzer_cmd == 1) {
            tone(BUZZER_PIN, 1800, 80);
        }
    }

    // Safety watchdog: halt all 4 wheels immediately if ESP32-S3 stops sending commands for > 350ms
    if (millis() - lastPacketTime > 350) {
        drivetrain.stopAll();
    }

    // 2. Run 50Hz (20ms) encoder RPM measurement and PID / PWM motor update
    drivetrain.update(20);

    // 3. Send actual PWM and RPM feedback back to ESP32-S3 Brain every 40ms (25Hz)
    static unsigned long lastFB = 0;
    unsigned long now = millis();
    if (now - lastFB >= 40) {
        lastFB = now;
        MotionFeedbackPacket fb = drivetrain.buildFeedbackPacket(now);
        sendPacket(Serial1, fb);
    }
}
