#include <Arduino.h>
#include <RobotProtocol.h>
#include <MotorPID.h>

// UART1 pins connected to ESP32-S3 Brain (GPIO 43/44) and onboard piezoelectric buzzer pin
const uint8_t MOTION_UART_TX = 16; // GP16 (TX) -> GPIO 44 (RX) on ESP32-S3
const uint8_t MOTION_UART_RX = 17; // GP17 (RX) -> GPIO 43 (TX) on ESP32-S3
const uint8_t BUZZER_PIN     = 22; // Onboard buzzer on Cytron Motion 2350 Pro

// Drivetrain controller and UART packet parser instances
MotorPID drivetrain;
PacketParser<ControlPacket, 0xAA, 0x55> controlParser;
unsigned long lastPacketTime = 0;

/**
 * @brief Initializes the 4-motor H-bridge & encoder PID subsystem, plays a startup
 *        confirmation tone on the buzzer, and starts UART1 communication with ESP32-S3.
 */
void setup() {
    // Initialize PWM, motor pins, encoder interrupts, and GPIO scanner
    drivetrain.begin();

    // Play a dual-tone startup chime to confirm the RP2350 driver board is booted
    pinMode(BUZZER_PIN, OUTPUT);
    tone(BUZZER_PIN, 2000, 100);
    delay(150);
    tone(BUZZER_PIN, 2500, 100);

    // Configure UART1 on GP16/GP17 at 115200 baud for communication with ESP32-S3
    Serial1.setTX(MOTION_UART_TX);
    Serial1.setRX(MOTION_UART_RX);
    Serial1.begin(115200);
}

/**
 * @brief Main real-time control loop running on the Cytron Motion 2350 Pro:
 *        0. Continuously polls 12 GPIO pins to detect encoder pulse activity for diagnostics
 *        1. Parses incoming ControlPackets from ESP32-S3 over UART1 (with a 350ms watchdog)
 *        2. Executes the 50Hz (20ms) encoder speed calculation and PID + Feed-Forward loop
 *        3. Sends a MotionFeedbackPacket back to ESP32-S3 every 40ms (25Hz)
 */
void loop() {
    // 0. Poll the 12 candidate GPIO pins to detect encoder edge transitions for the dashboard
    drivetrain.scanGpios();

    // 1. Read and process incoming control packets from ESP32-S3 over UART1
    ControlPacket pkt;
    while (controlParser.readFrom(Serial1, pkt)) {
        lastPacketTime = millis();
        // Update control mode (PID vs Direct PWM) and wheel target setpoints
        drivetrain.setTargets(pkt.pid_enable, pkt.target_val);

        // Sound the horn if requested by the gamepad
        if (pkt.buzzer_cmd == 1) {
            tone(BUZZER_PIN, 1800, 80);
        }
    }

    // Safety watchdog: stop all motors immediately if UART communication drops for > 350ms
    if (millis() - lastPacketTime > 350) {
        drivetrain.stopAll();
    }

    // 2. Run encoder RPM estimation and closed-loop PID / PWM update every 20ms (50Hz)
    drivetrain.update(20);

    // 3. Transmit motor & encoder diagnostic feedback packet to ESP32-S3 every 40ms (25Hz)
    static unsigned long lastFB = 0;
    unsigned long now = millis();
    if (now - lastFB >= 40) {
        lastFB = now;
        MotionFeedbackPacket fb = drivetrain.buildFeedbackPacket(now);
        sendPacket(Serial1, fb);
    }
}
