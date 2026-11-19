#include <Arduino.h>
#include <MecanumKinematics.h>

/**
 * @file src/ESP32_Brain/main.cpp
 * @brief High-level robot brain running on the DFRobot FireBeetle 2 ESP32-S3.
 *
 * =============================================================================
 * TEAM QUICK-START GUIDE: HOW TO CONTROL THE ROBOT IN ONE FUNCTION CALL
 * =============================================================================
 * All drivetrain commands take speed/rotation values in the range [-255 .. +255]
 * and automatically enable closed-loop PID so all 4 wheels stay synchronized.
 *
 * Sign Conventions:
 *   - vx    (-255..255) : +Forward,            -Backward
 *   - vy    (-255..255) : +Strafe Left,        -Strafe Right
 *   - omega (-255..255) : +Rotate Left (CCW),  -Rotate Right (CW)
 *   - angleDeg          : 0 = Forward, 90 = Left, 180 = Backward, -90 = Right
 *
 * -----------------------------------------------------------------------------
 * A. CONTINUOUS MOVEMENT (Best inside `loop()` for line following / sensors)
 *    Note: Keep calling `robot.update()` in `loop()` so UART packets stream at 50Hz.
 * -----------------------------------------------------------------------------
 *   robot.drive(vx, vy, omega);          // Move in any (vx, vy) + spin (omega)
 *   robot.moveAngle(45, 140, 0);         // Move diagonally Forward-Left at 140 PWM
 *   robot.moveForward(150);              // Go straight forward at 150 PWM
 *   robot.moveForward(150, -30);         // Curve right while moving forward
 *   robot.moveBackward(150);             // Go straight backward at 150 PWM
 *   robot.strafeLeft(150);               // Slide sideways left at 150 PWM
 *   robot.strafeRight(150);              // Slide sideways right at 150 PWM
 *   robot.rotate(120);                   // Spin in place Left (CCW); use -120 for Right (CW)
 *   robot.stop();                        // Stop all 4 wheels immediately
 *   robot.beep();                        // Sound a short beep on the driver board
 *
 * -----------------------------------------------------------------------------
 * B. DISTANCE, ANGLE & TIMED MOVEMENT (Auto-stops when finished)
 *    Last argument `blocking`:
 *      - `true`  = Function waits until the maneuver finishes before moving to the next line.
 *      - `false` = Function returns immediately so you can read sensors while moving;
 *                  check `robot.isBusy()` and keep calling `robot.update()` in `loop()`.
 * -----------------------------------------------------------------------------
 *   robot.moveDistance(150, 0, 50.0f, 0, true);        // Move forward 50 cm at vx=150, then stop
 *   robot.moveAngleDistance(90, 140, 30.0f, 0, true);  // Strafe Left (90 deg) for 30 cm, then stop
 *   robot.rotateAngle(120, 90.0f, true);               // Turn Left 90 deg at omega=120, then stop
 *   robot.rotateAngle(-120, 90.0f, true);              // Turn Right 90 deg at omega=-120, then stop
 *   robot.driveFor(120, 0, 40, 1500, true);            // Curve for 1500 ms, then stop
 *
 * -----------------------------------------------------------------------------
 * C. OPTIONAL SETTINGS & FEEDBACK
 * -----------------------------------------------------------------------------
 *   robot.setPidEnabled(true);           // true = Closed-loop PID (default), false = Raw PWM
 *   robot.isBusy();                      // true while a non-blocking timed/distance move runs
 *   robot.isDriverAlive();               // true if Motion 2350 Pro is replying over UART
 * =============================================================================
 */

// UART1 pins wired to Cytron Motion 2350 Pro (GP16/GP17)
// Note: GPIO 16 and GPIO 17 on ESP32-S3 are reserved for the LiDAR sensor.
constexpr int UART_RX_PIN = 44; // Connects to GP16 (TX) on Motion 2350 Pro
constexpr int UART_TX_PIN = 43; // Connects to GP17 (RX) on Motion 2350 Pro

HardwareSerial MotionSerial(1);
Drivetrain robot; // High-level Mecanum drivetrain controller

void setup() {
    // Release default UART0 from GPIO 43/44 so MotionSerial (UART1) can use them cleanly
    Serial.end();

    // Initialize high-speed UART link to the Motion 2350 Pro motor driver board
    MotionSerial.setRxBufferSize(512);
    MotionSerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    // Initialize drivetrain (PID mode enabled by default)
    robot.begin(MotionSerial);
}

void loop() {
    // Always keep this non-blocking update running on every loop iteration:
    // Streams 50Hz motor commands, handles auto-stop timers, and reads encoder feedback.
    robot.update();

    // -------------------------------------------------------------------------
    // EXAMPLE 1 (Commented out): One-time autonomous sequence using blocking calls
    // -------------------------------------------------------------------------
    /*
    static bool sequenceDone = false;
    if (!sequenceDone) {
        delay(1000);                                         // Wait 1s for motor driver boot chime
        robot.moveDistance(140, 0, 50.0f, 0, true);          // Move forward 50 cm at PWM=140, omega=0
        robot.rotateAngle(-120, 90.0f, true);                // Rotate 90 deg Clockwise (Right)
        robot.moveAngleDistance(90.0f, 140, 30.0f, 0, true); // Strafe Left 30 cm
        robot.beep();                                        // Confirm completion
        sequenceDone = true;
    }
    */

    // -------------------------------------------------------------------------
    // EXAMPLE 2 (Commented out): Continuous driving with your own sensor logic
    // -------------------------------------------------------------------------
    /*
    // Read your sensors here...
    bool shouldStop = false;
    if (shouldStop) {
        robot.stop();
    } else {
        robot.drive(130, 0, 0); // Cruise forward continuously at vx=130, vy=0, omega=0
    }
    */
}
