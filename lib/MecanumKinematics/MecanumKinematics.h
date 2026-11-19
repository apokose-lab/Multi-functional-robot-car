#pragma once
#include <Arduino.h>
#include <RobotProtocol.h>

/**
 * @file MecanumKinematics.h
 * @brief High-level, beginner-friendly Mecanum drivetrain controller and inverse kinematics engine.
 *
 * Designed for use on the ESP32-S3 Brain (`Drivetrain` alias is provided for readability).
 * It handles:
 *   1. Converting chassis commands `(vx, vy, omega)` or `(angleDeg, speedPwm, omega)` into
 *      synchronized 4-wheel targets with proportional normalization (no direction distortion).
 *   2. Automatically streaming 50Hz `ControlPacket` updates over UART to the Motion 2350 Pro
 *      driver board (satisfying its 350ms safety watchdog automatically).
 *   3. Supporting both:
 *        - Continuous driving (`drive`, `moveAngle`, `moveForward`, ...) for sensor-based loops
 *        - Distance / angle / timed maneuvers (`moveDistance`, `moveAngleDistance`, `rotateAngle`, `driveFor`)
 *          that automatically stop when finished (blocking or non-blocking).
 *
 * Sign Conventions (Standard Robot Frame):
 *   - `vx`    (-255..+255): +Forward,       -Backward
 *   - `vy`    (-255..+255): +Strafe Left,   -Strafe Right
 *   - `omega` (-255..+255): +Rotate CCW (Left), -Rotate CW (Right)
 *   - `angleDeg` (degrees): 0° = Forward, 90° = Left, 180° = Backward, -90° / 270° = Right
 */
class MecanumKinematics {
public:
    // =========================================================================
    // Physical & Timing Constants
    // =========================================================================
    static constexpr float PI_F                   = 3.14159265f;
    static constexpr float DEFAULT_MAX_RPM        = 120.0f;                 // Wheel RPM at command = 255 in PID mode
    static constexpr float WHEEL_DIAMETER_CM      = 5.8f;                   // 58mm Mecanum wheels
    static constexpr float WHEEL_CIRCUMFERENCE_CM = PI_F * WHEEL_DIAMETER_CM; // ~18.2212 cm per revolution
    static constexpr float CHASSIS_TURN_RADIUS_CM = 16.0f;                  // Effective radius (Lx + Ly) for rotateAngle()
    static constexpr uint32_t UART_SEND_INTERVAL_MS = 20;                   // 50Hz UART control refresh rate

    /**
     * @brief Constructs a MecanumKinematics / Drivetrain controller.
     * @param maxTargetRpm Maximum wheel speed (RPM) mapped to full command `255` when PID is enabled.
     */
    explicit MecanumKinematics(float maxTargetRpm = DEFAULT_MAX_RPM);

    // =========================================================================
    // 1. Setup & Loop Maintenance (Required)
    // =========================================================================

    /**
     * @brief Binds the drivetrain to the UART port connected to the Motion 2350 Pro driver board,
     *        enables closed-loop PID by default, and sends an initial stop packet.
     * @param serialPort HardwareSerial or Stream connected to the Motion 2350 Pro (e.g., MotionSerial).
     */
    void begin(Stream& serialPort);

    /**
     * @brief Non-blocking update tick. Call this on EVERY iteration of `loop()`!
     *        - Automatically stops timed/distance/angle maneuvers when their duration expires.
     *        - Reads incoming encoder RPM / PWM feedback packets from the Motion 2350 Pro.
     *        - Streams ControlPackets at 50Hz (every 20ms) so the motor driver stays synchronized.
     */
    void update();

    /**
     * @brief Waits (while calling `update()`) until an active timed/distance/angle maneuver finishes.
     */
    void waitUntilDone();

    // =========================================================================
    // 2. Continuous Movement Commands (Great for Line Following & Obstacle Loops)
    //    These cancel any active timer and keep moving until another command or `stop()`.
    // =========================================================================

    /**
     * @brief Drives the robot holonomically using X, Y, and rotational velocity commands.
     * @param vx    Forward/Backward speed (-255..255). + = Forward, - = Backward.
     * @param vy    Left/Right strafe speed (-255..255). + = Left, - = Right.
     * @param omega Rotation speed (-255..255). + = Rotate Left (CCW), - = Rotate Right (CW).
     */
    void drive(int16_t vx, int16_t vy, int16_t omega = 0);

    /**
     * @brief Drives the robot in ANY compass direction (0..360 deg) at a chosen speed while optionally spinning.
     * @param angleDeg Heading angle in degrees (0° = Forward, 90° = Left, 180° = Backward, -90°/270° = Right).
     * @param speedPwm Translation speed (-255..255). Negative values reverse the chosen direction.
     * @param omega    Optional rotation speed (-255..255; + = CCW/Left, - = CW/Right). Default = 0 (keep heading).
     */
    void moveAngle(float angleDeg, int16_t speedPwm, int16_t omega = 0);

    /**
     * @brief Drives straight forward (or curves gently if omega != 0).
     * @param pwm   Forward speed magnitude (0..255; negative moves backward).
     * @param omega Optional yaw rotation (-255..255). Default = 0.
     */
    void moveForward(int16_t pwm, int16_t omega = 0);

    /**
     * @brief Drives straight backward (or curves gently if omega != 0).
     * @param pwm   Backward speed magnitude (0..255).
     * @param omega Optional yaw rotation (-255..255). Default = 0.
     */
    void moveBackward(int16_t pwm, int16_t omega = 0);

    /**
     * @brief Strafes sideways to the left without rotating (unless omega != 0).
     * @param pwm   Strafe speed magnitude (0..255).
     * @param omega Optional yaw rotation (-255..255). Default = 0.
     */
    void strafeLeft(int16_t pwm, int16_t omega = 0);

    /**
     * @brief Strafes sideways to the right without rotating (unless omega != 0).
     * @param pwm   Strafe speed magnitude (0..255).
     * @param omega Optional yaw rotation (-255..255). Default = 0.
     */
    void strafeRight(int16_t pwm, int16_t omega = 0);

    /**
     * @brief Spins the robot in place.
     * @param omega Rotational speed (-255..255). + = Rotate Left (CCW), - = Rotate Right (CW).
     */
    void rotate(int16_t omega);

    /**
     * @brief Immediately stops all 4 wheels, cancels any timed maneuver, and sends a stop packet.
     */
    void stop();

    // =========================================================================
    // 3. Distance, Angle, and Duration Commands (Auto-Stop When Complete)
    //    Set `blocking = true` for simple step-by-step scripts, or `false` (default)
    //    to check sensors while moving and poll `isBusy()` in `loop()`.
    // =========================================================================

    /**
     * @brief Drives with `(vx, vy, omega)` for a specified duration in milliseconds, then stops automatically.
     * @param vx         Forward/Backward speed (-255..255).
     * @param vy         Left/Right strafe speed (-255..255).
     * @param omega      Rotation speed (-255..255).
     * @param durationMs Duration to drive in milliseconds.
     * @param blocking   true = Wait inside this call until finished; false = Return immediately (non-blocking).
     */
    void driveFor(int16_t vx, int16_t vy, int16_t omega, uint32_t durationMs, bool blocking = false);

    /**
     * @brief Moves along velocity vector `(vx, vy)` for a specific linear distance (cm), then stops.
     * @param vx         Forward/Backward speed component (-255..255).
     * @param vy         Left/Right strafe speed component (-255..255).
     * @param distanceCm Target travel distance in centimeters (must be > 0).
     * @param omega      Optional simultaneous yaw rotation (-255..255). Default = 0.
     * @param blocking   true = Wait until distance is completed; false = Non-blocking.
     */
    void moveDistance(int16_t vx, int16_t vy, float distanceCm, int16_t omega = 0, bool blocking = false);

    /**
     * @brief Moves at any heading angle (`angleDeg`) at `speedPwm` for `distanceCm` centimeters, then stops.
     * @param angleDeg   Direction angle in degrees (0° = Forward, 90° = Left, 180° = Backward, -90° = Right).
     * @param speedPwm   Speed magnitude (1..255).
     * @param distanceCm Target travel distance in centimeters (must be > 0).
     * @param omega      Optional simultaneous yaw rotation (-255..255). Default = 0.
     * @param blocking   true = Wait until distance is completed; false = Non-blocking.
     */
    void moveAngleDistance(float angleDeg, int16_t speedPwm, float distanceCm,
                           int16_t omega = 0, bool blocking = false);

    /**
     * @brief Rotates the robot in place by a target angle in degrees, then stops automatically.
     *        Direction rule: Positive `omega` & `angleDeg` rotate Left (CCW).
     *        If either `omega` or `angleDeg` is negative, the robot rotates Right (CW).
     * @param omega    Rotation speed command (-255..255).
     * @param angleDeg Target turn angle in degrees (e.g., 90.0f for a 90° turn, -90.0f for CW).
     * @param blocking true = Wait until rotation finishes; false = Non-blocking.
     */
    void rotateAngle(int16_t omega, float angleDeg, bool blocking = false);

    // =========================================================================
    // 4. Buzzer, PID Mode & Diagnostics
    // =========================================================================

    /**
     * @brief Triggers a short confirmation beep on the Motion 2350 Pro onboard buzzer.
     */
    void beep();

    /**
     * @brief Enables or disables closed-loop PID speed control.
     * @param enabled true (default) = Closed-loop PID (RPM matched across all 4 wheels),
     *                false = Open-loop direct PWM (-255..255).
     */
    void setPidEnabled(bool enabled);

    /**
     * @brief Returns true if closed-loop PID control is currently enabled.
     */
    bool isPidEnabled() const { return pidEnable_ == 1; }

    /**
     * @brief Returns true while a non-blocking timed/distance/angle maneuver is in progress.
     */
    bool isBusy() const { return timedMoveActive_; }

    /**
     * @brief Checks whether the Motion 2350 Pro driver board is responding over UART.
     * @param timeoutMs Maximum time (ms) since last valid feedback packet (default: 400ms).
     * @return true if driver feedback was received recently.
     */
    bool isDriverAlive(uint32_t timeoutMs = 400) const;

    /**
     * @brief Returns the latest computed target setpoint for wheel index 0..3 ([M1_FL, M2_FR, M3_RL, M4_RR]).
     */
    int16_t getTarget(uint8_t wheelIndex) const { return (wheelIndex < 4) ? currentTarget_[wheelIndex] : 0; }

    /**
     * @brief Returns the latest measured encoder RPM for wheel index 0..3 (from Motion 2350 Pro).
     */
    int16_t getActualRpm(uint8_t wheelIndex) const { return (wheelIndex < 4) ? latestFeedback_.actual_rpm[wheelIndex] : 0; }

    /**
     * @brief Returns the latest applied motor PWM (-255..255) for wheel index 0..3 (from Motion 2350 Pro).
     */
    int16_t getActualPwm(uint8_t wheelIndex) const { return (wheelIndex < 4) ? latestFeedback_.actual_pwm[wheelIndex] : 0; }

    /**
     * @brief Computes Mecanum inverse kinematics for `(vx, vy, omega)` with proportional scaling
     *        and packs the result into a checksummed `ControlPacket`.
     * @param vx        Forward/Backward command (-255..255).
     * @param vy        Left/Right strafe command (-255..255).
     * @param omega     Yaw rotation command (-255..255).
     * @param pidEnable 1 = Scale targets to RPM for PID, 0 = Raw PWM.
     * @param buzzerCmd 1 = Trigger buzzer beep, 0 = Silent.
     * @return Ready-to-send ControlPacket.
     */
    ControlPacket buildControlPacket(int16_t vx, int16_t vy, int16_t omega,
                                     uint8_t pidEnable = 1, uint8_t buzzerCmd = 0);

private:
    /**
     * @brief Immediately builds and transmits the current motion state over UART (if attached).
     */
    void sendCurrentPacket();

    float maxTargetRpm_;
    Stream* serial_ = nullptr;

    // Active chassis command state
    int16_t cmdVx_    = 0;
    int16_t cmdVy_    = 0;
    int16_t cmdOmega_ = 0;
    uint8_t pidEnable_     = 1; // PID enabled by default
    uint8_t buzzerPending_ = 0;

    // Timed / distance / angle maneuver tracking
    bool timedMoveActive_       = false;
    unsigned long moveStartMs_  = 0;
    uint32_t moveDurationMs_    = 0;
    unsigned long lastSendMs_   = 0;

    // Wheel targets and feedback telemetry from Motion 2350 Pro
    int16_t currentTarget_[4] = {0, 0, 0, 0};
    PacketParser<MotionFeedbackPacket, 0xBB, 0x66> feedbackParser_;
    MotionFeedbackPacket latestFeedback_{};
    unsigned long lastFeedbackMs_ = 0;
};

// Friendly alias so teammates can declare either `Drivetrain robot;` or `MecanumKinematics robot;`
using Drivetrain = MecanumKinematics;
