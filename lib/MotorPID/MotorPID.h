#pragma once
#include <Arduino.h>
#include <RobotProtocol.h>

/**
 * @brief Closed-loop PID + Feed-Forward motor controller and quadrature encoder driver
 *        for the 4-wheel Mecanum drivetrain on the Cytron Motion 2350 Pro (RP2350).
 */
class MotorPID {
public:
    static constexpr uint8_t NUM_MOTORS = 4;
    static constexpr uint8_t NUM_SCAN_PINS = 12;

    // Encoder resolution and PID + Feed-Forward tuning constants for FIT0450 motors
    static constexpr float PULSES_PER_REV = 680.0f;
    static constexpr float KP  = 1.2f;
    static constexpr float KI  = 2.5f;
    static constexpr float KD  = 0.02f;
    static constexpr float KFF = 1.6f;

    /**
     * @brief Initializes PWM frequency/range, motor H-bridge pins, encoder pull-ups,
     *        diagnostic GPIO scanner pins, and hardware encoder interrupts.
     */
    void begin();

    /**
     * @brief Polls 12 candidate GPIO pins for logic-level transitions to help diagnose
     *        encoder pin mapping and wiring live on the dashboard.
     */
    void scanGpios();

    /**
     * @brief Updates the active control mode (0 = Direct PWM, 1 = Closed-loop PID)
     *        and sets the target values (PWM or RPM) for all 4 motors.
     */
    void setTargets(uint8_t mode, const int16_t targets[NUM_MOTORS]);

    /**
     * @brief Immediately stops all 4 motors and resets PID integral/derivative states.
     */
    void stopAll();

    /**
     * @brief Periodic control loop (default 20ms / 50Hz) that computes encoder RPM,
     *        checks encoder wire health, and updates motor PWM via PID or open-loop control.
     * @param intervalMs Control loop period in milliseconds (default: 20ms).
     */
    void update(uint32_t intervalMs = 20);

    /**
     * @brief Assembles a MotionFeedbackPacket containing applied PWM, measured RPM,
     *        encoder wire health flags, and active GPIO pulse bitmask.
     * @param uptimeMs Current system uptime in milliseconds.
     * @return Populated MotionFeedbackPacket with valid XOR checksum.
     */
    MotionFeedbackPacket buildFeedbackPacket(uint32_t uptimeMs);

private:
    /**
     * @brief Drives a single motor H-bridge channel with direction inversion and PWM clamping.
     */
    void setMotorHardware(uint8_t i, int16_t pwm);

    uint8_t prevScanState_[NUM_SCAN_PINS] = {0};
    uint16_t gpioPulseMask_ = 0;

    uint8_t pidMode_ = 0;
    int16_t targetVal_[NUM_MOTORS]   = {0, 0, 0, 0};
    int16_t appliedPWM_[NUM_MOTORS]  = {0, 0, 0, 0};
    int16_t measuredRPM_[NUM_MOTORS] = {0, 0, 0, 0};
    uint8_t encHealth_[NUM_MOTORS]   = {0, 0, 0, 0};
    float filteredRPM_[NUM_MOTORS]   = {0, 0, 0, 0};

    long prevEncCount_[NUM_MOTORS]   = {0, 0, 0, 0};
    uint32_t prevPulseA_[NUM_MOTORS] = {0, 0, 0, 0};
    uint32_t prevPulseB_[NUM_MOTORS] = {0, 0, 0, 0};
    float integralErr_[NUM_MOTORS]   = {0, 0, 0, 0};
    float prevErr_[NUM_MOTORS]       = {0, 0, 0, 0};

    unsigned long lastPIDTime_ = 0;
};
