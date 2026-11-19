#pragma once
#include <Arduino.h>
#include <RobotProtocol.h>

/**
 * @file MotorPID.h
 * @brief Low-level 4-channel DC motor H-bridge driver, quadrature encoder reader,
 *        and closed-loop PID + Feed-Forward speed controller for the Cytron Motion 2350 Pro (RP2350).
 *
 * Motor indexing convention across the robot:
 *   - Index 0 (M1): Front-Left
 *   - Index 1 (M2): Front-Right
 *   - Index 2 (M3): Rear-Left
 *   - Index 3 (M4): Rear-Right
 */
class MotorPID {
public:
    static constexpr uint8_t NUM_MOTORS = 4;

    // Encoder resolution and closed-loop PID + Feed-Forward constants tuned for FIT0450 motors
    static constexpr float PULSES_PER_REV = 680.0f; // Encoder rising-edge pulses per wheel revolution
    static constexpr float KP  = 1.2f;              // Proportional gain (responds to immediate RPM error)
    static constexpr float KI  = 2.5f;              // Integral gain (eliminates steady-state RPM error)
    static constexpr float KD  = 0.02f;             // Derivative gain (damps overshoot)
    static constexpr float KFF = 1.6f;              // Feed-Forward gain (maps target RPM directly to baseline PWM)

    /**
     * @brief Configures 20kHz PWM output, H-bridge direction pins, encoder input pull-ups,
     *        and hardware rising-edge interrupts for all 4 motors.
     */
    void begin();

    /**
     * @brief Sets the active control mode and wheel target setpoints for all 4 motors.
     * @param mode    1 = Closed-loop PID (targets are in RPM), 0 = Open-loop direct PWM (-255..255).
     * @param targets Array of 4 setpoint values ordered [M1_FL, M2_FR, M3_RL, M4_RR].
     */
    void setTargets(uint8_t mode, const int16_t targets[NUM_MOTORS]);

    /**
     * @brief Immediately stops all 4 motors and resets PID integral/derivative states to zero.
     */
    void stopAll();

    /**
     * @brief Non-blocking periodic control loop (default 20ms / 50Hz).
     *        Samples encoder counts, calculates filtered wheel RPM, and updates H-bridge PWM outputs.
     * @param intervalMs Control loop period in milliseconds (default: 20ms).
     */
    void update(uint32_t intervalMs = 20);

    /**
     * @brief Builds a MotionFeedbackPacket containing current applied PWM and measured RPM
     *        for all 4 wheels, ready to transmit back to the ESP32-S3 Brain.
     * @param uptimeMs Current driver board uptime in milliseconds (e.g. millis()).
     * @return Populated MotionFeedbackPacket with a valid XOR checksum.
     */
    MotionFeedbackPacket buildFeedbackPacket(uint32_t uptimeMs);

private:
    /**
     * @brief Applies polarity mapping and drives the dual-pin H-bridge for a single motor channel.
     * @param i   Motor index (0..3).
     * @param pwm Signed PWM command (-255..255).
     */
    void setMotorHardware(uint8_t i, int16_t pwm);

    // Default to Closed-Loop PID mode (1) so all 4 wheels maintain matched speeds under load
    uint8_t pidMode_ = 1;
    int16_t targetVal_[NUM_MOTORS]   = {0, 0, 0, 0}; // Target RPM (if pidMode_ == 1) or raw PWM (if 0)
    int16_t appliedPWM_[NUM_MOTORS]  = {0, 0, 0, 0}; // Last PWM duty cycle written to each H-bridge
    int16_t measuredRPM_[NUM_MOTORS] = {0, 0, 0, 0}; // Integer snapshot of filtered wheel speed (RPM)
    float filteredRPM_[NUM_MOTORS]   = {0, 0, 0, 0}; // Exponential moving average (EMA) wheel RPM

    // Previous sample state for delta calculations
    long prevEncCount_[NUM_MOTORS]   = {0, 0, 0, 0};
    uint32_t prevPulseA_[NUM_MOTORS] = {0, 0, 0, 0};
    uint32_t prevPulseB_[NUM_MOTORS] = {0, 0, 0, 0};

    // PID controller internal history
    float integralErr_[NUM_MOTORS]   = {0, 0, 0, 0};
    float prevErr_[NUM_MOTORS]       = {0, 0, 0, 0};

    unsigned long lastPIDTime_ = 0;
};
