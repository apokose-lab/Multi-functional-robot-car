#pragma once
#include <Arduino.h>
#include <RobotProtocol.h>

/**
 * @brief Inverse kinematics solver for a 4-wheel Mecanum drivetrain.
 *        Converts chassis velocity inputs (vx, vy, omega) into individual wheel targets
 *        for Front-Left (M1), Front-Right (M2), Rear-Left (M3), and Rear-Right (M4).
 */
class MecanumKinematics {
public:
    static constexpr float DEFAULT_MAX_RPM = 120.0f;
    static constexpr int16_t TEST_WHEEL_PWM = 150;

    /**
     * @brief Constructs a MecanumKinematics instance.
     * @param maxTargetRpm Maximum wheel speed (RPM) corresponding to full input scale (255) in PID mode.
     */
    explicit MecanumKinematics(float maxTargetRpm = DEFAULT_MAX_RPM);

    /**
     * @brief Computes target values for all 4 wheels and packs them into a ControlPacket.
     * @param vx        Forward/Backward command (-255..255)
     * @param vy        Left/Right strafe command (-255..255)
     * @param omega     Rotational yaw command (-255..255)
     * @param pidEnable 1 = Scale targets to RPM for closed-loop PID, 0 = Use raw PWM (-255..255)
     * @param testWheel Single-wheel diagnostic mode: 1..4 spins only motor M1..M4 at +150 PWM; 0 = normal driving
     * @param buzzerCmd 1 = Trigger buzzer on the motor driver board, 0 = Silent
     * @return Fully populated ControlPacket with valid XOR checksum.
     */
    ControlPacket buildControlPacket(int16_t vx, int16_t vy, int16_t omega,
                                     uint8_t pidEnable, uint8_t testWheel,
                                     uint8_t buzzerCmd);

    /**
     * @brief Returns a pointer to the array of 4 latest computed wheel targets.
     */
    const int16_t* getCurrentTargets() const { return currentTarget_; }

    /**
     * @brief Returns the latest computed target value for a specific wheel index (0..3).
     */
    int16_t getTarget(uint8_t index) const { return (index < 4) ? currentTarget_[index] : 0; }

private:
    float maxTargetRpm_;
    int16_t currentTarget_[4] = {0, 0, 0, 0};
};
