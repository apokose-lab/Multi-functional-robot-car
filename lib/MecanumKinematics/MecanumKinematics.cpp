#include "MecanumKinematics.h"

MecanumKinematics::MecanumKinematics(float maxTargetRpm)
    : maxTargetRpm_(maxTargetRpm) {}

ControlPacket MecanumKinematics::buildControlPacket(int16_t vx, int16_t vy, int16_t omega,
                                                    uint8_t pidEnable, uint8_t testWheel,
                                                    uint8_t buzzerCmd) {
    ControlPacket pkt;

    // Single-wheel diagnostic test mode triggered via PS4 D-Pad (1=M1, 2=M2, 3=M3, 4=M4)
    if (testWheel >= 1 && testWheel <= 4) {
        // Force open-loop Direct PWM mode during single-wheel hardware testing
        pkt.pid_enable = 0;
        for (int i = 0; i < 4; i++) {
            currentTarget_[i] = (i == (testWheel - 1)) ? TEST_WHEEL_PWM : 0;
            pkt.target_val[i] = currentTarget_[i];
        }
    } else {
        // Standard Mecanum X-configuration inverse kinematics equations, clamped to [-255, 255]
        int16_t raw_fl = constrain(vx - vy - omega, -255, 255); // M1: Front-Left
        int16_t raw_fr = constrain(vx + vy + omega, -255, 255); // M2: Front-Right
        int16_t raw_rl = constrain(vx + vy - omega, -255, 255); // M3: Rear-Left
        int16_t raw_rr = constrain(vx - vy + omega, -255, 255); // M4: Rear-Right
        int16_t raw[4] = {raw_fl, raw_fr, raw_rl, raw_rr};

        pkt.pid_enable = pidEnable;
        for (int i = 0; i < 4; i++) {
            if (pidEnable == 1) {
                // Convert normalized [-255..255] demand into target wheel RPM [-maxTargetRpm..maxTargetRpm]
                currentTarget_[i] = (int16_t)((raw[i] / 255.0f) * maxTargetRpm_);
            } else {
                // Pass raw PWM duty cycle directly in open-loop mode
                currentTarget_[i] = raw[i];
            }
            pkt.target_val[i] = currentTarget_[i];
        }
    }

    // Attach buzzer trigger flag and compute packet XOR checksum
    pkt.buzzer_cmd = buzzerCmd;
    pkt.checksum   = calculateChecksum(pkt);
    return pkt;
}
