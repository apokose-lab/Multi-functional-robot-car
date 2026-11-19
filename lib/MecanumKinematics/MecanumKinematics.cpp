#include "MecanumKinematics.h"
#include <math.h>

/**
 * @file MecanumKinematics.cpp
 * @brief Implementation of the Mecanum drivetrain API, inverse kinematics solver,
 *        distance/angle motion timing, and 50Hz UART communication manager.
 */

MecanumKinematics::MecanumKinematics(float maxTargetRpm)
    : maxTargetRpm_(maxTargetRpm) {
    memset(&latestFeedback_, 0, sizeof(latestFeedback_));
}

void MecanumKinematics::begin(Stream& serialPort) {
    serial_         = &serialPort;
    cmdVx_          = 0;
    cmdVy_          = 0;
    cmdOmega_       = 0;
    pidEnable_      = 1; // Closed-loop PID enabled by default
    buzzerPending_  = 0;
    timedMoveActive_ = false;
    moveDurationMs_ = 0;
    lastSendMs_     = millis();
    lastFeedbackMs_ = 0;
    memset(&latestFeedback_, 0, sizeof(latestFeedback_));

    // Send an immediate zero-velocity stop packet so the motor driver syncs safely
    sendCurrentPacket();
}

void MecanumKinematics::update() {
    unsigned long now = millis();

    // 1. Check if an active timed / distance / angle maneuver has reached its target duration
    if (timedMoveActive_ && (now - moveStartMs_ >= moveDurationMs_)) {
        timedMoveActive_ = false;
        cmdVx_    = 0;
        cmdVy_    = 0;
        cmdOmega_ = 0;
        sendCurrentPacket();
        lastSendMs_ = now;
    }

    // 2. Parse any incoming encoder / PWM feedback packets from Motion 2350 Pro
    if (serial_ != nullptr) {
        while (feedbackParser_.readFrom(*serial_, latestFeedback_)) {
            lastFeedbackMs_ = now;
        }
    }

    // 3. Stream ControlPacket at 50Hz (every 20ms) so the motor driver keeps running smoothly
    if (serial_ != nullptr && (now - lastSendMs_ >= UART_SEND_INTERVAL_MS)) {
        lastSendMs_ = now;
        sendCurrentPacket();
    }
}

void MecanumKinematics::waitUntilDone() {
    while (timedMoveActive_) {
        update();
        delay(2);
    }
}

void MecanumKinematics::drive(int16_t vx, int16_t vy, int16_t omega) {
    timedMoveActive_ = false;
    cmdVx_    = constrain(vx, -255, 255);
    cmdVy_    = constrain(vy, -255, 255);
    cmdOmega_ = constrain(omega, -255, 255);
    sendCurrentPacket();
    lastSendMs_ = millis();
}

void MecanumKinematics::moveAngle(float angleDeg, int16_t speedPwm, int16_t omega) {
    float rad = angleDeg * (PI_F / 180.0f);
    int16_t clampedSpeed = constrain(speedPwm, -255, 255);
    int16_t vx = (int16_t)roundf((float)clampedSpeed * cosf(rad));
    int16_t vy = (int16_t)roundf((float)clampedSpeed * sinf(rad));
    drive(vx, vy, omega);
}

void MecanumKinematics::moveForward(int16_t pwm, int16_t omega) {
    drive(pwm, 0, omega);
}

void MecanumKinematics::moveBackward(int16_t pwm, int16_t omega) {
    drive(-pwm, 0, omega);
}

void MecanumKinematics::strafeLeft(int16_t pwm, int16_t omega) {
    drive(0, pwm, omega);
}

void MecanumKinematics::strafeRight(int16_t pwm, int16_t omega) {
    drive(0, -pwm, omega);
}

void MecanumKinematics::rotate(int16_t omega) {
    drive(0, 0, omega);
}

void MecanumKinematics::stop() {
    timedMoveActive_ = false;
    moveDurationMs_  = 0;
    cmdVx_           = 0;
    cmdVy_           = 0;
    cmdOmega_        = 0;
    sendCurrentPacket();
    lastSendMs_ = millis();
}

void MecanumKinematics::driveFor(int16_t vx, int16_t vy, int16_t omega,
                                 uint32_t durationMs, bool blocking) {
    if (durationMs == 0) {
        stop();
        return;
    }

    cmdVx_           = constrain(vx, -255, 255);
    cmdVy_           = constrain(vy, -255, 255);
    cmdOmega_        = constrain(omega, -255, 255);
    timedMoveActive_ = true;
    moveStartMs_     = millis();
    moveDurationMs_  = durationMs;

    sendCurrentPacket();
    lastSendMs_ = moveStartMs_;

    if (blocking) {
        waitUntilDone();
    }
}

void MecanumKinematics::moveDistance(int16_t vx, int16_t vy, float distanceCm,
                                     int16_t omega, bool blocking) {
    int16_t clampedVx = constrain(vx, -255, 255);
    int16_t clampedVy = constrain(vy, -255, 255);

    // Resultant translational command magnitude in [0..255] scale
    float speedCmd = sqrtf((float)clampedVx * (float)clampedVx + (float)clampedVy * (float)clampedVy);
    if (speedCmd > 255.0f) {
        speedCmd = 255.0f;
    }

    if (speedCmd < 1.0f || distanceCm <= 0.0f) {
        stop();
        return;
    }

    // Convert normalized command into linear chassis velocity (cm/s)
    // Formula: wheelRpm = (speedCmd / 255) * maxTargetRpm_
    //          linearCmPerSec = (wheelRpm / 60) * WHEEL_CIRCUMFERENCE_CM
    float wheelRpm = (speedCmd / 255.0f) * maxTargetRpm_;
    float linearCmPerSec = (wheelRpm / 60.0f) * WHEEL_CIRCUMFERENCE_CM;
    if (linearCmPerSec <= 0.01f) {
        stop();
        return;
    }

    uint32_t durationMs = (uint32_t)roundf((distanceCm / linearCmPerSec) * 1000.0f);
    driveFor(clampedVx, clampedVy, omega, durationMs, blocking);
}

void MecanumKinematics::moveAngleDistance(float angleDeg, int16_t speedPwm, float distanceCm,
                                          int16_t omega, bool blocking) {
    float rad = angleDeg * (PI_F / 180.0f);
    int16_t clampedSpeed = constrain(speedPwm, -255, 255);
    int16_t vx = (int16_t)roundf((float)clampedSpeed * cosf(rad));
    int16_t vy = (int16_t)roundf((float)clampedSpeed * sinf(rad));
    moveDistance(vx, vy, distanceCm, omega, blocking);
}

void MecanumKinematics::rotateAngle(int16_t omega, float angleDeg, bool blocking) {
    int16_t magOmega = abs(constrain(omega, -255, 255));
    float magAngle   = fabsf(angleDeg);

    if (magOmega == 0 || magAngle < 0.1f) {
        stop();
        return;
    }

    // Determine spin sign: negative in either omega or angleDeg rotates Clockwise (Right)
    int16_t signedOmega = ((omega < 0) || (angleDeg < 0.0f)) ? -magOmega : magOmega;

    // Tangential wheel speed (cm/s) at command magnitude `magOmega`
    float wheelRpm = ((float)magOmega / 255.0f) * maxTargetRpm_;
    float wheelSpeedCmPerSec = (wheelRpm / 60.0f) * WHEEL_CIRCUMFERENCE_CM;

    // Chassis angular speed (deg/s) = (wheelSpeed / CHASSIS_TURN_RADIUS_CM) * (180 / PI)
    float angularDegPerSec = (wheelSpeedCmPerSec / CHASSIS_TURN_RADIUS_CM) * (180.0f / PI_F);
    if (angularDegPerSec <= 0.1f) {
        stop();
        return;
    }

    uint32_t durationMs = (uint32_t)roundf((magAngle / angularDegPerSec) * 1000.0f);
    driveFor(0, 0, signedOmega, durationMs, blocking);
}

void MecanumKinematics::beep() {
    buzzerPending_ = 1;
    sendCurrentPacket();
    lastSendMs_ = millis();
}

void MecanumKinematics::setPidEnabled(bool enabled) {
    pidEnable_ = enabled ? 1 : 0;
    sendCurrentPacket();
    lastSendMs_ = millis();
}

bool MecanumKinematics::isDriverAlive(uint32_t timeoutMs) const {
    if (lastFeedbackMs_ == 0) {
        return false;
    }
    return (millis() - lastFeedbackMs_) < timeoutMs;
}

void MecanumKinematics::sendCurrentPacket() {
    if (serial_ == nullptr) {
        return;
    }
    ControlPacket pkt = buildControlPacket(cmdVx_, cmdVy_, cmdOmega_, pidEnable_, buzzerPending_);
    sendPacket(*serial_, pkt);
    buzzerPending_ = 0; // Clear one-shot buzzer trigger once transmitted
}

ControlPacket MecanumKinematics::buildControlPacket(int16_t vx, int16_t vy, int16_t omega,
                                                    uint8_t pidEnable, uint8_t buzzerCmd) {
    ControlPacket pkt;
    pkt.pid_enable = pidEnable;

    // Standard X-configuration Mecanum inverse kinematics equations:
    //   M1 (Front-Left)  = vx - vy - omega
    //   M2 (Front-Right) = vx + vy + omega
    //   M3 (Rear-Left)   = vx + vy - omega
    //   M4 (Rear-Right)  = vx - vy + omega
    float raw[4] = {
        (float)vx - (float)vy - (float)omega, // M1: Front-Left
        (float)vx + (float)vy + (float)omega, // M2: Front-Right
        (float)vx + (float)vy - (float)omega, // M3: Rear-Left
        (float)vx - (float)vy + (float)omega  // M4: Rear-Right
    };

    // Proportional wheel normalization:
    // If combined vx + vy + omega exceeds 255 on any wheel, scale all 4 wheels down
    // uniformly instead of hard-clipping so heading angle & spin ratio remain accurate.
    float maxMag = 0.0f;
    for (int i = 0; i < 4; i++) {
        float m = fabsf(raw[i]);
        if (m > maxMag) {
            maxMag = m;
        }
    }
    if (maxMag > 255.0f) {
        float scale = 255.0f / maxMag;
        for (int i = 0; i < 4; i++) {
            raw[i] *= scale;
        }
    }

    // Convert normalized [-255..255] wheel demand into RPM (PID mode) or raw PWM (open-loop)
    for (int i = 0; i < 4; i++) {
        if (pidEnable == 1) {
            currentTarget_[i] = (int16_t)roundf((raw[i] / 255.0f) * maxTargetRpm_);
        } else {
            currentTarget_[i] = (int16_t)roundf(raw[i]);
        }
        pkt.target_val[i] = currentTarget_[i];
    }

    pkt.buzzer_cmd = buzzerCmd;
    pkt.checksum   = calculateChecksum(pkt);
    return pkt;
}
