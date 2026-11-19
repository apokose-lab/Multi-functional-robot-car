#include "MotorPID.h"

/**
 * @file MotorPID.cpp
 * @brief Implementation of the 4-channel quadrature encoder and PID + Feed-Forward motor controller
 *        running on the Cytron Motion 2350 Pro (RP2350).
 */

namespace {
    // H-Bridge dual PWM pin pairs for each motor: {ForwardPin, ReversePin}
    const uint8_t M_PINS[MotorPID::NUM_MOTORS][2] = {
        {14, 15}, // M1: Front-Left
        {13, 12}, // M2: Front-Right
        {10, 11}, // M3: Rear-Left
        {8, 9}    // M4: Rear-Right
    };

    // Quadrature encoder Phase A and Phase B GPIO assignments
    const uint8_t ENC_A[MotorPID::NUM_MOTORS] = {4, 6, 18, 26}; // Phase A pins for M1..M4
    const uint8_t ENC_B[MotorPID::NUM_MOTORS] = {5, 7, 19, 27}; // Phase B pins for M1..M4

    // Hardware polarity multipliers (+1 or -1) to align physical rotation with logical forward motion
    const int8_t MOTOR_DIR[MotorPID::NUM_MOTORS] = {1, 1, 1, 1};
    const int8_t ENC_DIR[MotorPID::NUM_MOTORS]   = {1, 1, 1, -1}; // Invert M4 encoder sign so forward = +RPM

    // Volatile ISR counters updated asynchronously on encoder rising edges
    volatile long s_encCount[MotorPID::NUM_MOTORS]   = {0, 0, 0, 0};
    volatile uint32_t s_pulseA[MotorPID::NUM_MOTORS] = {0, 0, 0, 0};
    volatile uint32_t s_pulseB[MotorPID::NUM_MOTORS] = {0, 0, 0, 0};

    /**
     * @brief Rising-edge ISR handler for Encoder Phase A.
     *        Increments Phase A counter and updates signed quadrature position using Phase B's state.
     */
    inline void handleEncA(uint8_t i) {
        s_pulseA[i]++;
        s_encCount[i] += digitalReadFast(ENC_B[i]) ? -1 : 1;
    }

    /**
     * @brief Rising-edge ISR handler for Encoder Phase B.
     *        Tracks raw Phase B pulses as an independent speed-measurement fallback if Phase A drops out.
     */
    inline void handleEncB(uint8_t i) {
        s_pulseB[i]++;
    }

    // Per-channel ISR trampoline functions required by attachInterrupt()
    void isrA0() { handleEncA(0); } void isrB0() { handleEncB(0); }
    void isrA1() { handleEncA(1); } void isrB1() { handleEncB(1); }
    void isrA2() { handleEncA(2); } void isrB2() { handleEncB(2); }
    void isrA3() { handleEncA(3); } void isrB3() { handleEncB(3); }
}

void MotorPID::begin() {
    // Configure silent 20kHz PWM frequency and 8-bit duty cycle range (0..255)
    analogWriteFreq(20000);
    analogWriteRange(255);

    // Configure H-bridge PWM pins and encoder input pins with internal pull-ups
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        pinMode(M_PINS[i][0], OUTPUT);
        pinMode(M_PINS[i][1], OUTPUT);
        pinMode(ENC_A[i], INPUT_PULLUP);
        pinMode(ENC_B[i], INPUT_PULLUP);
    }

    // Attach interrupts on RISING edges to prevent +1/-1 pulse cancellation if a connector vibrates
    attachInterrupt(digitalPinToInterrupt(ENC_A[0]), isrA0, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_B[0]), isrB0, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_A[1]), isrA1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_B[1]), isrB1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_A[2]), isrA2, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_B[2]), isrB2, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_A[3]), isrA3, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_B[3]), isrB3, RISING);

    // Ensure all motors start safely stopped
    stopAll();
    lastPIDTime_ = millis();
}

void MotorPID::setMotorHardware(uint8_t i, int16_t pwm) {
    // Clamp PWM to signed 8-bit limits [-255, 255] and record applied output
    pwm = constrain(pwm, -255, 255);
    appliedPWM_[i] = pwm;

    // Apply motor polarity multiplier and drive the dual H-bridge pins
    int16_t hwPwm = pwm * MOTOR_DIR[i];
    if (hwPwm > 0) {
        analogWrite(M_PINS[i][0], hwPwm);
        analogWrite(M_PINS[i][1], 0);
    } else if (hwPwm < 0) {
        analogWrite(M_PINS[i][0], 0);
        analogWrite(M_PINS[i][1], -hwPwm);
    } else {
        analogWrite(M_PINS[i][0], 0);
        analogWrite(M_PINS[i][1], 0);
    }
}

void MotorPID::stopAll() {
    // Clear PID states, zero all target setpoints, and cut power to all 4 motors
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        integralErr_[i] = 0;
        prevErr_[i]     = 0;
        targetVal_[i]   = 0;
        setMotorHardware(i, 0);
    }
}

void MotorPID::setTargets(uint8_t mode, const int16_t targets[NUM_MOTORS]) {
    // Reset integral windup and derivative history when switching between PID and raw PWM modes
    if (mode != pidMode_) {
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            integralErr_[i] = 0;
            prevErr_[i]     = 0;
        }
        pidMode_ = mode;
    }

    // Store target values (RPM in PID mode, PWM duty cycle in raw PWM mode)
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        targetVal_[i] = targets[i];
    }
}

void MotorPID::update(uint32_t intervalMs) {
    unsigned long now = millis();
    if (now - lastPIDTime_ < intervalMs) {
        return; // Wait until the next 20ms (50Hz) control period
    }

    float dt = (now - lastPIDTime_) / 1000.0f;
    if (dt <= 0.0f) {
        dt = intervalMs / 1000.0f;
    }
    lastPIDTime_ = now;

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        // Atomically snapshot volatile ISR counters
        noInterrupts();
        long count  = s_encCount[i];
        uint32_t pA = s_pulseA[i];
        uint32_t pB = s_pulseB[i];
        interrupts();

        // Compute pulse counts elapsed during this dt window
        long delta  = (count - prevEncCount_[i]) * ENC_DIR[i];
        uint32_t dA = pA - prevPulseA_[i];
        uint32_t dB = pB - prevPulseB_[i];
        prevEncCount_[i] = count;
        prevPulseA_[i]   = pA;
        prevPulseB_[i]   = pB;

        // Fallback: if Phase A produces no pulses, use Phase B pulses to avoid uncontrolled full-throttle spin
        float pulses = (dA > 0) ? (float)delta : ((float)dB * ENC_DIR[i]);
        float rawRPM = (pulses / PULSES_PER_REV) * (60.0f / dt);

        // Smooth RPM measurements using an Exponential Moving Average (EMA) low-pass filter
        filteredRPM_[i] = 0.6f * filteredRPM_[i] + 0.4f * rawRPM;
        measuredRPM_[i] = (int16_t)filteredRPM_[i];

        if (pidMode_ == 1) {
            // Closed-loop PID + Feed-Forward RPM control
            float targetRPM = (float)targetVal_[i];
            if (fabsf(targetRPM) < 3.0f) {
                // Zero-speed deadband prevents audible humming or micro-oscillation while stopped
                integralErr_[i] = 0;
                prevErr_[i]     = 0;
                setMotorHardware(i, 0);
            } else {
                float err = targetRPM - filteredRPM_[i];
                // Accumulate integral error with anti-windup clamping
                integralErr_[i] = constrain(integralErr_[i] + err * dt, -80.0f, 80.0f);
                float deriv = (err - prevErr_[i]) / dt;
                prevErr_[i] = err;

                // Combine velocity Feed-Forward (KFF) with PID correction terms
                float outPWM = (KFF * targetRPM) + (KP * err) + (KI * integralErr_[i]) + (KD * deriv);
                setMotorHardware(i, (int16_t)constrain(outPWM, -255.0f, 255.0f));
            }
        } else {
            // Open-loop Direct PWM mode: drive H-bridges directly with target PWM
            integralErr_[i] = 0;
            prevErr_[i]     = 0;
            setMotorHardware(i, targetVal_[i]);
        }
    }
}

MotionFeedbackPacket MotorPID::buildFeedbackPacket(uint32_t uptimeMs) {
    MotionFeedbackPacket fb;
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        fb.actual_pwm[i] = appliedPWM_[i];
        fb.actual_rpm[i] = measuredRPM_[i];
    }
    fb.uptime_ms = uptimeMs;
    fb.checksum  = calculateChecksum(fb);
    return fb;
}
