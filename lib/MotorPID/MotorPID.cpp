#include "MotorPID.h"

namespace {
    // H-Bridge PWM pin pairs for the 4 motors (ordered to match physical chassis layout)
    const uint8_t M_PINS[MotorPID::NUM_MOTORS][2] = {
        {14, 15}, // M1: Front-Left
        {13, 12}, // M2: Front-Right
        {10, 11}, // M3: Rear-Left
        {8, 9}    // M4: Rear-Right
    };

    // Quadrature encoder Phase A and Phase B GPIO pins for M1..M4
    const uint8_t ENC_A[MotorPID::NUM_MOTORS] = {4, 6, 18, 26}; // Phase A for M1, M2, M3, M4
    const uint8_t ENC_B[MotorPID::NUM_MOTORS] = {5, 7, 19, 27}; // Phase B for M1, M2, M3, M4

    // Polarity multipliers (+1 or -1) for motor drive direction and encoder count direction
    const int8_t MOTOR_DIR[MotorPID::NUM_MOTORS] = {1, 1, 1, 1};
    const int8_t ENC_DIR[MotorPID::NUM_MOTORS]   = {1, 1, 1, -1}; // Invert M4 so forward motion reports positive RPM

    // List of 12 GPIO pins continuously scanned for logic transitions to report to the dashboard
    const uint8_t SCAN_PINS[MotorPID::NUM_SCAN_PINS] = {4, 5, 6, 7, 18, 19, 20, 21, 26, 27, 28, 29};

    // Volatile pulse counters updated inside hardware interrupts (independent Phase A/B tracking prevents pulse cancellation if one wire is loose)
    volatile long s_encCount[MotorPID::NUM_MOTORS]   = {0, 0, 0, 0};
    volatile uint32_t s_pulseA[MotorPID::NUM_MOTORS] = {0, 0, 0, 0};
    volatile uint32_t s_pulseB[MotorPID::NUM_MOTORS] = {0, 0, 0, 0};

    /**
     * @brief Rising-edge ISR handler for Encoder Phase A: increments Phase A activity counter
     *        and updates signed position count based on Phase B state.
     */
    inline void handleEncA(uint8_t i) {
        s_pulseA[i]++;
        s_encCount[i] += digitalReadFast(ENC_B[i]) ? -1 : 1;
    }

    /**
     * @brief Rising-edge ISR handler for Encoder Phase B: records Phase B pulse activity
     *        for wire-health diagnostics and single-wire speed fallback.
     */
    inline void handleEncB(uint8_t i) {
        s_pulseB[i]++;
    }

    // Hardware interrupt wrappers for each motor's Phase A and Phase B channels
    void isrA0() { handleEncA(0); } void isrB0() { handleEncB(0); }
    void isrA1() { handleEncA(1); } void isrB1() { handleEncB(1); }
    void isrA2() { handleEncA(2); } void isrB2() { handleEncB(2); }
    void isrA3() { handleEncA(3); } void isrB3() { handleEncB(3); }
}

void MotorPID::begin() {
    // Configure ultrasonic-quiet 20kHz PWM carrier frequency and 8-bit duty cycle range (0..255)
    analogWriteFreq(20000);
    analogWriteRange(255);

    // Initialize the 12 diagnostic scan pins with internal pull-ups and capture initial logic states
    for (uint8_t i = 0; i < NUM_SCAN_PINS; i++) {
        pinMode(SCAN_PINS[i], INPUT_PULLUP);
        prevScanState_[i] = digitalReadFast(SCAN_PINS[i]);
    }

    // Configure H-Bridge output pins and encoder input pins for all 4 motors
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        pinMode(M_PINS[i][0], OUTPUT);
        pinMode(M_PINS[i][1], OUTPUT);
        pinMode(ENC_A[i], INPUT_PULLUP);
        pinMode(ENC_B[i], INPUT_PULLUP);
    }

    // Trigger on RISING edge only (instead of CHANGE) to prevent +1/-1 self-cancellation when a wire is loose
    attachInterrupt(digitalPinToInterrupt(ENC_A[0]), isrA0, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_B[0]), isrB0, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_A[1]), isrA1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_B[1]), isrB1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_A[2]), isrA2, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_B[2]), isrB2, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_A[3]), isrA3, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_B[3]), isrB3, RISING);

    // Ensure all motors start in a safe, stopped state
    stopAll();
}

void MotorPID::scanGpios() {
    // Check each candidate GPIO for a state change since the last loop iteration
    for (uint8_t k = 0; k < NUM_SCAN_PINS; k++) {
        uint8_t s = digitalReadFast(SCAN_PINS[k]);
        if (s != prevScanState_[k]) {
            prevScanState_[k] = s;
            gpioPulseMask_ |= (1 << k); // Latch bit k so the next feedback packet reports activity on this pin
        }
    }
}

void MotorPID::setMotorHardware(uint8_t i, int16_t pwm) {
    // Clamp PWM duty cycle to valid 8-bit signed range [-255, 255]
    pwm = constrain(pwm, -255, 255);
    appliedPWM_[i] = pwm;

    // Apply motor polarity multiplier and drive the dual H-bridge inputs
    int16_t hw_pwm = pwm * MOTOR_DIR[i];
    if (hw_pwm > 0) {
        analogWrite(M_PINS[i][0], hw_pwm);
        analogWrite(M_PINS[i][1], 0);
    } else if (hw_pwm < 0) {
        analogWrite(M_PINS[i][0], 0);
        analogWrite(M_PINS[i][1], -hw_pwm);
    } else {
        analogWrite(M_PINS[i][0], 0);
        analogWrite(M_PINS[i][1], 0);
    }
}

void MotorPID::stopAll() {
    // Reset all PID error accumulators, clear target setpoints, and cut power to all motors
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        integralErr_[i] = 0;
        prevErr_[i] = 0;
        targetVal_[i] = 0;
        setMotorHardware(i, 0);
    }
}

void MotorPID::setTargets(uint8_t mode, const int16_t targets[NUM_MOTORS]) {
    // Clear PID integral windup and derivative history when switching between PWM and PID modes
    if (mode != pidMode_) {
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            integralErr_[i] = 0;
            prevErr_[i] = 0;
        }
        pidMode_ = mode;
    }
    // Store the new target values (PWM duty cycle or target RPM) for all 4 wheels
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        targetVal_[i] = targets[i];
    }
}

void MotorPID::update(uint32_t intervalMs) {
    unsigned long now = millis();
    if (now - lastPIDTime_ < intervalMs) {
        return; // Wait until the next 20ms (50Hz) control interval
    }

    float dt = (now - lastPIDTime_) / 1000.0f;
    lastPIDTime_ = now;

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        // Atomically snapshot volatile ISR counters with interrupts briefly disabled
        noInterrupts();
        long count = s_encCount[i];
        uint32_t pA = s_pulseA[i];
        uint32_t pB = s_pulseB[i];
        interrupts();

        // Compute pulse deltas since the previous control step
        long delta = (count - prevEncCount_[i]) * ENC_DIR[i];
        uint32_t dA = pA - prevPulseA_[i];
        uint32_t dB = pB - prevPulseB_[i];
        prevEncCount_[i] = count;
        prevPulseA_[i]   = pA;
        prevPulseB_[i]   = pB;

        // Evaluate Phase A and Phase B wire health bitmask for the diagnostic dashboard
        uint8_t health = 0;
        if (dA > 0) health |= 0x01; // Bit 0: Phase A wire is pulsing
        if (dB > 0) health |= 0x02; // Bit 1: Phase B wire is pulsing
        encHealth_[i] = health;

        // Fallback mechanism: if Phase A is disconnected/loose but Phase B is alive, estimate speed from Phase B pulses
        float pulses = (dA > 0) ? (float)delta : ((float)dB * ENC_DIR[i]);
        float rawRPM = (pulses / PULSES_PER_REV) * (60.0f / dt);

        // Apply low-pass exponential moving average (EMA) filter to smooth noisy RPM readings
        filteredRPM_[i] = 0.6f * filteredRPM_[i] + 0.4f * rawRPM;
        measuredRPM_[i] = (int16_t)filteredRPM_[i];

        if (pidMode_ == 1) {
            // Closed-loop PID + Feed-Forward RPM control mode
            float targetRPM = (float)targetVal_[i];
            if (abs(targetRPM) < 3.0f) {
                // Deadband around zero target RPM to prevent motor humming/hunting at rest
                integralErr_[i] = 0;
                prevErr_[i] = 0;
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
            // Open-loop Direct PWM control mode
            integralErr_[i] = 0;
            prevErr_[i] = 0;
            setMotorHardware(i, targetVal_[i]);
        }
    }
}

MotionFeedbackPacket MotorPID::buildFeedbackPacket(uint32_t uptimeMs) {
    MotionFeedbackPacket fb;
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        fb.actual_pwm[i] = appliedPWM_[i];
        fb.actual_rpm[i] = measuredRPM_[i];
        fb.enc_health[i] = encHealth_[i];
    }
    fb.active_gpios = gpioPulseMask_;
    gpioPulseMask_ = 0; // Reset latched GPIO transition mask after each transmission
    fb.uptime_ms = uptimeMs;
    fb.checksum  = calculateChecksum(fb);
    return fb;
}
