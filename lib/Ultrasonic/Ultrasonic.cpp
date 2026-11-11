#include "Ultrasonic.h"

namespace {
    // Shared state accessed inside the GPIO interrupt service routines (ISRs)
    uint8_t s_echoPins[UltrasonicArray::NUM_SENSORS] = {5, 8, 11};
    volatile uint32_t s_echoStart[UltrasonicArray::NUM_SENSORS]  = {0, 0, 0};
    volatile uint32_t s_lastEchoMs[UltrasonicArray::NUM_SENSORS] = {0, 0, 0};
    volatile uint16_t s_distanceCm[UltrasonicArray::NUM_SENSORS] = {0, 0, 0};

    /**
     * @brief Pin-change interrupt handler that measures the HIGH duration of an HC-SR04 Echo pulse.
     *        Placed in IRAM on ESP32-S3 for fast, deterministic interrupt execution.
     */
    void IRAM_ATTR handleEchoISR(uint8_t idx) {
        uint32_t now = micros();
        if (digitalRead(s_echoPins[idx])) {
            // Rising edge: record the start timestamp of the Echo pulse
            s_echoStart[idx] = now;
        } else {
            // Falling edge: compute pulse width in microseconds
            uint32_t dur = now - s_echoStart[idx];

            // Valid HC-SR04 echo pulses fall between ~100us and 38000us (even when no obstacle is in range)
            if (dur > 80 && dur < 40000) {
                s_lastEchoMs[idx] = millis(); // Mark this sensor as physically connected and responsive
                if (dur < 24000) {
                    // Convert round-trip time (us) to distance (cm): speed of sound ~0.0343 cm/us -> dur * 0.01715
                    s_distanceCm[idx] = (uint16_t)(dur * 0.01715f);
                } else {
                    // Echo timed out in open space (> 4 meters)
                    s_distanceCm[idx] = 400;
                }
            }
        }
    }

    // Individual ISR wrappers for each ultrasonic sensor channel
    void IRAM_ATTR echoISR0() { handleEchoISR(0); }
    void IRAM_ATTR echoISR1() { handleEchoISR(1); }
    void IRAM_ATTR echoISR2() { handleEchoISR(2); }
}

constexpr uint8_t UltrasonicArray::DEFAULT_TRIG_PINS[UltrasonicArray::NUM_SENSORS];
constexpr uint8_t UltrasonicArray::DEFAULT_ECHO_PINS[UltrasonicArray::NUM_SENSORS];

void UltrasonicArray::begin(const uint8_t trigPins[NUM_SENSORS], const uint8_t echoPins[NUM_SENSORS]) {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        trigPins_[i] = trigPins[i];
        echoPins_[i] = echoPins[i];
        s_echoPins[i] = echoPins[i];

        // Configure Trigger pin as OUTPUT (idle LOW)
        pinMode(trigPins_[i], OUTPUT);
        digitalWrite(trigPins_[i], LOW);

        // Configure Echo pin with internal pull-down so unplugged pins stay LOW instead of floating
        pinMode(echoPins_[i], INPUT_PULLDOWN);
    }

    // Attach pin-change interrupts (both rising and falling edges) for all 3 Echo pins
    attachInterrupt(digitalPinToInterrupt(echoPins_[0]), echoISR0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(echoPins_[1]), echoISR1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(echoPins_[2]), echoISR2, CHANGE);
}

void UltrasonicArray::update(uint32_t intervalMs) {
    unsigned long now = millis();
    // Trigger one sensor every intervalMs (35ms default) to avoid ultrasonic echo interference
    if (now - lastPingMs_ >= intervalMs) {
        lastPingMs_ = now;
        uint8_t pin = trigPins_[currentSensor_];

        // Generate a 10-microsecond HIGH trigger pulse required by HC-SR04
        digitalWrite(pin, HIGH);
        delayMicroseconds(10);
        digitalWrite(pin, LOW);

        // Advance to the next sensor in the round-robin sequence (0 -> 1 -> 2 -> 0)
        currentSensor_ = (currentSensor_ + 1) % NUM_SENSORS;
    }
}

void UltrasonicArray::readTelemetry(uint16_t outDistanceCm[NUM_SENSORS], uint8_t& outStatusMask, uint32_t timeoutMs) const {
    unsigned long now = millis();
    uint8_t mask = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        uint32_t lastMs = s_lastEchoMs[i];
        // A sensor is considered alive if it produced at least one valid echo within the last timeoutMs
        bool connected = (lastMs > 0) && ((now - lastMs) < timeoutMs);
        if (connected) {
            mask |= (1 << i);
            outDistanceCm[i] = s_distanceCm[i];
        } else {
            outDistanceCm[i] = 0;
        }
    }
    outStatusMask = mask;
}
