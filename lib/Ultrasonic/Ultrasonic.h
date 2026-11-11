#pragma once
#include <Arduino.h>

/**
 * @brief Non-blocking driver for an array of 3 HC-SR04 ultrasonic distance sensors.
 *        Uses hardware GPIO interrupts (ISR) to measure echo pulse widths without blocking the CPU,
 *        and triggers sensors sequentially in a round-robin schedule to prevent acoustic crosstalk.
 */
class UltrasonicArray {
public:
    static constexpr uint8_t NUM_SENSORS = 3;

    // Default pins for the 3 HC-SR04 sensors: [0]=Front, [1]=Left, [2]=Right
    static constexpr uint8_t DEFAULT_TRIG_PINS[NUM_SENSORS] = {4, 6, 10};
    static constexpr uint8_t DEFAULT_ECHO_PINS[NUM_SENSORS] = {5, 8, 11};

    /**
     * @brief Configures Trigger/Echo GPIO pins and attaches pin-change interrupts for echo timing.
     */
    void begin(const uint8_t trigPins[NUM_SENSORS] = DEFAULT_TRIG_PINS,
               const uint8_t echoPins[NUM_SENSORS] = DEFAULT_ECHO_PINS);

    /**
     * @brief Sends a 10us trigger pulse to one sensor at a time in a round-robin cycle.
     * @param intervalMs Time in milliseconds between consecutive sensor triggers (default: 35ms).
     */
    void update(uint32_t intervalMs = 35);

    /**
     * @brief Retrieves the latest distance measurements (in cm) and connection status bitmask.
     * @param outDistanceCm Output array receiving distances for [Front, Left, Right] in cm.
     * @param outStatusMask Output bitmask where Bit i = 1 if sensor i responded within timeoutMs.
     * @param timeoutMs     Maximum age (ms) of the last valid echo before marking a sensor disconnected.
     */
    void readTelemetry(uint16_t outDistanceCm[NUM_SENSORS], uint8_t& outStatusMask, uint32_t timeoutMs = 350) const;

private:
    uint8_t trigPins_[NUM_SENSORS] = {4, 6, 10};
    uint8_t echoPins_[NUM_SENSORS] = {5, 8, 11};
    unsigned long lastPingMs_ = 0;
    uint8_t currentSensor_ = 0;
};

using Ultrasonic = UltrasonicArray;
