#pragma once
#include <Arduino.h>

class Ultrasonic {
    private:
        uint8_t triggerPin;
        uint8_t echoPin;
        static constexpr unsigned long ECHO_TIMEOUT_US = 30000; // Timeout for pulseIn in microseconds
        static constexpr float SOUND_SPEED_CM_PER_US = 0.0343; // Speed of sound in cm/us (343 m/s)
        static constexpr float MIN_DISTANCE_CM = 2.0;
        static constexpr float MAX_DISTANCE_CM = 200.0;
        static constexpr float INVALID_DISTANCE_CM = -1.0;
        float calibrationOffsetCm;
        static constexpr float MIN_CALIBRATION_OFFSET_CM = -5.0;
        static constexpr float MAX_CALIBRATION_OFFSET_CM = 5.0;

    public:
        Ultrasonic(uint8_t triggerPin, uint8_t echoPin, float calibrationOffsetCm = 0.0);
        void begin();
        float readDistanceCm();
        void setCalibrationOffset(float offsetCm);
        float getCalibrationOffset() const;
};