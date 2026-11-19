#include "Ultrasonic.h"

Ultrasonic::Ultrasonic(uint8_t triggerPin, uint8_t echoPin, float calibrationOffsetCm)
    : triggerPin(triggerPin),
      echoPin(echoPin),
      calibrationOffsetCm(0.0) {

    setCalibrationOffset(calibrationOffsetCm);
}

void Ultrasonic::begin() {
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

void Ultrasonic::setCalibrationOffset(float offsetCm) {
    if (offsetCm < MIN_CALIBRATION_OFFSET_CM ||
        offsetCm > MAX_CALIBRATION_OFFSET_CM) {
        return;
    }

    calibrationOffsetCm = offsetCm;
}

float Ultrasonic::getCalibrationOffset() const {
    return calibrationOffsetCm;
}

float Ultrasonic::readDistanceCm() {
    // Send a 10 microsecond pulse to trigger the ultrasonic sensor
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    // read the echo pin and calculate the distance
    unsigned long duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
    if (duration == 0) {
        // No echo received within the timeout period
        return INVALID_DISTANCE_CM; // Indicate an error or out of range
    }


    float distanceCm = ((duration * SOUND_SPEED_CM_PER_US) / 2) + calibrationOffsetCm;
    if (distanceCm < MIN_DISTANCE_CM || distanceCm > MAX_DISTANCE_CM) {
        // Distance is out of the valid range
        return INVALID_DISTANCE_CM; // Indicate an error or out of range
    }
    return distanceCm;
}
