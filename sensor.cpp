#include "sensor.h"
#include <Arduino.h>

Sensor::Sensor(int pin) : pin(pin), threshold(0), whiteValue(0), blackValue(1023) {
    // Initialize with default values
}

// Legacy calibration method
void Sensor::calibrate(int samplesPerReading, int arbitraryNumber) {
    int value = 0;
    for (int j = 0; j < samplesPerReading; j++) {
        value += analogRead(pin);
    }
    threshold = (value / samplesPerReading) - arbitraryNumber;
}

// Helper method to take multiple samples and average them
int Sensor::takeSamples(int samplesPerReading) {
    int value = 0;
    for (int j = 0; j < samplesPerReading; j++) {
        value += analogRead(pin);
        delay(10);  // Small delay between readings for stability
        Serial.println("Taking samples...");
    }
    return value / samplesPerReading;
}

// Define a constant for white sensitivity bias
// A lower factor (e.g., < 0.5) shifts the threshold closer to the white value,
// requiring a stronger white reading to be considered "white".
const float WHITE_BIAS_FACTOR = 0.6; // Adjust this value (0.0 to 1.0)

void Sensor::calibrateBlack(int samplesPerReading) {
    blackValue = takeSamples(samplesPerReading);

    // Recalculate threshold using the white bias factor
    // Ensure calculation uses floating point before casting back to int
    threshold = static_cast<int>(whiteValue + (blackValue - whiteValue) * WHITE_BIAS_FACTOR);
}

void Sensor::calibrateWhite(int samplesPerReading) {
    whiteValue = takeSamples(samplesPerReading);

    // Recalculate threshold using the white bias factor
    // Ensure calculation uses floating point before casting back to int
    threshold = static_cast<int>(whiteValue + (blackValue - whiteValue) * WHITE_BIAS_FACTOR);
}

bool Sensor::isBlack() {
    int value = analogRead(pin);
    return value >= threshold;
}

int Sensor::getThreshold() const {
    return threshold;
}

int Sensor::getWhiteValue() const {
    return whiteValue;
}

int Sensor::getBlackValue() const {
    return blackValue;
}