#include "sensor.h"
#include <Arduino.h>

Sensor::Sensor(int pin) : pin(pin), threshold(0) {}

void Sensor::calibrate(int samplesPerReading, int arbitraryNumber) {
    int value = 0;
    for (int j = 0; j < samplesPerReading; j++) {
        value += analogRead(pin);
    }
    threshold = (value / samplesPerReading) - arbitraryNumber;
}

bool Sensor::isBlack() {
    int value = analogRead(pin);
    return value >= threshold;
}

int Sensor::getThreshold() const {
    return threshold;
}