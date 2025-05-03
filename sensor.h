#ifndef SENSOR_H
#define SENSOR_H

class Sensor {
private:
    int pin;
    int threshold;
    int whiteValue;
    int blackValue;
    
    int takeSamples(int samplesPerReading);

public:
    Sensor(int pin);

    // Legacy calibration method
    void calibrate(int samplesPerReading, int arbitraryNumber);

    // New calibration methods
    void calibrateBlack(int samplesPerReading = 10);
    void calibrateWhite(int samplesPerReading = 10);

    bool isBlack();
    int getThreshold() const;
    int getWhiteValue() const;
    int getBlackValue() const;
};

#endif // SENSOR_H