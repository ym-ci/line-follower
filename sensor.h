#ifndef SENSOR_H
#define SENSOR_H

class Sensor {
private:
    int pin;
    int threshold;

public:
    Sensor(int pin);

    void calibrate(int samplesPerReading, int arbitraryNumber);

    bool isBlack();

    int getThreshold() const;
};

#endif // SENSOR_H