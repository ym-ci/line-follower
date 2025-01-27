#ifndef MATHUTIL_H
#define MATHUTIL_H

#include <cmath>

// Static methods for clamping
int clamp(int value, int low, int high);
double clamp(double value, double low, double high);
float clamp(float value, float low, float high);

// Method for applying deadband
double applyDeadband(double value, double deadband, double maxMagnitude);
double applyDeadband(double value, double deadband);

// Methods for inputModulus and angleModulus
double inputModulus(double input, double minimumInput, double maximumInput);
double angleModulus(double angleRadians);

// Other utility methods
double interpolate(double startValue, double endValue, double t);
double inverseInterpolate(double startValue, double endValue, double q);
bool isNear(double expected, double actual, double tolerance);
bool isNear(double expected, double actual, double tolerance, double min, double max);

#endif // MATHUTIL_H