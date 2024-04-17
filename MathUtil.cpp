#include <cmath>
#include <stdexcept>

// Static methods for clamping
int clamp(int value, int low, int high) {
    return std::max(low, std::min(value, high));
}
double clamp(double value, double low, double high) {
    return std::max(low, std::min(value, high));
}
// Method for applying deadband
double applyDeadband(double value, double deadband, double maxMagnitude) {
    if (abs(value) > deadband) {
        if (maxMagnitude / deadband > 1.0e12) {
            return value > 0.0 ? value - deadband : value + deadband;
        }
        if (value > 0.0) {
            return maxMagnitude * (value - deadband) / (maxMagnitude - deadband);
        } else {
            return maxMagnitude * (value + deadband) / (maxMagnitude - deadband);
        }
    } else {
        return 0.0;
    }
}
double applyDeadband(double value, double deadband) {
    return applyDeadband(value, deadband, 1.0);
}
// Methods for inputModulus and angleModulus
double inputModulus(double input, double minimumInput, double maximumInput) {
    double modulus = maximumInput - minimumInput;
    int numMax = static_cast<int>((input - minimumInput) / modulus);
    input -= numMax * modulus;
    int numMin = static_cast<int>((input - maximumInput) / modulus);
    input -= numMin * modulus;
    return input;
}
double angleModulus(double angleRadians) {
    return inputModulus(angleRadians, -M_PI, M_PI);
}
// Other utility methods
double interpolate(double startValue, double endValue, double t) {
    return startValue + (endValue - startValue) * clamp(t, 0.0, 1.0);
}
double inverseInterpolate(double startValue, double endValue, double q) {
    double totalRange = endValue - startValue;
    if (totalRange <= 0) {
        return 0.0;
    }
    double queryToStart = q - startValue;
    if (queryToStart <= 0) {
        return 0.0;
    }
    return queryToStart / totalRange;
}
bool isNear(double expected, double actual, double tolerance) {
    if (tolerance < 0) {
        throw std::invalid_argument("Tolerance must be a non-negative number!");
    }
    return abs(expected - actual) < tolerance;
}
bool isNear(double expected, double actual, double tolerance, double min, double max) {
    if (tolerance < 0) {
        throw std::invalid_argument("Tolerance must be a non-negative number!");
    }
    double errorBound = (max - min) / 2.0;
    double error = inputModulus(expected - actual, -errorBound, errorBound);
    return abs(error) < tolerance;
}