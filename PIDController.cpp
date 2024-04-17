#include <cmath>
#include <stdexcept>
#include "MathUtil.h"

class PIDController {
private:
    double m_kp;
    double m_ki;
    double m_kd;
    double m_iZone = infinity();
    const double m_period;
    double m_maximumIntegral = 1.0;
    double m_minimumIntegral = -1.0;
    double m_maximumInput;
    double m_minimumInput;
    bool m_continuous;
    double m_positionError;
    double m_velocityError;
    double m_prevError;
    double m_totalError;
    double m_positionTolerance = 0.05;
    double m_velocityTolerance = infinity();
    double m_setpoint;
    double m_measurement;
    bool m_haveMeasurement = false;
    bool m_haveSetpoint = false;

public:
    PIDController(double kp, double ki, double kd) : m_kp(kp), m_ki(ki), m_kd(kd), m_period(0.02) {
        if (kp < 0.0 || ki < 0.0 || kd < 0.0) {
            throw std::invalid_argument("Kp, Ki, and Kd must be non-negative numbers!");
        }
    }

    PIDController(double kp, double ki, double kd, double period) : m_kp(kp), m_ki(ki), m_kd(kd), m_period(period) {
        if (kp < 0.0 || ki < 0.0 || kd < 0.0 || period <= 0.0) {
            throw std::invalid_argument("Kp, Ki, Kd, and period must be non-negative numbers, and period must be positive!");
        }
    }

    ~PIDController() {
        // Remove from registry
    }

    void setPID(double kp, double ki, double kd) {
        m_kp = kp;
        m_ki = ki;
        m_kd = kd;
    }

    void setP(double kp) {
        m_kp = kp;
    }

    void setI(double ki) {
        m_ki = ki;
    }

    void setD(double kd) {
        m_kd = kd;
    }

    void setIZone(double iZone) {
        if (iZone < 0) {
            throw std::invalid_argument("IZone must be a non-negative number!");
        }
        m_iZone = iZone;
    }

    double getP() const {
        return m_kp;
    }

    double getI() const {
        return m_ki;
    }

    double getD() const {
        return m_kd;
    }

    double getIZone() const {
        return m_iZone;
    }

    double getPeriod() const {
        return m_period;
    }

    double getPositionTolerance() const {
        return m_positionTolerance;
    }

    double getVelocityTolerance() const {
        return m_velocityTolerance;
    }

    void setSetpoint(double setpoint) {
        m_setpoint = setpoint;
        m_haveSetpoint = true;

        if (m_continuous) {
            double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
            m_positionError = fmod(m_setpoint - m_measurement + errorBound, errorBound * 2) - errorBound;
        } else {
            m_positionError = m_setpoint - m_measurement;
        }

        m_velocityError = (m_positionError - m_prevError) / m_period;
    }

    double getSetpoint() const {
        return m_setpoint;
    }

    bool atSetpoint() const {
        return m_haveMeasurement && m_haveSetpoint &&
               abs(m_positionError) < m_positionTolerance &&
               abs(m_velocityError) < m_velocityTolerance;
    }

    void enableContinuousInput(double minimumInput, double maximumInput) {
        m_continuous = true;
        m_minimumInput = minimumInput;
        m_maximumInput = maximumInput;
    }

    void disableContinuousInput() {
        m_continuous = false;
    }

    bool isContinuousInputEnabled() const {
        return m_continuous;
    }

    void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
        m_minimumIntegral = minimumIntegral;
        m_maximumIntegral = maximumIntegral;
    }

    void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, infinity());
    }

    void setTolerance(double positionTolerance, double velocityTolerance) {
        m_positionTolerance = positionTolerance;
        m_velocityTolerance = velocityTolerance;
    }

    double getPositionError() const {
        return m_positionError;
    }

    double getVelocityError() const {
        return m_velocityError;
    }

    double calculate(double measurement, double setpoint) {
        m_setpoint = setpoint;
        m_haveSetpoint = true;
        return calculate(measurement);
    }

    double calculate(double measurement) {
        m_measurement = measurement;
        m_prevError = m_positionError;
        m_haveMeasurement = true;

        if (m_continuous) {
            double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
            m_positionError = fmod(m_setpoint - m_measurement + errorBound, errorBound * 2) - errorBound;
        } else {
            m_positionError = m_setpoint - m_measurement;
        }

        m_velocityError = (m_positionError - m_prevError) / m_period;

        if (abs(m_positionError) > m_iZone) {
            m_totalError = 0;
        } else if (m_ki != 0) {
            m_totalError = clamp(m_totalError + m_positionError * m_period,
                                      m_minimumIntegral / m_ki, m_maximumIntegral / m_ki);
        }

        return m_kp * m_positionError + m_ki * m_totalError + m_kd * m_velocityError;
    }

    void reset() {
        m_positionError = 0;
        m_prevError = 0;
        m_totalError = 0;
        m_velocityError = 0;
        m_haveMeasurement = false;
    }
};
