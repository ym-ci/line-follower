#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <cmath>
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
    PIDController(double kp, double ki, double kd);
    PIDController(double kp, double ki, double kd, double period);
    ~PIDController();

    void setPID(double kp, double ki, double kd);
    void setP(double kp);
    void setI(double ki);
    void setD(double kd);
    void setIZone(double iZone);
    double getP() const;
    double getI() const;
    double getD() const;
    double getIZone() const;
    double getPeriod() const;
    double getPositionTolerance() const;
    double getVelocityTolerance() const;
    void setSetpoint(double setpoint);
    double getSetpoint() const;
    bool atSetpoint() const;
    void enableContinuousInput(double minimumInput, double maximumInput);
    void disableContinuousInput();
    bool isContinuousInputEnabled() const;
    void setIntegratorRange(double minimumIntegral, double maximumIntegral);
    void setTolerance(double positionTolerance);
    void setTolerance(double positionTolerance, double velocityTolerance);
    double getPositionError() const;
    double getVelocityError() const;
    double calculate(double measurement, double setpoint);
    double calculate(double measurement);
    void reset();
};

#endif // PID_CONTROLLER_H
