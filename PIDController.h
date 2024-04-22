#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController
{
private:
    float m_kp;
    float m_ki;
    float m_kd;
    float m_iZone;
    const float m_period;
    float m_maximumIntegral;
    float m_minimumIntegral;
    float m_maximumInput;
    float m_minimumInput;
    bool m_continuous;
    float m_positionError;
    float m_velocityError;
    float m_prevError;
    float m_totalError;
    float m_positionTolerance;
    float m_velocityTolerance;
    float m_setpoint;
    float m_measurement;
    bool m_haveMeasurement;
    bool m_haveSetpoint;

public:
    PIDController(float kp, float ki, float kd);
    PIDController(float kp, float ki, float kd, float period);

    void setPID(float kp, float ki, float kd);
    void setP(float kp);
    void setI(float ki);
    void setD(float kd);
    void setIZone(float iZone);

    float getP() const;
    float getI() const;
    float getD() const;
    float getIZone() const;
    float getPeriod() const;
    float getPositionTolerance() const;
    float getVelocityTolerance() const;
    float getSetpoint() const;
    float getPositionError() const;
    float getVelocityError() const;

    void setSetpoint(float setpoint);
    bool atSetpoint() const;

    void enableContinuousInput(float minimumInput, float maximumInput);
    void disableContinuousInput();
    bool isContinuousInputEnabled() const;

    void setIntegratorRange(float minimumIntegral, float maximumIntegral);
    void setTolerance(float positionTolerance);
    void setTolerance(float positionTolerance, float velocityTolerance);

    float calculate(float measurement, float setpoint);
    float calculate(float measurement);
    void reset();
};

#endif // PID_CONTROLLER_H
