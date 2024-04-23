#include <cmath>
#include "MathUtil.h"
#include "utils.h"

/*

    float m_kp;
    float m_ki;
    float m_kd;
    float m_iZone = infinity();
    // period is the time between controller updates or sothing like MSPT
    // 0.02 is the default value for period as RoboRIO runs at 50Hz
    // 0.02 prob needs to be way lower in my experience
    const float m_period;
    float m_maximumIntegral = 1.0;
    float m_minimumIntegral = -1.0;
    float m_maximumInput;
    float m_minimumInput;
    bool m_continuous;
    float m_positionError;
    float m_velocityError;
    float m_prevError;
    float m_totalError;
    float m_positionTolerance = 0.05;
    float m_velocityTolerance = infinity();
    float m_setpoint;
    float m_measurement;
    bool m_haveMeasurement = false;
    bool m_haveSetpoint = false;*/

struct PIDController
{
    float m_kp;
    float m_ki;
    float m_kd;
    float m_iZone = infinity();
    const float m_period;
    float m_maximumIntegral = 1.0;
    float m_minimumIntegral = -1.0;
    float m_maximumInput;
    float m_minimumInput;
    bool m_continuous;
    float m_positionError;
    float m_velocityError;
    float m_prevError;
    float m_totalError;
    float m_positionTolerance = 0.05;
    float m_velocityTolerance = infinity();
    float m_setpoint;
    float m_measurement;
    bool m_haveMeasurement = false;
    bool m_haveSetpoint = false;
    PIDController(float kp, float ki, float kd) : m_kp(kp), m_ki(ki), m_kd(kd), m_period(0.02)
    {
        if (kp < 0.0 || ki < 0.0 || kd < 0.0)
        {
            println("Kp, Ki, and Kd must be non-negative numbers!");
        }
    }

    PIDController(float kp, float ki, float kd, float period) : m_kp(kp), m_ki(ki), m_kd(kd), m_period(period)
    {
        if (kp < 0.0 || ki < 0.0 || kd < 0.0 || period <= 0.0)
        {
            println("Kp, Ki, and Kd must be non-negative numbers, and period must be a positive number!");
        }
    }

    void setPID(float kp, float ki, float kd)
    {
        m_kp = kp;
        m_ki = ki;
        m_kd = kd;
    }

    void setP(float kp)
    {
        m_kp = kp;
    }

    void setI(float ki)
    {
        m_ki = ki;
    }

    void setD(float kd)
    {
        m_kd = kd;
    }

    void setIZone(float iZone)
    {
        if (iZone < 0)
        {
            println("IZone must be a non-negative number!");
        }
        m_iZone = iZone;
    }

    float getP() const
    {
        return m_kp;
    }

    float getI() const
    {
        return m_ki;
    }

    float getD() const
    {
        return m_kd;
    }

    float getIZone() const
    {
        return m_iZone;
    }

    float getPeriod() const
    {
        return m_period;
    }

    float getPositionTolerance() const
    {
        return m_positionTolerance;
    }

    float getVelocityTolerance() const
    {
        return m_velocityTolerance;
    }

    void setSetpoint(float setpoint)
    {
        m_setpoint = setpoint;
        m_haveSetpoint = true;

        if (m_continuous)
        {
            float errorBound = (m_maximumInput - m_minimumInput) / 2.0;
            m_positionError = fmod(m_setpoint - m_measurement + errorBound, errorBound * 2) - errorBound;
        }
        else
        {
            m_positionError = m_setpoint - m_measurement;
        }

        m_velocityError = (m_positionError - m_prevError) / m_period;
    }

    float getSetpoint() const
    {
        return m_setpoint;
    }

    bool atSetpoint() const
    {
        return m_haveMeasurement && m_haveSetpoint &&
               abs(m_positionError) < m_positionTolerance &&
               abs(m_velocityError) < m_velocityTolerance;
    }

    void enableContinuousInput(float minimumInput, float maximumInput)
    {
        m_continuous = true;
        m_minimumInput = minimumInput;
        m_maximumInput = maximumInput;
    }

    void disableContinuousInput()
    {
        m_continuous = false;
    }

    bool isContinuousInputEnabled() const
    {
        return m_continuous;
    }

    void setIntegratorRange(float minimumIntegral, float maximumIntegral)
    {
        m_minimumIntegral = minimumIntegral;
        m_maximumIntegral = maximumIntegral;
    }

    void setTolerance(float positionTolerance)
    {
        setTolerance(positionTolerance, infinity());
    }

    void setTolerance(float positionTolerance, float velocityTolerance)
    {
        m_positionTolerance = positionTolerance;
        m_velocityTolerance = velocityTolerance;
    }

    float getPositionError() const
    {
        return m_positionError;
    }

    float getVelocityError() const
    {
        return m_velocityError;
    }

    float calculate(float measurement, float setpoint)
    {
        m_setpoint = setpoint;
        m_haveSetpoint = true;
        return calculate(measurement);
    }

    float calculate(float measurement)
    {
        m_measurement = measurement;
        m_prevError = m_positionError;
        m_haveMeasurement = true;

        if (m_continuous)
        {
            float errorBound = (m_maximumInput - m_minimumInput) / 2.0;
            m_positionError = fmod(m_setpoint - m_measurement + errorBound, errorBound * 2) - errorBound;
        }
        else
        {
            m_positionError = m_setpoint - m_measurement;
        }

        m_velocityError = (m_positionError - m_prevError) / m_period;

        if (abs(m_positionError) > m_iZone)
        {
            m_totalError = 0;
        }
        else if (m_ki != 0)
        {
            m_totalError = clamp(m_totalError + m_positionError * m_period,
                                 m_minimumIntegral / m_ki, m_maximumIntegral / m_ki);
        }

        return m_kp * m_positionError + m_ki * m_totalError + m_kd * m_velocityError;
    }

    void reset()
    {
        m_positionError = 0;
        m_prevError = 0;
        m_totalError = 0;
        m_velocityError = 0;
        m_haveMeasurement = false;
    }
};
