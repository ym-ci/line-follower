#include <cmath>
#include "MathUtil.h"
#include "utils.h"
#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd) : m_kp(kp), m_ki(ki), m_kd(kd), m_period(0.02)
{
    if (kp < 0.0 || ki < 0.0 || kd < 0.0)
    {
        println("Kp, Ki, and Kd must be non-negative numbers!");
    }
}

PIDController::PIDController(float kp, float ki, float kd, float period) : m_kp(kp), m_ki(ki), m_kd(kd), m_period(period)
{
    if (kp < 0.0 || ki < 0.0 || kd < 0.0 || period <= 0.0)
    {
        println("Kp, Ki, and Kd must be non-negative numbers, and period must be a positive number!");
    }
}

void PIDController::setPID(float kp, float ki, float kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

void PIDController::setP(float kp)
{
    m_kp = kp;
}

void PIDController::setI(float ki)
{
    m_ki = ki;
}

void PIDController::setD(float kd)
{
    m_kd = kd;
}

void PIDController::setIZone(float iZone)
{
    if (iZone < 0)
    {
        println("IZone must be a non-negative number!");
    }
    m_iZone = iZone;
}

float PIDController::getP() const
{
    return m_kp;
}

float PIDController::getI() const
{
    return m_ki;
}

float PIDController::getD() const
{
    return m_kd;
}

float PIDController::getIZone() const
{
    return m_iZone;
}

float PIDController::getPeriod() const
{
    return m_period;
}

float PIDController::getPositionTolerance() const
{
    return m_positionTolerance;
}

float PIDController::getVelocityTolerance() const
{
    return m_velocityTolerance;
}

void PIDController::setSetpoint(float setpoint)
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

float PIDController::getSetpoint() const
{
    return m_setpoint;
}

bool PIDController::atSetpoint() const
{
    return m_haveMeasurement && m_haveSetpoint &&
           abs(m_positionError) < m_positionTolerance &&
           abs(m_velocityError) < m_velocityTolerance;
}

void PIDController::enableContinuousInput(float minimumInput, float maximumInput)
{
    m_continuous = true;
    m_minimumInput = minimumInput;
    m_maximumInput = maximumInput;
}

void PIDController::disableContinuousInput()
{
    m_continuous = false;
}

bool PIDController::isContinuousInputEnabled() const
{
    return m_continuous;
}

void PIDController::setIntegratorRange(float minimumIntegral, float maximumIntegral)
{
    m_minimumIntegral = minimumIntegral;
    m_maximumIntegral = maximumIntegral;
}

void PIDController::setTolerance(float positionTolerance)
{
    setTolerance(positionTolerance, infinity());
}

void PIDController::setTolerance(float positionTolerance, float velocityTolerance)
{
    m_positionTolerance = positionTolerance;
    m_velocityTolerance = velocityTolerance;
}

float PIDController::getPositionError() const
{
    return m_positionError;
}

float PIDController::getVelocityError() const
{
    return m_velocityError;
}

float PIDController::calculate(float measurement, float setpoint)
{
    m_setpoint = setpoint;
    m_haveSetpoint = true;
    return calculate(measurement);
}

float PIDController::calculate(float measurement)
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

void PIDController::reset()
{
    m_positionError = 0;
    m_prevError = 0;
    m_totalError = 0;
    m_velocityError = 0;
    m_haveMeasurement = false;
}