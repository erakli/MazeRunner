#include "pid.h"

#include "utils.h"


PIDWrapper::PIDWrapper(double Kp, double Ki, double Kd, ErrorTypes errorType)
    : m_setpoint(0.0)
    , m_input(0.0)
    , m_output(0.0)
    , m_pid(&m_input, &m_output, &m_setpoint, Kp, Ki, Kd, DIRECT)
    , m_errorType(errorType) {
}


bool PIDWrapper::update(double input) {
    m_input = input;
    return m_pid.Compute(isAngularError());
}


void PIDWrapper::setTarget(double newTarget) {
    if (isAngularError())
        m_setpoint = constrainAngle(newTarget);
    else
        m_setpoint = newTarget;
}


void PIDWrapper::setRelative(double relativeTarget) {
    setTarget(m_input + relativeTarget);
}


void PIDWrapper::reset() {
    // TODO: not sure, is it right. Possibly, we can reset it with PID_v1
    m_setpoint = m_input;
}


double PIDWrapper::getOutput() const {
    return m_output;
}


PID* PIDWrapper::pid() {
    return &m_pid;
}


bool PIDWrapper::isAngularError() const {
    return (m_errorType == ErrorType_Angular);
}