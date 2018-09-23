#include "pid.h"

#include "utils.h"


PIDWrapper::PIDWrapper(double Kp, double Ki, double Kd, ErrorTypes errorType)
    : m_setpoint(0.0)
    , m_input(0.0)
    , m_output(0.0)
    , m_pid(&m_input, &m_output, &m_setpoint, Kp, Ki, Kd, DIRECT)
    , m_errorType(errorType) {
}


bool PIDWrapper::compute() {
    return m_pid.Compute(angularError());
}


void PIDWrapper::setTarget(double newTarget) {
    if (angularError())
        m_setpoint = constrainAngle(newTarget);
    else
        m_setpoint = newTarget;
}


void PIDWrapper::setRelative(double retativeTarget) {
    setTarget(m_input + retativeTarget);
}


void PIDWrapper::reset() {
    // TODO: not sure, is it right. Possibly, we can reset it with PID_v1
    m_setpoint = m_input;
}


void PIDWrapper::update(double input) {
    m_input = input;
}


double PIDWrapper::getOutput() const {
    return m_output;
}


PID* PIDWrapper::pid() {
    return &m_pid;
}


bool PIDWrapper::angularError() const {
    return (m_errorType == ErrorType_Angular);
}