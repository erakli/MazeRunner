#ifndef PID_H
#define PID_H

#include <pid_v1.h>

class PIDWrapper {
public:
    enum ErrorTypes {
        ErrorType_Normal = 0,
        ErrorType_Angular
    };

    PIDWrapper(double Kp = 1.0, double Ki = 0.0, double Kd = 0.0, 
               ErrorTypes errorType = ErrorType_Normal);

    bool compute();

    void setTarget(double newTarget);
    void setRelative(double retativeTarget);
    void reset();

    void update(double input);
    double getOutput() const;

    PID* pid();
private:
    bool angularError() const;

    double m_setpoint;
    double m_input;
    double m_output;

    PID m_pid;

    ErrorTypes m_errorType;
};

#endif