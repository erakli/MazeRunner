#ifndef PID_H
#define PID_H

#include <pid_v1.h>


// Extends functionality of Arduino PID_v1 class.
// Main purpose of this wrapper is to hide Setpoint, Input and Output variables
// into one entity and add methods like `setRelative` and `reset`. Also, this
// class can work with angular inputs.
//
// NOTE: this wrapper use modified PID_v1 library with extended `Compute`
// method, that supports angular error normalization.
// TODO: we could add this functionality into PID_v1 to create PID_v2 library
class PIDWrapper {
public:
    enum ErrorTypes {
        ErrorType_Normal = 0,
        ErrorType_Angular,
    };

    // Hides initialization of `PID` object and sets its pointers to
    // `PIDWrapper`'s members. All `PID` options are set to their defaults. Use
    // `PIDWrapper::pid` method to change them.
    PIDWrapper(double Kp = 1.0, double Ki = 0.0, double Kd = 0.0,
               ErrorTypes errorType = ErrorType_Normal);

    // Updates `Input` value and calls `PID::Compute` method. Should be called
    // as often as possible but not more than `PID::SampleTime` (it would has
    // no effect).
    bool update(double input);

    // Sets target value, that we need to achieve. Takes care about angular
    // normalization.
    void setTarget(double newTarget);

    // Sets target value according current position. If `PIDWrapper` has
    // error type `ErrorType_Angular`, than it also performs normalization
    // in case when `target` is beyond 180 degrees.
    void setRelative(double relativeTarget);

    // Sets target value to current input, thus output should be near zero.
    void reset();

    // Returns PID's `Output` value. No computation is performed.
    double getOutput() const;

    // Returns pointer to underlying `PID` object. Can be used for configuration
    // and some other specific actions.
    // NOTE: bad design, so be carefull
    PID* pid();

private:
    bool isAngularError() const;

    double m_setpoint;
    double m_input;
    double m_output;

    PID m_pid;

    ErrorTypes m_errorType;
};

#endif