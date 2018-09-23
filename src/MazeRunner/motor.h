#ifndef MOTOR_H
#define MOTOR_H

#include <WString.h>  // String
#include <stdint.h>   // uint16_t, etc


// Controls motors through 2-channel Motor Shield from iarduino.
// Make abstraction on most basic actions: changing direction and setting speed.
class Motor {
public:
    enum Directions {
        Direction_Forward,
        Direction_Backward,
    };

    // Initializes pins `pin` and `pwmPin` for controlling motor.
    // `reverseDirection` changes `Forward` direction of motor.
    // `Forward` direction of Motor must be corresponding with `Forward` of robot.
    Motor(uint8_t pin, uint8_t pwmPin, bool reverseDirection = false);

    // Immediately changes current speed. Values allowed: 0-255.
    void setSpeed(uint8_t speed);

    // Changes direction of motor. Should be called after stop and little delay.
    void setDirection(Directions direction);

    // These convenience methods only set current direction without motor stop or 
    // changing speed.
    void forward();
    void backward();

    // Sets current motor speed to 0. Direction is preserved.
    void stop();

    String name;

private:
    uint8_t m_pin;
    uint8_t m_pwmPin;
    uint8_t m_forwardCommand;
    uint8_t m_backwardCommand;
    bool m_reverseDirection;
};

#endif