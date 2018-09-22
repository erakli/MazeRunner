#ifndef MOTORS_CONTROLLER_H
#define MOTORS_CONTROLLER_H

#include "constants.h"
#include "motor.h"

// Motors' controller for 2 motors (currently only supports Left and Right)
class MotorsController {
public:
    enum Moves {
        Move_Stop = 0,
        Move_Forward,
        Move_Backward,
        Move_RotateLeft,
        Move_RotateRight
    };

    enum Motors {
        Motor_Left = 0,
        Motor_Right = 1
    };

    MotorsController(const Motor &left, const Motor &right);

    void move(Moves moveCommand, uint8_t leftSpeed = 0, uint8_t rightSpeed = 0);
    void setSpeed(uint8_t leftSpeed, uint8_t rightSpeed);
    void stop();

    void setDirections(Motor::Directions leftDirection, 
                       Motor::Directions rightDirection);

private:
    void setNames(const char *leftName, const char *rightName);

    Motor m_motors[MOTORS_NUM];
};

#endif