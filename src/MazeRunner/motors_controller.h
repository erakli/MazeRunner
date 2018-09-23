#ifndef MOTORS_CONTROLLER_H
#define MOTORS_CONTROLLER_H

#include "motor.h"
#include "parameters.h"

#include <stdint.h>  // uint8_t


#define MOTORS_NUM 2

// Controls a pair of Motors.
// Example:
//    motorsController.move(MotorsController::Move_Forward);
//    motorsController.setSpeed(100, 100);
//    motorsController.stop();
//
// Currently supports only Left and Right motors.
class MotorsController {
public:
    // TODO: possibly, it might be declared outside the class?
    enum Moves {
        Move_Stop = 0,
        Move_Forward,
        Move_Backward,
        Move_RotateLeft,  // rotate CCW
        Move_RotateRight  // rotate CW
    };

    enum Motors {
        Motor_Left = 0,
        Motor_Right = 1,
    };

    MotorsController(const Motor& left, const Motor& right);

    // Performs high-level move action by controlling Motors directions. Takes
    // care about stopping Motors before setting new direction. After call,
    // `setSpeed` method must be called.
    void move(Moves moveCommand);

    // Values allowed: 0-255.
    void setSpeed(uint8_t leftSpeed, uint8_t rightSpeed);

    // Stops all motors at once.
    void stop();

private:
    void setNames(const char* leftName, const char* rightName);

    Motor m_motors[MOTORS_NUM];
};

#endif