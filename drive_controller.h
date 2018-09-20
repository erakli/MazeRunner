#ifndef DRIVE_CONTROLLER_H
#define DRIVE_CONTROLLER_H

#include "constants.h"

class MotorsController;

class DriveController {
public:
    enum Moves {
        Move_Stop = 0,
        Move_Forward,
        Move_Backward,
        Move_Left,
        Move_Right
    };

    DriveController(const MotorsController &motorsController, 
                    uint8_t baseSpeed = DEFAULT_MOTORS_SPEED);

    void setBaseSpeed(uint8_t baseSpeed);

    void init();
    void update();
    void move(Moves moveCommand, uint8_t newSpeed = 0);

private:
    void initPID();

    void changeDirection();

    void setTargetAngle(float newAngle);
    void setRelativeTargetAngle(float relativeAngle);
    void resetTargetAngle();

    void updateCurrentMove();
    bool evalStraightCorrection();
    bool rotationMove();

    MotorsController m_motorsController;
    uint8_t m_baseSpeed;

    Moves m_currentMove;

    uint8_t m_leftSpeed;
    uint8_t m_rightSpeed;
};

#endif