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
    void straightLineMove();
    void evalStraightCorrection();

    MotorsController m_motorsController;
    uint8_t m_baseSpeed;

    Moves m_currentMove;

    int m_leftCorrection;
    int m_rightCorrection;
};

#endif