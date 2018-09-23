#ifndef DRIVE_CONTROLLER_H
#define DRIVE_CONTROLLER_H

#include "constants.h"
#include "motors_controller.h"
#include "pid.h"

class DriveController {
    enum Moves {
        Move_Stop = 0,
        Move_Forward,
        Move_Backward,
        Move_Left,
        Move_Right
    };

    enum PIDs {
        PID_Rotation = 0,
        PID_Straight = 1,
        PID_Distance = 2,
    };

#define PID_NUM 3

public:
    DriveController(const MotorsController& motorsController);

    void setBaseSpeed(uint8_t baseSpeed);

    void init();
    void update();

    void move(float distance);
    void turn(float angle);

private:
    void setPIDCoeffs();
    void setMove(Moves moveCommand);

    void initPID();

    void changeDirection();

    void updateCurrentMove();
    bool evalStraightMove();
    bool evalStraightCorrection();
    bool evalRotationMove();

    MotorsController m_motorsController;
    PIDWrapper m_pidArray[PID_NUM];
    uint8_t m_baseSpeed;

    Moves m_currentMove;

    uint8_t m_leftSpeed;
    uint8_t m_rightSpeed;
};

#endif