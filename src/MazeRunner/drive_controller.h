#ifndef DRIVE_CONTROLLER_H
#define DRIVE_CONTROLLER_H

#include "motors_controller.h"
#include "pid.h"


// TODO: bad design
#define PID_USED_NUM 3

// High-level contolling class. Performs contol in close-loop manner.
// It relies on data from sensors (gyro, ultrasonic) and performs high-level
// moves with PID contolling.
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

public:
    DriveController(const MotorsController& motorsController);

    // Sets new base speed, which will be used in `updateCurrentMove` function.
    void setBaseSpeed(uint8_t baseSpeed);

    // Initializes all PIDs and sensors. Must be called before any other
    // methods. For example, in `setup()`.
    void init();

    // Main method, that you need to use. Performs all sensor readings and
    // PID computations. After these updates, it calls `updateCurrentMove`.
    // As a result, it performs corrections to current motors' direction and
    // speed with the help of `MotorsController`.
    // Must be called as often as possible!
    void update();

    // Performs move on `distance` in cm from current position.
    // NOTE: This method is only responsible on initial setup of the movement
    // (motors directions, PID initialization). Actual movement occurs when
    // `update()` method is called.
    void move(float distance);

    // Performs turn on `angle` in degrees from current position.
    // NOTE: This method is only responsible on initial setup of the movement
    // (motors directions, PID initialization). Actual movement occurs when
    // `update()` method is called.
    void turn(float angle);

private:
    void setPIDCoeffs();
    void setMove(Moves moveCommand);

    void initPID();

    // Changes current movement direction on opposite.
    void changeDirection();

    // Must be called as soon, as new PID output obtained.
    void updateCurrentMove();

    // Computes `baseSpeed` according to `PID_Distance` output.
    bool evalStraightMove();

    // Computes `leftSpeed` and `rightSpeed` values according to `PID_Straight` 
    // output. Thus, it corrects deviations during straight line movement.
    bool evalStraightCorrection();

    // Computes `leftSpeed` and `rightSpeed` values according to `PID_Rotation`
    // output.
    bool evalRotationMove();

    MotorsController m_motorsController;
    PIDWrapper m_pidArray[PID_USED_NUM];
    uint8_t m_baseSpeed;

    Moves m_currentMove;

    uint8_t m_leftSpeed;
    uint8_t m_rightSpeed;
};

#endif