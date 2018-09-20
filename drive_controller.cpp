#include <Arduino.h>

#include <pid_v1.h>

#include "constants.h"
#include "utils.h"
#include "orientation.h"
#include "motors_controller.h"
#include "drive_controller.h"


// NOTE: PID handling has bad design

// Define Variables we'll be connecting to
double pidSetpoint;
double pidInput;
double pidOutput;

PID rotationPID(&pidInput, &pidOutput, &pidSetpoint, 
                PID_P, PID_I, PID_D, DIRECT);



DriveController::DriveController(const MotorsController &motorsController, 
                                 uint8_t baseSpeed) 
    : m_motorsController(motorsController)
    , m_baseSpeed(baseSpeed)
    , m_currentMove(Move_Stop)
    , m_leftSpeed(0)
    , m_rightSpeed(0)
{

}


void DriveController::setBaseSpeed(uint8_t baseSpeed) {
    m_baseSpeed = baseSpeed;
}


void DriveController::init() {
    initPID();
    initOrientation();
}


// must be called as soon as possible
void DriveController::update() {
    pidInput = getYaw();
    rotationPID.Compute(true);

#if PID_DEBUG
    Serial.print("out=");
    Serial.print(pidOutput);
    Serial.print(",");
#endif

    updateCurrentMove();
}


void DriveController::move(Moves moveCommand, uint8_t newSpeed) {
    if (newSpeed != 0)
        setBaseSpeed(newSpeed);

    bool needChangeMove = (m_currentMove != moveCommand);
    if (!needChangeMove) {
        return;
    }

    m_motorsController.stop();
    resetTargetAngle();
    m_currentMove = moveCommand;

    switch (moveCommand) {
        case Move_Forward:
            m_motorsController.move(MotorsController::Move_Forward);
        break;

        case Move_Backward:
            m_motorsController.move(MotorsController::Move_Backward);
        break;

        case Move_Left:
            m_motorsController.move(MotorsController::Move_RotateLeft);
            setRelativeTargetAngle(+ROTATION_ANGLE_90);
        break;

        case Move_Right:
            m_motorsController.move(MotorsController::Move_RotateRight);
            setRelativeTargetAngle(-ROTATION_ANGLE_90);
        break;

        case Move_Stop:
            // do nothing
        break;
    }
}


void DriveController::initPID() {
    rotationPID.SetOutputLimits(PID_OUTPUT_LIMIT_MIN, PID_OUTPUT_LIMIT_MAX);

    // TODO: not sure is it needed
    // установим частоту вычисления ПИДа
    // rotationPID.SetSampleTime(PID_SAMPLE_TIME); 

    // turn the PID on
    rotationPID.SetMode(AUTOMATIC);

    resetTargetAngle();
}


// this method changes current direction on opposite
void DriveController::changeDirection() {
    switch (m_currentMove) {
        case Move_Forward:
            move(Move_Backward);
        break;

        case Move_Backward:
            move(Move_Forward);
        break;

        // TODO: вот здесь надо подумать. выглядит не очень
        case Move_Left:
            m_motorsController.stop();
            m_motorsController.move(MotorsController::Move_RotateRight);
            m_currentMove = Move_Right;
        break;

        case Move_Right:
            m_motorsController.stop();
            m_motorsController.move(MotorsController::Move_RotateLeft);
            m_currentMove = Move_Left;
        break;
    }
}


void DriveController::setTargetAngle(float newAngle) {
    pidSetpoint = constrainAngle(newAngle);
}


void DriveController::setRelativeTargetAngle(float relativeAngle) {
    setTargetAngle(pidInput + relativeAngle);
}


void DriveController::resetTargetAngle() {
    // TODO: not sure, is it right
    pidSetpoint = pidInput;
}


// must be called as soon, as new PID output obtained 
void DriveController::updateCurrentMove() {
    bool needChangeSpeed = false;
    if (m_currentMove == Move_Forward || m_currentMove == Move_Backward)
        needChangeSpeed = evalStraightCorrection();
    else if (m_currentMove == Move_Left || m_currentMove == Move_Right)
        needChangeSpeed = rotationMove();
    else {
        // TODO: is it alright?
        return;
    }

    if (needChangeSpeed)
        m_motorsController.setSpeed(m_leftSpeed, m_rightSpeed);
}


bool DriveController::evalStraightCorrection() {
    int baseCorrection = abs(int(pidOutput / 2.0));

    // TODO: possibly need to be changed on abs(currentAngle - pidSetpoint)
    if (abs(pidOutput) < ANGLE_SETPOINT_DELTA) {
        return false;
    }

    int leftCorrection;
    int rightCorrection;

    bool turnLeft = (0.0 < pidOutput);
    bool turnRight = (pidOutput < 0.0);

    if (turnLeft) {
        leftCorrection = -1 * baseCorrection;
        rightCorrection = baseCorrection;
    } else if (turnRight) {
        leftCorrection = baseCorrection;
        rightCorrection = -1 * baseCorrection;
    } else {
        // do nothing
    }

    m_leftSpeed = max(m_baseSpeed + leftCorrection, 0);
    m_rightSpeed = max(m_baseSpeed + rightCorrection, 0);

    return true;
}


bool DriveController::rotationMove() {
    // TODO: test it. possibly we doesn't need divide
    uint8_t baseCorrection = abs(int(pidOutput / 2.0));

    // TODO: possibly need to be changed on abs(currentAngle - pidSetpoint)
    if (abs(pidOutput) < ANGLE_SETPOINT_DELTA) {
        move(Move_Stop);
        return false;
    }

    bool turnLeft = (0.0 < pidOutput);
    bool turnRight = (pidOutput < 0.0);

    if ((m_currentMove == Move_Left && turnRight) 
        || (m_currentMove == Move_Right && turnLeft)) 
    {
        changeDirection();
    }

    m_leftSpeed = baseCorrection;
    m_rightSpeed = baseCorrection;

    return true;
}

