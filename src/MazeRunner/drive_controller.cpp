#include "drive_controller.h"

#include <Arduino.h>  // max

#include "defines.h"
#include "orientation.h"
#include "utils.h"


DriveController::DriveController(const MotorsController& motorsController)
    : m_motorsController(motorsController)
    , m_baseSpeed(0)
    , m_currentMove(Move_Stop)
    , m_leftSpeed(0)
    , m_rightSpeed(0) {
    setPIDCoeffs();
}


void DriveController::setBaseSpeed(int baseSpeed) {
    if (baseSpeed > UINT8_MAX)
        m_baseSpeed = UINT8_MAX;
    else if (baseSpeed < MINIMAL_MOTOR_SPEED) {
        setMove(Move_Stop);  // TEST: don't know if this needed
        m_baseSpeed = 0;
    } else {
        m_baseSpeed = baseSpeed;
    }
}


void DriveController::init() {
    initPID();
    initOrientation();
}


// TODO: maybe, we  can disable readings and PID computation on distance, when
// we perform rotation, and on rotation, when we moving straight
void DriveController::update() {
    yawUpdate();
    distanceUpdate();

    double yaw = getYaw();
    m_pidArray[PID_Rotation].update(yaw);
    m_pidArray[PID_Straight].update(yaw);

    double distance = getDistance();
    m_pidArray[PID_Distance].update(distance);

#if PID_DEBUG
    // Serial.print("out=");
    // Serial.print(pidOutput);
    // Serial.print(",");
#endif

    updateCurrentMove();
}


void DriveController::move(float distance) {
    if (fuzzyIsNull(distance))
        setMove(Move_Stop);

    // TODO: the same code executes in updateCurrentMove. may need refactor
    if (0 < distance)
        setMove(Move_Forward);
    else if (distance < 0)
        setMove(Move_Backward);

    m_pidArray[PID_Distance].reset();
    m_pidArray[PID_Distance].setRelative(distance);

    // we must reset straight line correction with new moving direction
    m_pidArray[PID_Straight].reset();
}


void DriveController::turn(float angle) {
    if (fuzzyIsNull(angle))
        setMove(Move_Stop);

    // TODO: the same code executes in updateCurrentMove. may need refactor
    if (0 < angle)  // CCW = positive
        setMove(Move_Left);
    else if (angle < 0)  // CW = negative
        setMove(Move_Right);

    m_pidArray[PID_Rotation].reset();
    m_pidArray[PID_Rotation].setRelative(angle);
}


void DriveController::setPIDCoeffs() {
    // TODO: maybe better to rewrite with setters
    m_pidArray[PID_Rotation] =
        PIDWrapper(ROTATION_PID_P, ROTATION_PID_I, ROTATION_PID_D,
                   PIDWrapper::ErrorType_Angular);

    m_pidArray[PID_Straight] =
        PIDWrapper(STRAIGHT_PID_P, STRAIGHT_PID_I, STRAIGHT_PID_D,
                   PIDWrapper::ErrorType_Angular);

    m_pidArray[PID_Distance] =
        PIDWrapper(DISTANCE_PID_P, DISTANCE_PID_I, DISTANCE_PID_D,
                   PIDWrapper::ErrorType_Normal);
}


// NOTE: this code can be moved to MotorsController, but we assume,
// that latter has no state. On the other hand, DriveController is
// statefull class that must do some kind of state machine stuff
void DriveController::setMove(Moves moveCommand) {
    if (m_currentMove == moveCommand) {
        return;
    }

    m_currentMove = moveCommand;

    switch (m_currentMove) {
        case Move_Forward:
            m_motorsController.move(MotorsController::Move_Forward);
            break;

        case Move_Backward:
            m_motorsController.move(MotorsController::Move_Backward);
            break;

        case Move_Left:
            m_motorsController.move(MotorsController::Move_RotateLeft);
            break;

        case Move_Right:
            m_motorsController.move(MotorsController::Move_RotateRight);
            break;

        case Move_Stop:
            m_motorsController.stop();
            break;
    }
}


void DriveController::initPID() {
    m_pidArray[PID_Rotation].pid()->SetOutputLimits(
        ROTATION_PID_OUTPUT_LIMIT_MIN, ROTATION_PID_OUTPUT_LIMIT_MAX);

    m_pidArray[PID_Straight].pid()->SetOutputLimits(
        STRAIGHT_PID_OUTPUT_LIMIT_MIN, STRAIGHT_PID_OUTPUT_LIMIT_MAX);

    m_pidArray[PID_Distance].pid()->SetOutputLimits(
        DISTANCE_PID_OUTPUT_LIMIT_MIN, DISTANCE_PID_OUTPUT_LIMIT_MAX);

    // TODO: PID_NUM - possibly bad practice
    for (uint8_t i = 0; i < PID_USED_NUM; i++) {
        // TODO: not sure if this needed
        // m_pidArray[i].pid()->SetSampleTime(PID_SAMPLE_TIME);

        // turn the PID on
        m_pidArray[i].pid()->SetMode(AUTOMATIC);
        m_pidArray[i].reset();
    }
}


void DriveController::changeDirection() {
    switch (m_currentMove) {
        case Move_Forward:
            setMove(Move_Backward);
            break;

        case Move_Backward:
            setMove(Move_Forward);
            break;

        case Move_Left:
            setMove(Move_Right);
            break;

        case Move_Right:
            setMove(Move_Left);
            break;

        case Move_Stop:
            // do nothing
            break;
    }
}


void DriveController::updateCurrentMove() {
    bool needChangeSpeed = false;
    if (m_currentMove == Move_Forward || m_currentMove == Move_Backward) {
        needChangeSpeed = evalStraightMove();
        needChangeSpeed |= evalStraightCorrection();
    } else if (m_currentMove == Move_Left || m_currentMove == Move_Right)
        needChangeSpeed = evalRotationMove();
    else {
        // TODO: is it ok?
        return;
    }

    if (needChangeSpeed)
        m_motorsController.setSpeed(m_leftSpeed, m_rightSpeed);
}


bool DriveController::evalStraightMove() {
    double output = m_pidArray[PID_Distance].getOutput();
    uint16_t baseCorrection = abs(int(output));

    if (abs(output) < MINIMAL_MOTOR_SPEED) {
        setMove(Move_Stop);
        return false;
    }

    bool forward = (0.0 < output);
    bool backward = (output < 0.0);

    if ((m_currentMove == Move_Forward && backward) ||
        (m_currentMove == Move_Backward && forward)) {
        changeDirection();
    }

    if (baseCorrection != m_baseSpeed) {
        // TODO: possibly, we need temp variable for corrected speed
        setBaseSpeed(baseCorrection);
        return true;
    }

    return false;
}


bool DriveController::evalStraightCorrection() {
    double output = m_pidArray[PID_Straight].getOutput();
    int baseCorrection = abs(int(output / 2.0));

    // TODO: possibly need to be changed on abs(currentAngle - pidSetpoint)
    if (abs(output) < ANGLE_PID_OUTPUT_THERSHOLD) {
        return false;
    }

    int leftCorrection;
    int rightCorrection;

    bool turnLeft = (0.0 < output);
    bool turnRight = (output < 0.0);

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


bool DriveController::evalRotationMove() {
    double output = m_pidArray[PID_Rotation].getOutput();
    // TODO: test it. possibly we doesn't need divide
    uint8_t baseCorrection = abs(int(output / 2.0));

    // TODO: possibly need to be changed on abs(currentAngle - pidSetpoint)
    if (abs(output) < ANGLE_PID_OUTPUT_THERSHOLD) {
        setMove(Move_Stop);
        return false;
    }

    bool turnLeft = (0.0 < output);
    bool turnRight = (output < 0.0);

    if ((m_currentMove == Move_Left && turnRight) ||
        (m_currentMove == Move_Right && turnLeft)) {
        changeDirection();
    }

    m_leftSpeed = baseCorrection;
    m_rightSpeed = baseCorrection;
    return true;
}
