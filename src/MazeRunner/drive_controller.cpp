#include "drive_controller.h"

#include <Arduino.h>  // max

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


void DriveController::setBaseSpeed(uint8_t baseSpeed) {
    m_baseSpeed = baseSpeed;
}


void DriveController::init() {
    initPID();
    initOrientation();
}


// must be called as soon as possible
void DriveController::update() {
    yawUpdate();
    distanceUpdate();

    double yaw = getYaw();
    m_pidArray[PID_Rotation].update(yaw);
    m_pidArray[PID_Straight].update(yaw);

    double distance = getDistance();
    m_pidArray[PID_Distance].update(distance);

    m_pidArray[PID_Straight].compute();
    m_pidArray[PID_Rotation].compute();
    m_pidArray[PID_Distance].compute();

#if PID_DEBUG
    // Serial.print("out=");
    // Serial.print(pidOutput);
    // Serial.print(",");
#endif

    updateCurrentMove();
}


// move on `distance` in cm from current position
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

    // reset straight correction on new moving direction
    m_pidArray[PID_Straight].reset();
}


// turn on `angle` in degrees from current position
void DriveController::turn(float angle) {
    if (fuzzyIsNull(angle))
        setMove(Move_Stop);

    // TODO: the same code executes in updateCurrentMove. may need refactor
    // CCW = positive
    if (0 < angle)
        setMove(Move_Left);
    else if (angle < 0)
        setMove(Move_Right);

    m_pidArray[PID_Rotation].reset();
    m_pidArray[PID_Rotation].setRelative(angle);
}


void DriveController::setPIDCoeffs() {
    // TODO: maybe better to rewrite with setters
    m_pidArray[PID_Rotation] = PIDWrapper(ROTATION_PID_P, ROTATION_PID_I, ROTATION_PID_D,
                                          PIDWrapper::ErrorType_Angular);

    m_pidArray[PID_Straight] = PIDWrapper(STRAIGHT_PID_P, STRAIGHT_PID_I, STRAIGHT_PID_D,
                                          PIDWrapper::ErrorType_Angular);

    m_pidArray[PID_Distance] = PIDWrapper(
        DISTANCE_PID_P, DISTANCE_PID_I, DISTANCE_PID_D, PIDWrapper::ErrorType_Normal);
}


// NOTE: this code can be moved to MotorsController, but we assume,
// that it have no state. On the other hand, DriveController is
// statefull class that must do some kind spoof state machine stuff
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

    // TODO: not sure is it needed
    // установим частоту вычисления ПИДа
    // rotationPID.SetSampleTime(PID_SAMPLE_TIME);

    // TODO: PID_NUM - possibly bad practice
    for (uint8_t i = 0; i < PID_NUM; i++) {
        // turn the PID on
        m_pidArray[i].pid()->SetMode(AUTOMATIC);
        m_pidArray[i].reset();
    }
}


// this method changes current direction on opposite
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
    }
}


// must be called as soon, as new PID output obtained
void DriveController::updateCurrentMove() {
    bool needChangeSpeed = false;
    if (m_currentMove == Move_Forward || m_currentMove == Move_Backward) {
        needChangeSpeed = evalStraightMove();
        needChangeSpeed |= evalStraightCorrection();
    } else if (m_currentMove == Move_Left || m_currentMove == Move_Right)
        needChangeSpeed = evalRotationMove();
    else {
        // TODO: is it alright?
        return;
    }

    if (needChangeSpeed)
        m_motorsController.setSpeed(m_leftSpeed, m_rightSpeed);
}


bool DriveController::evalStraightMove() {
    double output = m_pidArray[PID_Distance].getOutput();
    uint8_t baseCorrection = abs(int(output));

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
        m_baseSpeed = baseCorrection;
        return true;
    }

    return false;
}


bool DriveController::evalStraightCorrection() {
    double output = m_pidArray[PID_Straight].getOutput();
    int baseCorrection = abs(int(output / 2.0));

    // TODO: possibly need to be changed on abs(currentAngle - pidSetpoint)
    if (abs(output) < ANGLE_SETPOINT_DELTA) {
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
    if (abs(output) < ANGLE_SETPOINT_DELTA) {
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
