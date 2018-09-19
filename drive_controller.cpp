#include <Arduino.h>

#include <pid_v1.h>

#include "constants.h"
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
    , m_leftCorrection(0)
    , m_rightCorrection(0)
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
}


void DriveController::move(Moves moveCommand, uint8_t newSpeed) {
    if (newSpeed != 0)
        setBaseSpeed(newSpeed);

    bool needChangeMove = (m_currentMove != moveCommand);
    if (needChangeMove) {
        m_motorsController.stop();
        m_currentMove = moveCommand;
    }

    switch (moveCommand) {
        case Move_Forward:
            if (needChangeMove) 
                m_motorsController.move(MotorsController::Move_Forward);
            straightLineMove();
        break;

        case Move_Backward:
            if (needChangeMove) 
                m_motorsController.move(MotorsController::Move_Backward);
            straightLineMove();
        break;

        case Move_Left:
            if (needChangeMove) 
                m_motorsController.move(MotorsController::Move_RotateLeft);
            // TODO: PID rotation
        break;

        case Move_Right:
            if (needChangeMove) 
                m_motorsController.move(MotorsController::Move_RotateLeft);
            // TODO: PID rotation
        break;

        case Move_Stop:
            // do nothing
        break;

        default:
            // do nothing
            ;
    }
}


void DriveController::initPID() {
    rotationPID.SetOutputLimits(PID_OUTPUT_LIMIT_MIN, PID_OUTPUT_LIMIT_MAX);

    // TODO: not sure is it needed
    // установим частоту вычисления ПИДа
    // rotationPID.SetSampleTime(PID_SAMPLE_TIME); 

    // turn the PID on
    rotationPID.SetMode(AUTOMATIC);
}


void DriveController::straightLineMove() {
    evalStraightCorrection();
    
    // protect from negative values
    // NOTE: we can change direction of motors on negative corrections
    int leftSpeed = m_baseSpeed + m_leftCorrection;        
    leftSpeed = max(leftSpeed, 0);

    int rightSpeed = m_baseSpeed + m_rightCorrection;
    rightSpeed = max(rightSpeed, 0);
    
    m_motorsController.setSpeed(leftSpeed, rightSpeed);
}


void DriveController::evalStraightCorrection() {
    // NOTE: используем глобальщину!
    int baseCorrection = abs(int(pidOutput / 2.0));

    if (ANGLE_SETPOINT_DELTA < abs(pidOutput)) { 
        bool turnLeft = (0.0 < pidOutput);
        bool turnRight = (pidOutput < 0.0);

        if (turnLeft) {
            m_leftCorrection = -1 * baseCorrection;
            m_rightCorrection = baseCorrection;
        } else if (turnRight) {
            m_leftCorrection = baseCorrection;
            m_rightCorrection = -1 * baseCorrection;
        } else {
            // do nothing
        }
    }
}
