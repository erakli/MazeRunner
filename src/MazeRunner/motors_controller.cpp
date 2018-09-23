#include "motors_controller.h"

#include <Arduino.h>  // delayMicroseconds


MotorsController::MotorsController(const Motor& left, const Motor& right)
    : m_motors({left, right}) {
    setNames("L", "R");
}


void MotorsController::move(Moves moveCommand, uint8_t leftSpeed, uint8_t rightSpeed) {
    stop();

#if MOTOR_DEBUG
    Serial.print("|MOVE|");
#endif

    switch (moveCommand) {
        case Move_Forward:
            m_motors[Motor_Left].forward();
            m_motors[Motor_Right].forward();
            break;

        case Move_Backward:
            m_motors[Motor_Left].backward();
            m_motors[Motor_Right].backward();
            break;

        case Move_RotateLeft:
            m_motors[Motor_Left].backward();
            m_motors[Motor_Right].forward();
            break;

        case Move_RotateRight:
            m_motors[Motor_Left].forward();
            m_motors[Motor_Right].backward();
            break;

        case Move_Stop:
            return;
            break;

        default:
            return;
    }  // end switch

    if (leftSpeed != 0 || rightSpeed != 0)
        setSpeed(leftSpeed, rightSpeed);

#if MOTOR_DEBUG
    Serial.print("|END MOVE|");
#endif
}


void MotorsController::setSpeed(uint8_t leftSpeed, uint8_t rightSpeed) {
    m_motors[Motor_Left].setSpeed(leftSpeed);
    m_motors[Motor_Right].setSpeed(rightSpeed);
}


void MotorsController::stop() {
    for (uint8_t i = 0; i < MOTORS_NUM; i++) {
        m_motors[i].stop();
    }

    delayMicroseconds(ANALOG_DELAY_AFTER_STOP);
}


void MotorsController::setDirections(Motor::Directions leftDirection,
                                     Motor::Directions rightDirection) {
    m_motors[Motor_Left].setDirection(leftDirection);
    m_motors[Motor_Right].setDirection(rightDirection);
}


void MotorsController::setNames(const char* leftName, const char* rightName) {
    m_motors[Motor_Left].name = leftName;
    m_motors[Motor_Right].name = rightName;
}
