#include "motors_controller.h"

#include <Arduino.h>  // delayMicroseconds

#include "defines.h"


MotorsController::MotorsController(const Motor& left, const Motor& right)
    : m_motors({left, right}) {
    // Might be used for debug purposes
    setNames("L", "R");
}


void MotorsController::move(Moves moveCommand) {
    // Before any changes in direction, we must stop Motors.
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
            // do nothing, because we already stopped motors
            return;

        default:
            return;
    }

#if MOTOR_DEBUG
    Serial.print("|END MOVE|");
#endif
}


// TODO: could be rewritten in key-value manner
void MotorsController::setSpeed(uint8_t leftSpeed, uint8_t rightSpeed) {
    m_motors[Motor_Left].setSpeed(leftSpeed);
    m_motors[Motor_Right].setSpeed(rightSpeed);
}


void MotorsController::stop() {
    for (uint8_t i = 0; i < MOTORS_NUM; i++) {
        m_motors[i].stop();
    }

    // We must do little delay before any changes in speed. By doing this
    // delay, we guarantee, that Motor has enough time to work out
    // necessary speed.
    delayMicroseconds(ANALOG_DELAY_AFTER_STOP);
}


void MotorsController::setNames(const char* leftName, const char* rightName) {
    m_motors[Motor_Left].name = leftName;
    m_motors[Motor_Right].name = rightName;
}
