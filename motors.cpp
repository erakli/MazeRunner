#include <Arduino.h>

#include "pins.h"
#include "constants.h"
#include "motors.h"

Motor g_motors[] = {
    Motor(DIR_RIGHT, PWM_RIGHT, false),
    Motor(DIR_LEFT, PWM_LEFT, true)
};

void MotorsSetDirection(Motor::Directions rightMotor, Motor::Directions leftMotor) {
    g_motors[Motor_Right].SetDirection(rightMotor);
    g_motors[Motor_Left].SetDirection(leftMotor);
}

void MotorsSetSpeed(uint8_t rightMotor, uint8_t leftMotor) {
    g_motors[Motor_Right].SetSpeed(rightMotor);
    g_motors[Motor_Left].SetSpeed(leftMotor);
}

void MotorsStop() {
    g_motors[Motor_Right].Stop();
    g_motors[Motor_Left].Stop();

    // TODO: надо этого как-то избежать
    delayMicroseconds(ANALOG_DELAY_AFTER_STOP);
}


void MotorsSetMove(Moves moveCommand, uint8_t speedRight, uint8_t speedLeft) {
    MotorsStop();

    switch (moveCommand) {

        case Move_Forward:  
            g_motors[Motor_Right].Forward();
            g_motors[Motor_Left].Forward();
        break;
      
        case Move_Backwards:
            g_motors[Motor_Right].Backward();
            g_motors[Motor_Left].Backward();
        break;

        case Move_Left:
            g_motors[Motor_Right].Forward();
            g_motors[Motor_Left].Backward();
        break;

        case Move_Right:
            g_motors[Motor_Right].Backward();
            g_motors[Motor_Left].Forward();
        break;

        case Move_Stop:
            return;
        break;
        
        default:
            return;

    } // end switch

    MotorsSetSpeed(speedRight, speedLeft);
}

// reverseDirection - изменить основное направление. Предполагается, что 
// направление Forward соответсвует движению вперёд робота
Motor::Motor(uint8_t pin, uint8_t pwmPin, bool reverseDirection) 
    : pin(pin)
    , pwmPin(pwmPin)
{
    pinMode(pin, OUTPUT);
    pinMode(pwmPin, OUTPUT);

    if (!reverseDirection) {
        forwardCommand = HIGH;
        backwardCommand = LOW;
    } else {
        forwardCommand = LOW;
        backwardCommand = HIGH;
    }
}


void Motor::SetSpeed(uint8_t speed) {
    analogWrite(pwmPin, speed);
}


void Motor::SetDirection(Directions direction) {
    switch (direction) {
        case Direction_Forward:
            digitalWrite(pin, forwardCommand);
        break;

        case Direction_Backward:
            digitalWrite(pin, backwardCommand);
        break;

        default: 
            ;
    }
}


void Motor::Forward() {
    SetDirection(Direction_Forward);
}


void Motor::Backward() {
    SetDirection(Direction_Backward);
}


void Motor::Stop() {
    SetSpeed(LOW);
}