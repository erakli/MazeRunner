#include "motor.h"

#include <Arduino.h>

#include "defines.h"


#if MOTOR_DEBUG
void printMotorDebug(const char* name, const char* message) {
    char buffer[50];
    int n = sprintf(buffer, "(%s)[%s],", name, message);
    Serial.write(buffer, n);
}
#endif


Motor::Motor(uint8_t pin, uint8_t pwmPin, bool reverseDirection)
    : m_pin(pin), m_pwmPin(pwmPin), m_reverseDirection(reverseDirection) {
    pinMode(pin, OUTPUT);

    // from:
    // https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/
    // You do not need to call pinMode() to set the pin as an output before
    // calling analogWrite(). The analogWrite function has nothing to do with
    // the analog pins or the analogRead function.

    // pinMode(pwmPin, OUTPUT);

    if (!m_reverseDirection) {
        m_forwardCommand = HIGH;
        m_backwardCommand = LOW;
    } else {
        m_forwardCommand = LOW;
        m_backwardCommand = HIGH;
    }
}


void Motor::setSpeed(uint8_t speed) {
#if MOTOR_DEBUG
    String speedMsg = String("speed=") + String(speed);
    printMotorDebug(name.c_str(), speedMsg.c_str());
#endif
    analogWrite(m_pwmPin, speed);
}


void Motor::setDirection(Directions direction) {
    switch (direction) {
        case Direction_Forward:
#if MOTOR_DEBUG
            printMotorDebug(name.c_str(), "F");
#endif
            digitalWrite(m_pin, m_forwardCommand);
            break;

        case Direction_Backward:
#if MOTOR_DEBUG
            printMotorDebug(name.c_str(), "B");
#endif
            digitalWrite(m_pin, m_backwardCommand);
            break;

        default:;
    }
}


void Motor::forward() {
    setDirection(Direction_Forward);
}


void Motor::backward() {
    setDirection(Direction_Backward);
}


void Motor::stop() {
#if MOTOR_DEBUG
    printMotorDebug(name.c_str(), "Stop");
#endif
    setSpeed(LOW);
}