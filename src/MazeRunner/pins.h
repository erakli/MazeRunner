/* This file contains named definitions for all Arduino pins used in project.
 */

#ifndef PINS_H
#define PINS_H

// iarduino MotorShield default pins
#define PIN_H1 7  // direction
#define PIN_H2 4
#define PIN_E1 6  // speed
#define PIN_E2 5

#define MOTOR_LEFT_DIR_PIN PIN_H2
#define MOTOR_LEFT_PWM_PIN PIN_E2

#define MOTOR_RIGHT_DIR_PIN PIN_H1
#define MOTOR_RIGHT_PWM_PIN PIN_E1

// Ultrasonic sensor pins
#define SONAR_TRIGGER_PIN 3
#define SONAR_ECHO_PIN 2

#endif