/* This module contains all numerical parameters that configures project.
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

// in uS. Delay that performed after motors has been stopped.
#define ANALOG_DELAY_AFTER_STOP 100

// in cm. All readings beyond this value are set to 0. From NewPing library.
#define SONAR_MAX_DISTANCE 300

// in uS. How frequently sonar distance would be updated.
#define SONAR_SPEED 24

// in deg/sec. Sets threshold, where all values below will be assumed as gyro
// noise and wiil be set to 0.
#define GYRO_Z_THRESHOLD 0.15

// If angular PID output would be less than this value, then contol wouldn't
// be performed.
// TODO: possibly, its name or usage should be changed
#define ANGLE_PID_OUTPUT_THERSHOLD 3

// Sample time, that defines how often PID output would be available.
#define PID_SAMPLE_TIME 5

#define ROTATION_PID_P 5.0
#define ROTATION_PID_I 0.1
#define ROTATION_PID_D 0.0
#define ROTATION_PID_OUTPUT_LIMIT_MIN -100
#define ROTATION_PID_OUTPUT_LIMIT_MAX 100

#define STRAIGHT_PID_P 1.0
#define STRAIGHT_PID_I 0.0
#define STRAIGHT_PID_D 0.0
#define STRAIGHT_PID_OUTPUT_LIMIT_MIN -100
#define STRAIGHT_PID_OUTPUT_LIMIT_MAX 100

#define DISTANCE_PID_P 1.0
#define DISTANCE_PID_I 0.0
#define DISTANCE_PID_D 0.0
#define DISTANCE_PID_OUTPUT_LIMIT_MIN -255
#define DISTANCE_PID_OUTPUT_LIMIT_MAX 255

// allowed values: 0-255. Motors wouldn't have speed less than this limit.
#define MINIMAL_MOTOR_SPEED 30

// in degrees.
#define ROTATION_ANGLE_90 90

// Typical values: 9600, 38400, 57600, 115200.
#define SERIAL_BAUDRATE 9600

#endif