#ifndef CONSTANTS_H
#define CONSTANTS_H

#define MOTORS_NUM 2

// микросекунды
#define ANALOG_DELAY_AFTER_STOP 100

// in cm
#define SONAR_MAX_DISTANCE 300

// in uS, how frequently ping distance is updated
#define PING_SPEED 24

#define GYRO_Z_THRESHOLD 0.15

// TODO: rename it
#define ANGLE_SETPOINT_DELTA 3
#define DISTANCE_OUTPUT_DELTA 3

// время, через которое будет вычисляться выход ПИДа
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

#define DEFAULT_MOTORS_SPEED 100

// motors wouldn't held less than this limit
#define MINIMAL_MOTOR_SPEED 30

// degrees
#define ROTATION_ANGLE_90 90

#define SERIAL_SPEED 9600

#define DEBUG true

#if DEBUG
#define MOTOR_DEBUG false
#define ORIENTATION_DEBUG false
#define PID_PRINT_DEBUG false
#endif

#endif