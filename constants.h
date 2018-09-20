#ifndef CONSTANTS_H
#define CONSTANTS_H

#define MOTORS_NUM 2

// микросекунды
#define ANALOG_DELAY_AFTER_STOP 100 

#define GYRO_Z_THRESHOLD 0.15

// TODO: rename it
#define ANGLE_SETPOINT_DELTA 3

#define PID_OUTPUT_LIMIT_MIN -100
#define PID_OUTPUT_LIMIT_MAX 100

// время, через которое будет вычисляться выход ПИДа
#define PID_SAMPLE_TIME 5 

#define PID_P 5.0
#define PID_I 0.1
#define PID_D 0.0

#define DEFAULT_MOTORS_SPEED 100

// motors wouldn't held less than this limit
#define MINIMAL_MOTOR_SPEED 30

// degrees
#define ROTATION_ANGLE_90 90

#define SERIAL_SPEED 9600

#define DEBUG true

#if DEBUG
    #define MOTOR_DEBUG true
    #define ORIENTATION_DEBUG true
    #define PID_PRINT_DEBUG true
#endif

#endif