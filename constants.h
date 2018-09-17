#ifndef CONSTANTS_H
#define CONSTANTS_H

#define ANALOG_DELAY_AFTER_STOP     100 // микросекунды

#define GYRO_Z_THRESHOLD 0.15

#define ANGLE_SETPOINT_DELTA 3

#define PID_OUTPUT_LIMIT_MIN -255
#define PID_OUTPUT_LIMIT_MAX 255

#define PID_SAMPLE_TIME 5 // время, через которое будет вычисляться выход ПИДа

#define BASE_SPEED 80

#endif