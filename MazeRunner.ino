#include <Arduino.h>
#include <pid_v1.h>

#include "constants.h"
#include "motors.h"
#include "orientation.h"

// Define Variables we'll be connecting to
double pidSetpoint;
double pidInput;
double pidOutput;

// Specify the links and initial tuning parameters
double Kp = 1;
double Ki = 0;
double Kd = 0;

PID rotationPID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);


void PIDInit() {
    rotationPID.SetOutputLimits(PID_OUTPUT_LIMIT_MIN, PID_OUTPUT_LIMIT_MAX);
    // rotationPID.SetSampleTime(PID_SAMPLE_TIME); // установим частоту вычисления ПИДа

    // turn the PID on
    rotationPID.SetMode(AUTOMATIC);
}


void PIDCheckFunction(float currentAngle) {
    pidInput = currentAngle;
    rotationPID.Compute(true);

    double desiredCommand = pidOutput;
    double baseCommand = abs(int(desiredCommand / 2.0));
    int rightCommand = BASE_SPEED;
    int leftCommand = BASE_SPEED;

    if (ANGLE_SETPOINT_DELTA < abs(desiredCommand)) { 
        if (desiredCommand < 0.0) {
            rightCommand += baseCommand;
            leftCommand -= baseCommand;
        } else if (0.0 < desiredCommand) {
            rightCommand -= baseCommand;
            leftCommand += baseCommand;
        } else {
            // do nothing
        }
    }

    MotorsSetSpeed(rightCommand, leftCommand);
}


void setup() {
    Serial.begin(9600);

    OrientationInit();
    PIDInit();

    // for test proporses, wait before motors start
    delay(1000);

    MotorsSetMove(Move_Forward, BASE_SPEED, BASE_SPEED);
}

void loop() {
    double currentAngle = GetYaw();
    PIDCheckFunction(currentAngle);
}
