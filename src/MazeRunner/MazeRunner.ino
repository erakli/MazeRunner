#include <Arduino.h>

#include "constants.h"
#include "pins.h"
#include "motor.h"
#include "motors_controller.h"
#include "drive_controller.h"


// array of motors
Motor g_motors[] = {
    Motor(MOTOR_LEFT_DIR_PIN, MOTOR_LEFT_PWM_PIN, true),
    Motor(MOTOR_RIGHT_DIR_PIN, MOTOR_RIGHT_PWM_PIN, false)
};

MotorsController motorsController(g_motors[0], g_motors[1]);
DriveController driveController(motorsController);

void setup() {
#if DEBUG
    Serial.begin(SERIAL_SPEED);
    while(!Serial) {
    } 
    driveController.init();
#endif

}


void loop() {

#if DEBUG
    driveController.update();
    if (Serial.available()) {
        char income = Serial.read();

        switch (income) {
            case '4':
                driveController.turn(90);
            break;

            case '6':
                driveController.turn(-90);
            break;
        }
    }
#endif
}
