#include "orientation.h"

#include <NewPing.h>
#include <iarduino_Position_BMX055.h>

#include "constants.h"
#include "pins.h"
#include "utils.h"  // constrainAngle

// BMA - акселерометр
// BMG - гироскоп.
// BMM - магнитометр.
// BMX - все датчики
iarduino_Position_BMX055 sensorG(BMG);
uint32_t g_yawTimer;
double g_yaw = 0;

NewPing sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN, SONAR_MAX_DISTANCE);
uint32_t g_pingTimer;
uint16_t g_sonarDistance = 0;

void initOrientation() {
    sensorG.begin();
    g_yawTimer = micros();
    g_pingTimer = millis();
}

void yawUpdate() {
    // sensor.read(BMX_DEG); - углы Эйлера в градусах (по умолчанию)
    // sensor.read(BMX_RAD); - углы Эйлера в радианах
    // sensor.read(BMX_M_S); - истинное ускорение в м/с²
    // read() - axisX, axisY, axisZ и temp.
    sensorG.read();
    double velZ = sensorG.axisZ;

    if (abs(velZ) < GYRO_Z_THRESHOLD)
        velZ = 0.0;
    g_yaw += velZ * ((double)(micros() - g_yawTimer) / 1000000.0);
    g_yaw = constrainAngle(g_yaw);
    g_yawTimer = micros();

#if ORIENTATION_DEBUG
    Serial.print("yaw=");
    Serial.print(g_yaw);
    Serial.print(",");
#endif
}

double getYaw() {
    return g_yaw;
}

// If ping received, set the sensor distance to array.
void echoCheckCallback() {
    if (sonar.check_timer()) {
        g_sonarDistance = sonar.ping_result / US_ROUNDTRIP_CM;
    }
}

// must be called as soon as possible
void distanceUpdate() {
    if (g_pingTimer <= millis()) {
        g_pingTimer += PING_SPEED;
        sonar.ping_timer(echoCheckCallback);
    }
}

uint16_t getDistance() {
    return g_sonarDistance;
}