#include "orientation.h"

// #define BMX055_DISABLE_MADGWICK
// #define BMX055_ENABLE_MAHONY
// #define BMX055_DISABLE_BMM
#include <iarduino_Position_BMX055.h>

#include "constants.h"
#include "utils.h" // constrainAngle

// BMA - акселерометр
// BMG - гироскоп.
// BMM - магнитометр.
// BMX - все датчики
iarduino_Position_BMX055 sensorG(BMG);

uint32_t g_yawTimer;
double g_yaw = 0;

void initOrientation() {
    sensorG.begin();
    g_yawTimer = micros();
}

double getYaw() {
    // sensor.read(BMX_DEG); - углы Эйлера в градусах (по умолчанию)
    // sensor.read(BMX_RAD); - углы Эйлера в радианах
    // sensor.read(BMX_M_S); - истинное ускорение в м/с²
    // read() - axisX, axisY, axisZ и temp.
    sensorG.read();
    double velZ = sensorG.axisZ;

    if(abs(velZ) < GYRO_Z_THRESHOLD)
        velZ = 0.0;
    g_yaw += velZ * ((double)(micros() - g_yawTimer) / 1000000.0);
    g_yaw = constrainAngle(g_yaw);
    g_yawTimer = micros();

#if ORIENTATION_DEBUG
    Serial.print("yaw=");
    Serial.print(g_yaw);
    Serial.print(",");
#endif

    return g_yaw;
}

double getDistance() {
    // TODO: add distance readings from Ultrasonic
    return 0.0;
}