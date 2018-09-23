#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <stdint.h>  // uint16_t

// Takes care on resources initialization. Must be called before any other 
// methods in this module
void initOrientation();

// Perform readings from gyro and obtains current yaw. Must be called as often
// as possible.
void yawUpdate();

// Returns current yaw (in degrees, (-180, +180)). No readings are performed.
double getYaw();

// Perform readings from sonar and obtains current distance. Must be called 
// as often as possible.
void distanceUpdate();

// Returns current sonar distance in cm. No readings are performed.
uint16_t getDistance();

#endif