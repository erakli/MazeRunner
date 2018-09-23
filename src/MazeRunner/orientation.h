#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <stdint.h>  // uint16_t


void initOrientation();
void yawUpdate();
double getYaw();
void distanceUpdate();
uint16_t getDistance();

#endif