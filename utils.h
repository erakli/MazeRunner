#ifndef UTILS_H
#define UTILS_H

#include <math.h> // fmod

double constrainAngle(double x) {
    // Normalize to [-180,180):
    x = fmod(x + 180, 360);
    if (x < 0)
        x += 360;
    return x - 180;
}

#endif