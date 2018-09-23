#include "utils.h"

#include <math.h>  // fmod

int sign(int x) {
    return (x > 0) - (x < 0);
}

double constrainAngle(double x) {
    // Normalize to [-180,180):
    x = fmod(x + 180, 360);
    if (x < 0)
        x += 360;
    return x - 180;
}

bool fuzzyIsNull(double d) {
    return fabs(d) <= 0.000000000001;
}

bool fuzzyIsNull(float f) {
    return fabs(f) <= 0.00001f;
}