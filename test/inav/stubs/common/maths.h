#pragma once
#include <stdlib.h>
static inline int scaleRange(int x, int srcMin, int srcMax, int destMin, int destMax) {
    long a = ((long)destMax - destMin) * ((long)x - srcMin);
    long b = (long)srcMax - srcMin;
    return (int)(a / b + destMin);
}
#define constrain(v, lo, hi) ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))
#define sq(x) ((x) * (x))
#define ABS(x) ((x) > 0 ? (x) : -(x))
