#pragma once
#include <stdint.h>
typedef uint32_t timeUs_t;
typedef uint32_t timeMs_t;
typedef int32_t timeDelta_t;
#define MS2US(x) ((x) * 1000)
static inline timeDelta_t cmpTimeUs(timeUs_t a, timeUs_t b) { return (timeDelta_t)(a - b); }
timeUs_t micros(void);
timeUs_t microsISR(void);
timeMs_t millis(void);
