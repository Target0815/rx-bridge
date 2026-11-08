#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "drivers/time.h"
#define RSSI_MAX_VALUE 1023
#define PWM_RANGE_MIDDLE 1500
typedef enum {
    RX_FRAME_PENDING = 0, RX_FRAME_COMPLETE = (1 << 0), RX_FRAME_FAILSAFE = (1 << 1),
    RX_FRAME_PROCESSING_REQUIRED = (1 << 2), RX_FRAME_DROPPED = (1 << 3),
} rxFrameState_e;
typedef enum { TRISTATE_AUTO = 0, TRISTATE_ON, TRISTATE_OFF } tristate_e;
typedef struct rxConfig_s { uint8_t serialrx_inverted; uint8_t halfDuplex; } rxConfig_t;
struct rxRuntimeConfig_s;
typedef struct rxRuntimeConfig_s rxRuntimeConfig_t;
typedef uint16_t (*rcReadRawDataFnPtr)(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan);
typedef uint8_t (*rcFrameStatusFnPtr)(rxRuntimeConfig_t *rxRuntimeConfig);
typedef bool (*rcProcessFrameFnPtr)(const rxRuntimeConfig_t *rxRuntimeConfig);
typedef struct rxRuntimeConfig_s {
    uint8_t channelCount;
    rcReadRawDataFnPtr rcReadRawFn;
    rcFrameStatusFnPtr rcFrameStatusFn;
    rcProcessFrameFnPtr rcProcessFrameFn;
    uint16_t *channelData;
    void *frameData;
    void *lqTracker;
} rxRuntimeConfig_t;
typedef struct {
    int16_t uplinkRSSI; uint8_t uplinkLQ; int8_t uplinkSNR; uint8_t rfMode; uint16_t uplinkTXPower; uint8_t activeAntenna;
} rxLinkStatistics_t;
extern rxLinkStatistics_t rxLinkStatistics;
void lqTrackerSet(void *lq, int value);
static inline bool tristateWithDefaultOnIsActive(uint8_t t) { return t != TRISTATE_OFF; }
static inline bool tristateWithDefaultOffIsActive(uint8_t t) { return t == TRISTATE_ON; }
