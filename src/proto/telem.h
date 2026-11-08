/*
 * Telemetry from the flight controller (CRSF frames) -> FrSky sensor values.
 *
 * Standard sensors use the same IDs and units as INAV's SmartPort telemetry
 * (telemetry/smartport.c), so EdgeTX / ETHOS and the INAV Lua scripts recognise them.
 * ArduPilot passthrough frames (CRSF custom frame 0x7F/0x80) are forwarded unchanged,
 * status texts are converted to passthrough 0x5000 chunks (as AP_Frsky_SPort_Passthrough).
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define TELEM_PT_QUEUE 96

enum {
    TV_BATT, TV_GPS, TV_VARIO, TV_BARO, TV_AIRSPEED, TV_ATTITUDE, TV_FMODE, TV_COUNT
};

typedef struct {
    uint32_t t_ms[TV_COUNT];        /* last update, 0 = never */
    /* battery */
    uint16_t volt_dv, curr_da;
    uint32_t cap_mah;
    uint8_t  remaining;
    /* gps */
    int32_t  lat, lon;              /* deg * 1e7 */
    uint16_t gspeed_dkmh, course_cdeg, alt_m_plus1000;
    uint8_t  sats;
    /* vario / baro */
    int16_t  vario_cms;
    int32_t  baro_alt_cm;
    /* airspeed */
    uint16_t airspeed_dkmh;
    /* attitude, rad * 10000 */
    int16_t  pitch, roll, yaw;
    /* flight mode */
    char     fmode[17];
    bool     is_ardupilot;          /* seen heartbeat or custom frames */

    /* passthrough queue (appid, data) */
    uint16_t pt_appid[TELEM_PT_QUEUE];
    uint32_t pt_data[TELEM_PT_QUEUE];
    uint8_t  pt_head, pt_tail;
    uint32_t pt_dropped;

    uint8_t  next_sensor;
    uint8_t  latlon_toggle;
    bool     prefer_pt;
    uint32_t frames;                /* telemetry frames accepted */
} telem_t;

void telem_init(telem_t *t);
/* frame: complete CRSF frame starting with the sync byte */
void telem_on_crsf(telem_t *t, const uint8_t *frame, int len, uint32_t now_ms);
/* next value for a FrSky poll; false if nothing fresh is available */
bool telem_next(telem_t *t, uint32_t now_ms, uint16_t *appid, uint32_t *data);

/* exposed for tests */
uint32_t telem_inav_modes(const char *fmode, bool *known);
int32_t  crsf_baro_alt_cm(uint16_t packed);
int16_t  crsf_vspeed_cms(int8_t packed);
