/*
 * FrSky receiver side: FBUS / F.Port 2 and F.Port 1 parsers, uplink (telemetry) frames.
 * Portable C, no hardware dependencies.
 *
 * Formats taken from INAV (src/main/rx/fport.c, fport2.c) and ArduPilot
 * (AP_RCProtocol_FPort.cpp, AP_RCProtocol_FPort2.cpp).
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define FRSKY_FC_PHY_ID        0x1B   /* physical ID the receiver polls for the flight controller */
#define FRSKY_PRIM_NULL        0x00
#define FRSKY_PRIM_DATA        0x10
#define FRSKY_FLAG_FRAME_LOST  0x04
#define FRSKY_FLAG_FAILSAFE    0x08
#define FRSKY_MAX_CH           24

typedef enum { FR_EVT_NONE = 0, FR_EVT_CONTROL, FR_EVT_POLL } frsky_evt_type_t;

typedef struct {
    frsky_evt_type_t type;
    /* control frame */
    uint8_t  nch;
    uint16_t ch[FRSKY_MAX_CH];      /* raw 11-bit SBUS values (172..1811 = 988..2012 us) */
    uint8_t  flags;
    uint8_t  rssi;
    /* poll (downlink) frame */
    uint8_t  phy;
    uint8_t  prim;
    uint16_t appid;
    uint32_t data;
} frsky_event_t;

typedef struct {
    uint8_t  buf[40];
    uint8_t  pos, need, nch;
    bool     downlink;
    bool     after_frame, first;
    uint32_t last_us;
    uint32_t crc_errors;
} fbus_parser_t;

typedef struct {
    uint8_t  buf[40];
    uint8_t  pos;
    bool     in_frame, esc;
    uint32_t last_us;
    uint32_t crc_errors;
} fport1_parser_t;

/* FrSky checksum: 0xFF - (byte sum with carry folding) */
uint8_t frsky_checksum(const uint8_t *p, int n);

void sbus_unpack(const uint8_t *p, int nch, uint16_t *ch);
void sbus_pack(const uint16_t *ch, int nch, uint8_t *out);   /* out: nch*11/8 bytes */

void fbus_init(fbus_parser_t *p);
bool fbus_feed(fbus_parser_t *p, uint8_t b, uint32_t t_us, frsky_event_t *ev);

void fport1_init(fport1_parser_t *p);
bool fport1_feed(fport1_parser_t *p, uint8_t b, uint32_t t_us, frsky_event_t *ev);

/* uplink frames from the "flight controller" (us) to the receiver; return length */
int fbus_build_uplink(uint8_t phy, uint8_t prim, uint16_t appid, uint32_t data, uint8_t *out /*10*/);
int fport1_build_uplink(uint8_t prim, uint16_t appid, uint32_t data, uint8_t *out /*20*/);

/* test helpers: build receiver frames (used by host tests and the simulator) */
int fbus_build_control(const uint16_t *ch, int nch, uint8_t flags, uint8_t rssi, uint8_t *out);
int fbus_build_downlink(uint8_t phy, uint8_t prim, uint16_t appid, uint32_t data, uint8_t *out);
int fport1_build_control(const uint16_t *ch, uint8_t flags, uint8_t rssi, uint8_t *out);
int fport1_build_request(uint8_t prim, uint16_t appid, uint32_t data, uint8_t *out);
