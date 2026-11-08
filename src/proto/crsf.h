/*
 * CRSF towards the flight controller: we act like a CRSF receiver.
 * Formats taken from INAV (rx/crsf.c, telemetry/crsf.c) and ArduPilot
 * (AP_RCProtocol_CRSF, AP_CRSF_Telem, AP_CRSF_Protocol.h).
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define CRSF_ADDR_FC          0xC8
#define CRSF_ADDR_RADIO       0xEA
#define CRSF_ADDR_RX          0xEC
#define CRSF_ADDR_TX          0xEE

#define CRSF_T_GPS            0x02
#define CRSF_T_VARIO          0x07
#define CRSF_T_BATTERY        0x08
#define CRSF_T_BARO_ALT       0x09
#define CRSF_T_AIRSPEED       0x0A
#define CRSF_T_HEARTBEAT      0x0B
#define CRSF_T_LINK_STATS     0x14
#define CRSF_T_RC_CHANNELS    0x16
#define CRSF_T_ATTITUDE       0x1E
#define CRSF_T_FLIGHT_MODE    0x21
#define CRSF_T_PING           0x28
#define CRSF_T_DEVICE_INFO    0x29
#define CRSF_T_MSP_REQ        0x7A
#define CRSF_T_MSP_RESP       0x7B
#define CRSF_T_AP_CUSTOM_OLD  0x7F
#define CRSF_T_AP_CUSTOM      0x80

#define CRSF_FRAME_MAX        64

typedef struct {
    uint8_t rssi1_dbm, rssi2_dbm;   /* positive numbers, meaning -dBm */
    uint8_t lq;                     /* % */
    int8_t  snr;
    uint8_t antenna, rf_mode, tx_power;
    uint8_t dl_rssi_dbm, dl_lq;
    int8_t  dl_snr;
} crsf_link_t;

typedef struct {
    uint8_t  buf[CRSF_FRAME_MAX + 2];
    uint8_t  pos;
    uint32_t last_us;
    uint32_t crc_errors;
} crsf_parser_t;

uint8_t crsf_crc8(const uint8_t *p, int n);

int crsf_build_rc(const uint16_t *ch16, uint8_t *out /*26*/);
int crsf_build_link_stats(const crsf_link_t *l, uint8_t *out /*14*/);
int crsf_build_device_info(uint8_t dest, const char *name, uint32_t fw_version, uint8_t *out /*64*/);

void crsf_parser_init(crsf_parser_t *p);
/* Feeds one byte. When a complete frame with valid CRC is available it is copied to
 * frame (sync byte first) and its total length returned, otherwise 0. */
int crsf_feed(crsf_parser_t *p, uint8_t b, uint32_t t_us, uint8_t *frame);
