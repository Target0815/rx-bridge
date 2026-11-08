/*
 * Conversion engine: FrSky FBUS / F.Port (receiver) <-> CRSF (flight controller).
 * Pure logic with injected time and output callbacks, so it runs unchanged on the
 * RP2040 and in the host simulator.
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "frsky.h"
#include "crsf.h"
#include "telem.h"

typedef enum { RXP_NONE = 0, RXP_FBUS, RXP_FPORT1 } rxproto_t;

typedef struct {
    void (*to_receiver)(const uint8_t *d, int n, void *ctx);   /* half-duplex line */
    void (*to_fc)(const uint8_t *d, int n, void *ctx);         /* CRSF to the FC RX pin */
    void *ctx;
} conv_io_t;

typedef struct {
    uint32_t rc_frames;         /* control frames from the receiver */
    uint32_t rc_forwarded;      /* CRSF RC frames sent */
    uint32_t failsafe_frames;
    uint32_t polls;             /* polls for our phy id */
    uint32_t uplinks;           /* telemetry frames sent to the receiver */
    uint32_t uplinks_late;      /* poll answered too late, skipped */
    uint32_t uplinks_null;
    uint32_t link_stats;
    uint32_t fc_frames;         /* valid CRSF frames from the FC */
    uint32_t pings;
    uint32_t fcid_requests;
} conv_stats_t;

typedef struct {
    conv_io_t io;
    fbus_parser_t fbus;
    fport1_parser_t fport1;
    crsf_parser_t crsf;
    telem_t tm;
    conv_stats_t st;

    rxproto_t proto;
    uint32_t last_valid_us;
    bool     ever_locked;

    /* RC */
    uint32_t last_rc_us;
    uint32_t rc_interval_us;    /* smoothed */
    bool     failsafe;
    uint8_t  rssi;
    uint8_t  nch;
    uint32_t lost_bits[4];      /* history of the last 128 frames, 1 = lost */
    uint8_t  hist_pos;
    uint8_t  lq;

    /* pending uplink */
    bool     up_pending;
    uint32_t up_due_us, up_deadline_us;
    uint8_t  up_prim;
    bool     fp1_rx_driven;
    uint8_t  fp1_consecutive;

    /* link statistics */
    bool     ls_pending;
    uint32_t ls_due_us, last_ls_us;

    /* device info reply for ArduPilot */
    bool     devinfo_pending;
    uint8_t  devinfo_dest;

    uint32_t t_prev_us, ms_acc_us, ms;

    /* identification of the flight controller firmware */
    char     fc_name[48];       /* e.g. "ArduPlane V4.5.7 (abc123)" or "INAV 8.0.1", empty = unknown */
    char     fc_variant[5];     /* MSP FC_VARIANT, e.g. "INAV" */
    uint8_t  fcid_step;         /* next request: 0 ping, 1 MSP variant, 2 MSP version */
    uint8_t  fcid_tries;
    uint32_t fcid_next_us;
    uint32_t last_fc_us;        /* last valid CRSF frame from the FC */
} conv_t;

#define CONV_UPLINK_MIN_DELAY_US   500u    /* INAV fport/fport2 */
#define CONV_UPLINK_MAX_DELAY_US  2000u    /* INAV fport 2000, fport2 3000, ArduPilot 2500 */
#define CONV_LINKSTATS_OFFSET_US  3000u    /* after an RC frame, INAV has a single frame buffer */
#define CONV_LINKSTATS_PERIOD_US 100000u
#define CONV_LOCK_TIMEOUT_US    1000000u
#define CONV_FW_VERSION         0x0003u    /* reported in CRSF device info: 0.3 */
#define CONV_FCID_PERIOD_US    1000000u    /* one identification request per second until known */
#define CONV_FCID_MAX_TRIES         30u
#define CONV_FC_SILENT_US      5000000u    /* FC telemetry gone this long: forget the identification */

void conv_init(conv_t *c, const conv_io_t *io);
void conv_rx_byte(conv_t *c, uint8_t b, uint32_t t_us);
void conv_fc_byte(conv_t *c, uint8_t b, uint32_t t_us);
void conv_poll(conv_t *c, uint32_t t_us);

/* FrSky RSSI value -> CRSF uplink RSSI (positive number = -dBm) */
uint8_t conv_rssi_to_dbm(uint8_t rssi);
