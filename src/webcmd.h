/*
 * Line protocol between the configuration web page (WebSerial) and the firmware.
 * Portable C, tested on the host.
 *
 * Page -> device, one command per line:
 *   #hello            device info + configuration
 *   #get              configuration
 *   #set <key>=<n>    mode (0 transparent, 1 converter), rcsys (0 FrSky),
 *                     inv (0/1), hold (1..100 us)
 *   #save             store in flash
 *   #reboot           restart (needed after changing the mode)
 *   #boot             restart into the USB bootloader
 *   #json <0|1>       status as JSON lines instead of text
 *
 * Device -> page: JSON objects, one per line, field "t" = hello | cfg | ack | st.
 * Lines not starting with '{' are plain console text.
 */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum { WC_NONE = 0, WC_HELLO, WC_GET, WC_SET, WC_SAVE, WC_REBOOT, WC_BOOT, WC_JSON, WC_ERR } wc_cmd_t;
typedef enum { WK_MODE = 0, WK_RCSYS, WK_INV, WK_HOLD } wc_key_t;

typedef struct {
    wc_cmd_t cmd;
    wc_key_t key;
    long     value;
    const char *err;        /* WC_ERR: reason (German, shown on the page) */
} wc_t;

typedef struct {
    uint32_t mode, run_mode, rcsys, inverted, hold_us;
    bool     saved;         /* configuration equals the flash content */
} wc_cfg_t;

typedef struct {            /* converter status, per-second values already as deltas */
    uint32_t secs, baud, rc_per_s, tlm_per_s, up_per_s, null_per_s;
    uint32_t late, crc_rx, crc_fc, pt_drop;
    const char *proto;      /* "FBUS", "F.Port 2", "F.Port" or NULL */
    const char *fc;         /* flight controller firmware, "" if not identified yet */
    uint8_t nch, lq, rssi;
    bool failsafe, ardupilot;
} wc_conv_st_t;

typedef struct {            /* transparent status */
    uint32_t secs, pulses, baud, raw_baud, fc_edges;
    bool inverted;
} wc_bridge_st_t;

void wc_parse(const char *line, wc_t *out);

int wc_fmt_hello(char *b, size_t n, const char *ver, const char *board, int pin_line, int pin_rx, int pin_tx, uint32_t mhz);
int wc_fmt_cfg(char *b, size_t n, const wc_cfg_t *c);
int wc_fmt_ack(char *b, size_t n, const char *cmd, bool ok, const char *msg);
int wc_fmt_conv_st(char *b, size_t n, const wc_conv_st_t *s);
int wc_fmt_bridge_st(char *b, size_t n, const wc_bridge_st_t *s);
