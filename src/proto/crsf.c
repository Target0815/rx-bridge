#include <string.h>
#include "crsf.h"

#define CRSF_GAP_US 1000u

static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
    crc ^= a;
    for (int i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    return crc;
}

uint8_t crsf_crc8(const uint8_t *p, int n) {
    uint8_t c = 0;
    for (int i = 0; i < n; i++) c = crc8_dvb_s2(c, p[i]);
    return c;
}

/* frame layout: sync len type payload crc, len = 1 (type) + payload + 1 (crc) */
static int finish(uint8_t *out, uint8_t type, int payload_len) {
    out[0] = CRSF_ADDR_FC;
    out[1] = (uint8_t)(payload_len + 2);
    out[2] = type;
    out[3 + payload_len] = crsf_crc8(&out[2], payload_len + 1);
    return payload_len + 4;
}

int crsf_build_rc(const uint16_t *ch16, uint8_t *out) {
    /* identical bit packing and value range as SBUS: 16 x 11 bit, LSB first */
    uint32_t acc = 0;
    int bits = 0, o = 3;
    for (int k = 0; k < 16; k++) {
        acc |= (uint32_t)(ch16[k] & 0x7FF) << bits;
        bits += 11;
        while (bits >= 8) { out[o++] = acc & 0xFF; acc >>= 8; bits -= 8; }
    }
    return finish(out, CRSF_T_RC_CHANNELS, 22);
}

int crsf_build_link_stats(const crsf_link_t *l, uint8_t *out) {
    uint8_t *p = &out[3];
    p[0] = l->rssi1_dbm;
    p[1] = l->rssi2_dbm;
    p[2] = l->lq;
    p[3] = (uint8_t)l->snr;
    p[4] = l->antenna;
    p[5] = l->rf_mode;
    p[6] = l->tx_power;
    p[7] = l->dl_rssi_dbm;
    p[8] = l->dl_lq;
    p[9] = (uint8_t)l->dl_snr;
    return finish(out, CRSF_T_LINK_STATS, 10);
}

int crsf_build_device_info(uint8_t dest, const char *name, uint32_t fw_version, uint8_t *out) {
    uint8_t *p = &out[3];
    int n = 0;
    p[n++] = dest;
    p[n++] = CRSF_ADDR_RX;
    size_t l = strlen(name);
    if (l > 30) l = 30;
    memcpy(&p[n], name, l);
    n += (int)l;
    p[n++] = 0;
    memcpy(&p[n], "RXBG", 4); n += 4;            /* serial number (not "ELRS") */
    memset(&p[n], 0, 4); n += 4;                 /* hardware id */
    for (int i = 3; i >= 0; i--) p[n++] = (fw_version >> (8 * i)) & 0xFF;   /* 00 00 major minor */
    p[n++] = 0;                                  /* parameter count */
    p[n++] = 0;                                  /* parameter version */
    return finish(out, CRSF_T_DEVICE_INFO, n);
}

void crsf_parser_init(crsf_parser_t *p) { memset(p, 0, sizeof *p); }

static bool is_sync(uint8_t b) {
    return b == CRSF_ADDR_FC || b == CRSF_ADDR_RADIO || b == CRSF_ADDR_RX || b == CRSF_ADDR_TX;
}

static void drop(crsf_parser_t *p, int n) {
    if (n >= p->pos) { p->pos = 0; return; }
    memmove(p->buf, p->buf + n, p->pos - n);
    p->pos -= n;
}

int crsf_feed(crsf_parser_t *p, uint8_t b, uint32_t t_us, uint8_t *frame) {
    if (p->pos && (uint32_t)(t_us - p->last_us) > CRSF_GAP_US) p->pos = 0;
    p->last_us = t_us;
    if (p->pos >= sizeof p->buf) p->pos = 0;
    p->buf[p->pos++] = b;

    for (;;) {
        if (p->pos == 0) return 0;
        if (!is_sync(p->buf[0])) { drop(p, 1); continue; }
        if (p->pos < 2) return 0;
        uint8_t len = p->buf[1];
        if (len < 2 || len > CRSF_FRAME_MAX - 2) { drop(p, 1); continue; }
        int total = len + 2;
        if (p->pos < total) return 0;
        if (crsf_crc8(&p->buf[2], len - 1) != p->buf[total - 1]) {
            p->crc_errors++;
            drop(p, 1);
            continue;
        }
        memcpy(frame, p->buf, total);
        drop(p, total);
        return total;
    }
}
