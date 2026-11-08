#include <string.h>
#include "frsky.h"

#define FRSKY_GAP_US 1000u      /* a gap this long always starts a new frame */
#define FBUS_START_GAP_US 150u  /* FBUS frames start after an idle gap or directly after a valid frame */

uint8_t frsky_checksum(const uint8_t *p, int n) {
    uint16_t s = 0;
    for (int i = 0; i < n; i++) {
        s += p[i];
        s = (s & 0xFF) + (s >> 8);
    }
    return (uint8_t)(0xFF - s);
}

static bool frsky_checksum_ok(const uint8_t *p, int n_incl_crc) {
    return frsky_checksum(p, n_incl_crc) == 0;
}

void sbus_unpack(const uint8_t *p, int nch, uint16_t *ch) {
    uint32_t acc = 0;
    int bits = 0, k = 0;
    for (int i = 0; k < nch; i++) {
        acc |= (uint32_t)p[i] << bits;
        bits += 8;
        while (bits >= 11 && k < nch) {
            ch[k++] = acc & 0x7FF;
            acc >>= 11;
            bits -= 11;
        }
    }
}

void sbus_pack(const uint16_t *ch, int nch, uint8_t *out) {
    uint32_t acc = 0;
    int bits = 0, o = 0;
    for (int k = 0; k < nch; k++) {
        acc |= (uint32_t)(ch[k] & 0x7FF) << bits;
        bits += 11;
        while (bits >= 8) {
            out[o++] = acc & 0xFF;
            acc >>= 8;
            bits -= 8;
        }
    }
    if (bits) out[o] = acc & 0xFF;
}

/* ------------------------------------------------------------------------ */
/* FBUS / F.Port 2                                                           */
/*   control : len(0x0D/0x18/0x23) 0xFF ch[nch*11/8] flags rssi crc         */
/*   downlink: 0x08 phy prim appid(2,LE) data(4,LE) crc                      */
/*   crc over everything after the length byte                               */
/* ------------------------------------------------------------------------ */
void fbus_init(fbus_parser_t *p) { memset(p, 0, sizeof *p); p->first = true; }

bool fbus_feed(fbus_parser_t *p, uint8_t b, uint32_t t_us, frsky_event_t *ev) {
    uint32_t gap = t_us - p->last_us;
    if (p->pos && gap > FRSKY_GAP_US) p->pos = 0;
    p->last_us = t_us;
    bool after_frame = p->after_frame;
    p->after_frame = false;

    if (p->pos == 0) {
        /* only accept a start byte after an idle gap or right after a valid frame;
         * this keeps the weak 8-bit checksum from locking onto noise */
        if (!after_frame && gap < FBUS_START_GAP_US && !p->first) return false;
        p->first = false;
        p->downlink = false;
        switch (b) {
        case 0x0D: p->nch = 8;  break;
        case 0x18: p->nch = 16; break;
        case 0x23: p->nch = 24; break;
        case 0x08: p->downlink = true; p->nch = 0; break;
        default: return false;
        }
        p->need = (uint8_t)(b + 3);         /* length byte + type + payload + crc */
        if (p->downlink) p->need = 10;
        p->buf[p->pos++] = b;
        return false;
    }
    if (p->pos == 1 && !p->downlink && b != 0xFF) {
        p->pos = 0;                          /* not a control frame, maybe a new start */
        return fbus_feed(p, b, t_us, ev);
    }
    p->buf[p->pos++] = b;
    if (p->pos < p->need) return false;
    p->pos = 0;

    if (!frsky_checksum_ok(&p->buf[1], p->need - 1)) {
        p->crc_errors++;
        return false;
    }
    p->after_frame = true;
    memset(ev, 0, sizeof *ev);
    if (p->downlink) {
        ev->type = FR_EVT_POLL;
        ev->phy = p->buf[1];
        ev->prim = p->buf[2];
        ev->appid = (uint16_t)(p->buf[3] | p->buf[4] << 8);
        ev->data = (uint32_t)p->buf[5] | (uint32_t)p->buf[6] << 8 | (uint32_t)p->buf[7] << 16 | (uint32_t)p->buf[8] << 24;
    } else {
        int chb = p->nch * 11 / 8;
        ev->type = FR_EVT_CONTROL;
        ev->nch = p->nch;
        sbus_unpack(&p->buf[2], p->nch, ev->ch);
        ev->flags = p->buf[2 + chb];
        ev->rssi = p->buf[3 + chb];
    }
    return true;
}

int fbus_build_uplink(uint8_t phy, uint8_t prim, uint16_t appid, uint32_t data, uint8_t *out) {
    out[0] = 0x08;
    out[1] = phy;
    out[2] = prim;
    out[3] = appid & 0xFF;
    out[4] = appid >> 8;
    for (int i = 0; i < 4; i++) out[5 + i] = (data >> (8 * i)) & 0xFF;
    out[9] = frsky_checksum(&out[1], 8);
    return 10;
}

int fbus_build_control(const uint16_t *ch, int nch, uint8_t flags, uint8_t rssi, uint8_t *out) {
    int chb = nch * 11 / 8;
    out[0] = (uint8_t)(chb + 2);
    out[1] = 0xFF;
    sbus_pack(ch, nch, &out[2]);
    out[2 + chb] = flags;
    out[3 + chb] = rssi;
    out[4 + chb] = frsky_checksum(&out[1], chb + 3);
    return chb + 5;
}

int fbus_build_downlink(uint8_t phy, uint8_t prim, uint16_t appid, uint32_t data, uint8_t *out) {
    return fbus_build_uplink(phy, prim, appid, data, out);   /* same layout in both directions */
}

/* ------------------------------------------------------------------------ */
/* F.Port 1: 0x7E len type payload crc 0x7E, byte stuffing 0x7D x^0x20       */
/*   control : len 0x19, type 0x00, ch[22] flags rssi                        */
/*   request : len 0x08, type 0x01, prim appid data                          */
/*   response: len 0x08, type 0x81, prim appid data (stuffed, no 0x7E)       */
/*   crc over len..payload                                                   */
/* ------------------------------------------------------------------------ */
void fport1_init(fport1_parser_t *p) { memset(p, 0, sizeof *p); }

static bool fport1_process(fport1_parser_t *p, frsky_event_t *ev) {
    const uint8_t *f = p->buf;
    int n = p->pos;
    if (n < 4 || f[0] != n - 2) return false;
    if (!frsky_checksum_ok(f, n)) { p->crc_errors++; return false; }
    memset(ev, 0, sizeof *ev);
    if (f[1] == 0x00 && f[0] == 0x19) {
        ev->type = FR_EVT_CONTROL;
        ev->nch = 16;
        sbus_unpack(&f[2], 16, ev->ch);
        ev->flags = f[24];
        ev->rssi = f[25];
        return true;
    }
    if (f[1] == 0x01 && f[0] == 0x08) {
        ev->type = FR_EVT_POLL;
        ev->phy = FRSKY_FC_PHY_ID;
        ev->prim = f[2];
        ev->appid = (uint16_t)(f[3] | f[4] << 8);
        ev->data = (uint32_t)f[5] | (uint32_t)f[6] << 8 | (uint32_t)f[7] << 16 | (uint32_t)f[8] << 24;
        return true;
    }
    return false;
}

bool fport1_feed(fport1_parser_t *p, uint8_t b, uint32_t t_us, frsky_event_t *ev) {
    if (p->in_frame && p->pos && (uint32_t)(t_us - p->last_us) > FRSKY_GAP_US) p->in_frame = false;
    p->last_us = t_us;

    if (b == 0x7E) {
        bool got = false;
        if (p->in_frame && p->pos >= 2) got = fport1_process(p, ev);
        p->in_frame = true;
        p->pos = 0;
        p->esc = false;
        return got;
    }
    if (!p->in_frame) return false;
    if (b == 0x7D) { p->esc = true; return false; }
    if (p->esc) { b ^= 0x20; p->esc = false; }
    if (p->pos >= sizeof p->buf) { p->in_frame = false; return false; }
    p->buf[p->pos++] = b;
    return false;
}

static int fport1_stuff(const uint8_t *raw, int n, uint8_t *out) {
    int o = 0;
    for (int i = 0; i < n; i++) {
        if (raw[i] == 0x7E || raw[i] == 0x7D) {
            out[o++] = 0x7D;
            out[o++] = raw[i] ^ 0x20;
        } else {
            out[o++] = raw[i];
        }
    }
    return o;
}

int fport1_build_uplink(uint8_t prim, uint16_t appid, uint32_t data, uint8_t *out) {
    uint8_t raw[10];
    raw[0] = 0x08;
    raw[1] = 0x81;
    raw[2] = prim;
    raw[3] = appid & 0xFF;
    raw[4] = appid >> 8;
    for (int i = 0; i < 4; i++) raw[5 + i] = (data >> (8 * i)) & 0xFF;
    raw[9] = frsky_checksum(raw, 9);
    return fport1_stuff(raw, 10, out);
}

int fport1_build_control(const uint16_t *ch, uint8_t flags, uint8_t rssi, uint8_t *out) {
    uint8_t raw[27];
    raw[0] = 0x19;
    raw[1] = 0x00;
    sbus_pack(ch, 16, &raw[2]);
    raw[24] = flags;
    raw[25] = rssi;
    raw[26] = frsky_checksum(raw, 26);
    int o = 0;
    out[o++] = 0x7E;
    o += fport1_stuff(raw, 27, &out[o]);
    out[o++] = 0x7E;
    return o;
}

int fport1_build_request(uint8_t prim, uint16_t appid, uint32_t data, uint8_t *out) {
    uint8_t raw[10];
    raw[0] = 0x08;
    raw[1] = 0x01;
    raw[2] = prim;
    raw[3] = appid & 0xFF;
    raw[4] = appid >> 8;
    for (int i = 0; i < 4; i++) raw[5 + i] = (data >> (8 * i)) & 0xFF;
    raw[9] = frsky_checksum(raw, 9);
    int o = 0;
    out[o++] = 0x7E;
    o += fport1_stuff(raw, 10, &out[o]);
    out[o++] = 0x7E;
    return o;
}
