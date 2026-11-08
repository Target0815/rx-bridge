#include <stdio.h>
#include <string.h>
#include "convert.h"

/* wrap-free millisecond clock derived from the 32-bit microsecond timestamps */
static uint32_t now_ms(conv_t *c, uint32_t t) {
    c->ms_acc_us += t - c->t_prev_us;
    c->t_prev_us = t;
    c->ms += c->ms_acc_us / 1000u;
    c->ms_acc_us %= 1000u;
    return c->ms + 1;
}

void conv_init(conv_t *c, const conv_io_t *io) {
    memset(c, 0, sizeof *c);
    c->io = *io;
    fbus_init(&c->fbus);
    fport1_init(&c->fport1);
    crsf_parser_init(&c->crsf);
    telem_init(&c->tm);
    c->lq = 0;
}

uint8_t conv_rssi_to_dbm(uint8_t rssi) {
    /* FrSky reports roughly 0..100 (ArduPilot assumes 0..50 for FBUS). Map 100 -> -30 dBm,
     * 0 -> -120 dBm; ArduPilot and the INAV OSD turn dBm back into a percentage. */
    if (rssi > 100) rssi = 100;
    return (uint8_t)(120 - (rssi * 9 + 5) / 10);
}

static void hist_push(conv_t *c, bool lost) {
    uint8_t w = c->hist_pos >> 5, b = c->hist_pos & 31;
    if (lost) c->lost_bits[w] |= 1u << b; else c->lost_bits[w] &= ~(1u << b);
    c->hist_pos = (uint8_t)((c->hist_pos + 1) % 100);
    int n = 0;
    for (int i = 0; i < 100; i++) n += (c->lost_bits[i >> 5] >> (i & 31)) & 1;
    c->lq = (uint8_t)(100 - n);
}

static void send_link_stats(conv_t *c, bool link_ok) {
    crsf_link_t l = {0};
    uint8_t dbm = link_ok ? conv_rssi_to_dbm(c->rssi) : 130;
    l.rssi1_dbm = l.rssi2_dbm = dbm;
    l.lq = link_ok ? c->lq : 0;
    l.rf_mode = 2;                  /* "150 Hz" class: F.Port/FBUS send ~110 frames/s */
    l.dl_rssi_dbm = dbm;
    l.dl_lq = l.lq;
    uint8_t f[16];
    int n = crsf_build_link_stats(&l, f);
    c->io.to_fc(f, n, c->io.ctx);
    c->st.link_stats++;
}

static void on_control(conv_t *c, const frsky_event_t *ev, uint32_t t) {
    c->st.rc_frames++;
    /* count frames that never arrived as lost */
    if (c->last_rc_us) {
        uint32_t gap = t - c->last_rc_us;
        if (c->rc_interval_us) {
            uint32_t slots = (gap + c->rc_interval_us / 2) / c->rc_interval_us;
            if (slots > 1 && slots < 100)
                for (uint32_t i = 1; i < slots; i++) hist_push(c, true);   /* frames that never arrived */
        }
        if (gap > 2000 && gap < 40000 && (!c->rc_interval_us || gap < c->rc_interval_us * 3 / 2))
            c->rc_interval_us = c->rc_interval_us ? (c->rc_interval_us * 7 + gap) / 8 : gap;
    }
    c->last_rc_us = t;
    c->rssi = ev->rssi;
    c->nch = ev->nch;
    c->failsafe = (ev->flags & FRSKY_FLAG_FAILSAFE) != 0;
    hist_push(c, (ev->flags & (FRSKY_FLAG_FRAME_LOST | FRSKY_FLAG_FAILSAFE)) != 0);

    if (c->failsafe) {
        /* like ELRS: no RC frames in failsafe, the FC detects signal loss itself */
        c->st.failsafe_frames++;
    } else {
        uint16_t ch[16];
        for (int i = 0; i < 16; i++) ch[i] = (i < ev->nch) ? ev->ch[i] : 992;
        uint8_t f[26];
        int n = crsf_build_rc(ch, f);
        c->io.to_fc(f, n, c->io.ctx);
        c->st.rc_forwarded++;
    }
    if (!c->ls_pending && (uint32_t)(t - c->last_ls_us) >= CONV_LINKSTATS_PERIOD_US - CONV_LINKSTATS_OFFSET_US) {
        /* place link statistics between two RC frames: INAV keeps only one CRSF frame buffer */
        uint32_t off = c->rc_interval_us ? c->rc_interval_us / 3 : CONV_LINKSTATS_OFFSET_US;
        if (off > CONV_LINKSTATS_OFFSET_US) off = CONV_LINKSTATS_OFFSET_US;
        if (off < 1000) off = 1000;
        c->ls_pending = true;
        c->ls_due_us = t + off;
    }
}

static void schedule_uplink(conv_t *c, uint8_t prim, uint32_t t) {
    c->up_pending = true;
    c->up_prim = prim;
    c->up_due_us = t + CONV_UPLINK_MIN_DELAY_US;
    c->up_deadline_us = t + CONV_UPLINK_MAX_DELAY_US;
}

static void on_poll(conv_t *c, const frsky_event_t *ev, uint32_t t) {
    if (c->proto == RXP_FBUS) {
        if (ev->phy != FRSKY_FC_PHY_ID) return;
        if (ev->prim != FRSKY_PRIM_NULL && ev->prim != FRSKY_PRIM_DATA) return;
        c->st.polls++;
        schedule_uplink(c, ev->prim, t);
    } else if (c->proto == RXP_FPORT1) {
        /* response rules as INAV / ArduPilot fport.c */
        if (ev->prim == FRSKY_PRIM_DATA) {
            c->fp1_rx_driven = true;
        } else if (ev->prim == FRSKY_PRIM_NULL) {
            if (c->fp1_rx_driven) return;
            if (c->fp1_consecutive >= 2) { c->fp1_consecutive = 0; return; }
            c->fp1_consecutive++;
        } else {
            return;
        }
        c->st.polls++;
        schedule_uplink(c, FRSKY_PRIM_DATA, t);
    }
}

static void lock(conv_t *c, rxproto_t p) {
    if (c->proto != p) {
        c->proto = p;
        c->fp1_rx_driven = false;
        c->fp1_consecutive = 0;
        c->up_pending = false;
    }
    c->ever_locked = true;
}

void conv_rx_byte(conv_t *c, uint8_t b, uint32_t t) {
    frsky_event_t ev;
    if (c->proto != RXP_FPORT1 && fbus_feed(&c->fbus, b, t, &ev)) {
        if (ev.type == FR_EVT_CONTROL) lock(c, RXP_FBUS);
        if (c->proto == RXP_FBUS) {
            c->last_valid_us = t;
            if (ev.type == FR_EVT_CONTROL) on_control(c, &ev, t);
            else on_poll(c, &ev, t);
        }
    }
    if (c->proto != RXP_FBUS && fport1_feed(&c->fport1, b, t, &ev)) {
        if (ev.type == FR_EVT_CONTROL) lock(c, RXP_FPORT1);
        if (c->proto == RXP_FPORT1) {
            c->last_valid_us = t;
            if (ev.type == FR_EVT_CONTROL) on_control(c, &ev, t);
            else on_poll(c, &ev, t);
        }
    }
}

/* ------------------------------------------------------------------------ */
/* flight controller identification                                          */
/*   ArduPilot answers a CRSF device ping with its name and version.         */
/*   INAV ignores pings but answers MSP over CRSF (MSP_FC_VARIANT/VERSION).  */
/* ------------------------------------------------------------------------ */
static void fcid_request(conv_t *c) {
    uint8_t f[16];
    int n;
    if (c->fcid_step == 0) {                             /* device ping, receiver -> FC */
        f[0] = CRSF_ADDR_FC; f[1] = 4; f[2] = CRSF_T_PING; f[3] = CRSF_ADDR_FC; f[4] = CRSF_ADDR_RX;
        f[5] = crsf_crc8(&f[2], 3);
        n = 6;
    } else {                                             /* MSP v1 request, as the INAV Lua scripts send it */
        uint8_t cmd = c->fcid_step == 1 ? 2 : 3;         /* MSP_FC_VARIANT, MSP_FC_VERSION */
        f[0] = CRSF_ADDR_FC; f[1] = 8; f[2] = CRSF_T_MSP_REQ; f[3] = CRSF_ADDR_FC; f[4] = CRSF_ADDR_RADIO;
        f[5] = 0x30;                                     /* MSP v1, start of message, sequence 0 */
        f[6] = 0;                                        /* payload size */
        f[7] = cmd;
        f[8] = cmd;                                      /* MSP v1 checksum (size ^ cmd) */
        f[9] = crsf_crc8(&f[2], 7);
        n = 10;
    }
    c->io.to_fc(f, n, c->io.ctx);
    c->st.fcid_requests++;
    /* alternate ping / MSP until one of them answers; after "variant" ask for the version */
    if (c->fcid_step == 0) c->fcid_step = 1;
    else if (c->fcid_step == 1) c->fcid_step = 0;
}

static void copy_name(char *dst, size_t n, const uint8_t *src, int len) {
    size_t k = 0;
    for (int i = 0; i < len && src[i] && k + 1 < n; i++)
        dst[k++] = (src[i] >= 0x20 && src[i] < 0x7F) ? (char)src[i] : '?';
    dst[k] = 0;
}

static void fcid_on_frame(conv_t *c, const uint8_t *f, int n) {
    int pl = f[1] - 2;                                   /* payload length after the type byte */
    if (f[2] == CRSF_T_DEVICE_INFO && pl >= 3 && f[4] == CRSF_ADDR_FC) {
        copy_name(c->fc_name, sizeof c->fc_name, &f[5], pl - 2);
    } else if (f[2] == CRSF_T_MSP_RESP && pl >= 5 && f[4] == CRSF_ADDR_FC) {
        const uint8_t *m = &f[5];                        /* status, size, cmd, data (MSP v1) */
        int ml = pl - 2;
        if (!(m[0] & 0x10) || (m[0] & 0x80) || ((m[0] >> 5) & 3) != 1 || ml < 3) return;
        uint8_t size = m[1], cmd = m[2];
        if (size > ml - 3) return;
        if (cmd == 2 && size >= 4) {
            copy_name(c->fc_variant, sizeof c->fc_variant, &m[3], 4);
            c->fcid_step = 2;                            /* now ask for the version */
            c->fcid_next_us = c->last_fc_us;             /* in the next free slot */
        } else if (cmd == 3 && size >= 3 && c->fc_variant[0]) {
            snprintf(c->fc_name, sizeof c->fc_name, "%s %u.%u.%u", c->fc_variant, m[3], m[4], m[5]);
        }
    }
    (void)n;
}

void conv_fc_byte(conv_t *c, uint8_t b, uint32_t t) {
    uint8_t f[CRSF_FRAME_MAX + 2];
    int n = crsf_feed(&c->crsf, b, t, f);
    if (!n) return;
    c->st.fc_frames++;
    c->last_fc_us = t;
    if (f[2] == CRSF_T_DEVICE_INFO || f[2] == CRSF_T_MSP_RESP) {
        fcid_on_frame(c, f, n);
        return;
    }
    if (f[2] == CRSF_T_PING) {
        /* extended frame: dest, origin. ArduPilot pings the receiver (0xEC) for its version */
        if (n >= 6 && (f[3] == CRSF_ADDR_RX || f[3] == 0x00)) {
            c->devinfo_pending = true;
            c->devinfo_dest = f[4];
            c->st.pings++;
        }
        return;
    }
    telem_on_crsf(&c->tm, f, n, now_ms(c, t));
}

void conv_poll(conv_t *c, uint32_t t) {
    (void)now_ms(c, t);
    if (c->proto != RXP_NONE && (uint32_t)(t - c->last_valid_us) > CONV_LOCK_TIMEOUT_US) {
        c->proto = RXP_NONE;
        c->up_pending = false;
        c->failsafe = false;
        c->last_rc_us = 0;
        c->rc_interval_us = 0;
    }

    if (c->up_pending && (int32_t)(t - c->up_due_us) >= 0) {
        c->up_pending = false;
        if ((int32_t)(t - c->up_deadline_us) > 0) {
            c->st.uplinks_late++;
        } else {
            uint16_t appid = 0;
            uint32_t data = 0;
            uint8_t prim = 0;
            if (c->up_prim == FRSKY_PRIM_DATA && telem_next(&c->tm, now_ms(c, t), &appid, &data)) prim = FRSKY_PRIM_DATA;
            else c->st.uplinks_null++;
            uint8_t f[24];
            int n = (c->proto == RXP_FBUS) ? fbus_build_uplink(FRSKY_FC_PHY_ID, prim, appid, data, f)
                                           : fport1_build_uplink(prim, appid, data, f);
            c->io.to_receiver(f, n, c->io.ctx);
            c->st.uplinks++;
        }
    }

    /* forget the identification when the FC stops sending (rebooted or replaced) */
    if (c->last_fc_us && (int32_t)(t - c->last_fc_us) > (int32_t)CONV_FC_SILENT_US) {
        c->last_fc_us = 0;
        c->fc_name[0] = c->fc_variant[0] = 0;
        c->fcid_step = 0;
        c->fcid_tries = 0;
    }

    if (c->ls_pending && (int32_t)(t - c->ls_due_us) >= 0) {
        c->ls_pending = false;
        c->last_ls_us = t;
        /* the identification requests use the link statistics slot (between two RC frames) */
        if (!c->fc_name[0] && c->last_fc_us && !c->failsafe && c->fcid_tries < CONV_FCID_MAX_TRIES &&
            (int32_t)(t - c->fcid_next_us) >= 0) {
            c->fcid_next_us = t + CONV_FCID_PERIOD_US;
            c->fcid_tries++;
            fcid_request(c);
        } else {
            send_link_stats(c, !c->failsafe);
        }
    } else if (c->ever_locked && !c->ls_pending &&
               (c->last_rc_us == 0 || (uint32_t)(t - c->last_rc_us) > 200000u) &&
               (uint32_t)(t - c->last_ls_us) > 500000u) {
        /* receiver silent: report LQ 0 twice per second */
        c->last_ls_us = t;
        send_link_stats(c, false);
    }

    if (c->devinfo_pending) {
        c->devinfo_pending = false;
        uint8_t f[CRSF_FRAME_MAX];
        int n = crsf_build_device_info(c->devinfo_dest, "rx-bridge", CONV_FW_VERSION, f);
        c->io.to_fc(f, n, c->io.ctx);
    }
}
