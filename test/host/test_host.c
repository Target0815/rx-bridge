/*
 * Host tests for the rx-bridge protocol core: parsers, telemetry mapping and a timed
 * simulation of receiver <-> converter <-> flight controller.
 */
#define _GNU_SOURCE
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../src/proto/convert.h"

static int fails, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ---------------------------------------------------------------------- */
/* capture of engine outputs                                               */
/* ---------------------------------------------------------------------- */
typedef struct { uint32_t t; uint8_t d[64]; int n; } out_t;
static out_t to_rx[20000], to_fc[60000];
static int n_rx, n_fc;
static uint32_t sim_t;

static void cap_rx(const uint8_t *d, int n, void *ctx) {
    (void)ctx;
    if (n_rx < 20000) { to_rx[n_rx].t = sim_t; memcpy(to_rx[n_rx].d, d, n); to_rx[n_rx].n = n; n_rx++; }
}
static void cap_fc(const uint8_t *d, int n, void *ctx) {
    (void)ctx;
    if (n_fc < 60000) { to_fc[n_fc].t = sim_t; memcpy(to_fc[n_fc].d, d, n); to_fc[n_fc].n = n; n_fc++; }
}

/* ---------------------------------------------------------------------- */
static void test_parsers(void) {
    printf("Parser\n");
    /* FBUS 8/16/24 channels */
    for (int nch = 8; nch <= 24; nch += 8) {
        uint16_t ch[24];
        for (int i = 0; i < nch; i++) ch[i] = (uint16_t)(100 + i * 77);
        uint8_t f[64];
        int n = fbus_build_control(ch, nch, 0x04, 55, f);
        fbus_parser_t p; fbus_init(&p);
        frsky_event_t ev; int got = 0;
        for (int i = 0; i < n; i++) got += fbus_feed(&p, f[i], (uint32_t)i * 22, &ev);
        CHECK(got == 1 && ev.nch == nch && ev.flags == 4 && ev.rssi == 55 && !memcmp(ev.ch, ch, nch * 2), "fbus %d ch", nch);
    }
    /* garbage between frames must not break framing; count false positives on pure noise */
    srand(11);
    fbus_parser_t p; fbus_init(&p);
    fport1_parser_t q; fport1_init(&q);
    crsf_parser_t cp; crsf_parser_init(&cp);
    frsky_event_t ev;
    uint8_t cf[80];
    long false_fbus = 0, false_fp1 = 0, false_crsf = 0;
    uint32_t t = 0;
    for (long i = 0; i < 2000000; i++) {
        uint8_t b = (uint8_t)rand();
        t += 22;
        false_fbus += fbus_feed(&p, b, t, &ev);
        false_fp1 += fport1_feed(&q, b, t, &ev);
        false_crsf += crsf_feed(&cp, b, t, cf) > 0;
    }
    printf("  2 MB random noise: false frames FBUS %ld, F.Port %ld, CRSF %ld\n", false_fbus, false_fp1, false_crsf);

    uint16_t ch[16];
    for (int i = 0; i < 16; i++) ch[i] = 992;
    int good = 0, sent = 0;
    fbus_init(&p);
    for (int k = 0; k < 500; k++) {
        uint8_t f[40];
        int n = fbus_build_control(ch, 16, 0, 90, f);
        int junk = rand() % 30;
        for (int i = 0; i < junk; i++) { t += 22; fbus_feed(&p, (uint8_t)rand(), t, &ev); }
        t += 3000;                                   /* real streams have a pause before each frame */
        for (int i = 0; i < n; i++) { t += 22; good += fbus_feed(&p, f[i], t, &ev) && ev.type == FR_EVT_CONTROL; }
        sent++;
    }
    CHECK(good == sent, "fbus resync %d/%d", good, sent);

    crsf_parser_init(&cp);
    good = 0;
    for (int k = 0; k < 500; k++) {
        uint8_t f[40];
        crsf_link_t l = { .lq = (uint8_t)k };
        int n = (k & 1) ? crsf_build_rc(ch, f) : crsf_build_link_stats(&l, f);
        int junk = rand() % 20;
        for (int i = 0; i < junk; i++) { t += 24; crsf_feed(&cp, (uint8_t)(rand() & 0x7F), t, cf); }
        for (int i = 0; i < n; i++) { t += 24; if (crsf_feed(&cp, f[i], t, cf) == n && !memcmp(cf, f, n)) good++; }
    }
    CHECK(good == 500, "crsf resync with junk %d/500", good);
    printf("  resync after junk: FBUS %d/500, CRSF %d/500\n", 500, good);
}

/* ---------------------------------------------------------------------- */
/* telemetry mapping: CRSF frames built like INAV telemetry/crsf.c          */
/* ---------------------------------------------------------------------- */
static int crsf_frame(uint8_t type, const uint8_t *pl, int n, uint8_t *out) {
    out[0] = 0xC8; out[1] = (uint8_t)(n + 2); out[2] = type;
    memcpy(&out[3], pl, n);
    out[3 + n] = crsf_crc8(&out[2], n + 1);
    return n + 4;
}
static void put16(uint8_t *p, uint16_t v) { p[0] = v >> 8; p[1] = v & 0xFF; }
static void put32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }

static bool find_sensor(telem_t *t, uint32_t now, uint16_t id, uint32_t *v, int nth) {
    for (int i = 0; i < 64; i++) {
        uint16_t a; uint32_t d;
        if (!telem_next(t, now, &a, &d)) return false;
        if (a == id && nth-- == 0) { *v = d; return true; }
    }
    return false;
}

static void test_telemetry(void) {
    printf("Telemetry mapping\n");
    telem_t t; telem_init(&t);
    uint8_t f[64], pl[32];
    /* battery 16.4 V, 23.7 A, 1234 mAh, 66 % */
    put16(pl, 164); put16(pl + 2, 237); pl[4] = 0; pl[5] = 0x04; pl[6] = 0xD2; pl[7] = 66;
    int n = crsf_frame(CRSF_T_BATTERY, pl, 8, f); telem_on_crsf(&t, f, n, 1000);
    /* GPS 48.1372, -11.5756, 72 km/h, course 270.5 deg, 520 m, 14 sats */
    put32(pl, (uint32_t)481372000); put32(pl + 4, (uint32_t)-115756000); put16(pl + 8, 720); put16(pl + 10, 27050);
    put16(pl + 12, 1520); pl[14] = 14;
    n = crsf_frame(CRSF_T_GPS, pl, 15, f); telem_on_crsf(&t, f, n, 1000);
    /* baro 123.4 m, +2.5 m/s packed */
    int8_t vp = (int8_t)lrintf(logf(250.0f / 100 + 1) / 0.026f);
    put16(pl, 1234 + 10000); pl[2] = (uint8_t)vp;
    n = crsf_frame(CRSF_T_BARO_ALT, pl, 3, f); telem_on_crsf(&t, f, n, 1000);
    /* attitude pitch 10 deg, roll -20 deg, yaw -90 deg */
    put16(pl, (uint16_t)(int16_t)lrintf(10 * 3.14159265f / 180 * 10000)); put16(pl + 2, (uint16_t)(int16_t)lrintf(-20 * 3.14159265f / 180 * 10000));
    put16(pl + 4, (uint16_t)(int16_t)lrintf(-90 * 3.14159265f / 180 * 10000));
    n = crsf_frame(CRSF_T_ATTITUDE, pl, 6, f); telem_on_crsf(&t, f, n, 1000);
    put16(pl, 720);                                   /* airspeed 72 km/h */
    n = crsf_frame(CRSF_T_AIRSPEED, pl, 2, f); telem_on_crsf(&t, f, n, 1000);
    n = crsf_frame(CRSF_T_FLIGHT_MODE, (const uint8_t *)"ANGL", 5, f); telem_on_crsf(&t, f, n, 1000);

    uint32_t v;
    CHECK(find_sensor(&t, 1100, 0x0210, &v, 0) && v == 1640, "VFAS %u", v);
    CHECK(find_sensor(&t, 1100, 0x0200, &v, 0) && v == 237, "CURR %u", v);
    CHECK(find_sensor(&t, 1100, 0x0600, &v, 0) && v == 66, "FUEL %u", v);
    CHECK(find_sensor(&t, 1100, 0x0100, &v, 0) && v == 12340, "ALT %u", v);
    CHECK(find_sensor(&t, 1100, 0x0110, &v, 0) && abs((int)v - 250) <= 6, "VARIO %d", (int)v);
    CHECK(find_sensor(&t, 1100, 0x0430, &v, 0) && v == 100, "PITCH %d", (int)v);
    CHECK(find_sensor(&t, 1100, 0x0440, &v, 0) && (int)v == -200, "ROLL %d", (int)v);
    CHECK(find_sensor(&t, 1100, 0x0840, &v, 0) && v == 27000, "HEADING %u", v);
    CHECK(find_sensor(&t, 1100, 0x0470, &v, 0) && v == 15, "MODES %u", v);
    CHECK(find_sensor(&t, 1100, 0x0480, &v, 0) && v == 1014, "GNSS %u", v);
    CHECK(find_sensor(&t, 1100, 0x0820, &v, 0) && v == 52000, "GPS_ALT %u", v);
    CHECK(find_sensor(&t, 1100, 0x0830, &v, 0) && abs((int)v - 38877) < 5, "SPEED %u (0.001 kt)", v);
    CHECK(find_sensor(&t, 1100, 0x0450, &v, 0) && v == 2705, "FPV %u", v);
    CHECK(find_sensor(&t, 1100, 0x0A00, &v, 0) && v == 389, "ASPD %u", v);
    /* lat/lon: decode like EdgeTX (minutes * 10000) */
    uint32_t a = 0, b = 0;
    CHECK(find_sensor(&t, 1100, 0x0800, &a, 0) && find_sensor(&t, 1100, 0x0800, &b, 0), "latlon");
    uint32_t lat_raw = (a & 0x80000000u) ? b : a, lon_raw = (a & 0x80000000u) ? a : b;
    double lat = (lat_raw & 0x3FFFFFFF) / 600000.0 * ((lat_raw & 0x40000000) ? -1 : 1);
    double lon = (lon_raw & 0x3FFFFFFF) / 600000.0 * ((lon_raw & 0x40000000) ? -1 : 1);
    CHECK(fabs(lat - 48.1372) < 2e-6 && fabs(lon + 11.5756) < 2e-6, "lat %.6f lon %.6f", lat, lon);
    printf("  VFAS/CURR/FUEL/ALT/VARIO/attitude/modes/GPS/airspeed ok, decoded %.6f %.6f\n", lat, lon);

    /* stale values are not sent */
    CHECK(!find_sensor(&t, 7000, 0x0210, &v, 0), "stale battery still sent");

    /* modes table */
    bool k;
    CHECK(telem_inav_modes("OK", &k) == 1 && k, "OK");
    CHECK(telem_inav_modes("!ERR", &k) == 2, "ERR");
    CHECK(telem_inav_modes("RTH", &k) == 1005, "RTH");
    CHECK(telem_inav_modes("CRUZ", &k) == 8205, "CRUZ");
    telem_inav_modes("FBWA", &k);
    CHECK(!k, "FBWA must be unknown");

    /* baro / vario packing helpers against the INAV / AP encoder */
    CHECK(crsf_baro_alt_cm((uint16_t)(((40000 + 5) / 10) | 0x8000)) == 400000, "alt m range");
    CHECK(crsf_baro_alt_cm(0) == -100000, "alt min");
    CHECK(crsf_vspeed_cms(0) == 0 && crsf_vspeed_cms(-48) < -240, "vspeed sign");
}

/* ArduPilot get_next_msg_chunk (AP_Frsky_SPort_Passthrough.cpp), extra_chunks = 1 */
static int ap_chunks(const char *text, uint8_t sev, uint32_t *out) {
    char buf[50] = {0};
    strncpy(buf, text, sizeof buf);
    int n = 0, idx = 0;
    for (;;) {
        uint32_t chunk = 0;
        uint8_t character = 0;
        for (int i = 0; i < 4 && idx < (int)sizeof buf; i++) {
            character = (uint8_t)buf[idx++];
            if (!character) break;
            chunk |= (uint32_t)character << (3 - i) * 8;
        }
        bool end = !character || idx == (int)sizeof buf;
        if (end) {
            chunk |= (uint32_t)(sev & 0x4) << 21;
            chunk |= (uint32_t)(sev & 0x2) << 14;
            chunk |= (uint32_t)(sev & 0x1) << 7;
        }
        out[n++] = chunk;
        out[n++] = chunk;
        if (end) return n;
    }
}

static void test_passthrough(void) {
    printf("ArduPilot passthrough\n");
    telem_t t; telem_init(&t);
    uint8_t f[64], pl[60];
    /* single packet 0xF0: appid 0x5006 data 0x12345678 (little endian) */
    pl[0] = 0xF0; pl[1] = 0x06; pl[2] = 0x50; pl[3] = 0x78; pl[4] = 0x56; pl[5] = 0x34; pl[6] = 0x12;
    int n = crsf_frame(CRSF_T_AP_CUSTOM_OLD, pl, 7, f); telem_on_crsf(&t, f, n, 100);
    /* multi packet 0xF2 with 3 packets */
    pl[0] = 0xF2; pl[1] = 3;
    for (int i = 0; i < 3; i++) { pl[2 + 6 * i] = (uint8_t)(1 + i); pl[3 + 6 * i] = 0x50; put32(&pl[4 + 6 * i], 0); pl[4 + 6 * i] = (uint8_t)(0xA0 + i); }
    n = crsf_frame(CRSF_T_AP_CUSTOM, pl, 20, f); telem_on_crsf(&t, f, n, 100);
    uint16_t a; uint32_t d;
    CHECK(telem_next(&t, 110, &a, &d) && a == 0x5006 && d == 0x12345678, "single %04x %08x", a, d);
    for (int i = 0; i < 3; i++) CHECK(telem_next(&t, 110, &a, &d) && a == 0x5001 + i && d == (uint32_t)(0xA0 + i), "multi %d", i);
    CHECK(t.is_ardupilot, "ardupilot detected");

    const char *msgs[] = { "PreArm: Throttle too high", "Mode FBWA", "ABCD", "", "EKF3 IMU0 is using GPS" };
    for (unsigned m = 0; m < 5; m++) {
        telem_init(&t);
        pl[0] = 0xF1; pl[1] = (uint8_t)(m + 2);
        size_t l = strlen(msgs[m]);
        memcpy(&pl[2], msgs[m], l + 1);
        n = crsf_frame(CRSF_T_AP_CUSTOM, pl, (int)(l + 3), f); telem_on_crsf(&t, f, n, 100);
        uint32_t ref[64];
        int nr = ap_chunks(msgs[m], (uint8_t)(m + 2), ref);
        int ok = 1;
        for (int i = 0; i < nr; i++) ok &= telem_next(&t, 110, &a, &d) && a == 0x5000 && d == ref[i];
        CHECK(ok && !telem_next(&t, 110, &a, &d), "text '%s'", msgs[m]);
    }
    printf("  single/multi packets forwarded, status texts chunked like ArduPilot\n");
}

/* ---------------------------------------------------------------------- */
/* timed simulation                                                        */
/* ---------------------------------------------------------------------- */
typedef struct { uint32_t t_start, t_end; bool failsafe; bool fc_poll; } rxlog_t;
static rxlog_t rxlog[4000];
static int n_rxlog;

static void send_bytes(conv_t *c, const uint8_t *d, int n, double byte_us, double *t) {
    for (int i = 0; i < n; i++) {
        *t += byte_us;
        sim_t = (uint32_t)*t;
        conv_rx_byte(c, d[i], sim_t);
        conv_poll(c, sim_t);
    }
}

static void idle(conv_t *c, double *t, double until) {
    while (*t < until) {
        *t += 10;
        sim_t = (uint32_t)*t;
        conv_poll(c, sim_t);
    }
}

static void fc_send(conv_t *c, const uint8_t *d, int n, double *t) {
    for (int i = 0; i < n; i++) { *t += 1e7 / 420000; sim_t = (uint32_t)*t; conv_fc_byte(c, d[i], sim_t); conv_poll(c, sim_t); }
}

static void run_sim(rxproto_t proto) {
    const bool fbus = proto == RXP_FBUS;
    printf("Simulation %s, 30 s\n", fbus ? "FBUS 460800" : "F.Port 1 115200");
    conv_t c;
    conv_io_t io = { cap_rx, cap_fc, NULL };
    conv_init(&c, &io);
    n_rx = n_fc = n_rxlog = 0;
    double t = 1000;
    const double byte_us = 1e7 / (fbus ? 460800 : 115200);
    const double frame_period = 9000;
    uint16_t ch[16];
    srand(5);
    int phy_cycle = 0, dropped = 0, control_sent = 0, fs_sent = 0, fc_polls = 0;
    double next_tlm = 0;

    for (int k = 0; t < 31e6; k++) {
        double t0 = 1000 + k * frame_period;
        bool silent = t0 > 20e6 && t0 < 23e6;                /* receiver switched off */
        bool failsafe = t0 > 12e6 && t0 < 13e6;
        bool drop = !silent && (rand() % 100) < 10;           /* 10 % of frames never arrive */
        idle(&c, &t, t0);
        if (silent) continue;
        if (drop) { dropped++; continue; }

        for (int i = 0; i < 16; i++) ch[i] = (uint16_t)(172 + ((k * 13 + i * 97) % 1640));
        uint8_t f[80];
        int n = fbus ? fbus_build_control(ch, 16, failsafe ? FRSKY_FLAG_FAILSAFE : 0, 77, f)
                     : fport1_build_control(ch, failsafe ? FRSKY_FLAG_FAILSAFE : 0, 77, f);
        rxlog[n_rxlog].t_start = (uint32_t)t;
        send_bytes(&c, f, n, byte_us, &t);
        rxlog[n_rxlog].t_end = (uint32_t)t;
        rxlog[n_rxlog].failsafe = failsafe;
        control_sent++;
        fs_sent += failsafe;

        /* downlink poll; FBUS cycles through physical IDs, F.Port always asks the FC */
        uint8_t q[32];
        int nq;
        uint8_t phy = (uint8_t)(phy_cycle++ % 3 == 0 ? FRSKY_FC_PHY_ID : 0x00 + (phy_cycle % 28));
        if (fbus) nq = fbus_build_downlink(phy, FRSKY_PRIM_DATA, 0, 0, q);
        else { nq = fport1_build_request(FRSKY_PRIM_DATA, 0, 0, q); phy = FRSKY_FC_PHY_ID; }
        if (!fbus) idle(&c, &t, t + 200);
        send_bytes(&c, q, nq, byte_us, &t);
        rxlog[n_rxlog].fc_poll = phy == FRSKY_FC_PHY_ID;
        fc_polls += phy == FRSKY_FC_PHY_ID;
        n_rxlog++;

        /* the FC sends telemetry like INAV: one frame every ~ 20 ms */
        if (t > next_tlm) {
            next_tlm = t + 20000;
            uint8_t pl[16], tf[32];
            static int which;
            int nt;
            switch (which++ % 3) {
            case 0: put16(pl, 125); put16(pl + 2, 80); pl[4] = 0; pl[5] = 1; pl[6] = 0; pl[7] = 90; nt = crsf_frame(CRSF_T_BATTERY, pl, 8, tf); break;
            case 1: put16(pl, 100); put16(pl + 2, 200); put16(pl + 4, 300); nt = crsf_frame(CRSF_T_ATTITUDE, pl, 6, tf); break;
            default: nt = crsf_frame(CRSF_T_FLIGHT_MODE, (const uint8_t *)"MANU", 5, tf); break;
            }
            double tt = t;
            fc_send(&c, tf, nt, &tt);
        }
    }
    idle(&c, &t, t + 2000);

    /* --- RC frames to the FC --- */
    int rc_out = 0, ls = 0, ls_zero = 0, rc_bad = 0, ls_too_close = 0;
    uint32_t max_lat = 0;
    crsf_parser_t cp; crsf_parser_init(&cp);
    int li = 0;
    uint32_t last_rc_out = 0;
    for (int i = 0; i < n_fc; i++) {
        uint8_t fr[80];
        int got = 0;
        for (int j = 0; j < to_fc[i].n; j++) got = crsf_feed(&cp, to_fc[i].d[j], to_fc[i].t + j * 24, fr);
        if (got != to_fc[i].n) { rc_bad++; continue; }
        if (fr[2] == CRSF_T_RC_CHANNELS) {
            rc_out++;
            while (li < n_rxlog && rxlog[li].t_end < to_fc[i].t - 50) li++;
            if (li < n_rxlog) { uint32_t lat = to_fc[i].t - rxlog[li].t_end; if (lat > max_lat) max_lat = lat; }
            last_rc_out = to_fc[i].t;
        } else if (fr[2] == CRSF_T_LINK_STATS) {
            ls++;
            if (fr[5] == 0) ls_zero++;
            /* must not follow an RC frame within 1 ms (INAV single buffer) */
            if (last_rc_out && to_fc[i].t - last_rc_out < 1000) ls_too_close++;
        }
    }
    int expected_rc = control_sent - fs_sent;
    CHECK(rc_out == expected_rc, "RC frames out %d, expected %d", rc_out, expected_rc);
    CHECK(rc_bad == 0, "invalid CRSF frames %d", rc_bad);
    CHECK(ls_too_close == 0, "link stats too close to RC frame: %d", ls_too_close);
    /* identification requests take the place of a link statistics frame; at most 30 of them */
    CHECK(c.st.fcid_requests <= CONV_FCID_MAX_TRIES, "identification requests %lu", (unsigned long)c.st.fcid_requests);
    CHECK(ls + (int)c.st.fcid_requests > 250 && ls + (int)c.st.fcid_requests < 320, "link stats count %d + %lu", ls, (unsigned long)c.st.fcid_requests);
    CHECK(ls_zero >= 5, "LQ 0 reports during failsafe / silence: %d", ls_zero);
    printf("  RC: %d frames in, %d failsafe, %d dropped -> %d CRSF frames out, max latency %u us\n",
           control_sent, fs_sent, dropped, rc_out, max_lat);
    printf("  link stats: %d (%d with LQ 0), none within 1 ms after an RC frame, LQ last %u %% (10 %% dropped)\n",
           ls, ls_zero, c.lq);
    CHECK(c.lq >= 82 && c.lq <= 98, "LQ %u with 10%% loss", c.lq);

    /* --- telemetry uplinks to the receiver --- */
    int answered = 0, early = 0, late = 0, wrong = 0, data = 0;
    for (int i = 0; i < n_rx; i++) {
        uint32_t ts = to_rx[i].t;
        int j = -1;
        for (int k = 0; k < n_rxlog; k++) if (rxlog[k].t_end <= ts) j = k;
        if (j < 0 || !rxlog[j].fc_poll) { wrong++; continue; }
        /* poll end = t_end of control + poll bytes; use the logged downlink end */
        uint32_t poll_end = rxlog[j].t_end + (uint32_t)((fbus ? 10 : 12) * byte_us) + (fbus ? 0 : 200);
        int32_t dt = (int32_t)(ts - poll_end);
        if (dt < 480) early++;
        if (dt > 2000) late++;
        answered++;
        if (fbus) {
            fbus_parser_t p; fbus_init(&p); frsky_event_t ev; int ok = 0;
            for (int b = 0; b < to_rx[i].n; b++) ok |= fbus_feed(&p, to_rx[i].d[b], b * 22, &ev);
            if (!ok || ev.phy != FRSKY_FC_PHY_ID) wrong++;
            if (ok && ev.prim == FRSKY_PRIM_DATA) data++;
        } else {
            data += to_rx[i].d[2] == 0x10;
        }
    }
    CHECK(early == 0 && late == 0 && wrong == 0, "uplinks early %d late %d wrong %d", early, late, wrong);
    CHECK(answered >= fc_polls - fs_sent - 10, "answered %d of %d polls", answered, fc_polls);
    CHECK(data > answered / 2, "uplinks with data %d / %d", data, answered);
    printf("  telemetry: %d polls for the FC, %d answered (%d with data), all 500..2000 us after the poll\n",
           fc_polls, answered, data);
}

static void test_ping(void) {
    printf("ArduPilot device ping\n");
    conv_t c;
    conv_io_t io = { cap_rx, cap_fc, NULL };
    conv_init(&c, &io);
    n_fc = 0;
    uint8_t ping[8] = { 0xC8, 4, CRSF_T_PING, CRSF_ADDR_RX, CRSF_ADDR_FC, 0 };
    ping[5] = crsf_crc8(&ping[2], 3);
    double t = 100;
    fc_send(&c, ping, 6, &t);
    sim_t = (uint32_t)t;
    conv_poll(&c, sim_t);
    CHECK(n_fc == 1, "device info sent %d", n_fc);
    if (n_fc != 1) return;
    const uint8_t *f = to_fc[0].d;
    CHECK(f[0] == 0xC8 && f[2] == CRSF_T_DEVICE_INFO, "frame header");
    /* ArduPilot AP_CRSF_Telem::process_device_info_frame */
    const uint8_t *payload = &f[5];
    uint8_t dest = f[3], origin = f[4];
    size_t offset = strnlen((const char *)payload, 42);
    bool elrs = !strncmp((const char *)&payload[offset + 1], "ELRS", 4);
    uint8_t major = payload[offset + 11], minor = payload[offset + 12];
    CHECK(dest == 0xC8 && origin == 0xEC, "addresses %02x %02x", dest, origin);
    CHECK(!elrs && major == 0 && minor == 3, "version %u.%u elrs %d", major, minor, elrs);
    CHECK(crsf_crc8(&f[2], f[1] - 1) == f[f[1] + 1], "crc");
    printf("  answered with device info \"%s\", firmware %u.%u, ArduPilot stops pinging\n", payload, major, minor);
}


static void test_fcid_ardupilot(void) {
    printf("FC-Erkennung ArduPilot (Geraeteabfrage)\n");
    conv_t c;
    conv_io_t io = { cap_rx, cap_fc, NULL };
    conv_init(&c, &io);
    n_fc = 0;
    uint16_t ch[16];
    for (int i = 0; i < 16; i++) ch[i] = 992;
    int ping_at = -1;
    for (int k = 0; k < 400 && ping_at < 0; k++) {
        double t = 1000 + k * 9000.0;
        uint8_t f[40];
        int n = fbus_build_control(ch, 16, 0, 80, f);
        send_bytes(&c, f, n, 21.7, &t);
        uint8_t hb[8] = { 0xC8, 4, CRSF_T_HEARTBEAT, 0, 0xC8, 0 };       /* ArduPilot heartbeat */
        hb[5] = crsf_crc8(&hb[2], 3);
        double tt = t + 1500;
        fc_send(&c, hb, 6, &tt);
        idle(&c, &t, 1000 + (k + 1) * 9000.0);
        for (int i = 0; i < n_fc; i++) if (to_fc[i].d[2] == CRSF_T_PING) { ping_at = i; break; }
    }
    CHECK(ping_at >= 0, "no ping sent");
    if (ping_at < 0) return;
    const uint8_t *p = to_fc[ping_at].d;
    /* AP_CRSF_Telem::process_ping_frame: destination 0 or 0xC8; reply goes to the origin */
    CHECK(p[0] == 0xC8 && p[1] == 4 && (p[3] == 0xC8 || p[3] == 0) && p[4] == CRSF_ADDR_RX &&
          crsf_crc8(&p[2], 3) == p[5], "ping frame");
    /* the RC frame before must be at least 1 ms earlier (INAV single buffer) */
    for (int i = ping_at - 1; i >= 0; i--)
        if (to_fc[i].d[2] == CRSF_T_RC_CHANNELS) { CHECK(to_fc[ping_at].t - to_fc[i].t >= 1000, "ping too close to RC frame"); break; }

    /* AP_CRSF_Telem::calc_device_info */
    uint8_t di[64];
    const char *name = "ArduPlane V4.5.7 (5e1a7b3c)";
    int n = 0;
    di[n++] = 0xC8; di[n++] = 0; di[n++] = CRSF_T_DEVICE_INFO; di[n++] = p[4]; di[n++] = 0xC8;
    memcpy(&di[n], name, strlen(name) + 1); n += (int)strlen(name) + 1;
    di[n++] = 4; di[n++] = 5; di[n++] = 7; di[n++] = 0;      /* serial: version */
    di[n++] = 3; di[n++] = 9; di[n++] = 0; di[n++] = 0;      /* hardware id */
    di[n++] = 0; di[n++] = 0; di[n++] = 0; di[n++] = 0;      /* software id */
    di[n++] = 0; di[n++] = 0;                                /* params, version */
    di[1] = (uint8_t)(n - 1);
    di[n] = crsf_crc8(&di[2], n - 2); n++;
    double t = to_fc[ping_at].t + 3000.0;
    fc_send(&c, di, n, &t);
    CHECK(!strcmp(c.fc_name, name), "fc name '%s'", c.fc_name);
    printf("  Abfrage gesendet, Antwort ausgewertet: \"%s\"\n", c.fc_name);

    /* no more requests once known; forgotten after 5 s without FC frames */
    int before = (int)c.st.fcid_requests;
    double tt = t;
    for (int k = 0; k < 400; k++) {
        uint8_t f[40];
        int nn = fbus_build_control(ch, 16, 0, 80, f);
        tt += 9000;
        send_bytes(&c, f, nn, 21.7, &tt);
        idle(&c, &tt, tt + 7000);
    }
    CHECK(c.st.fcid_requests == (uint32_t)before, "requests after identification");
    CHECK(c.fc_name[0] == 0, "name not forgotten after FC silence");
    printf("  keine weiteren Anfragen, nach 5 s ohne FC-Daten wieder vergessen\n");
}

int main(void) {
    test_parsers();
    test_telemetry();
    test_passthrough();
    run_sim(RXP_FBUS);
    run_sim(RXP_FPORT1);
    test_ping();
    test_fcid_ardupilot();
    printf("Host tests: %d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
