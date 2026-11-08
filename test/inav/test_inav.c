/*
 * Cross-check of rx-bridge protocol code against the ORIGINAL INAV parsers.
 *
 *  - CRSF frames produced by rx-bridge are decoded by INAV rx/crsf.c
 *  - FBUS / F.Port frames produced by the rx-bridge test generators are decoded by INAV
 *    rx/fport2.c / rx/fport.c (so the generators match what INAV - and real receivers - use)
 *  - The telemetry uplink frames INAV writes are compared byte for byte with rx-bridge's
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "platform.h"
#include "rx/rx.h"
#include "harness.h"
#include "../../src/proto/frsky.h"
#include "../../src/proto/crsf.h"
#include "../../src/proto/convert.h"

/* INAV symbols */
bool crsfRxInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig);
void crsfDataReceive(uint16_t c, void *data);
uint8_t crsfFrameStatus(rxRuntimeConfig_t *rxRuntimeConfig);
uint16_t crsfReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan);
extern uint32_t crsfChannelData[];
bool fportRxInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig);
bool fport2RxInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, bool isFBUS);
int h_msp_task(uint8_t *frame);
extern int h_msp_requests;

static int fails, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void feed(const uint8_t *d, int n, uint32_t byte_us) {
    for (int i = 0; i < n; i++) { h_rx_cb(d[i], NULL); h_now_us += byte_us; }
}

static void test_crsf(void) {
    printf("INAV rx/crsf.c <- rx-bridge CRSF output\n");
    rxConfig_t cfg = { 0 };
    rxRuntimeConfig_t rt = { 0 };
    CHECK(crsfRxInit(&cfg, &rt), "crsfRxInit");
    srand(7);
    uint16_t ch[16];
    for (int round = 0; round < 200; round++) {
        for (int i = 0; i < 16; i++) ch[i] = round == 0 ? (uint16_t)(172 + i * 100) : (uint16_t)(rand() & 0x7FF);
        if (round == 1) { ch[0] = 172; ch[1] = 992; ch[2] = 1811; ch[3] = 0; ch[4] = 2047; }
        uint8_t f[32];
        int n = crsf_build_rc(ch, f);
        h_now_us += 5000;
        feed(f, n, 24);
        uint8_t st = crsfFrameStatus(&rt);
        CHECK(st == RX_FRAME_COMPLETE, "rc frame status %d", st);
        for (int i = 0; i < 16; i++) CHECK(crsfChannelData[i] == ch[i], "ch%d %u != %u", i, crsfChannelData[i], ch[i]);
        if (round == 1)
            printf("  172 -> %u us, 992 -> %u us, 1811 -> %u us\n", crsfReadRawRC(&rt, 0), crsfReadRawRC(&rt, 1), crsfReadRawRC(&rt, 2));
    }

    crsf_link_t l = { .rssi1_dbm = 67, .rssi2_dbm = 80, .lq = 87, .snr = 5, .antenna = 0, .rf_mode = 2 };
    uint8_t f[20];
    int n = crsf_build_link_stats(&l, f);
    h_now_us += 5000;
    feed(f, n, 24);
    uint8_t st = crsfFrameStatus(&rt);
    CHECK(st == RX_FRAME_PENDING, "link stats must not count as RC frame");
    CHECK(rxLinkStatistics.uplinkRSSI == -67 && rxLinkStatistics.uplinkLQ == 87 && rxLinkStatistics.rfMode == 2,
          "link stats decoded rssi %d lq %u mode %u", rxLinkStatistics.uplinkRSSI, rxLinkStatistics.uplinkLQ, rxLinkStatistics.rfMode);
    printf("  link stats: RSSI %d dBm, LQ %u %%, rf mode %u\n", rxLinkStatistics.uplinkRSSI, rxLinkStatistics.uplinkLQ, rxLinkStatistics.rfMode);

    /* INAV keeps exactly one frame: a second frame directly behind an RC frame overwrites it
     * before the RX task has polled. This is why rx-bridge spaces link statistics. */
    for (int i = 0; i < 16; i++) ch[i] = 1500;
    uint8_t rc[32];
    int nr = crsf_build_rc(ch, rc);
    h_now_us += 5000;
    feed(rc, nr, 24);
    feed(f, n, 24);
    st = crsfFrameStatus(&rt);
    printf("  RC frame immediately followed by link stats, no poll in between: RC %s\n",
           st == RX_FRAME_COMPLETE ? "received" : "LOST");
    CHECK(st != RX_FRAME_COMPLETE, "expected INAV to lose the RC frame (documents the constraint)");
    h_now_us += 5000;
    feed(rc, nr, 24);
    st = crsfFrameStatus(&rt);
    h_now_us += 1000;
    feed(f, n, 24);
    uint8_t st2 = crsfFrameStatus(&rt);
    CHECK(st == RX_FRAME_COMPLETE && st2 == RX_FRAME_PENDING, "spaced frames");
    printf("  with a poll in between: RC %s\n", st == RX_FRAME_COMPLETE ? "received" : "LOST");
}

static void test_fbus(void) {
    printf("INAV rx/fport2.c (FBUS) <- rx-bridge test generator, uplink compared\n");
    rxConfig_t cfg = { .serialrx_inverted = 1, .halfDuplex = TRISTATE_OFF };
    rxRuntimeConfig_t rt = { 0 };
    CHECK(fport2RxInit(&cfg, &rt, true), "fport2RxInit");
    const double byte_us = 10e6 / 460800;
    srand(3);
    for (int round = 0; round < 100; round++) {
        uint16_t ch[16];
        for (int i = 0; i < 16; i++) ch[i] = (uint16_t)(rand() & 0x7FF);
        uint8_t flags = round == 5 ? FRSKY_FLAG_FAILSAFE : (round == 6 ? FRSKY_FLAG_FRAME_LOST : 0);
        uint8_t rssi = (uint8_t)(rand() % 101);
        uint8_t f[64], d[16];
        int n = fbus_build_control(ch, 16, flags, rssi, f);
        int nd = fbus_build_downlink(FRSKY_FC_PHY_ID, FRSKY_PRIM_DATA, 0, 0, d);
        h_now_us += 6000;
        for (int i = 0; i < n; i++) { h_rx_cb(f[i], NULL); h_now_us += (uint32_t)byte_us; }
        for (int i = 0; i < nd; i++) { h_rx_cb(d[i], NULL); h_now_us += (uint32_t)byte_us; }
        uint8_t st = rt.rcFrameStatusFn(&rt);
        uint8_t want = RX_FRAME_COMPLETE | (flags & FRSKY_FLAG_FAILSAFE ? RX_FRAME_FAILSAFE : 0) |
                       (flags & FRSKY_FLAG_FRAME_LOST ? RX_FRAME_DROPPED : 0);
        CHECK((st & ~RX_FRAME_PROCESSING_REQUIRED) == want, "control status %x want %x", st, want);
        for (int i = 0; i < 16; i++) CHECK(rt.channelData[i] == ch[i], "ch%d", i);
        CHECK(h_lq == rssi * 1023 / 100, "rssi");

        /* downlink poll: INAV answers after >= 500 us */
        st = rt.rcFrameStatusFn(&rt);
        CHECK(!(st & RX_FRAME_PROCESSING_REQUIRED), "must not answer before 500 us");
        h_now_us += 520;
        st = rt.rcFrameStatusFn(&rt);
        CHECK(st & RX_FRAME_PROCESSING_REQUIRED, "should answer after 500 us");
        h_next_payload.frameId = 0x10;
        h_next_payload.valueId = (uint16_t)(0x0100 + round * 7);
        h_next_payload.data = (uint32_t)rand() * 2654435761u;
        h_tx_len = 0;
        rt.rcProcessFrameFn(&rt);
        uint8_t mine[16];
        int nm = fbus_build_uplink(FRSKY_FC_PHY_ID, 0x10, h_next_payload.valueId, h_next_payload.data, mine);
        CHECK(h_tx_len == nm && !memcmp(h_tx, mine, nm), "uplink differs from INAV (len %d/%d)", h_tx_len, nm);

        /* our own parser must decode the same bytes */
        fbus_parser_t p; fbus_init(&p);
        frsky_event_t ev; int got = 0;
        for (int i = 0; i < n; i++) if (fbus_feed(&p, f[i], (uint32_t)(i * byte_us), &ev)) got++;
        CHECK(got == 1 && ev.type == FR_EVT_CONTROL && ev.flags == flags && ev.rssi == rssi && !memcmp(ev.ch, ch, sizeof ch), "own parser");
    }
    printf("  100 control+poll cycles decoded by INAV, 100 uplinks identical to INAV\n");
}

static void test_fport1(void) {
    printf("INAV rx/fport.c (F.Port 1) <- rx-bridge test generator, uplink compared\n");
    rxConfig_t cfg = { .serialrx_inverted = 1, .halfDuplex = TRISTATE_OFF };
    rxRuntimeConfig_t rt = { 0 };
    CHECK(fportRxInit(&cfg, &rt), "fportRxInit");
    const double byte_us = 10e6 / 115200;
    srand(4);
    int stuffed = 0;
    for (int round = 0; round < 100; round++) {
        uint16_t ch[16];
        for (int i = 0; i < 16; i++) ch[i] = (uint16_t)(rand() & 0x7FF);
        if (round == 0) { ch[0] = 0x07E; ch[1] = 0x7D0 >> 1; }
        uint8_t rssi = (uint8_t)(round == 1 ? 0x7E : rand() % 101);
        uint8_t f[80], q[32];
        int n = fport1_build_control(ch, 0, rssi, f);
        for (int i = 0; i < n; i++) stuffed += f[i] == 0x7D;
        h_now_us += 9000;
        for (int i = 0; i < n; i++) { h_rx_cb(f[i], NULL); h_now_us += (uint32_t)byte_us; }
        uint8_t st = rt.rcFrameStatusFn(&rt);
        CHECK(st & RX_FRAME_COMPLETE, "control status %x (round %d)", st, round);
        for (int i = 0; i < 16; i++) CHECK(rt.channelData[i] == ch[i], "ch%d", i);

        int nq = fport1_build_request(FRSKY_PRIM_DATA, 0, 0, q);
        h_now_us += 300;
        for (int i = 0; i < nq; i++) { h_rx_cb(q[i], NULL); h_now_us += (uint32_t)byte_us; }
        st = rt.rcFrameStatusFn(&rt);
        h_now_us += 520;
        st = rt.rcFrameStatusFn(&rt);
        CHECK(st & RX_FRAME_PROCESSING_REQUIRED, "should answer telemetry request");
        h_next_payload.frameId = 0x10;
        h_next_payload.valueId = round == 2 ? 0x7E7D : (uint16_t)(0x0210 + round);
        h_next_payload.data = round == 3 ? 0x7D7E7D7Eu : (uint32_t)rand() * 2246822519u;
        h_tx_len = 0;
        rt.rcProcessFrameFn(&rt);
        uint8_t mine[24];
        int nm = fport1_build_uplink(0x10, h_next_payload.valueId, h_next_payload.data, mine);
        CHECK(h_tx_len == nm && !memcmp(h_tx, mine, nm), "uplink differs from INAV (len %d/%d, round %d)", h_tx_len, nm, round);

        fport1_parser_t p; fport1_init(&p);
        frsky_event_t ev; int got = 0;
        for (int i = 0; i < n; i++) if (fport1_feed(&p, f[i], (uint32_t)(i * byte_us), &ev)) got++;
        CHECK(got == 1 && ev.rssi == rssi && !memcmp(ev.ch, ch, sizeof ch), "own parser");
    }
    printf("  100 control+request cycles decoded by INAV (%d escaped bytes), 100 uplinks identical to INAV\n", stuffed);
}


/* rx-bridge engine -> INAV rx/crsf.c + telemetry/msp_shared.c -> back into rx-bridge */
static uint8_t q[4096]; static uint32_t qt[4096]; static int qn;
static void cap_fc(const uint8_t *d, int n, void *ctx) { (void)ctx; for (int i = 0; i < n && qn < 4096; i++) { q[qn] = d[i]; qt[qn++] = h_now_us; } }
static void cap_rx(const uint8_t *d, int n, void *ctx) { (void)d; (void)n; (void)ctx; }

static void test_fcid(void) {
    printf("FC-Erkennung: rx-bridge fragt per MSP ueber CRSF, INAV (msp_shared.c) antwortet\n");
    rxConfig_t cfg = { 0 };
    rxRuntimeConfig_t rt = { 0 };
    CHECK(crsfRxInit(&cfg, &rt), "crsfRxInit");
    conv_t c;
    conv_io_t io = { cap_rx, cap_fc, NULL };
    conv_init(&c, &io);
    uint16_t ch[16];
    for (int i = 0; i < 16; i++) ch[i] = 992;
    int rc_sent = 0, rc_inav = 0;
    uint32_t t0 = 1000;
    char found_at[16] = "";
    for (int k = 0; k < 600; k++) {                      /* 600 frames * 9 ms = 5.4 s */
        uint32_t t = t0 + k * 9000;
        uint8_t f[64], tf[16];
        int n = fbus_build_control(ch, 16, 0, 90, f);
        qn = 0;
        for (int i = 0; i < n; i++) { h_now_us = t + i * 22; conv_rx_byte(&c, f[i], h_now_us); conv_poll(&c, h_now_us); }
        if (k % 2 == 0) {                                 /* INAV sends telemetry: battery frame every 18 ms */
            tf[0] = 0xC8; tf[1] = 10; tf[2] = 0x08; memset(&tf[3], 0, 8); tf[11] = crsf_crc8(&tf[2], 9);
            for (int i = 0; i < 12; i++) conv_fc_byte(&c, tf[i], t + 2000 + i * 24);
        }
        /* run time forward in 50 us steps; deliver our output to INAV like a UART would */
        int qi = 0;
        for (uint32_t tt = t + n * 22; tt < t + 9000; tt += 50) {
            h_now_us = tt;
            conv_poll(&c, tt);
            while (qi < qn) {
                h_now_us = qt[qi] > tt ? qt[qi] : tt;
                crsfDataReceive(q[qi++], NULL);
                /* INAV RX task polls about every 50 us here */
                uint8_t st = crsfFrameStatus(&rt);
                if (st == RX_FRAME_COMPLETE) rc_inav++;
            }
            uint8_t reply[64];
            int rn = h_msp_task(reply);                   /* INAV telemetry task answers */
            for (int i = 0; i < rn; i++) conv_fc_byte(&c, reply[i], tt + 300 + i * 24);
        }
        rc_sent = (int)c.st.rc_forwarded;
        if (c.fc_name[0] && !found_at[0]) snprintf(found_at, sizeof found_at, "%.1f s", (k * 9000) / 1e6);
    }
    CHECK(!strcmp(c.fc_name, "INAV 8.0.1"), "fc name '%s'", c.fc_name);
    CHECK(rc_inav == rc_sent, "INAV received %d of %d RC frames while identifying", rc_inav, rc_sent);
    CHECK(c.st.fcid_requests >= 2 && c.st.fcid_requests <= 6, "requests %lu", (unsigned long)c.st.fcid_requests);
    printf("  erkannt: \"%s\" nach %s, %lu Anfragen (%d per MSP bei INAV angekommen), INAV hat alle %d Kanal-Frames erhalten\n",
           c.fc_name, found_at, (unsigned long)c.st.fcid_requests, h_msp_requests, rc_inav);
}

int main(void) {
    test_crsf();
    test_fbus();
    test_fport1();
    test_fcid();
    printf("INAV cross-check: %d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
