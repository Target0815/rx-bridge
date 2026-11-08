/* Tests of the web line protocol. With an argument it writes the JSON lines the firmware
 * produces to that file; the browser test replays them to the configuration page. */
#include <stdio.h>
#include <string.h>
#include "../../src/webcmd.h"

static int fails, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static wc_t P(const char *s) { wc_t w; wc_parse(s, &w); return w; }

int main(int argc, char **argv) {
    CHECK(P("#hello").cmd == WC_HELLO, "hello");
    CHECK(P("  #get\r").cmd == WC_GET, "get with ws/cr");
    CHECK(P("#save").cmd == WC_SAVE && P("#reboot").cmd == WC_REBOOT && P("#boot").cmd == WC_BOOT, "save/reboot/boot");
    CHECK(P("#bootloader").cmd == WC_ERR, "prefix must not match");
    CHECK(P("#json 1").cmd == WC_JSON && P("#json 1").value == 1 && P("#json 2").cmd == WC_ERR, "json");
    wc_t w = P("#set mode=1");
    CHECK(w.cmd == WC_SET && w.key == WK_MODE && w.value == 1, "set mode");
    w = P("#set hold = 20 ");
    CHECK(w.cmd == WC_SET && w.key == WK_HOLD && w.value == 20, "set hold spaces");
    CHECK(P("#set hold=0").cmd == WC_ERR && P("#set hold=101").cmd == WC_ERR, "hold range");
    CHECK(P("#set inv=1").cmd == WC_SET && P("#set inv=x").cmd == WC_ERR, "inv");
    CHECK(P("#set rcsys=0").cmd == WC_SET && P("#set rcsys=1").cmd == WC_ERR, "rcsys only FrSky");
    CHECK(P("#set mode=2").cmd == WC_ERR && P("#set foo=1").cmd == WC_ERR && P("#set mode").cmd == WC_ERR, "errors");
    CHECK(P("?").cmd == WC_NONE && P("#xyz").cmd == WC_ERR, "none/unknown");

    char b[400];
    FILE *f = argc > 1 ? fopen(argv[1], "w") : NULL;
#define OUT(fn, ...) do { int n = fn(b, sizeof b, __VA_ARGS__); CHECK(n > 0 && n < (int)sizeof b, #fn " length %d", n); if (f) fprintf(f, "%s\n", b); } while (0)
    OUT(wc_fmt_hello, "0.3.2", "waveshare_rp2040_zero", 26, 27, 28, 125);
    wc_cfg_t c0 = { 0, 0, 0, 1, 10, true };
    OUT(wc_fmt_cfg, &c0);
    wc_bridge_st_t bs = { 12, 1874, 460800, 461123, 2210, true };
    OUT(wc_fmt_bridge_st, &bs);
    wc_bridge_st_t bq = { 13, 0, 0, 0, 0, true };
    OUT(wc_fmt_bridge_st, &bq);
    OUT(wc_fmt_ack, "set", true, "Betriebsart wird nach Speichern und Neustart aktiv");
    wc_cfg_t c1 = { 1, 0, 0, 1, 10, false };
    OUT(wc_fmt_cfg, &c1);
    OUT(wc_fmt_ack, "save", true, "gespeichert");
    wc_cfg_t c2 = { 1, 1, 0, 1, 10, true };
    OUT(wc_fmt_cfg, &c2);
    wc_conv_st_t cs = { .secs = 42, .baud = 460800, .rc_per_s = 111, .tlm_per_s = 48, .up_per_s = 37, .null_per_s = 2,
                        .proto = "FBUS", .fc = "ArduPlane V4.5.7 (5e1a7b3c)", .nch = 16, .lq = 98, .rssi = 87, .ardupilot = true };
    OUT(wc_fmt_conv_st, &cs);
    wc_conv_st_t cf = cs; cf.secs = 43; cf.failsafe = true; cf.rc_per_s = 0; cf.lq = 0; cf.crc_rx = 3; cf.late = 1;
    OUT(wc_fmt_conv_st, &cf);
    wc_conv_st_t cn = { .secs = 44, .baud = 115200, .fc = "Q\"uote\\x" };
    OUT(wc_fmt_conv_st, &cn);
    OUT(wc_fmt_ack, "error", false, "hold: 1 bis 100 us");
    if (f) fclose(f);
    printf("webcmd: %d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
