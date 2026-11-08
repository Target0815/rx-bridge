/*
 * rx-bridge – FrSky S.Port / F.Port / FBUS receivers on flight controllers without
 * UART inverter (STM32F4).
 *
 * Mode 0, transparent: bidirectional inverter running entirely in PIO hardware
 *         (step 1). The FC is configured for S.Port / F.Port / FBUS.
 * Mode 1, converter: F.Port / FBUS from the receiver is decoded and sent to the FC as
 *         CRSF; telemetry from the FC (CRSF) goes back to the receiver (step 2).
 *         The protocol core runs alone on core 1, core 0 only does USB and the LED.
 *
 * Wiring (see config.h), UART crossed as usual:
 *   GPIO26  LINE – S.Port / F.Port / FBUS wire of the receiver (through 470 Ohm)
 *   GPIO27  RX   – module input,  to TX of the flight controller
 *   GPIO28  TX   – module output, to RX of the flight controller
 */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "config.h"
#include "proto/convert.h"
#include "webcmd.h"
#include "bridge.pio.h"
#include "uart.pio.h"
#ifdef PICO_DEFAULT_WS2812_PIN
#include "ws2812.pio.h"
#endif

/* ------------------------------------------------------------------------ */
/* persistent settings                                                       */
/* ------------------------------------------------------------------------ */
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t inverted;      /* 1 = receiver line is inverted (FrSky default) */
    uint32_t hold_us;       /* transparent mode: hold time after the FC stops sending */
    uint32_t mode;          /* 0 = transparent, 1 = converter (since version 2) */
    uint32_t rcsys;         /* receiver system, 0 = FrSky (since version 3) */
    uint32_t checksum;
} settings_t;

#define SETTINGS_MAGIC   0x52584247u   /* "RXBG" */
#define SETTINGS_VERSION 3u
#define SETTINGS_OFFSET  (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

static settings_t settings;
static volatile bool core1_running;
static uint32_t running_mode;              /* mode the firmware was started in */

static uint32_t settings_sum_v1(const uint32_t *w) {   /* magic, version, inverted, hold_us */
    return w[0] ^ w[1] ^ (w[2] * 0x9E3779B9u) ^ (w[3] * 0x85EBCA6Bu) ^ 0xA5A5A5A5u;
}
static uint32_t settings_sum_v2(const uint32_t *w) {   /* + mode */
    return settings_sum_v1(w) ^ (w[4] * 0xC2B2AE35u);
}
static uint32_t settings_sum(const settings_t *s) {
    return settings_sum_v2(&s->magic) ^ (s->rcsys * 0x27D4EB2Fu);
}

static void settings_load(void) {
    const settings_t *f = (const settings_t *)(XIP_BASE + SETTINGS_OFFSET);
    const uint32_t *w = (const uint32_t *)f;
    settings.magic = SETTINGS_MAGIC;
    settings.version = SETTINGS_VERSION;
    settings.inverted = DEFAULT_INVERTED;
    settings.hold_us = DEFAULT_HOLD_US;
    settings.mode = DEFAULT_MODE;
    settings.rcsys = 0;
    if (f->magic != SETTINGS_MAGIC) return;
    if (f->version == 1 && w[4] == settings_sum_v1(w)) {          /* from 0.1.x */
        settings.inverted = w[2];
        settings.hold_us = w[3];
    } else if (f->version == 2 && w[5] == settings_sum_v2(w) && w[4] <= 1) {   /* from 0.2.x */
        settings.inverted = w[2];
        settings.hold_us = w[3];
        settings.mode = w[4];
    } else if (f->version == SETTINGS_VERSION && f->checksum == settings_sum(f) && f->mode <= 1 && f->rcsys == 0) {
        settings = *f;
    }
    if (settings.hold_us < HOLD_US_MIN || settings.hold_us > HOLD_US_MAX) settings.hold_us = DEFAULT_HOLD_US;
}

static bool settings_save(void) {
    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof page);
    settings.checksum = settings_sum(&settings);
    memcpy(page, &settings, sizeof settings);
    /* Flash access stalls code execution from flash (XIP). In converter mode core 1 is
     * parked for the few ms; the FC will see a short signal gap. */
    if (core1_running) {
        watchdog_enable(3000, true);
        multicore_lockout_start_blocking();
    }
    uint32_t irq = save_and_disable_interrupts();
    flash_range_erase(SETTINGS_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(SETTINGS_OFFSET, page, FLASH_PAGE_SIZE);
    restore_interrupts(irq);
    if (core1_running) {
        multicore_lockout_end_blocking();
        watchdog_enable(CONV_WATCHDOG_MS, true);
    }
    const settings_t *f = (const settings_t *)(XIP_BASE + SETTINGS_OFFSET);
    return memcmp(f, &settings, sizeof settings) == 0;
}

static bool settings_saved(void) {
    settings_t tmp = settings;
    tmp.checksum = settings_sum(&tmp);
    return memcmp((const void *)(XIP_BASE + SETTINGS_OFFSET), &tmp, sizeof tmp) == 0;
}

/* ------------------------------------------------------------------------ */
/* common pin setup                                                          */
/* ------------------------------------------------------------------------ */
static void line_polarity_apply(bool inverted) {
    /* PIO works in the logical UART domain (idle high). For an inverted line the GPIO
     * overrides flip input and output, and the idle level is held by a pull-down. */
    gpio_set_inover(PIN_LINE, inverted ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
    gpio_set_outover(PIN_LINE, inverted ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
    gpio_set_pulls(PIN_LINE, !inverted, inverted);
}

static void pins_init(PIO line_pio, PIO fc_pio) {
    gpio_init(PIN_RX);                      /* input from the FC's TX, idle high */
    gpio_set_dir(PIN_RX, GPIO_IN);
    gpio_pull_up(PIN_RX);
    pio_gpio_init(line_pio, PIN_LINE);
    pio_gpio_init(fc_pio, PIN_TX);
    gpio_set_drive_strength(PIN_LINE, GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_drive_strength(PIN_TX, GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_slew_rate(PIN_LINE, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_TX, GPIO_SLEW_RATE_FAST);
    line_polarity_apply(settings.inverted);
}

/* ------------------------------------------------------------------------ */
/* mode 0: transparent PIO bridge                                            */
/* ------------------------------------------------------------------------ */
static PIO const bpio = pio0;
enum { SM_MIRROR = 0, SM_TXDRV = 1, SM_PULSE = 2, SM_EDGE = 3 };
static uint off_txdrv;

static uint32_t hold_count(uint32_t hold_us) {
    return (uint32_t)((uint64_t)clock_get_hz(clk_sys) * hold_us / 2000000u);   /* 2 clocks per count */
}

static void txdrv_start(uint32_t hold_us) {
    pio_sm_set_enabled(bpio, SM_TXDRV, false);
    pio_sm_config c = fc_tx_drive_program_get_default_config(off_txdrv);
    sm_config_set_in_pins(&c, PIN_RX);
    sm_config_set_jmp_pin(&c, PIN_RX);
    sm_config_set_set_pins(&c, PIN_LINE, 1);
    sm_config_set_clkdiv(&c, 1.0f);
    pio_sm_init(bpio, SM_TXDRV, off_txdrv, &c);
    pio_sm_set_pins_with_mask(bpio, SM_TXDRV, 1u << PIN_LINE, 1u << PIN_LINE);
    pio_sm_set_pindirs_with_mask(bpio, SM_TXDRV, 0, 1u << PIN_LINE);
    pio_sm_put_blocking(bpio, SM_TXDRV, hold_count(hold_us));
    pio_sm_exec(bpio, SM_TXDRV, pio_encode_pull(false, false));
    pio_sm_exec(bpio, SM_TXDRV, pio_encode_mov(pio_y, pio_osr));
    pio_sm_set_enabled(bpio, SM_TXDRV, true);
}

static void bridge_start(void) {
    pins_init(bpio, bpio);
    uint off_mirror = pio_add_program(bpio, &line_mirror_program);
    off_txdrv = pio_add_program(bpio, &fc_tx_drive_program);
    uint off_pulse = pio_add_program(bpio, &pulse_meter_program);
    uint off_edge = pio_add_program(bpio, &edge_counter_program);
    pio_claim_sm_mask(bpio, 0xF);

    pio_sm_config c = line_mirror_program_get_default_config(off_mirror);   /* receiver -> FC */
    sm_config_set_in_pins(&c, PIN_LINE);
    sm_config_set_out_pins(&c, PIN_TX, 1);
    sm_config_set_clkdiv(&c, 1.0f);
    pio_sm_init(bpio, SM_MIRROR, off_mirror, &c);
    pio_sm_set_pins_with_mask(bpio, SM_MIRROR, 1u << PIN_TX, 1u << PIN_TX);
    pio_sm_set_consecutive_pindirs(bpio, SM_MIRROR, PIN_TX, 1, true);
    pio_sm_set_enabled(bpio, SM_MIRROR, true);

    txdrv_start(settings.hold_us);                                           /* FC -> receiver */

    c = pulse_meter_program_get_default_config(off_pulse);
    sm_config_set_in_pins(&c, PIN_LINE);
    sm_config_set_jmp_pin(&c, PIN_LINE);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_sm_init(bpio, SM_PULSE, off_pulse, &c);
    pio_sm_set_enabled(bpio, SM_PULSE, true);

    c = edge_counter_program_get_default_config(off_edge);
    sm_config_set_in_pins(&c, PIN_RX);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_sm_init(bpio, SM_EDGE, off_edge, &c);
    pio_sm_set_enabled(bpio, SM_EDGE, true);
}

typedef struct { uint32_t pulses, fc_edges, min_clk; } bstats_t;
static bstats_t bwin;
static uint32_t b_last_line_ms, b_last_fc_ms;

static void bridge_stats_poll(uint32_t now_ms, uint32_t min_valid_clk) {
    while (!pio_sm_is_rx_fifo_empty(bpio, SM_PULSE)) {
        uint32_t clk = 2u * (0xFFFFFFFFu - pio_sm_get(bpio, SM_PULSE)) + 4u;
        if (clk < min_valid_clk) continue;
        bwin.pulses++;
        if (clk < bwin.min_clk) bwin.min_clk = clk;
        b_last_line_ms = now_ms;
    }
    while (!pio_sm_is_rx_fifo_empty(bpio, SM_EDGE)) {
        (void)pio_sm_get(bpio, SM_EDGE);
        bwin.fc_edges++;
        b_last_fc_ms = now_ms;
    }
}

static const uint32_t std_bauds[] = {
    9600, 19200, 38400, 57600, 100000, 115200, 125000, 200000, 230400, 250000,
    400000, 416666, 420000, 460800, 500000, 921600, 1000000, 1870000, 2000000
};

static uint32_t baud_guess(uint32_t min_clk, uint32_t *raw) {
    if (min_clk == UINT32_MAX) { *raw = 0; return 0; }
    *raw = (uint32_t)(clock_get_hz(clk_sys) / min_clk);
    uint32_t best = 0, best_err = UINT32_MAX;
    for (unsigned i = 0; i < count_of(std_bauds); i++) {
        uint32_t b = std_bauds[i], err = *raw > b ? *raw - b : b - *raw;
        if (err < best_err) { best_err = err; best = b; }
    }
    return (best_err * 100u <= best * 6u) ? best : 0;
}

/* ------------------------------------------------------------------------ */
/* mode 1: converter, runs on core 1                                         */
/* ------------------------------------------------------------------------ */
static PIO const lpio = pio0;               /* receiver line */
static PIO const fpio = pio1;               /* flight controller (shares PIO1 with the LED) */
enum { SM_LRX = 0, SM_LTX = 1 };
enum { SM_FRX = 1, SM_FTX = 2 };
static uint off_lrx, off_ltx;

typedef struct { uint8_t b[512]; uint16_t h, t; } ring_t;
static ring_t q_line, q_fc;

static void ring_put(ring_t *r, const uint8_t *d, int n) {
    for (int i = 0; i < n; i++) {
        uint16_t nx = (uint16_t)((r->h + 1) % sizeof r->b);
        if (nx == r->t) return;                 /* full: drop the rest */
        r->b[r->h] = d[i];
        r->h = nx;
    }
}

static conv_t conv;
static uint32_t line_baud = FBUS_BAUD;
static uint32_t line_busy_until;

typedef struct {
    conv_stats_t st;
    rxproto_t proto;
    uint32_t baud;
    uint8_t lq, rssi, nch;
    bool failsafe;
    uint32_t last_rc_ms, last_fc_ms;
    uint32_t tlm_frames, pt_dropped;
    bool ardupilot;
    uint32_t fbus_crc, fport_crc, crsf_crc;
    char fc_name[48];
} cstats_t;
static volatile uint32_t cs_seq;
static cstats_t cs_shared;

static void cb_to_receiver(const uint8_t *d, int n, void *ctx) {
    (void)ctx;
    ring_put(&q_line, d, n);
    /* our own bytes come back on the receive state machine: ignore them */
    uint32_t now = time_us_32();
    uint32_t start = (int32_t)(line_busy_until - now) > 0 ? line_busy_until : now;
    line_busy_until = start + (uint32_t)((n + 2) * 10000000ull / line_baud);
}

static void cb_to_fc(const uint8_t *d, int n, void *ctx) {
    (void)ctx;
    ring_put(&q_fc, d, n);
}

static void uart_sm_init_rx(PIO pio, uint sm, uint off, uint pin, uint32_t baud) {
    pio_sm_config c = uart_rx8_program_get_default_config(off);
    sm_config_set_in_pins(&c, pin);
    sm_config_set_jmp_pin(&c, pin);
    sm_config_set_in_shift(&c, true, false, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / (8.0f * baud));
    pio_sm_init(pio, sm, off, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static void line_set_baud(uint32_t baud) {
    line_baud = baud;
    float div = (float)clock_get_hz(clk_sys) / (8.0f * baud);
    pio_sm_set_enabled(lpio, SM_LRX, false);
    pio_sm_set_enabled(lpio, SM_LTX, false);
    pio_sm_exec(lpio, SM_LTX, pio_encode_set(pio_pindirs, 0));
    pio_sm_clear_fifos(lpio, SM_LRX);
    pio_sm_clear_fifos(lpio, SM_LTX);
    pio_sm_set_clkdiv(lpio, SM_LRX, div);
    pio_sm_set_clkdiv(lpio, SM_LTX, div);
    pio_sm_restart(lpio, SM_LRX);
    pio_sm_restart(lpio, SM_LTX);
    pio_sm_exec(lpio, SM_LRX, pio_encode_jmp(off_lrx));
    pio_sm_exec(lpio, SM_LTX, pio_encode_jmp(off_ltx));
    pio_sm_clkdiv_restart(lpio, SM_LRX);
    pio_sm_clkdiv_restart(lpio, SM_LTX);
    pio_sm_set_enabled(lpio, SM_LRX, true);
    pio_sm_set_enabled(lpio, SM_LTX, true);
    q_line.h = q_line.t = 0;
}

static void converter_hw_init(void) {
    pins_init(lpio, fpio);

    off_lrx = pio_add_program(lpio, &uart_rx8_program);
    off_ltx = pio_add_program(lpio, &hd_uart_tx8_program);
    pio_sm_claim(lpio, SM_LRX);
    pio_sm_claim(lpio, SM_LTX);
    uart_sm_init_rx(lpio, SM_LRX, off_lrx, PIN_LINE, line_baud);

    pio_sm_config c = hd_uart_tx8_program_get_default_config(off_ltx);
    sm_config_set_set_pins(&c, PIN_LINE, 1);
    sm_config_set_out_pins(&c, PIN_LINE, 1);
    sm_config_set_sideset_pins(&c, PIN_LINE);
    sm_config_set_out_shift(&c, true, false, 32);
    sm_config_set_mov_status(&c, STATUS_TX_LESSTHAN, 1);
    sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / (8.0f * line_baud));
    pio_sm_init(lpio, SM_LTX, off_ltx, &c);
    pio_sm_set_pins_with_mask(lpio, SM_LTX, 1u << PIN_LINE, 1u << PIN_LINE);   /* idle level */
    pio_sm_set_pindirs_with_mask(lpio, SM_LTX, 0, 1u << PIN_LINE);            /* released */
    pio_sm_set_enabled(lpio, SM_LTX, true);

    uint off_frx = pio_add_program(fpio, &uart_rx8_program);
    uint off_ftx = pio_add_program(fpio, &uart_tx8_program);
    pio_sm_claim(fpio, SM_FRX);
    pio_sm_claim(fpio, SM_FTX);
    uart_sm_init_rx(fpio, SM_FRX, off_frx, PIN_RX, CRSF_BAUD);
    c = uart_tx8_program_get_default_config(off_ftx);
    sm_config_set_out_pins(&c, PIN_TX, 1);
    sm_config_set_sideset_pins(&c, PIN_TX);
    sm_config_set_out_shift(&c, true, false, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / (8.0f * CRSF_BAUD));
    pio_sm_init(fpio, SM_FTX, off_ftx, &c);
    pio_sm_set_pins_with_mask(fpio, SM_FTX, 1u << PIN_TX, 1u << PIN_TX);
    pio_sm_set_pindirs_with_mask(fpio, SM_FTX, 1u << PIN_TX, 1u << PIN_TX);
    pio_sm_set_enabled(fpio, SM_FTX, true);
}

static void publish(uint32_t now_us) {
    cs_seq++;
    __dmb();
    cs_shared.st = conv.st;
    cs_shared.proto = conv.proto;
    cs_shared.baud = line_baud;
    cs_shared.lq = conv.lq;
    cs_shared.rssi = conv.rssi;
    cs_shared.nch = conv.nch;
    cs_shared.failsafe = conv.failsafe;
    cs_shared.last_rc_ms = conv.last_rc_us ? conv.last_rc_us / 1000u : 0;
    cs_shared.last_fc_ms = conv.crsf.last_us / 1000u;
    cs_shared.tlm_frames = conv.tm.frames;
    cs_shared.pt_dropped = conv.tm.pt_dropped;
    cs_shared.ardupilot = conv.tm.is_ardupilot;
    cs_shared.fbus_crc = conv.fbus.crc_errors;
    cs_shared.fport_crc = conv.fport1.crc_errors;
    cs_shared.crsf_crc = conv.crsf.crc_errors;
    memcpy(cs_shared.fc_name, conv.fc_name, sizeof cs_shared.fc_name);
    cs_shared.fc_name[sizeof cs_shared.fc_name - 1] = 0;
    (void)now_us;
    __dmb();
    cs_seq++;
}

static void core1_converter(void) {
    multicore_lockout_victim_init();
    conv_io_t io = { cb_to_receiver, cb_to_fc, NULL };
    conv_init(&conv, &io);
    uint32_t baud_switch_us = time_us_32(), next_pub = 0;
    watchdog_enable(CONV_WATCHDOG_MS, true);

    for (;;) {
        uint32_t t = time_us_32();
        while (!pio_sm_is_rx_fifo_empty(lpio, SM_LRX)) {
            uint8_t b = (uint8_t)(pio_sm_get(lpio, SM_LRX) >> 24);
            if ((int32_t)(line_busy_until - t) > 0) continue;       /* echo of our own uplink */
            conv_rx_byte(&conv, b, t);
        }
        while (!pio_sm_is_rx_fifo_empty(fpio, SM_FRX))
            conv_fc_byte(&conv, (uint8_t)(pio_sm_get(fpio, SM_FRX) >> 24), t);
        conv_poll(&conv, t);

        while (q_line.t != q_line.h && !pio_sm_is_tx_fifo_full(lpio, SM_LTX)) {
            pio_sm_put(lpio, SM_LTX, q_line.b[q_line.t]);
            q_line.t = (uint16_t)((q_line.t + 1) % sizeof q_line.b);
        }
        while (q_fc.t != q_fc.h && !pio_sm_is_tx_fifo_full(fpio, SM_FTX)) {
            pio_sm_put(fpio, SM_FTX, q_fc.b[q_fc.t]);
            q_fc.t = (uint16_t)((q_fc.t + 1) % sizeof q_fc.b);
        }

        /* no receiver protocol locked: alternate between FBUS and 115200 (F.Port 1/2) */
        if (conv.proto != RXP_NONE) {
            baud_switch_us = t;
        } else if ((uint32_t)(t - baud_switch_us) > 300000u) {
            baud_switch_us = t;
            line_set_baud(line_baud == FBUS_BAUD ? FPORT_BAUD : FBUS_BAUD);
            fbus_init(&conv.fbus);
            fport1_init(&conv.fport1);
        }

        if ((int32_t)(t - next_pub) >= 0) {
            next_pub = t + 50000;
            publish(t);
        }
        watchdog_update();
    }
}

static bool cstats_get(cstats_t *out) {
    for (int tries = 0; tries < 10; tries++) {
        uint32_t s = cs_seq;
        if (s & 1) continue;
        __dmb();
        memcpy(out, (const void *)&cs_shared, sizeof *out);
        __dmb();
        if (cs_seq == s) return true;
    }
    return false;
}

/* ------------------------------------------------------------------------ */
/* status LED                                                                */
/* ------------------------------------------------------------------------ */
#ifdef PICO_DEFAULT_WS2812_PIN
static void led_init(void) {
#ifdef PICO_DEFAULT_WS2812_POWER_PIN
    gpio_init(PICO_DEFAULT_WS2812_POWER_PIN);
    gpio_set_dir(PICO_DEFAULT_WS2812_POWER_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_WS2812_POWER_PIN, 1);
#endif
    uint off = pio_add_program(pio1, &ws2812_program);
    pio_sm_claim(pio1, 0);
    pio_gpio_init(pio1, PICO_DEFAULT_WS2812_PIN);
    pio_sm_set_consecutive_pindirs(pio1, 0, PICO_DEFAULT_WS2812_PIN, 1, true);
    pio_sm_config c = ws2812_program_get_default_config(off);
    sm_config_set_sideset_pins(&c, PICO_DEFAULT_WS2812_PIN);
    sm_config_set_out_shift(&c, false, true, 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    int cycles = ws2812_T1 + ws2812_T2 + ws2812_T3;
    sm_config_set_clkdiv(&c, clock_get_hz(clk_sys) / (800000.0f * cycles));
    pio_sm_init(pio1, 0, off, &c);
    pio_sm_set_enabled(pio1, 0, true);
}
/* only blue and white: identical on RGB and GRB LEDs */
static void led_set(uint8_t white, uint8_t blue) {
    uint32_t g = white, r = white, b = (uint32_t)white + blue > 255 ? 255 : white + blue;
    if (!pio_sm_is_tx_fifo_full(pio1, 0)) pio_sm_put(pio1, 0, ((g << 16) | (r << 8) | b) << 8);
}
#elif defined(PICO_DEFAULT_LED_PIN)
static void led_init(void) { gpio_init(PICO_DEFAULT_LED_PIN); gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT); }
static void led_set(uint8_t white, uint8_t blue) { gpio_put(PICO_DEFAULT_LED_PIN, (white | blue) != 0); }
#else
static void led_init(void) {}
static void led_set(uint8_t white, uint8_t blue) { (void)white; (void)blue; }
#endif

static void led_show(uint32_t now, bool line, bool fc, bool failsafe) {
    static uint8_t lw = 0xFF, lb = 0xFF;
    static uint32_t last;
    uint8_t w, b;
    if (failsafe)  { w = 0;  b = (now % 200) < 100 ? 30 : 0; }      /* failsafe: fast blue blinking */
    else if (fc)   { w = 40; b = 0; }                              /* FC sends telemetry: white */
    else if (line) { w = 0;  b = 30; }                             /* receiver signal: blue */
    else           { w = (now % 1000) < 50 ? 12 : 0; b = 0; }      /* nothing: short blink */
    if (w != lw || b != lb || now - last >= 100) { led_set(w, b); lw = w; lb = b; last = now; }
}

/* ------------------------------------------------------------------------ */
/* USB console: single-key commands for terminals, '#' lines for the web page */
/* ------------------------------------------------------------------------ */
#ifndef RXB_BOARD
#define RXB_BOARD "unbekannt"
#endif

static const uint32_t hold_presets[] = { 2, 5, 10, 20, 50 };
static bool json_mode;
static char jbuf[400];

static const char *mode_name(uint32_t m) { return m ? "Konverter (F.Port/FBUS -> CRSF)" : "Inverter"; }

static void print_help(void) {
    printf("\n" FW_NAME " " FW_VERSION " – Befehle:\n"
           "  ?  Hilfe und Einstellungen\n"
           "  m  Betriebsart wechseln (Inverter / Konverter), speichert und startet neu\n"
           "  i  Empfaengersignal umschalten (invertiert / nicht invertiert)\n"
           "  h  Haltezeit wechseln (nur Inverter: 2/5/10/20/50 us)\n"
           "  s  Einstellungen dauerhaft speichern\n"
           "  q  Statusausgabe ein/aus\n"
           "  b  Neustart in den USB-Bootloader (Firmware-Update)\n"
           "Komfortabler: Konfigurationsseite web/index.html in Chrome oder Edge oeffnen.\n");
}

static void print_settings(void) {
    printf("Betriebsart: %s", mode_name(running_mode));
    if (settings.mode != running_mode) printf(" (nach Neustart: %s)", mode_name(settings.mode));
    printf("\nRC-System: FrSky | Pins: LEITUNG=GPIO%d  RX=GPIO%d (an TX der FC)  TX=GPIO%d (an RX der FC)\n",
           PIN_LINE, PIN_RX, PIN_TX);
    printf("Leitung %s", settings.inverted ? "invertiert (FrSky)" : "normal");
    if (!running_mode) printf(" | Haltezeit %lu us", (unsigned long)settings.hold_us);
    else printf(" | zur FC: CRSF %d Baud", CRSF_BAUD);
    printf(" | Takt %lu MHz | %s\n", (unsigned long)(clock_get_hz(clk_sys) / 1000000u),
           settings_saved() ? "gespeichert" : "NICHT gespeichert");
}

static void send_cfg(void) {
    wc_cfg_t c = { settings.mode, running_mode, settings.rcsys, settings.inverted, settings.hold_us, settings_saved() };
    wc_fmt_cfg(jbuf, sizeof jbuf, &c);
    puts(jbuf);
}

static void send_ack(const char *cmd, bool ok, const char *msg) {
    wc_fmt_ack(jbuf, sizeof jbuf, cmd, ok, msg);
    puts(jbuf);
}

static void set_hold(uint32_t us) {
    settings.hold_us = us;
    if (!running_mode) txdrv_start(us);
}

static void do_reboot(bool bootloader) {
    sleep_ms(150);                       /* let USB deliver the last message */
    /* Both reboot paths use the watchdog. In converter mode core 1 feeds it continuously,
     * which would cancel the reboot and leave core 0 waiting forever: stop core 1 first. */
    if (core1_running) {
        multicore_reset_core1();
        core1_running = false;
    }
    if (bootloader) reset_usb_boot(0, 0);
    watchdog_reboot(0, 0, 10);
    for (;;) tight_loop_contents();
}

static void handle_web(const char *line) {
    wc_t w;
    wc_parse(line, &w);
    switch (w.cmd) {
    case WC_HELLO:
        wc_fmt_hello(jbuf, sizeof jbuf, FW_VERSION, RXB_BOARD, PIN_LINE, PIN_RX, PIN_TX, clock_get_hz(clk_sys) / 1000000u);
        puts(jbuf);
        send_cfg();
        break;
    case WC_GET: send_cfg(); break;
    case WC_SET:
        switch (w.key) {
        case WK_MODE:  settings.mode = (uint32_t)w.value; break;
        case WK_RCSYS: settings.rcsys = (uint32_t)w.value; break;
        case WK_INV:   settings.inverted = (uint32_t)w.value; line_polarity_apply(settings.inverted); break;
        case WK_HOLD:  set_hold((uint32_t)w.value); break;
        }
        send_ack("set", true, settings.mode != running_mode ? "Betriebsart wird nach Speichern und Neustart aktiv" : "");
        send_cfg();
        break;
    case WC_SAVE: {
        bool ok = settings_save();
        send_ack("save", ok, ok ? (running_mode ? "gespeichert (kurze Signalpause zur FC)" : "gespeichert") : "Fehler beim Speichern");
        send_cfg();
        break;
    }
    case WC_REBOOT: send_ack("reboot", true, "Neustart"); do_reboot(false); break;
    case WC_BOOT:   send_ack("boot", true, "USB-Bootloader"); do_reboot(true); break;
    case WC_JSON:   json_mode = w.value != 0; send_ack("json", true, ""); break;
    case WC_ERR:    send_ack("error", false, w.err); break;
    default: break;
    }
}

static void handle_key(int ch, bool *status_on) {
    switch (ch) {
    case '?': print_help(); print_settings(); break;
    case 'q': *status_on = !*status_on; printf("Status %s\n", *status_on ? "ein" : "aus"); break;
    case 'i':
        settings.inverted = !settings.inverted;
        line_polarity_apply(settings.inverted);
        print_settings();
        break;
    case 'h':
        if (running_mode) { printf("Haltezeit gibt es nur im Inverter-Betrieb.\n"); break; }
        {
            unsigned i = 0;
            while (i < count_of(hold_presets) && hold_presets[i] <= settings.hold_us) i++;
            set_hold(hold_presets[i % count_of(hold_presets)]);
            print_settings();
        }
        break;
    case 'm':
        settings.mode = !running_mode;
        printf("Neue Betriebsart: %s. Speichern und Neustart ...\n", mode_name(settings.mode));
        if (!settings_save()) printf("FEHLER beim Speichern!\n");
        do_reboot(false);
        break;
    case 's':
        if (running_mode) printf("Hinweis: Speichern unterbricht das Signal zur FC fuer einige ms.\n");
        printf(settings_save() ? "Gespeichert.\n" : "FEHLER beim Speichern!\n");
        break;
    case 'b':
        printf("Bootloader ...\n");
        do_reboot(true);
        break;
    default: break;
    }
}

/* characters are single-key commands, unless a line starts with '#' */
static void console_input(int ch, bool *status_on) {
    static char line[64];
    static int len;
    static bool in_line;
    if (ch == '\r' || ch == '\n') {
        if (in_line) { line[len] = 0; handle_web(line); }
        in_line = false;
        len = 0;
        return;
    }
    if (!in_line && ch == '#') { in_line = true; len = 0; }
    if (in_line) {
        if (len < (int)sizeof line - 1) line[len++] = (char)ch;
        return;
    }
    handle_key(ch, status_on);
}

static const char *proto_name(rxproto_t p, uint32_t baud) {
    if (p == RXP_FBUS) return baud == FBUS_BAUD ? "FBUS" : "F.Port 2";
    if (p == RXP_FPORT1) return "F.Port";
    return NULL;
}

/* called every second; the deltas are always updated so the first line after a new
 * connection covers one second and not the whole time nobody was listening */
static void report_converter(uint32_t now_ms, bool print) {
    static cstats_t prev;
    cstats_t s;
    if (!cstats_get(&s)) return;
    wc_conv_st_t w = {
        .secs = now_ms / 1000, .baud = s.baud, .proto = proto_name(s.proto, s.baud), .nch = s.nch,
        .rc_per_s = s.st.rc_forwarded - prev.st.rc_forwarded, .lq = s.lq, .rssi = s.rssi, .failsafe = s.failsafe,
        .tlm_per_s = s.tlm_frames - prev.tlm_frames, .ardupilot = s.ardupilot,
        .up_per_s = s.st.uplinks - prev.st.uplinks, .null_per_s = s.st.uplinks_null - prev.st.uplinks_null,
        .late = s.st.uplinks_late - prev.st.uplinks_late,
        .crc_rx = s.fbus_crc + s.fport_crc - prev.fbus_crc - prev.fport_crc,
        .crc_fc = s.crsf_crc - prev.crsf_crc, .pt_drop = s.pt_dropped - prev.pt_dropped,
        .fc = s.fc_name,
    };
    prev = s;
    if (!print) return;
    if (json_mode) {
        wc_fmt_conv_st(jbuf, sizeof jbuf, &w);
        puts(jbuf);
        return;
    }
    if (!w.proto)
        printf("[%5lu s] Empfaenger: kein F.Port/FBUS-Signal (suche, %lu Baud)", (unsigned long)w.secs, (unsigned long)w.baud);
    else
        printf("[%5lu s] %s %lu Bd, %u Kan. | Kanaele %lu/s | LQ %u %% | RSSI %u | Failsafe %s", (unsigned long)w.secs,
               w.proto, (unsigned long)w.baud, w.nch, (unsigned long)w.rc_per_s, w.lq, w.rssi, w.failsafe ? "JA" : "nein");
    printf(" | FC %s, Telemetrie %lu/s | zum Empfaenger %lu/s, davon %lu leer\n",
           w.fc[0] ? w.fc : (w.ardupilot ? "ArduPilot" : "unbekannt"), (unsigned long)w.tlm_per_s,
           (unsigned long)w.up_per_s, (unsigned long)w.null_per_s);
    if (w.late || w.crc_rx || w.crc_fc || w.pt_drop)
        printf("          Fehler: Pruefsumme Empf. %lu, FC %lu | Antwort zu spaet %lu | Passthrough verworfen %lu\n",
               (unsigned long)w.crc_rx, (unsigned long)w.crc_fc, (unsigned long)w.late, (unsigned long)w.pt_drop);
}

static void report_bridge(uint32_t now_ms) {
    uint32_t raw, baud = baud_guess(bwin.min_clk, &raw);
    wc_bridge_st_t w = { now_ms / 1000, bwin.pulses, baud, raw, bwin.fc_edges, settings.inverted != 0 };
    memset(&bwin, 0, sizeof bwin);
    bwin.min_clk = UINT32_MAX;
    if (json_mode) {
        wc_fmt_bridge_st(jbuf, sizeof jbuf, &w);
        puts(jbuf);
        return;
    }
    if (w.pulses == 0)
        printf("[%5lu s] Leitung: kein Signal", (unsigned long)w.secs);
    else if (w.baud)
        printf("[%5lu s] Leitung: %6lu Pulse/s, ~%lu Baud", (unsigned long)w.secs, (unsigned long)w.pulses, (unsigned long)w.baud);
    else
        printf("[%5lu s] Leitung: %6lu Pulse/s, ~%lu Baud (keine Standardrate)", (unsigned long)w.secs,
               (unsigned long)w.pulses, (unsigned long)w.raw_baud);
    printf(" | FC sendet: %s (%lu Flanken/s) | %s\n", w.fc_edges ? "ja" : "nein", (unsigned long)w.fc_edges,
           w.inverted ? "inv" : "norm");
}

int main(void) {
    settings_load();
    running_mode = settings.mode;
    /* bring the signal path up first, before USB and everything else */
    if (running_mode) {
        converter_hw_init();
        core1_running = true;
        multicore_launch_core1(core1_converter);
    } else {
        bridge_start();
    }
    led_init();
    stdio_init_all();

    const uint32_t min_valid_clk = clock_get_hz(clk_sys) / 2500000u;
    bool status_on = true, was_connected = false;
    uint32_t next_report = 1000;
    bwin.min_clk = UINT32_MAX;
    b_last_line_ms = b_last_fc_ms = 0x80000000u;

    for (;;) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (running_mode) {
            cstats_t s;
            if (cstats_get(&s)) {
                uint32_t now_us_ms = time_us_32() / 1000u;
                bool line = s.proto != RXP_NONE && s.last_rc_ms && (now_us_ms - s.last_rc_ms) < 200;
                bool fc = s.last_fc_ms && (now_us_ms - s.last_fc_ms) < 60;
                led_show(now, line, fc, line && s.failsafe);
            }
        } else {
            bridge_stats_poll(now, min_valid_clk);
            led_show(now, (now - b_last_line_ms) < 200, (now - b_last_fc_ms) < 60, false);
        }

        int ch;
        while ((ch = getchar_timeout_us(0)) >= 0) console_input(ch, &status_on);

        bool connected = stdio_usb_connected();
        if (!connected && was_connected) json_mode = false;    /* next terminal gets text again */
        was_connected = connected;

        if ((int32_t)(now - next_report) >= 0) {
            next_report = now + 1000;
            if (running_mode) {
                report_converter(now, status_on && connected);
            } else if (status_on && connected) {
                report_bridge(now);
            } else {
                memset(&bwin, 0, sizeof bwin);
                bwin.min_clk = UINT32_MAX;
            }
        }
        sleep_us(200);
    }
}
