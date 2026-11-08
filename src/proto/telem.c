#include <math.h>
#include <string.h>
#include "telem.h"
#include "crsf.h"

#define STALE_MS 5000u

/* FrSky / INAV SmartPort data IDs */
#define FID_ALT       0x0100
#define FID_VARIO     0x0110
#define FID_CURR      0x0200
#define FID_VFAS      0x0210
#define FID_PITCH     0x0430
#define FID_ROLL      0x0440
#define FID_FPV       0x0450
#define FID_MODES     0x0470
#define FID_GNSS      0x0480
#define FID_FUEL      0x0600
#define FID_LATLONG   0x0800
#define FID_GPS_ALT   0x0820
#define FID_SPEED     0x0830
#define FID_HEADING   0x0840
#define FID_ASPD      0x0A00
#define FID_PT_TEXT   0x5000

void telem_init(telem_t *t) { memset(t, 0, sizeof *t); }

static uint16_t be16(const uint8_t *p) { return (uint16_t)(p[0] << 8 | p[1]); }
static uint32_t be24(const uint8_t *p) { return (uint32_t)p[0] << 16 | (uint32_t)p[1] << 8 | p[2]; }
static uint32_t be32(const uint8_t *p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }
static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }
static uint32_t le32(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

/* altitude_packed: < 0x8000: dm + 10000, >= 0x8000: metres (INAV / ArduPilot / TBS) */
int32_t crsf_baro_alt_cm(uint16_t packed) {
    if (packed & 0x8000) return (int32_t)(packed & 0x7FFF) * 100;
    return ((int32_t)packed - 10000) * 10;
}

/* vertical_speed_packed = ln(|v|/Kl + 1) / Kr, Kl = 100 cm/s, Kr = 0.026 */
int16_t crsf_vspeed_cms(int8_t packed) {
    float v = (expf(fabsf((float)packed) * 0.026f) - 1.0f) * 100.0f;
    return (int16_t)lrintf(packed < 0 ? -v : v);
}

static void pt_push(telem_t *t, uint16_t appid, uint32_t data) {
    uint8_t next = (uint8_t)((t->pt_head + 1) % TELEM_PT_QUEUE);
    if (next == t->pt_tail) {                      /* full: drop oldest */
        t->pt_tail = (uint8_t)((t->pt_tail + 1) % TELEM_PT_QUEUE);
        t->pt_dropped++;
    }
    t->pt_appid[t->pt_head] = appid;
    t->pt_data[t->pt_head] = data;
    t->pt_head = next;
}

/* status text -> 0x5000 chunks: 4 chars per chunk (first char in the MSB), severity in
 * bits 23/15/7 of the last chunk, each chunk twice (AP_Frsky_SPort_Passthrough) */
static void pt_text(telem_t *t, uint8_t severity, const char *s, int maxlen) {
    int i = 0;
    for (;;) {
        uint32_t chunk = 0;
        bool end = false;
        for (int k = 0; k < 4; k++) {
            char c = (i < maxlen) ? s[i] : 0;
            if (i < maxlen) i++;
            if (!c) { end = true; break; }
            chunk |= (uint32_t)(uint8_t)c << ((3 - k) * 8);
        }
        if (i >= maxlen) end = true;
        if (end) {
            chunk |= (uint32_t)(severity & 0x4) << 21;
            chunk |= (uint32_t)(severity & 0x2) << 14;
            chunk |= (uint32_t)(severity & 0x1) << 7;
        }
        pt_push(t, FID_PT_TEXT, chunk);
        pt_push(t, FID_PT_TEXT, chunk);
        if (end) break;
    }
}

void telem_on_crsf(telem_t *t, const uint8_t *f, int len, uint32_t now_ms) {
    if (len < 4) return;
    uint8_t type = f[2];
    const uint8_t *p = &f[3];
    int pl = f[1] - 2;
    if (pl < 0 || pl + 4 > len) return;
    if (now_ms == 0) now_ms = 1;

    switch (type) {
    case CRSF_T_BATTERY:
        if (pl < 8) return;
        t->volt_dv = be16(p);
        t->curr_da = be16(p + 2);
        t->cap_mah = be24(p + 4);
        t->remaining = p[7];
        t->t_ms[TV_BATT] = now_ms;
        break;
    case CRSF_T_GPS:
        if (pl < 15) return;
        t->lat = (int32_t)be32(p);
        t->lon = (int32_t)be32(p + 4);
        t->gspeed_dkmh = be16(p + 8);
        t->course_cdeg = be16(p + 10);
        t->alt_m_plus1000 = be16(p + 12);
        t->sats = p[14];
        t->t_ms[TV_GPS] = now_ms;
        break;
    case CRSF_T_VARIO:
        if (pl < 2) return;
        t->vario_cms = (int16_t)be16(p);
        t->t_ms[TV_VARIO] = now_ms;
        break;
    case CRSF_T_BARO_ALT:
        if (pl < 2) return;
        t->baro_alt_cm = crsf_baro_alt_cm(be16(p));
        t->t_ms[TV_BARO] = now_ms;
        if (pl >= 3) {
            t->vario_cms = crsf_vspeed_cms((int8_t)p[2]);
            t->t_ms[TV_VARIO] = now_ms;
        }
        break;
    case CRSF_T_AIRSPEED:
        if (pl < 2) return;
        t->airspeed_dkmh = be16(p);
        t->t_ms[TV_AIRSPEED] = now_ms;
        break;
    case CRSF_T_ATTITUDE:
        if (pl < 6) return;
        t->pitch = (int16_t)be16(p);
        t->roll = (int16_t)be16(p + 2);
        t->yaw = (int16_t)be16(p + 4);
        t->t_ms[TV_ATTITUDE] = now_ms;
        break;
    case CRSF_T_FLIGHT_MODE: {
        int n = pl < 16 ? pl : 16;
        memcpy(t->fmode, p, n);
        t->fmode[n] = 0;
        t->t_ms[TV_FMODE] = now_ms;
        break;
    }
    case CRSF_T_HEARTBEAT:
        t->is_ardupilot = true;
        break;
    case CRSF_T_AP_CUSTOM:
    case CRSF_T_AP_CUSTOM_OLD:
        t->is_ardupilot = true;
        if (pl < 1) return;
        if (p[0] == 0xF0 && pl >= 7) {                 /* single passthrough packet */
            pt_push(t, le16(p + 1), le32(p + 3));
        } else if (p[0] == 0xF2 && pl >= 2) {          /* multi packet */
            int n = p[1];
            for (int i = 0; i < n && 2 + 6 * i + 6 <= pl; i++)
                pt_push(t, le16(p + 2 + 6 * i), le32(p + 4 + 6 * i));
        } else if (p[0] == 0xF1 && pl >= 3) {          /* status text */
            pt_text(t, p[1], (const char *)(p + 2), pl - 2);
        }
        break;
    default:
        return;
    }
    t->frames++;
}

/* INAV flight mode text (telemetry/crsf.c) -> INAV SmartPort MODES value (smartport.c) */
static const struct { const char *s; uint32_t v; } inav_modes[] = {
    { "ACRO", 0 },        { "MANU", 40 },       { "ANGL", 10 },      { "HOR", 20 },
    { "ANGH", 2000000 },  { "AH", 210 },        { "HOLD", 400 },     { "LOTR", 800000 },
    { "CRSH", 8000 },     { "CRUZ", 8200 },     { "WP", 2000 },      { "RTH", 1000 },
    { "WRTH", 1000000 },  { "LAND", 100000 },   { "!FS!", 40000 },   { "TURT", 200000 },
    { "GEO", 400000 },    { "HRST", 0 },
};

uint32_t telem_inav_modes(const char *fmode, bool *known) {
    *known = true;
    if (!strcmp(fmode, "OK")) return 1;                         /* disarmed, arming possible */
    if (!strcmp(fmode, "WAIT") || !strcmp(fmode, "!ERR")) return 2;
    for (unsigned i = 0; i < sizeof inav_modes / sizeof inav_modes[0]; i++)
        if (!strcmp(fmode, inav_modes[i].s)) return inav_modes[i].v + 5;   /* armed */
    *known = false;
    return 0;
}

static bool fresh(const telem_t *t, int k, uint32_t now) {
    return t->t_ms[k] && (uint32_t)(now - t->t_ms[k]) < STALE_MS;
}

static int32_t r4_to_decideg(int16_t r) { return (int32_t)lrintf((float)r * (1800.0f / 31415.9265f)); }

/* standard sensor round robin; returns false if the entry has no fresh data */
static bool sensor_value(telem_t *t, int idx, uint32_t now, uint16_t *id, uint32_t *v) {
    switch (idx) {
    case 0:  if (!fresh(t, TV_BATT, now)) return false; *id = FID_VFAS; *v = t->volt_dv * 10u; return true;
    case 1:  if (!fresh(t, TV_BATT, now)) return false; *id = FID_CURR; *v = t->curr_da; return true;
    case 2:  if (!fresh(t, TV_BATT, now)) return false; *id = FID_FUEL; *v = t->remaining; return true;
    case 3:  if (!fresh(t, TV_BARO, now)) return false; *id = FID_ALT; *v = (uint32_t)t->baro_alt_cm; return true;
    case 4:  if (!fresh(t, TV_VARIO, now)) return false; *id = FID_VARIO; *v = (uint32_t)(int32_t)t->vario_cms; return true;
    case 5:  if (!fresh(t, TV_ATTITUDE, now)) return false;
             *id = FID_HEADING;
             { int32_t d = r4_to_decideg(t->yaw); if (d < 0) d += 3600; *v = (uint32_t)(d * 10); }
             return true;
    case 6:  if (!fresh(t, TV_ATTITUDE, now)) return false; *id = FID_PITCH; *v = (uint32_t)r4_to_decideg(t->pitch); return true;
    case 7:  if (!fresh(t, TV_ATTITUDE, now)) return false; *id = FID_ROLL; *v = (uint32_t)r4_to_decideg(t->roll); return true;
    case 8: {
        if (!fresh(t, TV_FMODE, now) || t->is_ardupilot) return false;
        char m[17];
        strcpy(m, t->fmode);
        size_t l = strlen(m);
        if (l && m[l - 1] == '*') m[l - 1] = 0;
        bool known;
        uint32_t mv = telem_inav_modes(m, &known);
        if (!known) return false;
        *id = FID_MODES; *v = mv; return true;
    }
    case 9:  if (!fresh(t, TV_GPS, now)) return false;
             *id = FID_GNSS;
             *v = (t->sats > 99 ? 99 : t->sats) + ((t->sats >= 4 && (t->lat || t->lon)) ? 1000u : 0u);
             return true;
    case 10:
    case 11: {
        if (!fresh(t, TV_GPS, now) || (!t->lat && !t->lon)) return false;
        *id = FID_LATLONG;
        uint32_t u;
        if (t->latlon_toggle++ & 1) {
            u = (uint32_t)(t->lon < 0 ? -(int64_t)t->lon : t->lon);
            u = ((u + u / 2) / 25) | 0x80000000u;
            if (t->lon < 0) u |= 0x40000000u;
        } else {
            u = (uint32_t)(t->lat < 0 ? -(int64_t)t->lat : t->lat);
            u = (u + u / 2) / 25;
            if (t->lat < 0) u |= 0x40000000u;
        }
        *v = u;
        return true;
    }
    case 12: if (!fresh(t, TV_GPS, now)) return false; *id = FID_GPS_ALT; *v = (uint32_t)(((int32_t)t->alt_m_plus1000 - 1000) * 100); return true;
    case 13: if (!fresh(t, TV_GPS, now)) return false; *id = FID_SPEED; *v = (uint32_t)((uint64_t)t->gspeed_dkmh * 53996u / 1000u); return true;
    case 14: if (!fresh(t, TV_GPS, now)) return false; *id = FID_FPV; *v = t->course_cdeg / 10u; return true;
    case 15: if (!fresh(t, TV_AIRSPEED, now)) return false; *id = FID_ASPD; *v = (uint32_t)((t->airspeed_dkmh * 10u + 9u) / 18.52f); return true;
    default: return false;
    }
}
#define SENSOR_COUNT 16

bool telem_next(telem_t *t, uint32_t now_ms, uint16_t *appid, uint32_t *data) {
    bool have_pt = t->pt_head != t->pt_tail;
    if (have_pt && t->prefer_pt) {
        t->prefer_pt = false;
        goto send_pt;
    }
    for (int i = 0; i < SENSOR_COUNT; i++) {
        int idx = t->next_sensor;
        t->next_sensor = (uint8_t)((t->next_sensor + 1) % SENSOR_COUNT);
        if (sensor_value(t, idx, now_ms, appid, data)) {
            t->prefer_pt = true;
            return true;
        }
    }
    if (!have_pt) return false;
send_pt:
    *appid = t->pt_appid[t->pt_tail];
    *data = t->pt_data[t->pt_tail];
    t->pt_tail = (uint8_t)((t->pt_tail + 1) % TELEM_PT_QUEUE);
    return true;
}
