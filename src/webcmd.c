#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "webcmd.h"

static const char *skip_ws(const char *s) {
    while (*s == ' ' || *s == '\t') s++;
    return s;
}

static bool word(const char **s, const char *w) {
    size_t l = strlen(w);
    if (strncmp(*s, w, l) != 0) return false;
    char c = (*s)[l];
    if (c && c != ' ' && c != '\t' && c != '=' && c != '\r' && c != '\n') return false;
    *s += l;
    return true;
}

static bool number(const char *s, long *v) {
    s = skip_ws(s);
    char *end;
    if (!*s) return false;
    *v = strtol(s, &end, 10);
    if (end == s) return false;
    end = (char *)skip_ws(end);
    return *end == 0 || *end == '\r' || *end == '\n';
}

void wc_parse(const char *line, wc_t *o) {
    memset(o, 0, sizeof *o);
    const char *s = skip_ws(line);
    if (*s != '#') { o->cmd = WC_NONE; return; }
    s++;
    o->cmd = WC_ERR;
    o->err = "unbekannter Befehl";
    if (word(&s, "hello")) { o->cmd = WC_HELLO; return; }
    if (word(&s, "get"))   { o->cmd = WC_GET; return; }
    if (word(&s, "save"))  { o->cmd = WC_SAVE; return; }
    if (word(&s, "reboot")){ o->cmd = WC_REBOOT; return; }
    if (word(&s, "boot"))  { o->cmd = WC_BOOT; return; }
    if (word(&s, "json")) {
        if (!number(s, &o->value) || o->value < 0 || o->value > 1) { o->err = "json erwartet 0 oder 1"; return; }
        o->cmd = WC_JSON;
        return;
    }
    if (word(&s, "set")) {
        s = skip_ws(s);
        static const struct { const char *k; wc_key_t key; long lo, hi; const char *err; } keys[] = {
            { "mode",  WK_MODE,  0, 1,   "mode: 0 (Inverter) oder 1 (Konverter)" },
            { "rcsys", WK_RCSYS, 0, 0,   "rcsys: nur 0 (FrSky) verfuegbar" },
            { "inv",   WK_INV,   0, 1,   "inv: 0 oder 1" },
            { "hold",  WK_HOLD,  1, 100, "hold: 1 bis 100 us" },
        };
        for (unsigned i = 0; i < sizeof keys / sizeof keys[0]; i++) {
            const char *p = s;
            if (!word(&p, keys[i].k)) continue;
            p = skip_ws(p);
            if (*p != '=') { o->err = "Syntax: #set key=wert"; return; }
            long v;
            if (!number(p + 1, &v) || v < keys[i].lo || v > keys[i].hi) { o->err = keys[i].err; return; }
            o->cmd = WC_SET;
            o->key = keys[i].key;
            o->value = v;
            return;
        }
        o->err = "unbekannte Einstellung";
        return;
    }
}

int wc_fmt_hello(char *b, size_t n, const char *ver, const char *board, int pl, int pr, int pt, uint32_t mhz) {
    return snprintf(b, n, "{\"t\":\"hello\",\"fw\":\"rx-bridge\",\"ver\":\"%s\",\"board\":\"%s\","
                          "\"pins\":{\"line\":%d,\"rx\":%d,\"tx\":%d},\"mhz\":%lu,\"rcsys\":[\"FrSky\"]}",
                    ver, board, pl, pr, pt, (unsigned long)mhz);
}

int wc_fmt_cfg(char *b, size_t n, const wc_cfg_t *c) {
    return snprintf(b, n, "{\"t\":\"cfg\",\"mode\":%lu,\"run\":%lu,\"rcsys\":%lu,\"inv\":%lu,\"hold\":%lu,\"saved\":%s}",
                    (unsigned long)c->mode, (unsigned long)c->run_mode, (unsigned long)c->rcsys,
                    (unsigned long)c->inverted, (unsigned long)c->hold_us, c->saved ? "true" : "false");
}

int wc_fmt_ack(char *b, size_t n, const char *cmd, bool ok, const char *msg) {
    /* msg comes from the firmware itself (no quotes or backslashes) */
    return snprintf(b, n, "{\"t\":\"ack\",\"cmd\":\"%s\",\"ok\":%s,\"msg\":\"%s\"}", cmd, ok ? "true" : "false", msg ? msg : "");
}

/* JSON string literal with escaping; the FC name comes from the flight controller */
static void json_str(char *out, size_t n, const char *s) {
    size_t k = 0;
    if (n < 3) { if (n) out[0] = 0; return; }
    out[k++] = '"';
    for (; s && *s && k + 3 < n; s++) {
        unsigned char ch = (unsigned char)*s;
        if (ch == '"' || ch == '\\') { out[k++] = '\\'; out[k++] = (char)ch; }
        else if (ch >= 0x20 && ch < 0x7F) out[k++] = (char)ch;
    }
    out[k++] = '"';
    out[k] = 0;
}

int wc_fmt_conv_st(char *b, size_t n, const wc_conv_st_t *s) {
    char fc[110];
    json_str(fc, sizeof fc, s->fc ? s->fc : "");
    return snprintf(b, n,
        "{\"t\":\"st\",\"run\":1,\"s\":%lu,\"proto\":%s%s%s,\"baud\":%lu,\"nch\":%u,\"rc\":%lu,\"lq\":%u,\"rssi\":%u,"
        "\"fs\":%d,\"tlm\":%lu,\"ap\":%d,\"up\":%lu,\"null\":%lu,\"late\":%lu,\"crc_rx\":%lu,\"crc_fc\":%lu,\"ptdrop\":%lu,\"fc\":%s}",
        (unsigned long)s->secs, s->proto ? "\"" : "", s->proto ? s->proto : "null", s->proto ? "\"" : "",
        (unsigned long)s->baud, s->nch, (unsigned long)s->rc_per_s, s->lq, s->rssi, s->failsafe ? 1 : 0,
        (unsigned long)s->tlm_per_s, s->ardupilot ? 1 : 0, (unsigned long)s->up_per_s, (unsigned long)s->null_per_s,
        (unsigned long)s->late, (unsigned long)s->crc_rx, (unsigned long)s->crc_fc, (unsigned long)s->pt_drop, fc);
}

int wc_fmt_bridge_st(char *b, size_t n, const wc_bridge_st_t *s) {
    return snprintf(b, n, "{\"t\":\"st\",\"run\":0,\"s\":%lu,\"pulses\":%lu,\"baud\":%lu,\"raw\":%lu,\"edges\":%lu,\"inv\":%d}",
                    (unsigned long)s->secs, (unsigned long)s->pulses, (unsigned long)s->baud,
                    (unsigned long)s->raw_baud, (unsigned long)s->fc_edges, s->inverted ? 1 : 0);
}
