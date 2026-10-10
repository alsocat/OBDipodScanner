/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Car gauges: full-screen gauges, a live data list and fault codes, sent
 * by a car OBD/CAN logger, either as a USB host (the iPod connects
 * charge-only and offers a USB serial port while this runs) or on the dock
 * connector serial pins.
 *
 * Data lines are space-separated key=value integers, one letter per value:
 *   B boost 0.1 psi   R rpm            S speed km/h      C coolant C
 *   I intake air C    T throttle 0.1%  P pedal 0.1%      L load 0.1%
 *   A timing 0.1 deg  V battery 0.01V  F fuel 0.1%       M lambda 0.001
 *   E short trim 0.1% G long trim 0.1% K catalyst C      O outside C
 * Fault codes come as a line starting "@D":
 *   @D mil=1 age=125 scan=3/15 engine:P0053:A central:02150A:S
 *   (mil -1 = not read yet; age = seconds since the last scan, -1 = none,
 *   -2 = from before the logger last started with no clock; scan only while
 *   one runs; flags A active, P pending, S stored)
 * Sends "SCAN\n" back to ask for a new fault scan.
 *
 * Wheel / << >>: change page   Select: reset peak, or scan on the codes page
 * Play: demo mode   Menu: exit
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/
#include "plugin.h"
#include "fixedpoint.h"
#include "lib/helper.h"

#define SERIAL_RATE     57600
#define STALE_TICKS     (HZ)          /* no line for this long = no data */
#define WHEEL_STEP      2             /* wheel clicks per page */
#define SAVE_FILE       PLUGIN_APPS_DATA_DIR "/boost_gauge.dat"

/* gauge arc: 240 degrees, starting lower left, clockwise */
#define ARC_START       150
#define ARC_SWEEP       240

#define FG              LCD_RGBPACK(255, 255, 255)
#define DIM             LCD_RGBPACK(90, 90, 90)
#define BG              LCD_RGBPACK(0, 0, 0)
#define RED             LCD_RGBPACK(230, 40, 30)

/* ---------- received values ---------- */

static int val[26];
static bool have[26];
static long last_line_tick = -1000 * HZ;
static bool demo = false;

#define V(k)    val[(k) - 'A']
#define HAS(k)  have[(k) - 'A']

#define MAX_CODES 16
static struct code {
    char module[12];
    char code[8];
    char flags;
} codes[MAX_CODES];
static int ncodes;
static bool codes_known;
static int mil = -1, scan_age = -1, scan_step, scan_steps;
static long codes_tick;
static long scan_asked_tick = -1000 * HZ;

/* ---------- pages ---------- */

/* every value in tenths of its display unit */
static int g_boost(void)    { return V('B'); }
static int g_rpm(void)      { return V('R') * 10; }
static int g_speed(void)    { return V('S') * 6214 / 1000; }
static int g_coolant(void)  { return V('C') * 18 + 320; }
static int g_iat(void)      { return V('I') * 18 + 320; }
static int g_battery(void)  { return V('V') / 10; }
static int g_afr(void)      { return V('M') * 147 / 1000; }
static int g_timing(void)   { return V('A'); }
static int g_throttle(void) { return V('T'); }
static int g_load(void)     { return V('L'); }
static int g_trims(void)    { return V('E') + V('G'); }
static int g_fuel(void)     { return V('F'); }
static int g_cat(void)      { return V('K') * 18 + 320; }
static int g_pedal(void)    { return V('P'); }
static int g_ambient(void)  { return V('O') * 18 + 320; }

static const struct gauge {
    const char *title, *unit;
    char key;
    int (*get)(void);
    int min, max, minor, major, red;    /* tenths; red 0 = none */
    int label_div;                      /* scale labels shown / this */
    int dec;                            /* decimals in the readout */
} gauges[] = {
    { "BOOST",       "PSI",   'B', g_boost,    -150,   250,   10,   50,     0,    1, 1 },
    { "RPM",         "RPM",   'R', g_rpm,         0, 70000, 2500, 10000, 65000, 1000, 0 },
    { "SPEED",       "MPH",   'S', g_speed,       0,  1600,   50,   200,     0,    1, 0 },
    { "COOLANT",     "F",     'C', g_coolant,  1000,  2600,  100,   400,  2300,    1, 0 },
    { "INTAKE AIR",  "F",     'I', g_iat,         0,  2000,  100,   500,  1500,    1, 0 },
    { "BATTERY",     "VOLTS", 'V', g_battery,   100,   160,    2,    10,     0,    1, 1 },
    { "AIR / FUEL",  "AFR",   'M', g_afr,       100,   200,    5,    20,     0,    1, 1 },
    { "TIMING",      "DEG",   'A', g_timing,   -100,   400,   20,   100,     0,    1, 1 },
    { "THROTTLE",    "%",     'T', g_throttle,    0,  1000,   50,   200,     0,    1, 0 },
    { "LOAD",        "%",     'L', g_load,        0,  1000,   50,   200,     0,    1, 0 },
    { "FUEL TRIMS",  "%",     'E', g_trims,    -250,   250,   20,   100,     0,    1, 1 },
    { "FUEL LEVEL",  "%",     'F', g_fuel,        0,  1000,   50,   250,     0,    1, 0 },
    { "CATALYST",    "F",     'K', g_cat,      4000, 18000, 1000,  4000, 16000,    1, 0 },
};
#define NGAUGES ((int)ARRAYLEN(gauges))
#define PAGE_LIVE  NGAUGES
#define PAGE_CODES (NGAUGES + 1)
#define NPAGES     (NGAUGES + 2)

static int page;
static int peak[NGAUGES];
static int needle[NGAUGES];

static int cx, cy, radius;

static int iabs(int v)
{
    return v < 0 ? -v : v;
}

/* tenths as "-1.5" or, with no decimals, "-2" */
static void fmt_tenths(char *s, size_t n, int v, int dec)
{
    if (dec)
        rb->snprintf(s, n, "%s%d.%d", v < 0 ? "-" : "", iabs(v) / 10, iabs(v) % 10);
    else
        rb->snprintf(s, n, "%d", (v + (v < 0 ? -5 : 5)) / 10);
}

static bool fresh(void)
{
    return TIME_BEFORE(*rb->current_tick, last_line_tick + STALE_TICKS);
}

/* ---------- input ---------- */

static void parse_codes(char *p)
{
    ncodes = 0;
    scan_steps = 0;
    while (*p) {
        while (*p == ' ')
            p++;
        char *tok = p;
        while (*p && *p != ' ')
            p++;
        if (*p)
            *p++ = 0;
        if (!rb->strncmp(tok, "mil=", 4))
            mil = rb->atoi(tok + 4);
        else if (!rb->strncmp(tok, "age=", 4))
            scan_age = rb->atoi(tok + 4);
        else if (!rb->strncmp(tok, "scan=", 5)) {
            scan_step = rb->atoi(tok + 5);
            char *slash = rb->strchr(tok, '/');
            scan_steps = slash ? rb->atoi(slash + 1) : 0;
        } else if (ncodes < MAX_CODES) {
            char *c1 = rb->strchr(tok, ':');
            char *c2 = c1 ? rb->strchr(c1 + 1, ':') : NULL;
            if (c2) {
                struct code *c = &codes[ncodes++];
                *c1 = *c2 = 0;
                rb->strlcpy(c->module, tok, sizeof(c->module));
                rb->strlcpy(c->code, c1 + 1, sizeof(c->code));
                c->flags = c2[1];
            }
        }
    }
    codes_known = true;
    codes_tick = *rb->current_tick;
}

static void parse_line(char *p)
{
    if (p[0] == '@') {
        if (p[1] == 'D')
            parse_codes(p + 2);
        return;
    }
    while (*p) {
        while (*p == ' ')
            p++;
        if (!p[0] || p[1] != '=')
            break;
        char key = p[0];
        p += 2;
        bool neg = (*p == '-');
        if (neg)
            p++;
        int v = 0;
        while (*p >= '0' && *p <= '9')
            v = v * 10 + (*p++ - '0');
        if (key >= 'A' && key <= 'Z') {
            val[key - 'A'] = neg ? -v : v;
            have[key - 'A'] = true;
        }
        while (*p && *p != ' ')
            p++;
    }
    last_line_tick = *rb->current_tick;
}

#ifdef HAVE_SERIAL_RAW_RX
static char line[512];
static int line_len;

static void read_serial(void)
{
    unsigned char buf[128];
    int n;
    while ((n = rb->serial_raw_read(buf, sizeof(buf))) > 0) {
        for (int i = 0; i < n; i++) {
            char c = buf[i];
            if (c == '\n' || c == '\r') {
                if (line_len) {
                    line[line_len] = 0;
                    parse_line(line);
                }
                line_len = 0;
            } else if (line_len < (int)sizeof(line) - 1) {
                line[line_len++] = c;
            } else {
                line_len = 0; /* garbage, wait for the next line */
            }
        }
    }
}
#endif

static void request_scan(void)
{
#ifdef HAVE_SERIAL_RAW_RX
    rb->serial_raw_write((const unsigned char *)"SCAN\n", 5);
#endif
    scan_asked_tick = *rb->current_tick;
}

/* Fake pulls through the gears so the screens can be tried without a car */
static void demo_tick(void)
{
    static int t;
    char buf[160];
    t = (t + 1) % 240;
    int gear_t = t % 80;
    int rpm = 2000 + gear_t * 60;
    int boost = gear_t < 10 ? -80 + gear_t * 20 : 200 + (gear_t % 7) * 3;
    int thr = gear_t < 70 ? 1000 : 80;
    if (t >= 230) {
        rpm = 1800;
        boost = -110;
        thr = 0;
    }
    rb->snprintf(buf, sizeof(buf),
                 "B=%d R=%d S=%d C=91 I=31 T=%d P=%d L=%d A=%d V=1412 F=640 "
                 "M=%d E=%d G=23 K=%d O=18",
                 boost, rpm, 40 + t / 3, thr, thr, thr * 8 / 10, 60 + gear_t,
                 thr > 500 ? 820 : 1000, (t % 9) - 4, 640 + t / 2);
    parse_line(buf);
    if (t % 30 == 0) {
        rb->strcpy(buf, "@D mil=1 age=420 engine:P0053:A central:02150A:S");
        parse_line(buf);
    }
}

/* ---------- drawing helpers ---------- */

static void polar(int deg, int r, int *x, int *y)
{
    deg %= 360;
    *x = cx + (r * fp14_cos(deg)) / 16384;
    *y = cy + (r * fp14_sin(deg)) / 16384;
}

static void thick_line(int x1, int y1, int x2, int y2, int w)
{
    for (int dx = -w / 2; dx <= w / 2; dx++)
        for (int dy = -w / 2; dy <= w / 2; dy++)
            rb->lcd_drawline(x1 + dx, y1 + dy, x2 + dx, y2 + dy);
}

/* seven-segment digit, a..g = bits 0..6 */
static const unsigned char seg_map[10] = {
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f
};

static void draw_segments(int x, int y, int w, int h, int t, unsigned char s)
{
    int hh = h / 2;
    if (s & 0x01) rb->lcd_fillrect(x + t, y, w - 2 * t, t);              /* a */
    if (s & 0x02) rb->lcd_fillrect(x + w - t, y + t, t, hh - t);         /* b */
    if (s & 0x04) rb->lcd_fillrect(x + w - t, y + hh, t, hh - t);        /* c */
    if (s & 0x08) rb->lcd_fillrect(x + t, y + h - t, w - 2 * t, t);      /* d */
    if (s & 0x10) rb->lcd_fillrect(x, y + hh, t, hh - t);                /* e */
    if (s & 0x20) rb->lcd_fillrect(x, y + t, t, hh - t);                 /* f */
    if (s & 0x40) rb->lcd_fillrect(x + t, y + hh - t / 2, w - 2 * t, t); /* g */
}

/* big seven-segment number centred on x */
static void draw_big_number(int x, int y, const char *s)
{
    const int w = 22, h = 38, t = 5, gap = 5, dot = 7;
    int total = 0;
    for (const char *p = s; *p; p++)
        total += (*p == '.' ? dot : w) + gap;
    x -= (total - gap) / 2;

    for (const char *p = s; *p; p++) {
        if (*p == '.') {
            rb->lcd_fillrect(x + 1, y + h - t, t, t);
            x += dot + gap;
        } else if (*p == '-') {
            draw_segments(x, y, w, h, t, 0x40);
            x += w + gap;
        } else if (*p >= '0' && *p <= '9') {
            draw_segments(x, y, w, h, t, seg_map[*p - '0']);
            x += w + gap;
        }
    }
}

static void put_centered(int x, int y, const char *s)
{
    int w, h;
    rb->lcd_getstringsize(s, &w, &h);
    rb->lcd_putsxy(x - w / 2, y - h / 2, s);
}

static void put_right(int x, int y, const char *s)
{
    int w, h;
    rb->lcd_getstringsize(s, &w, &h);
    rb->lcd_putsxy(x - w, y, s);
}

static int font_height(void)
{
    int w, h;
    rb->lcd_getstringsize("Ag", &w, &h);
    return h;
}

static void draw_header(const char *title)
{
    rb->lcd_setfont(FONT_UI);
    rb->lcd_set_foreground(FG);
    rb->lcd_putsxy(4, 2, title);
    rb->lcd_setfont(FONT_SYSFIXED);
    if (demo)
        put_right(LCD_WIDTH - 4, 2, "DEMO");
}

/* one dot per page along the bottom, the current one filled */
static void draw_page_dots(void)
{
    const int step = 9;
    int x = LCD_WIDTH / 2 - (NPAGES - 1) * step / 2;
    for (int i = 0; i < NPAGES; i++, x += step) {
        rb->lcd_set_foreground(i == page ? FG : DIM);
        if (i == page)
            rb->lcd_fillrect(x - 2, LCD_HEIGHT - 6, 5, 5);
        else
            rb->lcd_drawrect(x - 2, LCD_HEIGHT - 6, 5, 5);
    }
}

/* ---------- pages ---------- */

static int gauge_angle(const struct gauge *g, int v)
{
    if (v < g->min)
        v = g->min;
    if (v > g->max)
        v = g->max;
    return ARC_START + (long long)(v - g->min) * ARC_SWEEP / (g->max - g->min);
}

static void draw_gauge(int i)
{
    const struct gauge *g = &gauges[i];
    bool ok = fresh() && HAS(g->key);
    char buf[32];

    draw_header(g->title);
    rb->lcd_setfont(FONT_SYSFIXED);

    /* scale */
    for (int v = g->min; v <= g->max; v += g->minor) {
        int a = gauge_angle(g, v), x1, y1, x2, y2;
        bool major = ((v - g->min) % g->major == 0);
        rb->lcd_set_foreground(g->red && v >= g->red ? RED : major ? FG : DIM);
        polar(a, radius, &x1, &y1);
        polar(a, radius - (major ? 14 : 6), &x2, &y2);
        thick_line(x1, y1, x2, y2, major ? 3 : 1);
        if (major) {
            polar(a, radius - 26, &x1, &y1);
            int lab = v / 10 / g->label_div;
            rb->snprintf(buf, sizeof(buf), "%d", lab);
            put_centered(x1, y1, buf);
        }
    }

    /* peak marker */
    if (peak[i] > g->min) {
        int x1, y1, x2, y2;
        rb->lcd_set_foreground(RED);
        polar(gauge_angle(g, peak[i]), radius + 2, &x1, &y1);
        polar(gauge_angle(g, peak[i]), radius + 9, &x2, &y2);
        thick_line(x1, y1, x2, y2, 3);
    }

    /* needle */
    if (ok) {
        int x1, y1, x2, y2;
        rb->lcd_set_foreground(RED);
        polar(gauge_angle(g, needle[i]), 16, &x1, &y1);
        polar(gauge_angle(g, needle[i]), radius - 4, &x2, &y2);
        thick_line(x1, y1, x2, y2, 3);
    }
    rb->lcd_set_foreground(FG);
    rb->lcd_fillrect(cx - 6, cy - 6, 12, 12);

    /* readout */
    if (ok) {
        fmt_tenths(buf, sizeof(buf), g->get(), g->dec);
        draw_big_number(cx, cy + 16, buf);
        put_centered(cx, cy + 64, g->unit);
    } else {
        put_centered(cx, cy + 36, "NO DATA");
        put_centered(cx, cy + 50, demo ? "" : "waiting for the car");
    }

    /* peak, top right */
    if (peak[i] > g->min) {
        char p[16];
        fmt_tenths(p, sizeof(p), peak[i], g->dec);
        rb->snprintf(buf, sizeof(buf), "PEAK %s", p);
        put_right(LCD_WIDTH - 4, demo ? 12 : 2, buf);
    }
}

static void draw_live(void)
{
    static const struct { const char *label; char key; int (*get)(void);
                          const char *unit; int dec; } rows[] = {
        { "Boost",      'B', g_boost,    "psi", 1 },
        { "RPM",        'R', g_rpm,      "",    0 },
        { "Speed",      'S', g_speed,    "mph", 0 },
        { "Coolant",    'C', g_coolant,  "F",   0 },
        { "Intake air", 'I', g_iat,      "F",   0 },
        { "Outside",    'O', g_ambient,  "F",   0 },
        { "Battery",    'V', g_battery,  "V",   1 },
        { "Air/fuel",   'M', g_afr,      "",    1 },
        { "Timing",     'A', g_timing,   "deg", 1 },
        { "Throttle",   'T', g_throttle, "%",   0 },
        { "Pedal",      'P', g_pedal,    "%",   0 },
        { "Load",       'L', g_load,     "%",   0 },
        { "Fuel trims", 'E', g_trims,    "%",   1 },
        { "Fuel level", 'F', g_fuel,     "%",   0 },
        { "Catalyst",   'K', g_cat,      "F",   0 },
    };
    const int n = ARRAYLEN(rows);
    char v[16], buf[32];

    draw_header("LIVE");
    rb->lcd_setfont(FONT_UI);
    int h = font_height() + 1;
    int top = h + 8;
    int per_col = (LCD_HEIGHT - 12 - top) / h;
    if (per_col * 2 < n) {
        rb->lcd_setfont(FONT_SYSFIXED);
        h = 12;
        per_col = (LCD_HEIGHT - 12 - top) / h;
    }
    int col_w = LCD_WIDTH / 2;

    for (int i = 0; i < n; i++) {
        int col = i / per_col, row = i % per_col;
        if (col > 1)
            break;
        int x = col * col_w + 6, y = top + row * h;
        rb->lcd_set_foreground(DIM);
        rb->lcd_putsxy(x, y, rows[i].label);
        rb->lcd_set_foreground(FG);
        if (fresh() && HAS(rows[i].key)) {
            fmt_tenths(v, sizeof(v), rows[i].get(), rows[i].dec);
            rb->snprintf(buf, sizeof(buf), "%s %s", v, rows[i].unit);
        } else {
            rb->strcpy(buf, "-");
        }
        put_right(x + col_w - 14, y, buf);
    }
    rb->lcd_set_foreground(DIM);
    rb->lcd_vline(col_w, top, LCD_HEIGHT - 12);
}

static void draw_codes(void)
{
    char buf[64];
    draw_header("CODES");
    rb->lcd_setfont(FONT_UI);
    int h = font_height() + 2;
    int y = h + 8;
    bool linked = codes_known &&
                  TIME_BEFORE(*rb->current_tick, codes_tick + 5 * HZ);

    if (!linked) {
        rb->lcd_set_foreground(DIM);
        rb->lcd_putsxy(6, y, "No connection to the car");
        return;
    }

    /* status */
    rb->lcd_set_foreground(mil > 0 ? RED : FG);
    rb->lcd_putsxy(6, y, mil > 0 ? "Check engine light ON" :
                         mil == 0 ? "Check engine light off" : "Check engine light: not read yet");
    y += h;
    rb->lcd_set_foreground(DIM);
    if (scan_steps) {
        rb->snprintf(buf, sizeof(buf), "Scanning modules %d of %d...", scan_step, scan_steps);
        rb->lcd_putsxy(6, y, buf);
        y += h;
        rb->lcd_drawrect(6, y, LCD_WIDTH - 12, 8);
        rb->lcd_set_foreground(FG);
        rb->lcd_fillrect(8, y + 2, (LCD_WIDTH - 16) * scan_step / scan_steps, 4);
        y += 14;
    } else if (scan_age == -1) {
        rb->lcd_putsxy(6, y, "No scan yet");
        y += h;
    } else if (scan_age < 0) {
        rb->lcd_putsxy(6, y, "Scanned on an earlier drive");
        y += h;
    } else {
        if (scan_age < 120)
            rb->snprintf(buf, sizeof(buf), "Scanned %d s ago", scan_age);
        else if (scan_age < 7200)
            rb->snprintf(buf, sizeof(buf), "Scanned %d min ago", scan_age / 60);
        else
            rb->snprintf(buf, sizeof(buf), "Scanned %d h ago", scan_age / 3600);
        rb->lcd_putsxy(6, y, buf);
        y += h;
    }
    y += 4;

    /* the codes */
    if (scan_age != -1 && !scan_steps) {
        if (!ncodes) {
            rb->lcd_set_foreground(FG);
            rb->lcd_putsxy(6, y, "No fault codes");
        }
        for (int i = 0; i < ncodes && y + h < LCD_HEIGHT - 26; i++, y += h) {
            const struct code *c = &codes[i];
            const char *st = c->flags == 'A' ? "ACTIVE" : c->flags == 'P' ? "PENDING" : "STORED";
            rb->lcd_set_foreground(DIM);
            rb->lcd_putsxy(6, y, c->module);
            rb->lcd_set_foreground(c->flags == 'A' ? RED : FG);
            rb->lcd_putsxy(LCD_WIDTH / 2 - 30, y, c->code);
            put_right(LCD_WIDTH - 6, y, st);
        }
    }

    /* footer */
    rb->lcd_setfont(FONT_SYSFIXED);
    rb->lcd_set_foreground(DIM);
    bool asked = TIME_BEFORE(*rb->current_tick, scan_asked_tick + 10 * HZ);
    put_centered(LCD_WIDTH / 2, LCD_HEIGHT - 18,
                 scan_steps ? "" : asked ? "Scan requested..."
                                         : "SELECT: scan now");
}

static void draw(void)
{
    rb->lcd_set_background(BG);
    rb->lcd_clear_display();
    if (page < NGAUGES)
        draw_gauge(page);
    else if (page == PAGE_LIVE)
        draw_live();
    else
        draw_codes();
    draw_page_dots();
    rb->lcd_update();
}

/* ---------- main ---------- */

static void load_page(void)
{
    int fd = rb->open(SAVE_FILE, O_RDONLY);
    if (fd >= 0) {
        unsigned char c;
        if (rb->read(fd, &c, 1) == 1 && c < NPAGES)
            page = c;
        rb->close(fd);
    }
}

static void save_page(void)
{
    int fd = rb->open(SAVE_FILE, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd >= 0) {
        unsigned char c = page;
        rb->write(fd, &c, 1);
        rb->close(fd);
    }
}

static void turn_page(int d)
{
    page = (page + d + NPAGES) % NPAGES;
}

enum plugin_status plugin_start(const void *parameter)
{
    (void)parameter;

    cx = LCD_WIDTH / 2;
    cy = LCD_HEIGHT / 2 + 6;
    radius = LCD_HEIGHT / 2 - 16;
    for (int i = 0; i < NGAUGES; i++)
        peak[i] = needle[i] = gauges[i].min;
    load_page();

    backlight_ignore_timeout();
#ifdef HAVE_SERIAL_RAW_RX
    rb->serial_raw_rx(SERIAL_RATE);
#else
    demo = true;  /* no serial here (simulator) */
#endif

    enum plugin_status ret = PLUGIN_OK;
    int wheel = 0;
    bool done = false;
    while (!done) {
#ifdef HAVE_SERIAL_RAW_RX
        read_serial();
#endif
        if (demo)
            demo_tick();

        if (fresh()) {
            for (int i = 0; i < NGAUGES; i++) {
                if (!HAS(gauges[i].key))
                    continue;
                int v = gauges[i].get();
                /* ease the needle so 20 Hz updates still look smooth */
                needle[i] += (v - needle[i]) / 2;
                if (v > peak[i])
                    peak[i] = v;
            }
        }
        draw();

        int button = rb->button_get_w_tmo(HZ / 30);
        switch (button) {
            case BUTTON_SCROLL_FWD:
            case BUTTON_SCROLL_FWD | BUTTON_REPEAT:
                if (++wheel >= WHEEL_STEP) {
                    wheel = 0;
                    turn_page(1);
                }
                break;
            case BUTTON_SCROLL_BACK:
            case BUTTON_SCROLL_BACK | BUTTON_REPEAT:
                if (--wheel <= -WHEEL_STEP) {
                    wheel = 0;
                    turn_page(-1);
                }
                break;
            case BUTTON_RIGHT | BUTTON_REL:
                turn_page(1);
                break;
            case BUTTON_LEFT | BUTTON_REL:
                turn_page(-1);
                break;
            case BUTTON_MENU | BUTTON_REL:
                done = true;
                break;
            case BUTTON_SELECT | BUTTON_REL:
                if (page == PAGE_CODES)
                    request_scan();
                else if (page < NGAUGES)
                    peak[page] = gauges[page].min;
                break;
            case BUTTON_PLAY | BUTTON_REL:
                demo = !demo;
                rb->memset(have, 0, sizeof(have));
                codes_known = false;
                last_line_tick = *rb->current_tick - 1000 * HZ;
                break;
            default:
                if (rb->default_event_handler(button) == SYS_USB_CONNECTED) {
                    ret = PLUGIN_USB_CONNECTED;
                    done = true;
                }
                break;
        }
    }

#ifdef HAVE_SERIAL_RAW_RX
    rb->serial_raw_rx(0);
#endif
    save_page();
    backlight_use_settings();
    return ret;
}
