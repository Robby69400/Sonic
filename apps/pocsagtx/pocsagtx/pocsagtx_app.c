/* POCSAG TX: test pages for the POCSAG receivers (POCSAG RX, POCSAG Scan,
 * Sonic's scan), sent in direct FSK by the BK4819's own FSK modem, as BEAM
 * sends its packets. Falcon app, API level 3 with the Falcon memory extension.
 *
 * - Frequency: a channel of the scan list named POCSAG (left/right), else the
 *   VFO's TX frequency. Transmission only in the 433.050-434.790 MHz band,
 *   at the lowest power: the other channels show "433 MHz ONLY".
 * - Pages: the test messages of Armel's POCSAG RX (test/pocsag.py), keys 1-5.
 * - Stream: 576 preamble bits, then batches of the sync codeword and 16
 *   codewords (BCH(31,21) + even parity), as test/pocsag.py builds them. The
 *   modem sends its own preamble and a 4-byte sync word, both 0xAA: the
 *   whole POCSAG transmission is then in the data, written to the FIFO at
 *   the pace the modem sends it.
 * Keys: left/right channel, 1-5 message, 0 auto repeat, * 1200/512 bauds,
 * PTT send, EXIT quit. */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "../app_api.h"
#include "pocsagtx_assets.h"

#define F_LO     43305000u /* 10 Hz: the 433 MHz license-free band */
#define F_HI     43479000u
#define LIST_MAX 32u
#define MAXW     48u       /* codewords without the sync words: 3 batches */
#define SYNC     0x7CD215D8u
#define IDLE     0x7A89C197u
#define PRE_DATA 52u       /* preamble bytes in the data: 576 bits with the
                              modem's 16 preamble and 4 sync bytes */
#define FIFO_PRE 32u       /* words written before the start (BEAM: 36) */
#define PA_BIAS  12u       /* lowest power: below LOW1 */
#define LNA_VHF  0x04u
#define LNA_UHF  0x08u

static const uint16_t AUTO_S[4] = {0, 10, 30, 60};

static struct {
    const app_api_t *A;
    uint16_t *ch;           /* the list's channels (app_main's stack) */
    uint32_t *w;            /* the codewords (app_main's stack) */
    uint32_t last;          /* ticks of the last send */
    uint16_t sent;
    uint8_t n, nw;          /* channels, codewords */
    uint8_t cfg[5];         /* saved: channel, message, auto, 512, (unused) */
    uint8_t prevKey, running, redraw;
    /* the byte stream */
    uint8_t pre, bi, b;
} g;
#define CI    g.cfg[0]
#define MSG   g.cfg[1]
#define AUTO  g.cfg[2]
#define B512  g.cfg[3]

void *memset(void *d, int v, size_t n)
{
    uint8_t *p = d;
    while (n--) *p++ = (uint8_t)v;
    return d;
}

__attribute__((noinline)) static char *puti(char *o, int32_t v) { return g.A->append_i32(o, v); }
static char *put(char *o, const char *s) { while (*s) *o++ = *s++; return o; }

/* ---- channels ---- */
static void loadList(void)
{
    const app_api_t *A = g.A;
    /* F4HWN (Labs 6.3.0: a 396-byte table) has no memory extension: the VFO */
    if (A->api_size < offsetof(app_api_t, memory_read) + 4u || A->falcon_memory_tag != 0x314d4346u) return;
    unsigned total = A->memory_read(0xffffu, 0);
    uint32_t base = total == 4096u ? 0x1FE000u : 0x8000u;
    unsigned list = 0;
    uint32_t nm[2];
    do {
        if (++list > 30u) return;
        A->sys_storage_read(0x8900u + (list - 1u) * 10u, nm, 6);
    } while (nm[0] != 0x53434F50u || (uint16_t)nm[1] != 0x4741u); /* "POCSAG" */
    unsigned n = 0;
    for (unsigned c = 0; c < total && n < LIST_MAX; c++) {
        uint16_t at;
        A->sys_storage_read(base + c * 2u, &at, 2);
        if ((at >> 8) == list && A->memory_read((uint16_t)c, 0)) g.ch[n++] = (uint16_t)c;
    }
    g.n = (uint8_t)n;
}
static uint32_t freq(char *name)
{
    if (!g.n) return g.A->tx_freq();
    return g.A->memory_read(g.ch[CI], name);
}

/* ---- POCSAG codewords (test/pocsag.py) ---- */
static uint32_t codeword(uint32_t info)
{
    uint32_t r = info << 10;
    for (int i = 30; i >= 10; i--)
        if (r >> i & 1u) r ^= 0x769u << (i - 10);
    uint32_t w = info << 11 | r << 1, p = w;
    p ^= p >> 16; p ^= p >> 8; p ^= p >> 4; p ^= p >> 2; p ^= p >> 1;
    return w | (p & 1u);
}
static void add(uint32_t v) { if (g.nw < MAXW) g.w[g.nw++] = v; }

static uint32_t acc;
static uint8_t nb;
static void bit(unsigned b)
{
    acc = acc << 1 | b;
    if (++nb == 20u) { add(codeword(1u << 20 | acc)); acc = nb = 0; }
}

static void build(void)
{
    const app_api_t *A = g.A;
    uint8_t rec[MSG_STRIDE], num[NUMSET_LEN];
    A->asset_read((uint16_t)(MSGS + MSG * MSG_STRIDE), rec, MSG_STRIDE);
    A->asset_read(NUMSET, num, NUMSET_LEN);
    g.nw = 0;
    const uint8_t *p = rec + 1;
    for (unsigned k = rec[0]; k--;) {
        uint32_t ric = p[0] | p[1] << 8 | (uint32_t)p[2] << 16;
        unsigned func = p[4], len = p[5];
        p += 6;
        while ((g.nw >> 1 & 7u) != (ric & 7u)) add(IDLE);   /* the RIC's frame */
        add(codeword((ric >> 3) << 2 | func));
        acc = nb = 0;
        for (; len--; p++) {
            unsigned v = *p & 127u, n = 7;
            if (!func) {                                     /* numeric: 4 bits */
                for (v = 0; v < 15u && num[v] != *p; v++) ;
                n = 4;
            }
            while (n--) { bit(v & 1u); v >>= 1; }            /* least significant first */
        }
        while (nb) bit(!func && (nb & 2u));                  /* numeric: spaces */
    }
    add(IDLE);
    while (g.nw & 15u) add(IDLE);
}

/* The stream, byte after byte: the rest of the preamble, then each batch's
 * sync codeword and 16 codewords, most significant first. */
static uint8_t nextByte(void)
{
    if (g.pre) { g.pre--; return 0xAAu; }
    unsigned o = g.bi;
    uint32_t c = o < 4u ? SYNC : g.w[g.b * 16u + (o >> 2) - 1u];
    if (++g.bi == 68u) { g.bi = 0; g.b++; }
    return (uint8_t)(c >> (24u - ((o & 3u) << 3)));
}
static uint16_t nextWord(void)
{
    unsigned first = nextByte();
    return (uint16_t)(nextByte() << 8 | first);       /* the modem sends the low byte first */
}

/* ---- transmission ---- */
static bool send(void)
{
    const app_api_t *A = g.A;
    uint32_t f = freq(0);
    if (f < F_LO || f > F_HI || A->tx_state() > 1u) return false;   /* 1: the VFO's own band */
    build();
    const unsigned bytes = PRE_DATA + (g.nw >> 4) * 68u, words = bytes >> 1;
    g.pre = PRE_DATA; g.bi = g.b = 0;

    A->tx_prepare();                                  /* PA_ENABLE still off */
    A->bk_write(0x38, (uint16_t)f);
    A->bk_write(0x39, (uint16_t)(f >> 16));
    A->bk_write(0x33, (uint16_t)((A->bk_read(0x33) & ~LNA_VHF) | LNA_UHF));
    A->bk_write(0x30, 0);
    A->bk_write(0x30, 0xC1FEu);
    A->bk_write(0x36, PA_BIAS << 8 | 0x80u | 0x22u);  /* lowest power */
    A->bk_write(0x70, 0x00E0u);                       /* tone 2 on: FSK deviation */
    A->bk_write(0x72, B512 ? 0x14A6u : 0x3065u);      /* 512 / 1200 bauds */
    A->bk_write(0x58, 0x00C1u);                       /* FSK on, TX FSK 1.2K: direct FSK */
    A->bk_write(0x5C, 0x5625u);                       /* no CRC */
    A->bk_write(0x5D, (uint16_t)(((bytes - 1u) & 255u) << 8 | ((bytes - 1u) >> 8 & 7u) << 5));
    A->bk_write(0x5A, 0xAAAAu);                       /* sync word: more preamble */
    A->bk_write(0x5B, 0xAAAAu);
    A->bk_write(0x3F, 0x8000u);                       /* TX finished interrupt */
    A->bk_write(0x59, 0x80F8u);                       /* clear the TX FIFO; 16 preamble bytes, */
    A->bk_write(0x59, 0x00F8u);                       /* 4 sync bytes */
    unsigned j = 0;
    for (; j < FIFO_PRE; j++) A->bk_write(0x5F, nextWord());
    A->tx_carrier(true);
    A->delay_ms(10);
    A->bk_write(0x59, 0x08F8u);                       /* FSK TX */
    /* Word j leaves the FIFO 10 word times (preamble and sync) plus j word
     * times after the start: write it when word j - FIFO_PRE leaves, so the
     * FIFO keeps FIFO_PRE words. */
    const uint32_t per = B512 ? 31250u : 13333u, t0 = A->ticks_ms();   /* us per word */
    uint32_t due = (10u + 1u) * per;                  /* word FIFO_PRE */
    for (uint32_t end = (words + 30u) * per + 300000u; (A->ticks_ms() - t0) * 1000u < end;) {
        if (j < words) {
            if ((A->ticks_ms() - t0) * 1000u >= due) { A->bk_write(0x5F, nextWord()); j++; due += per; }
        } else if (A->bk_read(0x0C) & 1u) {
            A->bk_write(0x02, 0);
            if (A->bk_read(0x02) & 0x8000u) break;    /* sent */
        }
    }
    A->bk_write(0x02, 0);
    A->bk_write(0x59, 0x0068u);
    A->bk_write(0x3F, 0);
    A->bk_write(0x70, 0);
    A->tx_carrier(false);
    A->bk_write(0x36, 0x0088u);
    A->tx_end();                                      /* back to RX */
    g.sent++;
    g.last = A->ticks_ms();
    return true;
}

/* ---- screen ---- */
static void draw(bool tx)
{
    const app_api_t *A = g.A;
    char s[UI_SIZE] __attribute__((aligned(4)));
    char str[34], name[12];
    char *o;
    A->asset_read(0, s, UI_SIZE);
    A->display_clear();
    A->status_clear();
    A->print_inverse(s + T_TITLE, 2, 0, true, true, (uint8_t)(2u + 4u * T_TITLE_CHARS));
    if (tx) A->print_inverse("TX", 46, 0, true, true, 54);
    if (g.n) { o = put(str, "CH "); o = puti(o, CI + 1); *o++ = '/'; o = puti(o, g.n); *o = 0;
               A->print_tiny(str, 60, 1, true, false); }
    A->draw_battery();

    o = puti(str, MSG + 1); *o++ = ' ';
    A->asset_read((uint16_t)(T_MSG + MSG * T_MSG_STRIDE), o, T_MSG_STRIDE);
    A->print_normal(str, 0, 0, 1);

    name[0] = 0;
    uint32_t f = freq(name);
    o = put(str, g.n ? name : s + T_VFO);
    *o++ = ' '; *o++ = ' ';
    o = puti(o, B512 ? 512 : 1200); o = put(o, s + T_BD);
    *o = 0;
    A->print_normal(str, 0, 0, 2);

    uint8_t *b = A->fb[0];
    for (unsigned x = 0; x < 128u; x += 2u) b[384u + x] |= 0x80u;   /* dotted line, y = 31 */
    A->draw_frequency(f, 0, 4);

    if (f < F_LO || f > F_HI) {
        A->print_inverse(s + T_ONLY, 2, 6, false, true, (uint8_t)(2u + 4u * 12u));   /* line 6: y = 49 */
        return;
    }
    o = put(str, s + T_AUTO);
    if (AUTO) { o = puti(o, AUTO_S[AUTO]); *o++ = 's'; } else o = put(o, s + T_OFF);
    o = put(o, s + T_SENT); o = puti(o, g.sent);
    if (AUTO && g.sent) {
        int32_t left = AUTO_S[AUTO] - (int32_t)((A->ticks_ms() - g.last) * 0x10625u >> 26); /* /1000 */
        o = put(o, s + T_NEXT); o = puti(o, left < 0 ? 0 : left); *o++ = 's';
    }
    *o = 0;
    A->print_tiny(str, 0, 49, false, true);
}

/* ---- keys ---- */
static void keys(void)
{
    const app_api_t *A = g.A;
    uint8_t key = A->get_key();
    if (key == APP_KEY_INVALID || key == g.prevKey) { g.prevKey = key; return; }
    g.prevKey = key;
    g.redraw = 1;
    int d = A->nav_dir(key);
    if (d && g.n) CI = d > 0 ? (CI + 1u < g.n ? CI + 1u : 0) : (CI ? CI - 1u : g.n - 1u);
    else if (key >= APP_KEY_1 && key < APP_KEY_1 + MSG_COUNT) MSG = (uint8_t)(key - APP_KEY_1);
    else if (key == APP_KEY_0) { AUTO = (uint8_t)((AUTO + 1u) & 3u); g.last = A->ticks_ms(); }
    else if (key == APP_KEY_STAR) B512 ^= 1u;
    else if (key == APP_KEY_PTT) { draw(true); A->blit_status(); A->blit_full(); send(); }
    else if (key == APP_KEY_EXIT) g.running = 0;
}

__attribute__((section(".text.entry"), used))
void app_main(const app_api_t *api)
{
    uint16_t ch[LIST_MAX];
    uint32_t w[MAXW];
    g.A = api;
    g.ch = ch;
    g.w = w;
    api->cfg_load(g.cfg, sizeof g.cfg);
    loadList();
    if (CI >= g.n) CI = 0;
    if (MSG >= MSG_COUNT) MSG = 0;
    if (AUTO > 3u) AUTO = 0;                          /* erased config: no auto repeat */
    B512 = B512 == 1u;
    api->backlight_on();
    g.running = 1;
    g.redraw = 1;
    uint32_t tick = api->ticks_ms();
    while (g.running) {
        keys();
        if (AUTO && api->ticks_ms() - g.last >= AUTO_S[AUTO] * 1000u) {
            draw(true); api->blit_status(); api->blit_full();
            if (!send()) g.last = api->ticks_ms();
            g.redraw = 1;
        }
        api->backlight_update();
        if (api->ticks_ms() - tick >= 500u) { tick = api->ticks_ms(); g.redraw = 1; api->battery_sample(); }
        if (g.redraw) {
            g.redraw = 0;
            draw(false);
            api->blit_status();
            api->blit_full();
        }
        api->delay_ms(10);
    }
    api->cfg_save(g.cfg, sizeof g.cfg);
}
