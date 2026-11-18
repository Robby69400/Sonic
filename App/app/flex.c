/* flex.c — scan FLEX résident (scanlist « FLEX »), décodage 1600 bps / 2 niveaux
 *
 * Basé sur pocsag.c. Seule la couche de décodage change :
 *   - Synchronisation sur 0xA6C6AAAA + mode code 0x870C
 *   - Désentrelacement des 88 mots de la phase
 *   - BCH(31,21) FLEX (info dans bits bas, inversion par rapport à POCSAG)
 *   - Parcours BIW → adresses → VIW → mots de message
 *   - Réassemblage multi-trames par capcode
 *
 * Non implémenté (documenté) :
 *   - 3200/6400 bps, 4 niveaux FSK
 *   - Multi-phases (B/C/D)
 *   - Adresses longues (2 mots)
 *   - Vérification K (checksum fragment)
 */

#ifdef ENABLE_FLEX

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "driver/bk4819.h"
#include "driver/bk4819-regs.h"
#include "driver/gpio.h"
#include "driver/keyboard.h"
#include "driver/py25q16.h"
#include "driver/st7565.h"
#include "driver/system.h"
#include "driver/backlight.h"
#include "ui/helper.h"
#include "ui/main.h"
#include "ui/status.h"
#include "board.h"
#include "misc.h"
#include "radio.h"
#include "settings.h"
#include "scheduler.h"
#include "helper/battery.h"
#include "py32f0xx.h"

#include "app/flex.h"

extern char gListName[MR_CHANNELS_LIST][10];

/* ============================================================
 * Constantes FLEX
 * ============================================================ */
#define FLEX_SYNC_MARKER    0xA6C6AAAAu
#define FLEX_SYNC_ERR       3u
#define FLEX_MODE_1600_2    0x870Cu
#define FLEX_PHASE_WORDS    88u
#define FLEX_PHASE_BITS     2816u   /* 88 × 32 */

#define BIAS_CODE      2048u
#define FS             9600u
#define CYC_PER_SAMPLE (48000000u / FS)
#define HOUSE_EVERY    240u

#define REG_2B      0x2B
#define REG_73      0x73
#define REG_48      0x48
#define REG_67      0x67

#define STEP_1600   8192u   /* identique à STEP_1200 POCSAG, adapté au 1600 bps */
#define DUP_1600    128u

#define MSG_MAX     254u
#define GAP         0x7Fu
#define BAD_RUN     3u

#define FLEX_MIN_MSG_LEN    10u

/* Types de page FLEX (VIW bits 4..6) */
#define FLEX_PAGE_TONE          0u
#define FLEX_PAGE_STANDARD_NUM  1u
#define FLEX_PAGE_SPECIAL_NUM   2u
#define FLEX_PAGE_ALPHA         5u
#define FLEX_PAGE_BINARY        6u

/* Réassemblage multi-trames */
#define FLEX_MAX_FRAGS  4u
#define FLEX_FRAG_TIMEOUT_SAMPLES  (FS * 2u)  /* 2 s sans fragment → abandon */

/* ============================================================
 * Stockage flash (identique POCSAG)
 * ============================================================ */
#define POCSAG_FLASH_BASE        0x012000u
#define POCSAG_FLASH_END         0x018000u
#define MAX_MSGS                 200u
#define POCSAG_META_FLASH_ADDR   (POCSAG_FLASH_BASE)
#define POCSAG_PAGES_FLASH_ADDR  (POCSAG_META_FLASH_ADDR + 16u)

#define NOTEXT_X        28
#define TITLE_X         2u
#define CAPS_END        127u
#define ROW_CHARS       18u
#define CW_COMPACT      14u
#define ROWS            6u

#define KEY_REPEAT_DELAY_MS   400u
#define KEY_REPEAT_PERIOD_MS  100u
#define KEY_DEBOUNCE_MAX_MS   120u

/* ============================================================
 * Paramètres du scan (identiques POCSAG)
 * ============================================================ */
#define FLEX_LIST_NAME      "FLEX"
#define FLEX_MAX_CH         64u
#define FLEX_SETTLE_MS      5u
#define FLEX_DWELL_MIN_MS   40u
#define FLEX_DWELL_MAX_MS   100u
#define FLEX_RSSI_SKIP_MARGIN   6u
#define FLEX_RSSI_CAL_CHANNELS  16u
#define FLEX_LOCK_GOOD      24u
#define FLEX_LOCK_RATIO     4u
#define FLEX_SYNC_WAIT_MS   1500u
#define FLEX_END_MS         100u
#define FLEX_MAX_RX_MS      2000u

#define MS_SAMPLES(ms)  ((uint32_t)(ms) * (FS / 1000u))

/* ============================================================
 * Structures
 * ============================================================ */
typedef struct {
    uint32_t ric;
    uint32_t freq;
    uint16_t offset;
    uint16_t len;
    uint8_t  flags;
    uint8_t  _pad[3];
} page_t;

#define PAGE_FLAG_FUNC(f)    ((uint8_t)((f) & 3u))
#define PAGE_FLAG_ALPHA(a)   ((uint8_t)(((a) & 1u) << 2))
#define PAGE_FLAG_BAUD(b)    ((uint8_t)(((b) & 1u) << 3))

#define PAGE_GET_FUNC(p)     ((unsigned)((p)->flags & 3u))
#define PAGE_GET_ALPHA(p)    ((unsigned)(((p)->flags >> 2) & 1u))
#define PAGE_GET_BAUD(p)     ((unsigned)(((p)->flags >> 3) & 1u))

static inline void page_set_func(page_t *p, unsigned f) {
    p->flags = (uint8_t)((p->flags & ~3u) | PAGE_FLAG_FUNC(f));
}
static inline void page_set_alpha(page_t *p, unsigned a) {
    p->flags = (uint8_t)((p->flags & ~(1u << 2)) | PAGE_FLAG_ALPHA(a));
}
static inline void page_set_baud(page_t *p, unsigned b) {
    p->flags = (uint8_t)((p->flags & ~(1u << 3)) | PAGE_FLAG_BAUD(b));
}

/* Structure sl_t adaptée FLEX */
typedef struct {
    int32_t  amp;
    int32_t  rprev, phase;
    uint32_t sr;
    uint32_t inv;
    uint32_t ric;
    uint32_t at;
    uint16_t len;
    uint8_t  num;
    uint8_t  nb;
    uint8_t  has;
    uint8_t  alpha;
    uint16_t alen;
    uint8_t  aacc, ana, ataint;
    uint8_t  acc, na;
    uint8_t  taint;
    uint8_t  badrun;
    uint16_t ok;
    uint16_t step;
    uint16_t dup;
    uint8_t  pageType;
    uint16_t expectLen;
    char     text[MSG_MAX];
    char     atext[MSG_MAX];
} sl_t;

typedef struct {
    uint32_t magic;
    uint32_t crc;
    uint16_t pool_head;
    uint16_t count;
    uint16_t head;
    uint16_t _pad;
} pocsag_hdr_t;

#define POCSAG_META_MAGIC  0x504F4354u

_Static_assert(sizeof(page_t) == 16u, "page_t doit faire 16 octets");
_Static_assert(sizeof(pocsag_hdr_t) == 16u, "hdr doit faire 16 octets");

#define POCSAG_POOL_FLASH_ADDR  (POCSAG_PAGES_FLASH_ADDR + MAX_MSGS * (uint32_t)sizeof(page_t))
#define POCSAG_POOL_SIZE        (POCSAG_FLASH_END - POCSAG_POOL_FLASH_ADDR)

_Static_assert(POCSAG_POOL_FLASH_ADDR < POCSAG_FLASH_END, "pool non vide");
_Static_assert(POCSAG_POOL_SIZE < 65536u, "offset pool uint16");

/* ============================================================
 * État global
 * ============================================================ */
static sl_t          g_sl[3];
static pocsag_hdr_t  g_hdr;
static bool          g_hdr_dirty;
static page_t        g_cur_page;
static bool          g_cur_valid;
static char          g_pool_ram[MSG_MAX];
static char          s_merged[MSG_MAX];
static char          s_old[MSG_MAX];

/* Réassemblage FLEX */
static struct {
    uint32_t capcode;
    uint16_t totalLen;
    uint16_t haveLen;
    uint8_t  type;
    uint8_t  active;
    uint32_t lastSample;
    char     text[MSG_MAX];
} s_frags[FLEX_MAX_FRAGS];

/* Machine d'état FLEX (par slicer) */
static uint32_t s_wireWords[FLEX_PHASE_WORDS];
static unsigned s_wordIdx;
static uint32_t s_syncReg;
static uint8_t  s_state;  /* 0=hunt, 1=mode, 2=fiw, 3=data */
static uint16_t s_fiwCycle, s_fiwFrame;
static uint32_t s_fiwReg;
static uint8_t  s_fiwCount;

struct {
    uint8_t  prevKey, redraw;
    uint8_t  repKey;
    uint32_t repNext;
    uint32_t invalidAt;
    bool     keyDown;
    bool     running;
    uint16_t cur;
    uint8_t  cw;
    uint8_t  refresh;
    uint8_t  top, lim;
    uint16_t nOk;
    uint8_t  led;
    uint16_t ledCnt;
    uint16_t nSync;
    uint16_t busyFor;
    uint32_t n;
    uint32_t current_freq;
    uint16_t nCh, nHits;
    uint16_t lastGood, lastBad, lastQ;
    uint16_t nWords;
    uint8_t  battOk;
    uint16_t baud;
    uint8_t  baudLocked;
    uint16_t rfCls;
    uint8_t  rfRun;
    uint8_t  rfStreak;
    uint8_t  rssiFloor;
    uint16_t rssiCalCount;
    bool     rssiFloorSet;
    uint16_t lastRssi;
    sl_t     *sl;
} g = { .cw = 0, .baud = 1600 };

volatile bool g_flexScanStop = false;

/* ============================================================
 * Helpers (identiques POCSAG)
 * ============================================================ */
static void *pocsag_memset(void *d, int c, size_t n) {
    uint8_t *p = d; while (n) p[--n] = (uint8_t)c; return d;
}
static void *pocsag_memcpy(void *d, const void *s, size_t n) {
    uint8_t *p = d; const uint8_t *q = s;
    while (n) { --n; p[n] = q[n]; } return d;
}
static unsigned ones(uint32_t v){
    unsigned n = 0;
    for(; v; v &= v - 1u) n++;
    return n;
}

/* ============================================================
 * Flash (identique POCSAG)
 * ============================================================ */
static uint32_t pocsag_hdr_crc(const pocsag_hdr_t *h)
{
    uint32_t c = 0xA5A5A5A5u;
    c ^= (uint32_t)h->pool_head << 16;
    c ^= (uint32_t)h->count;
    c ^= (uint32_t)h->head << 8;
    return c;
}
static void pocsag_hdr_flush(void)
{
    if (!g_hdr_dirty) return;
    g_hdr.magic = POCSAG_META_MAGIC;
    g_hdr.crc   = pocsag_hdr_crc(&g_hdr);
    PY25Q16_WriteBuffer(POCSAG_META_FLASH_ADDR, &g_hdr, sizeof(g_hdr), false);
    g_hdr_dirty = false;
}
static void pocsag_hdr_load(void)
{
    PY25Q16_ReadBuffer(POCSAG_META_FLASH_ADDR, &g_hdr, sizeof(g_hdr));
    if (g_hdr.magic != POCSAG_META_MAGIC || g_hdr.count > MAX_MSGS ||
        g_hdr.crc != pocsag_hdr_crc(&g_hdr)) {
        pocsag_memset(&g_hdr, 0, sizeof(g_hdr));
        g_hdr.magic = POCSAG_META_MAGIC;
        g_hdr_dirty = true;
    } else {
        g_hdr_dirty = false;
    }
    g_cur_valid = false;
}
static uint16_t pocsag_phys(unsigned k)
{
    return (uint16_t)((g_hdr.head + k) % MAX_MSGS);
}
static void pocsag_page_load(unsigned k, page_t *p)
{
    uint16_t phys = pocsag_phys(k);
    PY25Q16_ReadBuffer(POCSAG_PAGES_FLASH_ADDR + (uint32_t)phys * sizeof(page_t),
                       p, sizeof(page_t));
}
static void pocsag_page_store_phys(uint16_t phys, const page_t *p)
{
    PY25Q16_WriteBuffer(POCSAG_PAGES_FLASH_ADDR + (uint32_t)phys * sizeof(page_t),
                        p, sizeof(page_t), false);
}
static void pocsag_cur_load(void)
{
    if (g_cur_valid || !g_hdr.count) return;
    if (g.cur >= g_hdr.count) g.cur = g_hdr.count - 1u;
    pocsag_page_load(g.cur, &g_cur_page);
    g_cur_valid = true;
}
static void pocsag_pool_read(uint16_t off, char *dst, unsigned n)
{
    while (n) {
        unsigned chunk = n;
        if ((uint32_t)off + chunk > POCSAG_POOL_SIZE)
            chunk = (unsigned)(POCSAG_POOL_SIZE - off);
        PY25Q16_ReadBuffer(POCSAG_POOL_FLASH_ADDR + off, dst, chunk);
        dst += chunk; off = 0; n -= chunk;
    }
}
static void pocsag_pool_write(uint16_t off, const char *src, unsigned n)
{
    while (n) {
        unsigned chunk = n;
        if ((uint32_t)off + chunk > POCSAG_POOL_SIZE)
            chunk = (unsigned)(POCSAG_POOL_SIZE - off);
        PY25Q16_WriteBuffer(POCSAG_POOL_FLASH_ADDR + off, src, chunk, false);
        src += chunk; off = 0; n -= chunk;
    }
}
static void pocsag_flash_wipe(void)
{
    for (uint32_t a = POCSAG_FLASH_BASE; a < POCSAG_FLASH_END; a += SECTOR_SIZE)
        PY25Q16_SectorErase(a);
    pocsag_memset(&g_hdr, 0, sizeof(g_hdr));
    g_hdr.magic = POCSAG_META_MAGIC;
    g_hdr_dirty = true;
    pocsag_hdr_flush();
    g.cur = 0;
    g_cur_valid = false;
}

/* ============================================================
 * Capture audio PA4 (identique POCSAG)
 * ============================================================ */
#define APP_AFC_ADC_CH_PA4  4u
#define APP_AFC_SMP4_POS    12u
#define APP_AFC_SMP8_POS    24u
#define APP_AFC_DAC_BIAS    ((1u << 0) | (1u << 1) | (1u << 2) | (7u << 3))
#define APP_AFC_BIAS_CODE   2048u

static struct {
    uint32_t moder, rcc, dac, dhr, sqr3, smpr3;
    bool     open;
} g_afc;

static void pocsag_afc_select_pa4(void)
{
    ADC1->SMPR3 = (g_afc.smpr3 & ~(7u << APP_AFC_SMP4_POS)) |
                  (((g_afc.smpr3 >> APP_AFC_SMP8_POS) & 7u) << APP_AFC_SMP4_POS);
    ADC1->SQR3  = (g_afc.sqr3 & ~0x1Fu) | APP_AFC_ADC_CH_PA4;
}
static void pocsag_afc_select_battery(void)
{
    ADC1->SQR3  = g_afc.sqr3;
    ADC1->SMPR3 = g_afc.smpr3;
}
static void pocsag_af_capture_open(void)
{
    if (g_afc.open) return;
    g_afc.moder = GPIOA->MODER;
    g_afc.rcc   = RCC->APBENR1;
    g_afc.dac   = DAC1->CR;
    g_afc.dhr   = DAC1->DHR12R1;
    g_afc.sqr3  = ADC1->SQR3;
    g_afc.smpr3 = ADC1->SMPR3;
    g_afc.open  = true;
    GPIOA->MODER  = g_afc.moder | (3u << 8);
    RCC->APBENR1  = g_afc.rcc | RCC_APBENR1_DACEN;
    DAC1->CR      = APP_AFC_DAC_BIAS;
    DAC1->DHR12R1 = APP_AFC_BIAS_CODE;
    DAC1->SWTRIGR = 1u;
    pocsag_afc_select_pa4();
}
static uint16_t pocsag_af_capture_sample(void)
{
    ADC1->CR2 |= ADC_CR2_SWSTART | ADC_CR2_EXTTRIG;
    for (uint16_t n = 0; !(ADC1->SR & ADC_SR_EOC) && n < 2000u; n++) {}
    return (uint16_t)(ADC1->DR & 0x0FFFu);
}
static void pocsag_af_capture_close(void)
{
    if (!g_afc.open) return;
    g_afc.open = false;
    pocsag_afc_select_battery();
    DAC1->CR      = g_afc.dac;
    DAC1->DHR12R1 = g_afc.dhr;
    RCC->APBENR1  = g_afc.rcc;
    GPIOA->MODER  = g_afc.moder;
}
static bool pocsag_afc_yield_adc(void)
{
    if (!g_afc.open) return false;
    pocsag_afc_select_battery();
    return true;
}
static void pocsag_afc_resume_adc(void) { pocsag_afc_select_pa4(); }

/* ============================================================
 * Compteur 48 MHz (identique POCSAG)
 * ============================================================ */
#define APP_CYC_HZ                 48000000u
#define APP_CYC_DIAGNOSTIC_WRAP_S  10u
#define APP_CYC_DIAGNOSTIC_START \
    (0u - APP_CYC_HZ * APP_CYC_DIAGNOSTIC_WRAP_S)
static uint32_t g_cyc_base;

static uint32_t pocsag_cycles_raw(void)
{
    uint32_t tick, val;
    do {
        tick = gGlobalSysTickCounter;
        val  = SysTick->VAL;
    } while (tick != gGlobalSysTickCounter);
    if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) {
        val = SysTick->VAL;
        tick++;
    }
    return tick * (SysTick->LOAD + 1u) + (SysTick->LOAD - val);
}
static uint32_t pocsag_cycles_48mhz(void)
{
    return pocsag_cycles_raw() + g_cyc_base;
}
static inline uint32_t pocsag_ticks_ms(void)
{
    return gGlobalSysTickCounter * 10u;
}

/* ============================================================
 * RSSI et scan (identique POCSAG)
 * ============================================================ */
static uint16_t pocsag_read_rssi(void)
{
    return (uint16_t)(BK4819_ReadRegister((BK4819_REGISTER_t)REG_67) & 0x01FFu);
}
static uint32_t pocsag_channel_freq(uint16_t ch)
{
    uint32_t freq = 0;
    PY25Q16_ReadBuffer(ADRESS_CHANNELS + (uint32_t)ch * 16,
                       &freq, sizeof(freq));
    if (freq == 0xFFFFFFFFu) return 0u;
    if (freq < 1400000u || freq > 116000000u) return 0u;
    return freq;
}
static bool pocsag_name_match(const char *n, const char *name)
{
    uint8_t k = 0;
    while (name[k] != '\0') {
        if (k >= 10) return false;
        if (n[k] != name[k]) return false;
        k++;
    }
    for (uint8_t j = k; j < 10; j++) {
        char c = n[j];
        if (c != ' ' && c != '\0' && (uint8_t)c != 0xFF) return false;
    }
    return true;
}
static int8_t pocsag_find_scanlist(const char *name)
{
    for (uint8_t i = 0; i < MR_CHANNELS_LIST; i++)
        if (pocsag_name_match(gListName[i], name)) return (int8_t)i;
    return -1;
}
static bool pocsag_channel_in_list(uint16_t ch, int8_t listIdx)
{
    ChannelAttributes_t att;
    MR_LoadChannelAttributesFromFlash(ch, &att);
    return (int8_t)(att.scanlist - 1) == listIdx;
}
static uint16_t pocsag_load_channels(int8_t listIdx)
{
    uint16_t n = 0;
    for (uint16_t ch = MR_CHANNEL_FIRST; ch <= MR_CHANNEL_LAST; ch++) {
        uint32_t freq = pocsag_channel_freq(ch);
        if (freq && pocsag_channel_in_list(ch, listIdx)) n++;
    }
    return n;
}
static uint32_t pocsag_channel_freq_at(int8_t listIdx, uint16_t idx)
{
    uint16_t n = 0;
    for (uint16_t ch = MR_CHANNEL_FIRST; ch <= MR_CHANNEL_LAST; ch++) {
        uint32_t freq = pocsag_channel_freq(ch);
        if (freq && pocsag_channel_in_list(ch, listIdx)) {
            if (n == idx) return freq;
            n++;
        }
    }
    return 0;
}

/* ============================================================
 * BCH FLEX (réutilise fix() POCSAG via inversion)
 * ============================================================ */
#define GEN         0x769u

static uint32_t syndrome(uint32_t cw){
    uint32_t r = cw >> 1;
    for(unsigned i = 30; i >= 10u; i--) if(r & (1u << i)) r ^= GEN << (i - 10u);
    return r;
}
static unsigned fix(uint32_t *pcw){
    uint32_t cw = *pcw, s = syndrome(cw);
    unsigned q = 1;
    if(s){
        uint32_t t = 1;
        unsigned i = 1;
        while(t != s){
            t <<= 1;
            if(t & 0x400u) t ^= GEN;
            if(++i > 31u) return 0;
        }
        cw ^= 1u << i;
        q = 2;
    }
    if(ones(cw) & 1u){
        if(s) return 0;
        cw ^= 1u;
        q = 2;
    }
    *pcw = cw;
    return q;
}

static uint32_t flex_reverse31(uint32_t x) {
    uint32_t r = 0;
    for (unsigned i = 0; i < 31; i++) {
        r = (r << 1) | (x & 1u);
        x >>= 1;
    }
    return r;
}

/* Décode un mot FLEX de 32 bits.
 * Retourne les 21 bits d'info et le nombre d'erreurs (0,1,2) ou -1. */
static int flex_bch_decode(uint32_t cw, uint32_t *info) {
    uint32_t rev = flex_reverse31(cw & 0x7FFFFFFFu);
    unsigned q = fix(&rev);
    if (q == 0) return -1;
    *info = flex_reverse31(rev) & 0x1FFFFFu;
    return (int)(q - 1u);
}

/* ============================================================
 * Décodage FLEX
 * ============================================================ */
static const char NUMERIC[16] = {'0','1','2','3','4','5','6','7','8','9','*','U',' ','-',')','('};

/* preferAlpha : adapté POCSAG, simplifié pour FLEX (pas de func) */
static bool preferAlpha(const sl_t *m, unsigned nn){
    if(m->alen < 6u) return false;
    int score = 0, letters = 0;
    unsigned spaces = 0, words = 0, run = 0;
    for(unsigned i = 0; i < m->alen; i++){
        unsigned ch = (uint8_t)m->atext[i];
        if(ch == GAP) continue;
        if((ch | 32u) - 'a' < 26u){ score += 3; letters++; run++; continue; }
        if(run >= 2u) words++;
        run = 0;
        if(ch == ' '){ score++; spaces++; }
        else return false;
    }
    if(run >= 2u) words++;
    for(unsigned i = 0; i < nn; i++){
        unsigned ch = (uint8_t)m->text[i];
        if(ch == GAP) continue;
        if(ch - '0' < 10u) score -= 2;
        else if(ch == ' ' || ch == '-') score--;
        else score += 4;
    }
    return (unsigned)letters >= 4u && spaces >= 2u && words >= 3u &&
           (unsigned)letters * 3u >= m->alen * 2u && score >= 9;
}

/* closePage : écrit en flash (identique POCSAG, adapté FLEX) */
static void closePage(sl_t *m){
    if(!m->has) return;
    m->has = 0;
    unsigned nn = m->len;
    while(nn && m->text[nn - 1u] == ' ') nn--;
    m->alpha = (uint8_t)preferAlpha(m, nn);
    if(m->alpha){
        pocsag_memcpy(m->text, m->atext, m->alen);
        m->len = m->alen;
    }

    const uint8_t *t = (const uint8_t *)m->text;
    unsigned n = m->len, gaps = 0;
    while(n && (t[n - 1u] == GAP))
        n--;
    for(unsigned i = 0; i < n; i++) gaps += t[i] == GAP;

    if(n < FLEX_MIN_MSG_LEN) return;
    if(2u * gaps > n) return;

    g.redraw |= 2u;  /* REDRAW_FRAME */

    /* Fusion avec les 2 dernières pages (même RIC) */
    for(unsigned j = 0; j < g_hdr.count && j < 2u; j++){
        page_t q;
        pocsag_page_load(j, &q);
        if(q.ric != m->ric) continue;

        for(unsigned i = 0; i < n; i++)
            s_merged[i] = ((uint8_t)t[i] == GAP) ? '?' : (char)t[i];

        pocsag_pool_read(q.offset, s_old, n);
        for (unsigned i = 0; i < n; i++)
            if (i < q.len && (uint8_t)s_old[i] != GAP)
                s_merged[i] = s_old[i];
        pocsag_pool_write(q.offset, s_merged, n);
        if(n > q.len) q.len = (uint16_t)n;
        pocsag_page_store_phys(pocsag_phys(j), &q);
        g_hdr_dirty = true;
        pocsag_hdr_flush();
        if (j == 0) g_cur_valid = false;
        return;
    }

    /* Nouvelle page */
    unsigned msg_len = n;
    if ((uint32_t)g_hdr.pool_head + msg_len > POCSAG_POOL_SIZE)
        g_hdr.pool_head = 0;
    uint16_t current_offset = g_hdr.pool_head;

    for(unsigned i = 0; i < n; i++)
        g_pool_ram[i] = ((uint8_t)t[i] == GAP) ? '?' : (char)t[i];
    pocsag_pool_write(current_offset, g_pool_ram, n);
    g_hdr.pool_head = (uint16_t)((current_offset + msg_len) % POCSAG_POOL_SIZE);

    g_hdr.head = (uint16_t)((g_hdr.head + MAX_MSGS - 1u) % MAX_MSGS);
    if (g_hdr.count < MAX_MSGS) g_hdr.count++;

    page_t p;
    p.ric = m->ric;
    p.freq = g.current_freq;
    p.offset = current_offset;
    p.len = (uint16_t)n;
    p.flags = 0;
    p._pad[0] = p._pad[1] = p._pad[2] = 0;
    page_set_func(&p, 0);
    page_set_alpha(&p, m->alpha);
    page_set_baud(&p, 0);
    pocsag_page_store_phys(g_hdr.head, &p);

    g.nOk++;
    g.cur = 0;
    g.top = 0;
    g_cur_valid = false;
    g_hdr_dirty = true;
    pocsag_hdr_flush();
}

/* Traite une phase FLEX complète après désentrelacement et BCH */
static void flex_process_phase(sl_t *m, const uint32_t *wireWords)
{
    uint32_t words[FLEX_PHASE_WORDS];
    int      corr[FLEX_PHASE_WORDS];

    /* Désentrelacement : reconstruction des 88 mots de 32 bits.
     * Formule de référence multimon-ng :
     *   index mot  = ((c>>5) & 0xFFF8) | (c & 7)
     *   position   = (c>>3) & 31
     * où c est le compteur de bits sur le wire (0..2815). */
    pocsag_memset(words, 0, sizeof(words));
    for (unsigned c = 0; c < FLEX_PHASE_BITS; c++) {
        unsigned wi = ((c >> 5) & 0xFFF8u) | (c & 7u);
        unsigned bp = (c >> 3) & 31u;
        if (wi >= FLEX_PHASE_WORDS) continue;
        unsigned bit = (wireWords[c >> 5] >> (c & 31u)) & 1u;
        if (bit) words[wi] |= (1u << bp);
    }

    /* BCH sur chaque mot */
    for (unsigned i = 0; i < FLEX_PHASE_WORDS; i++) {
        uint32_t info = 0;
        int e = flex_bch_decode(words[i], &info);
        corr[i] = e;
        words[i] = info;
    }

    /* BIW = mot 0 */
    uint32_t biw = words[0];
    if (biw == 0 || biw == 0x1FFFFFu) return;  /* idle */

    unsigned voffset = (biw >> 10) & 0x3Fu;
    unsigned aoffset = ((biw >> 8) & 0x03u) + 1u;
    if (aoffset < 1u || voffset <= aoffset || voffset >= FLEX_PHASE_WORDS)
        return;

    /* Parcours des adresses */
    for (unsigned a = aoffset; a < voffset; a++) {
        if (corr[a] < 0) continue;
        uint32_t aw = words[a];
        if (aw == 0 || aw == 0x1FFFFFu) continue;  /* idle */

        uint32_t capcode = aw - 0x8000u;
        unsigned v = voffset + (a - aoffset);
        if (v >= FLEX_PHASE_WORDS || corr[v] < 0) continue;

        uint32_t viw = words[v];
        unsigned type = (viw >> 4) & 0x07u;
        unsigned mw1 = (viw >> 7) & 0x7Fu;
        unsigned mlen = (viw >> 14) & 0x7Fu;

        if (type == FLEX_PAGE_TONE) continue;
        if (mw1 == 0 || mlen == 0 || mw1 + mlen > FLEX_PHASE_WORDS) continue;

        /* Extraire le texte */
        char buf[MSG_MAX];
        unsigned n = 0;
        if (type == FLEX_PAGE_ALPHA) {
            uint32_t bits = 0;
            unsigned nbits = 0;
            for (unsigned w = mw1; w < mw1 + mlen; w++) {
                if (corr[w] < 0) {
                    if (n < MSG_MAX) buf[n++] = GAP;
                    continue;
                }
                bits |= (words[w] & 0x1FFFFFu) << nbits;
                nbits += 21;
                while (nbits >= 7u && n < MSG_MAX) {
                    unsigned ch = bits & 0x7Fu;
                    bits >>= 7; nbits -= 7;
                    if (ch >= 32u && ch < 127u) buf[n++] = (char)ch;
                    else buf[n++] = GAP;
                }
            }
        } else if (type == FLEX_PAGE_STANDARD_NUM || type == FLEX_PAGE_SPECIAL_NUM) {
            for (unsigned w = mw1; w < mw1 + mlen; w++) {
                if (corr[w] < 0) {
                    if (n < MSG_MAX) buf[n++] = GAP;
                    continue;
                }
                for (unsigned k = 0; k < 5u && n < MSG_MAX; k++) {
                    unsigned nib = (words[w] >> (k * 4u)) & 0xFu;
                    buf[n++] = NUMERIC[nib];
                }
            }
        } else {
            continue;
        }

        /* Réassemblage par capcode */
        int fragIdx = -1;
        for (unsigned f = 0; f < FLEX_MAX_FRAGS; f++) {
            if (s_frags[f].active && s_frags[f].capcode == capcode) {
                fragIdx = (int)f; break;
            }
        }
        if (fragIdx < 0) {
            for (unsigned f = 0; f < FLEX_MAX_FRAGS; f++) {
                if (!s_frags[f].active) { fragIdx = (int)f; break; }
            }
            if (fragIdx < 0) fragIdx = 0;
            s_frags[fragIdx].capcode = capcode;
            s_frags[fragIdx].haveLen = 0;
            s_frags[fragIdx].type = (uint8_t)type;
            s_frags[fragIdx].active = 1;
        }
        s_frags[fragIdx].lastSample = g.n;

        unsigned copy = n;
        if (s_frags[fragIdx].haveLen + copy > MSG_MAX)
            copy = MSG_MAX - s_frags[fragIdx].haveLen;
        pocsag_memcpy(s_frags[fragIdx].text + s_frags[fragIdx].haveLen, buf, copy);
        s_frags[fragIdx].haveLen += copy;

        /* Finaliser immédiatement : en FLEX 1600/2 sans info de continuation
         * explicite dans le VIW (simplification), chaque vecteur est traité
         * comme un message complet. Le réassemblage multi-trames réel
         * nécessiterait les flags C/F du VIW (non implémenté ici). */
        m->ric = capcode;
        m->len = s_frags[fragIdx].haveLen;
        pocsag_memcpy(m->text, s_frags[fragIdx].text, m->len);
        m->has = 1;
        closePage(m);
        s_frags[fragIdx].active = 0;
    }
}

/* Machine d'état bit-à-bit FLEX */
static void flex_bit(sl_t *m, uint32_t b)
{
    switch (s_state) {
    case 0:  /* hunt sync */
        s_syncReg = (s_syncReg << 1) | b;
        if (ones(s_syncReg ^ FLEX_SYNC_MARKER) <= FLEX_SYNC_ERR) {
            s_state = 1;
            s_syncReg = 0;
            m->nb = 0;
            g.nSync++;
            g.baudLocked = 1u;
        }
        break;
    case 1:  /* mode code (16 bits) */
        s_syncReg = (s_syncReg << 1) | b;
        if (++m->nb >= 16u) {
            uint16_t mode = (uint16_t)(s_syncReg & 0xFFFFu);
            if (mode == FLEX_MODE_1600_2) {
                s_state = 2;
                s_fiwReg = 0;
                s_fiwCount = 0;
            } else {
                s_state = 0;
            }
            m->nb = 0;
            s_syncReg = 0;
        }
        break;
    case 2:  /* FIW (32 bits) */
        s_fiwReg = (s_fiwReg << 1) | b;
        if (++s_fiwCount >= 32u) {
            s_fiwCycle = (uint16_t)((s_fiwReg >> 4) & 0x0Fu);
            s_fiwFrame = (uint16_t)((s_fiwReg >> 8) & 0x7Fu);
            s_state = 3;
            s_wordIdx = 0;
            pocsag_memset(s_wireWords, 0, sizeof(s_wireWords));
            m->nb = 0;
            m->sr = 0;
        }
        break;
    case 3:  /* data words (88 × 32 bits) */
        m->sr = (m->sr << 1) | b;
        if (++m->nb >= 32u) {
            s_wireWords[s_wordIdx >> 5] |= (m->sr & 1u) << (s_wordIdx & 31u);
            s_wordIdx++;
            m->nb = 0;
            m->sr = 0;
            if (s_wordIdx >= FLEX_PHASE_BITS) {
                flex_process_phase(m, s_wireWords);
                s_state = 0;
                s_wordIdx = 0;
            }
        }
        break;
    }
}

static void slice(sl_t *m, int32_t y){
    int32_t a = y < 0 ? -y : y;
    if(a > m->amp) m->amp += (a - m->amp) >> 2;
    else m->amp -= m->amp >> 10;
    int32_t t = (m->amp * m->num) >> 3;
    int32_t r = m->rprev;
    if(y > t) r = 1;
    else if(y < -t) r = -1;
    m->phase += m->step;
    if((r > 0) != (m->rprev > 0)) m->phase += (32768 + (m->step >> 1) - m->phase) >> 3;
    m->rprev = r;
    if(m->phase >= 65536){
        m->phase -= 65536;
        flex_bit(m, r <= 0);
    }
}

/* ============================================================
 * Affichage (identique POCSAG, adapté FLEX)
 * ============================================================ */
static void printFreq(char *s, uint32_t f)
{
    sprintf(s, "%u.%05u", (unsigned)(f / 100000u), (unsigned)(f % 100000u));
}

__attribute__((noinline))
static void draw(void)
{
    char s1[6]  __attribute__((aligned(4)));
    char s2[64] __attribute__((aligned(4)));
    char f[16];
    unsigned lim = 0;

    UI_DisplayClear();
    UI_StatusClear();

    pocsag_cur_load();
    const page_t *p = &g_cur_page;

    printFreq(f, g_hdr.count ? p->freq : g.current_freq);
    unsigned x = TITLE_X;
    sprintf(s1, "%u", 1600);
    unsigned n1 = (unsigned)strlen(s1);
    GUI_DisplaySmallestInverse(s1, (uint8_t)x, 0, true, true,
                               (uint8_t)(x + 4u * n1));
    x += 4u * n1 + 4u * 4u;

    sprintf(s2, "%s", f);
    GUI_DisplaySmallestInverse(s2, (uint8_t)x, 0, true, true,
                               (uint8_t)(x + 4u * (unsigned)strlen(s2)));

    {
        char r[20];
        if (g.rssiFloorSet)
            sprintf(r, "R%u/%u", g.lastRssi, (unsigned)g.rssiFloor + FLEX_RSSI_SKIP_MARGIN);
        else
            sprintf(r, "R%u/--", g.lastRssi);
        GUI_DisplaySmallestInverse(r, 80, 0, true, true, (uint8_t)(80 + 4u * (unsigned)strlen(r)));
    }

    if (g.refresh == 0 || !g.battOk) {
        bool yielded = pocsag_afc_yield_adc();
        uint16_t v, cur;
        BOARD_ADC_GetBatteryInfo(&v, &cur);
        BOARD_ADC_GetBatteryInfo(&gBatteryVoltages[gBatteryCheckCounter++ % 4],
                                 &gBatteryCurrent);
        if (yielded) pocsag_afc_resume_adc();
        BATTERY_GetReadings(false);
        g.battOk = 1;
    }
    sprintf(s1, "%u%%", (unsigned)BATTERY_VoltsToPercent(gBatteryVoltageAverage));
    GUI_DisplaySmallest(s1, 128, 0, true, true);

    if (g_hdr.count) {
        sprintf(s2, "%lu", (unsigned long)p->ric);
        UI_PrintStringSmallBold(s2, 0, 0, 0);

        unsigned capsEnd = CAPS_END;
        char *o = s2 + sprintf(s2, "%u/%u", g.cur + 1u, g_hdr.count);
        capsEnd -= 4u * (unsigned)(o - s2);
        GUI_DisplaySmallestInverse(s2, (uint8_t)capsEnd, 0, false, true, CAPS_END);

        const unsigned w       = g.cw ? (ROW_CHARS + CW_COMPACT) : ROW_CHARS;
        const unsigned maxRows = g.cw ? 8u : ROWS;
        const unsigned y0      = g.cw ? 8u : 0u;
        unsigned rows = 0;

        for (unsigned i = 0; i < p->len; i += w, rows++) {
            unsigned n = p->len - i;
            unsigned r = rows - g.top;
            if (n > w) n = w;

            unsigned rd = 0;
            while (rd < n) {
                uint16_t off = (uint16_t)((p->offset + i + rd) % POCSAG_POOL_SIZE);
                unsigned chunk = n - rd;
                if ((uint32_t)off + chunk > POCSAG_POOL_SIZE)
                    chunk = (unsigned)(POCSAG_POOL_SIZE - off);
                pocsag_pool_read(off, s2 + rd, chunk);
                rd += chunk;
            }

            for (unsigned j = 0; j < n; j++)
                if ((uint8_t)s2[j] == GAP) s2[j] = '?';
            s2[n] = '\0';

            if (r >= maxRows) continue;

            if (g.cw) {
                GUI_DisplaySmallest(s2, 0, (uint8_t)(y0 + r * 6u), false, true);
            } else {
                UI_PrintStringSmallBold(s2, 0, 0, (uint8_t)(1u + r));
            }
        }

        if (!g.cw && rows > ROWS)
            lim = rows - ROWS;
    } else {
        sprintf(s2, "SCAN %u CH", g.nCh);
        UI_PrintStringSmallBold(s2, 0, 0, 1);
        if (g.rssiFloorSet) {
            sprintf(s2, "RSSI %u  SKIP %u", g.lastRssi,
                    (unsigned)g.rssiFloor + FLEX_RSSI_SKIP_MARGIN);
            UI_PrintStringSmallBold(s2, 0, 0, 2);
        } else {
            UI_PrintStringSmallBold("CALIB...", 0, 0, 2);
        }
    }

    g.lim = (uint8_t)lim;
}

/* ============================================================
 * Clavier et housekeeping (identique POCSAG)
 * ============================================================ */
static void handleKeys(void)
{
    const uint32_t now = pocsag_ticks_ms();
    uint8_t key = KEYBOARD_GetKey();

    if (key == KEY_INVALID) {
        if (g.keyDown) {
            if (!g.invalidAt) {
                g.invalidAt = now ? now : 1u;
            } else if ((uint32_t)(now - g.invalidAt) >= KEY_DEBOUNCE_MAX_MS) {
                g.keyDown   = false;
                g.repKey    = 0;
                g.invalidAt = 0;
                g.prevKey   = KEY_INVALID;
            }
        } else {
            g.prevKey = KEY_INVALID;
        }
        return;
    }
    g.invalidAt = 0;

    const bool newPress = !g.keyDown || (key != g.prevKey);
    if (newPress) {
        g.prevKey = key;
        g.repKey  = key;
        g.keyDown = true;
        g.repNext = now + KEY_REPEAT_DELAY_MS;
    }

    bool fire = newPress;
    if (!fire && key == g.repKey && (key == KEY_UP || key == KEY_DOWN)) {
        if ((int32_t)(now - g.repNext) >= 0) {
            g.repNext = now + KEY_REPEAT_PERIOD_MS;
            fire = true;
        }
    }
    if (!fire) return;

    BACKLIGHT_TurnOn();
    g.redraw |= 1u;

    int d = (key == KEY_DOWN) - (key == KEY_UP);
    if (d) {
        unsigned top = (unsigned)(g.top + d);
        if (top <= g.lim) g.top = (uint8_t)top;
        else {
            int nc = (int)g.cur + d;
            if (nc >= 0 && (unsigned)nc < g_hdr.count) {
                g.cur = (uint16_t)nc;
                g.top = 0;
                g_cur_valid = false;
            }
        }
    }
    else if (key == KEY_EXIT) {
        g.running = false;
        g_flexScanStop = true;
    }
    else if (key == KEY_2) {
        g.cur = 0; g.nOk = 0; g.top = 0;
        g.nSync = 0; g.nHits = 0; g.nWords = 0;
        pocsag_flash_wipe();
    }
    else if (key == KEY_STAR) {
        g.cw ^= 1u;
        g.top = 0;
        g_cur_valid = false;
        g.redraw |= 1u;
    }
}

static void backlightTick(void){
    BACKLIGHT_Update();
    if (gNextTimeslice_500ms) {
        gNextTimeslice_500ms = false;
        if (gBacklightCountdown_500ms > 0) --gBacklightCountdown_500ms;
        if (gEeprom.BACKLIGHT_TIME < 61 && gBacklightCountdown_500ms == 0) BACKLIGHT_TurnOff();
    }
}

static void houseLight(void){
    handleKeys();
    backlightTick();
}

static void house(void){
    handleKeys();
    if (g_flexScanStop) return;
    if (g.redraw & 2u) BACKLIGHT_TurnOn();
    backlightTick();
    if (++g.refresh >= 100u) g.redraw |= 1u;
    if (!g.redraw) return;
    g.redraw = 0;
    g.refresh = 0;
    draw();
    ST7565_BlitStatusLine();
    ST7565_BlitFullScreen();
}

/* ============================================================
 * Écoute d'un canal
 * ============================================================ */
static void listen(uint32_t dwell_ms){
    sl_t *sl = g_sl;
    int32_t box4[4] = {0, 0, 0, 0}, sum4 = 0, dc = (int32_t)(BIAS_CODE << 4);
    unsigned bi = 0;
    pocsag_memset(sl, 0, sizeof(g_sl));
    for (unsigned k = 0; k < 3u; k++) {
        sl_t *m = &sl[k];
        m->num = (uint8_t)(2u + k);
        m->step = STEP_1600;
        m->dup  = DUP_1600;
    }
    g.sl = sl;
    uint16_t good = 0, bad = 0, q = 0;
    uint8_t run = 0;

    const uint32_t start = g.n;
    const uint16_t sync0 = g.nSync;
    uint32_t lastBatch = 0;
    bool active = false;
    uint32_t next = pocsag_cycles_48mhz();
    uint16_t cnt = 0;

    g.rfCls = 0; g.rfRun = 0; g.rfStreak = 0;
    g.baudLocked = 0;
    g.busyFor = 0;

    /* Reset machine d'état FLEX */
    s_state = 0;
    s_syncReg = 0;
    s_wordIdx = 0;

    while (!g_flexScanStop) {
        while ((int32_t)(pocsag_cycles_48mhz() - next) < 0) {}
        next += CYC_PER_SAMPLE;
        g.n++;
        int32_t adc = pocsag_af_capture_sample();
        dc += ((adc << 4) - dc) >> 8;
        int32_t x = adc - (dc >> 4);
        sum4 += x - box4[bi & 3u];
        box4[bi & 3u] = x;
        bi = (bi + 1u) & 3u;
        int32_t before = sl[0].rprev;
        for (unsigned k = 0; k < 3u; k++) slice(&sl[k], sum4);

        if (run < 255u) run++;
        if (sl[0].rprev != before) {
            const unsigned m = run & 7u, err = m <= 4u ? m : 8u - m;
            if (run >= 6u && run <= 44u && err <= 1u) good++;
            else bad++;
            if (run >= 3u && run <= 44u && (m == 3u || m == 4u || m == 5u)) q++;
            run = 0;
        }
        if (++g.ledCnt >= 960u) {
            g.ledCnt = 0;
            if (g.led) { g.led--; BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, g.led & 1u); }
        }

        bool inBatch = false;
        for (unsigned k = 0; k < 3u; k++)
            if (s_state != 0) inBatch = true;
        if (inBatch) lastBatch = g.n;

        const uint32_t elapsed = g.n - start;
        if (!active) {
            if (g.nSync != sync0 ||
                (good >= FLEX_LOCK_GOOD && bad * FLEX_LOCK_RATIO <= good)) {
                active = true;
                g.nHits++;
                lastBatch = g.n;
            } else if (elapsed >= MS_SAMPLES(dwell_ms)) {
                g.lastGood = good; g.lastBad = bad; g.lastQ = q;
                return;
            }
            continue;
        }
        if (g.nSync == sync0) {
            if (elapsed >= MS_SAMPLES(FLEX_SYNC_WAIT_MS)) break;
        } else if (!inBatch && g.n - lastBatch >= MS_SAMPLES(FLEX_END_MS)) {
            break;
        }
        if (elapsed >= MS_SAMPLES(FLEX_MAX_RX_MS)) break;

        if (cnt < HOUSE_EVERY) { cnt++; continue; }
        cnt = 0;
        houseLight();
        while ((int32_t)(pocsag_cycles_48mhz() - next) >= 0) {
            next += CYC_PER_SAMPLE;
            g.n++;
            for (unsigned k = 0; k < 3u; k++) {
                sl[k].phase += sl[k].step;
                if (sl[k].phase >= 65536) { sl[k].phase -= 65536; flex_bit(&sl[k], 0); }
            }
        }
    }
    for (unsigned k = 0; k < 3u; k++) {
        closePage(&sl[k]);
        g.nWords += sl[k].ok;
    }
}

/* ============================================================
 * Accord rapide et lancement
 * ============================================================ */
static uint16_t g_rx30;

static void pocsag_tune(uint32_t f)
{
    BK4819_SetFrequency(f);
    BK4819_PickRXFilterPathBasedOnFrequency(f);
    BK4819_WriteRegister(BK4819_REG_30, 0x200);
    BK4819_WriteRegister(BK4819_REG_30, g_rx30);
}

void APP_RunFlexScan(void)
{
    int8_t listIdx = pocsag_find_scanlist(FLEX_LIST_NAME);
    g.nCh = listIdx < 0 ? 0 : pocsag_load_channels(listIdx);
    if (!g.nCh) {
        UI_DisplayClear();
        UI_PrintStringSmallBold("NO " FLEX_LIST_NAME " LIST", 0, 0, 2);
        ST7565_BlitFullScreen();
        SYSTEM_DelayMs(1500);
        g_flexScanStop = true;
        return;
    }

    pocsag_hdr_load();

    const uint16_t r2b = BK4819_ReadRegister((BK4819_REGISTER_t)REG_2B);
    const uint16_t r73 = BK4819_ReadRegister((BK4819_REGISTER_t)REG_73);
    const uint16_t r48 = BK4819_ReadRegister((BK4819_REGISTER_t)REG_48);

    GPIO_DisableAudioPath();
    pocsag_af_capture_open();

    uint32_t first_freq = pocsag_channel_freq_at(listIdx, 0);
    BK4819_ToggleGpioOut(BK4819_GPIO0_PIN28_RX_ENABLE, true);
    BK4819_SetFrequency(first_freq);
    BK4819_PickRXFilterPathBasedOnFrequency(first_freq);
    BK4819_SetFilterBandwidth(BK4819_FILTER_BW_WIDE, false);
    BK4819_RX_TurnOn();
    g_rx30 = BK4819_ReadRegister(BK4819_REG_30);
    BK4819_WriteRegister((BK4819_REGISTER_t)REG_2B, (uint16_t)((r2b | 0x0700u) & ~0x0007u));
    BK4819_WriteRegister((BK4819_REGISTER_t)REG_73, (uint16_t)(r73 | 0x0010u));
    BK4819_SetAF(BK4819_AF_FM);
    BACKLIGHT_TurnOn();
    g_cyc_base = APP_CYC_DIAGNOSTIC_START - pocsag_cycles_raw();
    g.redraw = 1u;

    g.baud = 1600;
    g.baudLocked = 0;
    g.rssiFloor = 0;
    g.rssiCalCount = 0;
    g.rssiFloorSet = false;
    g.lastRssi = 0;
    g.repKey = 0;
    g.repNext = 0;
    g.invalidAt = 0;
    g.keyDown = false;

    uint32_t tuned = first_freq;
    SYSTEM_DelayMs(FLEX_SETTLE_MS);
    for (uint16_t i = 0; !g_flexScanStop; i = (uint16_t)((i + 1u) % g.nCh)) {
        uint32_t freq = pocsag_channel_freq_at(listIdx, i);
        if (!freq) continue;
        g.current_freq = freq;
        if (g.current_freq != tuned) {
            tuned = g.current_freq;
            pocsag_tune(tuned);
            SYSTEM_DelayMs(FLEX_SETTLE_MS);
        }

        const uint16_t rssi = pocsag_read_rssi();
        g.lastRssi = rssi;

        if (g.rssiCalCount < FLEX_RSSI_CAL_CHANNELS) {
            if (g.rssiCalCount == 0 || rssi < g.rssiFloor)
                g.rssiFloor = (uint8_t)rssi;
            g.rssiCalCount++;
            g.rssiFloorSet = true;
        }

        if (g.rssiFloorSet &&
            rssi <= (uint16_t)g.rssiFloor + FLEX_RSSI_SKIP_MARGIN) {
            g.redraw |= 1u;
            continue;
        }

        uint32_t dwell = FLEX_DWELL_MIN_MS;
        if (rssi > (uint16_t)g.rssiFloor + 2u * FLEX_RSSI_SKIP_MARGIN)
            dwell = FLEX_DWELL_MAX_MS;

        const uint16_t before = g.nOk;
        listen(dwell);
        if (g.nOk != before) {
            g.cur = 0; g.top = 0;
            g_cur_valid = false;
            g.led = 4u;
            g.redraw |= 2u;
        }
        house();
    }

    GPIO_DisableAudioPath();
    BK4819_SetAF(BK4819_AF_MUTE);
    pocsag_af_capture_close();
    BK4819_WriteRegister((BK4819_REGISTER_t)REG_2B, r2b);
    BK4819_WriteRegister((BK4819_REGISTER_t)REG_73, r73);
    BK4819_WriteRegister((BK4819_REGISTER_t)REG_48, r48);
    BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false);
    BACKLIGHT_TurnOn();
}

#endif /* ENABLE_FLEX */