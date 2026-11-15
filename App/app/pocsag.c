/* pocsag.c — scan POCSAG résident (scanlist « POCSAG »), décodage 1200 bauds
 *
 * Le décodeur (filtre, 3 slicers à hystérésis, DPLL, mot de synchro dans
 * les deux polarités, BCH avec correction d'un bit, assemblage des pages) est
 * celui de l'app POCSAG RX, inchangé.
 *
 * SCAN
 *  - Les canaux de la scanlist « POCSAG » sont lus une fois au lancement.
 *  - Sur chaque canal : accord, POCSAG_SETTLE_MS de stabilisation, puis
 *    POCSAG_DWELL_MS d'écoute : signal à 1200 bauds (préambule ou lots,
 *    même pris en cours d'émission) ou mot de synchro ?
 *  - Rien : canal suivant. Trouvé : on décode jusqu'à la fin de l'émission
 *    (plus aucun lot pendant POCSAG_END_MS), puis on affiche la ou les pages
 *    reçues avec la fréquence en en-tête, et le scan reprend.
 *  - Les pages restent affichées (10 dernières, toutes fréquences) pendant
 *    que le scan continue ; une nouvelle page rallume l'écran.
 *
 * AUDIO
 *  - Le décodeur lit la sortie audio du BK4829 (broche AF, vue par l'ADC sur
 *    PA4). Cette sortie doit donc rester active : AF en FM et gain AF
 *    (REG_48) inchangé. Couper l'AF ou mettre REG_48 à 0 coupe aussi le
 *    signal du décodeur.
 *  - Le haut-parleur est coupé par l'ampli (GPIO_DisableAudioPath), qui est
 *    en aval : le scan est muet sans toucher au signal du décodeur.
 *  - REG_2B, REG_73 et REG_48 sont rétablis à la sortie.
 *
 * Touches : EXIT quitte · UP/DOWN pages (puis défilement) · * vue compacte ·
 *           1 tient le canal en cours (H) · 2 efface les pages ·
 *           3 diagnostic en bas d'écran.
 */

#ifdef ENABLE_POCSAG

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

#include "app/pocsag.h"

extern char gListName[MR_CHANNELS_LIST][10];

/* ============================================================
 * Constantes POCSAG (app POCSAG RX)
 * ============================================================ */
#define BIAS_CODE      2048u
#define FS             9600u
#define CYC_PER_SAMPLE (48000000u / FS)
#define HOUSE_EVERY    480u    /* 50 ms entre deux services touches/écran */
#define BUSY_MAX       19200u
#define LED_STEP       960u
#define LED_STEPS      4u
#define SYNC_SLOT      8u
#define REFRESH_SLOTS  100u
#define REDRAW_ON      1u
#define REDRAW_FRAME   2u

#define REG_2B      0x2B   /* <10:8> = 1 : RX HPF300, LPF3K, désaccentuation off */
#define REG_73      0x73   /* <4> = 1 : AFC off (il suivrait les bits)            */
#define REG_48      0x48   /* gain AF : le laisser, le décodeur en dépend          */

#define NSL         3
#define STEP_1200   8192u
#define DUP_1200    128u
#define PLL_SHIFT   3
#define DIRTY_WORDS 8u

#define SYNC        0x7CD215D8u
#define IDLE        0x7A89C197u
#define GEN         0x769u
#define SYNC_ERR    2u
#define BATCH_WORDS 16u
#define HUNT        0xFFu
#define FUNC_NUMERIC 0u
#define AUTO_MIN_ALPHA 6u
#define AUTO_MIN_LETTERS 4u
#define AUTO_MARGIN    9
#define MSG_MAX     254u
#define GAP         0x7Fu
#define BAD_RUN     3u

#define POOL_SIZE   1586u
#define MAX_MSGS    10u

/* Layout texte */
#define TONE_STR        "TONE ONLY"
#define LOST_STR        "TEXT LOST"
#define NOTEXT_X        28
#define TITLE_X         2u
#define CAPS_END        127u
#define ROW_CHARS       18u
#define CW_COMPACT      14u
#define ROWS            3u

/* ============================================================
 * Paramètres du scan
 * ============================================================ */
#define POCSAG_LIST_NAME    "POCSAG"
#define POCSAG_MAX_CH       64u     /* canaux de la scanlist gardés          */
#define POCSAG_SETTLE_MS    40u     /* après l'accord, avant d'écouter       */
#define POCSAG_DWELL_MS     50u     /* recherche du préambule par canal      */
/* Signal à 1200 bauds : les changements du slicer tombent sur des multiples
 * de la durée d'un bit (8 échantillons), au préambule (1 0 1 0) comme dans
 * les lots. Le bruit les place n'importe où. On compte les écarts à +/-1
 * échantillon d'un multiple (good) et les autres (bad). Détecte aussi une
 * émission prise en cours, après son préambule. */
#define POCSAG_LOCK_GOOD    24u
#define POCSAG_LOCK_RATIO   4u      /* bad * 4 <= good                       */
/* Après le préambule, le premier mot de synchro arrive au plus tard à la
 * fin du préambule (480 ms normalisés, plus long sur certains réseaux). */
#define POCSAG_SYNC_WAIT_MS 1500u
/* Fin d'émission : plus aucun slicer dans un lot depuis ce temps. */
#define POCSAG_END_MS       600u
#define POCSAG_MAX_RX_MS    30000u  /* plafond d'une émission                */

#define MS_SAMPLES(ms)  ((uint32_t)(ms) * (FS / 1000u))

/* ============================================================
 * Structures
 * ============================================================ */
typedef struct {
    uint32_t ric;
    uint32_t t;
    uint32_t at;
    uint32_t freq;
    int16_t  rssi;
    uint16_t offset;
    uint16_t len;
    uint8_t  func, alpha;
} page_t;

typedef struct {
    int32_t  amp;
    int32_t  rprev, phase;
    uint32_t sr;
    uint32_t inv;
    uint32_t ric;
    uint32_t at;
    uint8_t  num;
    uint8_t  dirty;
    uint8_t  nb;
    uint8_t  word;
    uint8_t  has;
    uint8_t  func, len, alpha;
    uint8_t  alen, aacc, ana, ataint, actrl;
    uint8_t  acc, na;
    uint8_t  taint;
    uint8_t  badrun;
    uint8_t  excuse;
    uint16_t ok;
    uint16_t step;
    uint16_t dup;
    uint8_t  live;
    char     text[MSG_MAX];
    char     atext[MSG_MAX];
} sl_t;

/* Buffers statiques : pas sur la pile (HardFault sinon). Les pages et le
 * pool sont gardés d'un canal à l'autre. */
static char     g_local_pool[POOL_SIZE];
static page_t   g_local_pages[MAX_MSGS];
static sl_t     g_sl[NSL];
static uint16_t g_local_pool_head;
static uint32_t g_chFreq[POCSAG_MAX_CH];

static char     *const g_pool = g_local_pool;
static uint16_t *const g_pool_head = &g_local_pool_head;
static page_t   *const g_pages = g_local_pages;

static struct {
    uint8_t  prevKey, redraw;
    bool     running;
    uint8_t  count, cur;
    uint8_t  cw;
    uint8_t  refresh;
    uint8_t  top, lim;
    uint16_t nOk;
    uint8_t  dbg;
    uint8_t  led;
    uint16_t ledCnt;
    uint16_t nSync;
    uint16_t busyFor;
    uint32_t n;
    uint32_t current_freq;
    uint16_t nCh, nHits;
    uint16_t lastGood, lastBad, lastQ; /* flips au pas d'un bit / autres / au demi-bit, dernière fenêtre */
    uint16_t nWords;                   /* mots BCH bons, cumulés (le compteur des slicers repart à chaque canal) */
    uint8_t  hold;                     /* touche 1 : reste sur le canal en cours */
    uint8_t  battOk;                   /* une mesure de batterie faite */
    sl_t     *sl;
} g;

volatile bool g_pocsagScanStop = false;

/* ============================================================
 * Helpers
 * ============================================================ */
static void *pocsag_memset(void *d, int c, size_t n) {
    uint8_t *p = d; while (n) p[--n] = (uint8_t)c; return d;
}
static void *pocsag_memcpy(void *d, const void *s, size_t n) {
    uint8_t *p = d; const uint8_t *q = s;
    while (n) { --n; p[n] = q[n]; } return d;
}

/* ============================================================
 * Capture audio PA4
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

static void pocsag_afc_resume_adc(void)
{
    pocsag_afc_select_pa4();
}

/* ============================================================
 * Compteur 48 MHz
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

/* ============================================================
 * Accès matériel
 * ============================================================ */
static inline int16_t pocsag_rssi_dbm(void)
{
    return BK4819_GetRSSI_dBm() + dBmCorrTable[gRxVfo->Band];
}

static inline uint32_t pocsag_ticks_ms(void)
{
    return gGlobalSysTickCounter * 10u;
}

/* ============================================================
 * Scan
 * ============================================================ */
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
    for (uint8_t i = 0; i < MR_CHANNELS_LIST; i++) {
        if (pocsag_name_match(gListName[i], name))
            return (int8_t)i;
    }
    return -1;
}

static bool pocsag_channel_in_list(uint16_t ch, int8_t listIdx)
{
    ChannelAttributes_t att;
    MR_LoadChannelAttributesFromFlash(ch, &att);
    return (int8_t)(att.scanlist - 1) == listIdx;
}

/* ============================================================
 * Décodage (app POCSAG RX, inchangé)
 * ============================================================ */
static bool preferAlpha(const sl_t *m, unsigned nn){
    if(m->func != FUNC_NUMERIC || m->alen < AUTO_MIN_ALPHA) return false;
    int score = -(int)m->actrl * 8, letters = 0;
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
    return (unsigned)letters >= AUTO_MIN_LETTERS && spaces >= 2u && words >= 3u &&
           (unsigned)letters * 3u >= m->alen * 2u && score >= AUTO_MARGIN;
}

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
    unsigned n = m->len, gaps = 0, lost = 0;
    while(n && (t[n - 1u] == GAP || (m->func == FUNC_NUMERIC && t[n - 1u] == ' '))) lost |= t[--n] == GAP;
    for(unsigned i = 0; i < n; i++) gaps += t[i] == GAP;
    if((!n) || 2u * gaps > n) return;
    g.redraw |= REDRAW_FRAME;

    for(unsigned j = 0; j < g.count; j++){
        page_t *q = &g_pages[j];
        if(q->ric != m->ric || q->func != m->func ||
           (uint32_t)(m->at - q->at + m->dup) >= 2u * m->dup) continue;
        if(q->alpha != m->alpha){
            if(!m->alpha) return;
            q->alpha = 1;
            q->len = (uint8_t)n;
            for(unsigned i = 0; i < n; i++){
                g_pool[(q->offset + i) % POOL_SIZE] = ((uint8_t)t[i] == GAP) ? '?' : t[i];
            }
            return;
        }
        for(unsigned i = 0; i < n; i++)
            if(i >= q->len || (uint8_t)g_pool[(q->offset + i) % POOL_SIZE] == GAP)
                g_pool[(q->offset + i) % POOL_SIZE] = (char)((uint8_t)t[i] == GAP ? '?' : t[i]);
        if(n > q->len) q->len = (uint8_t)n;
        return;
    }

    unsigned msg_len = (n > 0) ? n : 1u;
    if ((uint32_t)(*g_pool_head) + msg_len > POOL_SIZE) {
        *g_pool_head = 0;
    }
    uint16_t current_offset = *g_pool_head;

    if (n > 0) {
        for(unsigned i = 0; i < n; i++) {
            g_pool[(*g_pool_head) % POOL_SIZE] = ((uint8_t)t[i] == GAP) ? '?' : t[i];
            *g_pool_head = (*g_pool_head + 1) % POOL_SIZE;
        }
    } else {
        g_pool[(*g_pool_head) % POOL_SIZE] = (char)lost;
        *g_pool_head = (*g_pool_head + 1) % POOL_SIZE;
    }

    if(g.count < MAX_MSGS) g.count++;
    for(unsigned k = g.count - 1u; k; k--) g_pages[k] = g_pages[k - 1u];

    page_t *p = &g_pages[0];
    p->rssi = pocsag_rssi_dbm();
    p->ric = m->ric;
    p->func = m->func;
    p->alpha = m->alpha;
    p->at = m->at;
    p->t = pocsag_ticks_ms();
    p->freq = g.current_freq;
    p->offset = current_offset;
    p->len = (uint16_t)n;

    g.nOk++;
    g.cur = 0;
    g.top = 0;
}

static unsigned ones(uint32_t v){
    unsigned n = 0;
    for(; v; v &= v - 1u) n++;
    return n;
}

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

static const char NUMERIC[16] = {'0','1','2','3','4','5','6','7','8','9','*','U',' ','-',')','('};

static void addc(sl_t *m, unsigned ch){
    if(m->len < MSG_MAX) m->text[m->len++] = (char)ch;
}
static void adda(sl_t *m, unsigned ch){
    if(m->alen < MSG_MAX) m->atext[m->alen++] = (char)ch;
}

static void codeword(sl_t *m, uint32_t cw){
    unsigned good = fix(&cw);
    if(!good) m->dirty = DIRTY_WORDS;
    else if(m->dirty){
        m->dirty--;
        if(good == 2u) good = 0;
    }
    unsigned n = m->func == FUNC_NUMERIC ? 4u : 7u;
    if(!good || cw + 1u < 2u){
        if(!m->has) return;
        if(++m->badrun >= BAD_RUN){ closePage(m); return; }
        unsigned left = m->na + 20u;
        for(; left >= n; left -= n) addc(m, GAP);
        m->na = (uint8_t)left;
        m->acc = 0;
        m->taint = left != 0u;
        if(m->func == FUNC_NUMERIC){
            left = m->ana + 20u;
            for(; left >= 7u; left -= 7u) adda(m, GAP);
            m->ana = (uint8_t)left;
            m->aacc = 0;
            m->ataint = left != 0u;
        }
        return;
    }
    m->ok++;
    m->live = 1;
    if(cw == IDLE){ closePage(m); return; }
    if(cw >> 31){
        if(!m->has) return;
        m->badrun = 0;
        for(unsigned i = 30; i > 10u; i--){
            unsigned bit = (cw >> i) & 1u;
            m->acc |= (uint8_t)(bit << m->na);
            if(++m->na == n){
                unsigned ch = m->acc;
                if(n == 4u) ch = (uint8_t)NUMERIC[ch];
                if(m->taint) addc(m, GAP);
                else if(ch - 32u < 95u) addc(m, ch);
                m->taint = 0;
                m->acc = m->na = 0;
            }
            if(m->func == FUNC_NUMERIC){
                m->aacc |= (uint8_t)(bit << m->ana);
                if(++m->ana == 7u){
                    unsigned ch = m->aacc;
                    if(m->ataint) adda(m, GAP);
                    else if(ch - 32u < 95u) adda(m, ch);
                    else if(m->actrl < 255u) m->actrl++;
                    m->ataint = 0;
                    m->aacc = m->ana = 0;
                }
            }
        }
        return;
    }
    closePage(m);
    m->ric = (((cw >> 13) & 0x3FFFFu) << 3) | (m->word >> 1);
    m->func = (uint8_t)((cw >> 11) & 3u);
    m->at = g.n;
    m->has = 1;
    m->acc = m->na = m->len = m->taint = m->badrun = m->alpha = 0;
    m->alen = m->aacc = m->ana = m->ataint = m->actrl = 0;
}

__attribute__((noinline))
static void seqBit(sl_t *m, uint32_t b){
    uint32_t sr = (m->sr << 1) | b;
    m->sr = sr;
    if(m->word == HUNT){
        unsigned e = ones(sr ^ SYNC);
        if(e <= SYNC_ERR || e >= 32u - SYNC_ERR){
            m->inv = e > 16u ? 0xFFFFFFFFu : 0u;
            m->word = 0;
            m->nb = 0;
            g.nSync++;
        }
        return;
    }
    if(++m->nb < 32u) return;
    m->nb = 0;
    uint32_t cw = sr ^ m->inv;
    if(m->word == BATCH_WORDS){
        if(ones(cw ^ SYNC) <= SYNC_ERR || (m->excuse && m->live)) m->word = 0;
        else { closePage(m); m->word = HUNT; }
        m->excuse = m->live = 0;
        return;
    }
    codeword(m, cw);
    m->word++;
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
    if((r > 0) != (m->rprev > 0)) m->phase += (32768 + (m->step >> 1) - m->phase) >> PLL_SHIFT;
    m->rprev = r;
    if(m->phase >= 65536){
        m->phase -= 65536;
        seqBit(m, r <= 0);
    }
}

/* ============================================================
 * Affichage
 * ============================================================ */
static void printFreq(char *s, uint32_t f)
{
    sprintf(s, "%u.%05u", (unsigned)(f / 100000u), (unsigned)(f % 100000u));
}

__attribute__((noinline))
static void draw(void){
    char s[64] __attribute__((aligned(4)));
    const page_t *p = &g_pages[g.cur];
    unsigned lim = 0;

    UI_DisplayClear();
    UI_StatusClear();

    /* En-tête : fréquence de la page affichée, sinon celle du scan. */
    printFreq(s, g.count ? p->freq : g.current_freq);
    GUI_DisplaySmallestInverse(s, TITLE_X, 0, true, true,
                               (uint8_t)(TITLE_X + 4u * (unsigned)strlen(s)));

    {
        /* Batterie : l'ADC repasse sur sa voie le temps de la mesure. La
         * première conversion après le changement de voie est jetée (elle
         * peut encore porter l'audio de PA4), puis le calcul est celui de
         * la barre d'état (BATTERY_GetReadings). Une mesure toutes les 5 s. */
        if (g.refresh == 0 || !g.battOk) {
            bool yielded = pocsag_afc_yield_adc();
            uint16_t v, cur;
            BOARD_ADC_GetBatteryInfo(&v, &cur);
            BOARD_ADC_GetBatteryInfo(&gBatteryVoltages[gBatteryCheckCounter++ % 4], &gBatteryCurrent);
            if (yielded) pocsag_afc_resume_adc();
            BATTERY_GetReadings(false);
            g.battOk = 1;
        }
        sprintf(s, "%u%%", (unsigned)BATTERY_VoltsToPercent(gBatteryVoltageAverage));
        GUI_DisplaySmallest(s, 128, 0, true, true);
    }

    if (g.count) {
        sprintf(s, "%lu", (unsigned long)p->ric);
        UI_PrintStringSmallBold(s, 0, 0, 0);

        unsigned capsEnd = CAPS_END;
        char *o = s + sprintf(s, "%u/%u", g.cur + 1u, g.count);
        capsEnd -= 4u * (unsigned)(o - s);
        GUI_DisplaySmallestInverse(s, (uint8_t)capsEnd, 0, false, true, CAPS_END);

        unsigned w = ROW_CHARS + g.cw, rows = 0;
        for (unsigned i = 0; i < p->len; i += w, rows++) {
            unsigned n = p->len - i, r = rows - g.top;
            if (n > w) n = w;
            for (unsigned j = 0; j < n; j++) {
                char ch = g_pool[(p->offset + i + j) % POOL_SIZE];
                s[j] = (uint8_t)ch == GAP ? '?' : ch;
            }
            s[n] = '\0';
            if (g.cw) { if (r < 8u) GUI_DisplaySmallest(s, 0, (uint8_t)(8u + r * 6u), false, true); }
            else if (r < ROWS) UI_PrintStringSmallBold(s, 0, 0, (uint8_t)(1u + r));
        }
        if (!p->len)
            UI_PrintStringSmallBold(g_pool[p->offset % POOL_SIZE] ? LOST_STR : TONE_STR, NOTEXT_X, 0, 2);
        if (!g.cw && rows > ROWS) lim = rows - ROWS;
    } else {
        sprintf(s, "SCAN %u CH", g.nCh);
        UI_PrintStringSmallBold(s, 0, 0, 2);
    }

    if (g.dbg) {
        /* Touche 3, diagnostic :
         *  A  niveau audio vu par le décodeur (crête, codes ADC filtrés) :
         *     ~0 = aucun signal sur PA4 (réception ou AF coupée)
         *  R  RSSI du canal (dBm)   P signaux détectés   S synchros
         *  puis mots BCH bons par slicer. */
        sprintf(s, "A%ld R%d P%u S%u W%u", (long)g.sl[0].amp, pocsag_rssi_dbm(),
                g.nHits, g.nSync, g.nWords);
        GUI_DisplaySmallest(s, 0, 50, false, true);
        /* G/B : flips au pas d'un bit / autres sur la dernière fenêtre de
         * 50 ms (détection : G >= 24 et B * 4 <= G). Q : flips au pas d'un
         * demi-bit (4, 12, 20 échantillons) : élevé = émission à 2400 bauds,
         * non décodée. H : canal tenu. */
        sprintf(s, "G%u B%u Q%u%s", g.lastGood, g.lastBad, g.lastQ, g.hold ? " H" : "");
        GUI_DisplaySmallest(s, 0, 43, false, true);
    }
    g.lim = (uint8_t)lim;
}

static void handleKeys(void){
    uint8_t key = KEYBOARD_GetKey();
    if (key == KEY_INVALID || key == g.prevKey) { g.prevKey = key; return; }
    g.prevKey = key;
    if (!BACKLIGHT_IsOn() && gEeprom.BACKLIGHT_TIME) { /* 1re touche : rallume seulement */
        BACKLIGHT_TurnOn();
        return;
    }
    BACKLIGHT_TurnOn();
    g.redraw |= REDRAW_ON;
    int d = (key == KEY_DOWN) - (key == KEY_UP);
    if (d) {
        unsigned top = (unsigned)(g.top + d);
        if (top <= g.lim) g.top = (uint8_t)top;
        else if ((unsigned)(g.cur + d) < g.count) { g.cur = (uint8_t)(g.cur + d); g.top = 0; }
    }
    else if (key == KEY_EXIT) {
        g.running = false;
        g_pocsagScanStop = true;
    }
    else if (key == KEY_2) {
        g.count = g.cur = 0; g.nOk = 0; g.top = 0;
        *g_pool_head = 0;
        pocsag_memset(g_pool, 0, POOL_SIZE);
        pocsag_memset(g_pages, 0, sizeof(page_t) * MAX_MSGS);
        g.nSync = 0; g.nHits = 0; g.nWords = 0;
        for (unsigned k = 0; k < NSL; k++) g.sl[k].ok = 0;
    }
    else if (key == KEY_3) g.dbg ^= 1u;
    else if (key == KEY_1) g.hold ^= 1u;
    else if (key == KEY_STAR) { g.cw ^= CW_COMPACT; g.top = 0; }
}

/* Rétroéclairage : extinction après la durée réglée (BACKLIGHT_TIME), comme
 * le spectre de Sonic ; BACKLIGHT_Update fait le fondu. */
static void backlightTick(void){
    BACKLIGHT_Update();
    if (gNextTimeslice_500ms) {
        gNextTimeslice_500ms = false;
        if (gBacklightCountdown_500ms > 0) --gBacklightCountdown_500ms;
        if (gEeprom.BACKLIGHT_TIME < 61 && gBacklightCountdown_500ms == 0) BACKLIGHT_TurnOff();
    }
}

/* Pendant une réception : touches et rétroéclairage seulement. Le dessin
 * de l'écran (polices en flash externe, envoi à l'écran) prendrait des
 * dizaines de millisecondes, donc des bits perdus : il attend la fin de
 * l'émission. */
static void houseLight(void){
    handleKeys();
    backlightTick();
}

static void house(void){
    handleKeys();
    if (g_pocsagScanStop) return;
    if (g.redraw & REDRAW_FRAME) BACKLIGHT_TurnOn();
    backlightTick();
    if (++g.refresh >= REFRESH_SLOTS || g.dbg) g.redraw |= REDRAW_ON; /* diagnostic : en direct */
    if (!g.redraw) return;
    g.redraw = 0;
    g.refresh = 0;
    draw();
    ST7565_BlitStatusLine();
    ST7565_BlitFullScreen();
}

/* ============================================================
 * Écoute d'un canal
 *
 * Phase 1 (POCSAG_DWELL_MS) : signal à 1200 bauds ou mot de synchro ?
 *   Non : retour tout de suite, canal suivant.
 * Phase 2 : décodage. L'écoute dure tant que des lots arrivent ; elle
 *   s'arrête POCSAG_END_MS après le dernier lot (fin d'émission), ou si le
 *   premier mot de synchro ne vient pas dans POCSAG_SYNC_WAIT_MS.
 * Les échantillons sautés pendant le service touches/écran sont rattrapés
 * par les horloges des slicers (alignement des mots gardé), comme l'app.
 * ============================================================ */
static void listen(void){
    sl_t *sl = g_sl;
    int32_t box4[4] = {0, 0, 0, 0}, sum4 = 0, dc = (int32_t)(BIAS_CODE << 4);
    unsigned bi = 0;
    pocsag_memset(sl, 0, sizeof(g_sl));
    for (unsigned k = 0; k < NSL; k++) {
        sl_t *m = &sl[k];
        m->num = (uint8_t)(2u + k);
        m->word = HUNT;
        m->step = STEP_1200;
        m->dup = DUP_1200;
    }
    g.sl = sl;
    uint16_t good = 0, bad = 0, q = 0;
    uint8_t run = 0;

    const uint32_t start = g.n;
    const uint16_t sync0 = g.nSync;
    uint32_t lastBatch = 0;        /* dernier échantillon vu dans un lot  */
    bool active = false;           /* préambule ou synchro trouvé          */
    uint32_t next = pocsag_cycles_48mhz();
    uint16_t cnt = 0;

    while (!g_pocsagScanStop) {
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
        for (unsigned k = 0; k < NSL; k++) slice(&sl[k], sum4);
        if (run < 255u) run++;
        if (sl[0].rprev != before) {
            const unsigned m = run & 7u, err = m <= 4u ? m : 8u - m;
            if (run >= 6u && run <= 44u && err <= 1u) good++;
            else bad++;
            if (run >= 3u && run <= 44u && (m == 3u || m == 4u || m == 5u)) q++;
            run = 0;
        }
        if (++g.ledCnt >= LED_STEP) {
            g.ledCnt = 0;
            if (g.led) { g.led--; BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, g.led & 1u); }
        }

        bool inBatch = false;
        for (unsigned k = 0; k < NSL; k++)
            if (sl[k].word != HUNT) inBatch = true;
        if (inBatch) lastBatch = g.n;

        const uint32_t elapsed = g.n - start;
        if (!active) {
            if (g.nSync != sync0 ||
                (good >= POCSAG_LOCK_GOOD && bad * POCSAG_LOCK_RATIO <= good)) {
                active = true;
                g.nHits++;
                lastBatch = g.n;
            } else if (elapsed >= MS_SAMPLES(POCSAG_DWELL_MS)) {
                g.lastGood = good; g.lastBad = bad; g.lastQ = q;
                return;                                   /* canal vide */
            }
            continue;                                     /* pas de service : 50 ms */
        }
        if (g.nSync == sync0) {                           /* pas encore de synchro */
            if (elapsed >= MS_SAMPLES(POCSAG_SYNC_WAIT_MS)) break;
        } else if (!inBatch && g.n - lastBatch >= MS_SAMPLES(POCSAG_END_MS)) {
            break;                                        /* fin d'émission */
        }
        if (elapsed >= MS_SAMPLES(POCSAG_MAX_RX_MS)) break;

        if (cnt < HOUSE_EVERY) { cnt++; continue; }
        bool wait = false;
        for (unsigned k = 0; k < NSL; k++) {
            unsigned w = sl[k].word;
            if (w != HUNT && (w != BATCH_WORDS || sl[k].nb >= SYNC_SLOT)) wait = true;
        }
        if (wait && ++g.busyFor < BUSY_MAX) continue;
        g.busyFor = 0;
        cnt = 0;
        houseLight();
        while ((int32_t)(pocsag_cycles_48mhz() - next) >= 0) {
            next += CYC_PER_SAMPLE;
            g.n++;
            for (unsigned k = 0; k < NSL; k++) {
                if (sl[k].word == BATCH_WORDS) sl[k].excuse = 1;
                sl[k].phase += sl[k].step;
                if (sl[k].phase >= 65536) { sl[k].phase -= 65536; seqBit(&sl[k], 0); }
            }
        }
    }
    for (unsigned k = 0; k < NSL; k++) {
        closePage(&sl[k]);                                 /* page en cours */
        g.nWords += sl[k].ok;
    }
}

/* ============================================================
 * Accord rapide (comme SetF du spectre) : fréquence, filtre, relance
 * du synthé par REG_30.
 * La valeur de réception de REG_30 est lue une fois, après
 * BK4819_RX_TurnOn. Attention : BK4819_WriteRegister met reg_30_cache à
 * jour et ignore une écriture égale au cache. Écrire 0x200 puis
 * reg_30_cache réécrirait 0x200 (ignoré) et laisserait la réception coupée.
 * ============================================================ */
static uint16_t g_rx30;

static void pocsag_tune(uint32_t f)
{
    BK4819_SetFrequency(f);
    BK4819_PickRXFilterPathBasedOnFrequency(f);
    BK4819_WriteRegister(BK4819_REG_30, 0x200);
    BK4819_WriteRegister(BK4819_REG_30, g_rx30);
}

static uint16_t pocsag_load_channels(int8_t listIdx)
{
    uint16_t n = 0;
    for (uint16_t ch = MR_CHANNEL_FIRST; ch <= MR_CHANNEL_LAST && n < POCSAG_MAX_CH; ch++) {
        uint32_t freq = pocsag_channel_freq(ch);
        if (freq && pocsag_channel_in_list(ch, listIdx))
            g_chFreq[n++] = freq;
    }
    return n;
}

/* ============================================================
 * APP_RunPocsagScan : tourne jusqu'à EXIT (g_pocsagScanStop).
 * ============================================================ */
void APP_RunPocsagScan(void)
{
    int8_t listIdx = pocsag_find_scanlist(POCSAG_LIST_NAME);
    g.nCh = listIdx < 0 ? 0 : pocsag_load_channels(listIdx);
    if (!g.nCh) {
        UI_DisplayClear();
        UI_PrintStringSmallBold("NO " POCSAG_LIST_NAME " LIST", 0, 0, 2);
        ST7565_BlitFullScreen();
        SYSTEM_DelayMs(1500);
        g_pocsagScanStop = true;
        return;
    }

    const uint16_t r2b = BK4819_ReadRegister((BK4819_REGISTER_t)REG_2B);
    const uint16_t r73 = BK4819_ReadRegister((BK4819_REGISTER_t)REG_73);
    const uint16_t r48 = BK4819_ReadRegister((BK4819_REGISTER_t)REG_48);

    GPIO_DisableAudioPath();                       /* HP muet, AF intact */
    pocsag_af_capture_open();
    BK4819_ToggleGpioOut(BK4819_GPIO0_PIN28_RX_ENABLE, true);
    BK4819_SetFrequency(g_chFreq[0]);
    BK4819_PickRXFilterPathBasedOnFrequency(g_chFreq[0]);
    BK4819_SetFilterBandwidth(BK4819_FILTER_BW_WIDE, false);
    BK4819_RX_TurnOn();
    g_rx30 = BK4819_ReadRegister(BK4819_REG_30);  /* réception complète */
    BK4819_WriteRegister((BK4819_REGISTER_t)REG_2B, (uint16_t)((r2b | 0x0700u) & ~0x0007u));
    BK4819_WriteRegister((BK4819_REGISTER_t)REG_73, (uint16_t)(r73 | 0x0010u));
    BK4819_SetAF(BK4819_AF_FM);
    BACKLIGHT_TurnOn();
    g_cyc_base = APP_CYC_DIAGNOSTIC_START - pocsag_cycles_raw();
    g.redraw = REDRAW_ON;

    uint32_t tuned = g_chFreq[0];
    SYSTEM_DelayMs(POCSAG_SETTLE_MS);
    for (uint16_t i = 0; !g_pocsagScanStop; i = g.hold ? i : (uint16_t)((i + 1u) % g.nCh)) {
        g.current_freq = g_chFreq[i];
        if (g.current_freq != tuned) {             /* un seul canal, ou tenu : pas de réaccord */
            tuned = g.current_freq;
            pocsag_tune(tuned);
            SYSTEM_DelayMs(POCSAG_SETTLE_MS);
        }

        const uint16_t before = g.nOk;
        listen();
        if (g.nOk != before) {                     /* nouvelle(s) page(s) */
            g.cur = 0; g.top = 0;
            g.led = LED_STEPS;
            g.redraw |= REDRAW_FRAME;
        }
        house();                                   /* touches, écran, rétroéclairage */
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

#endif /* ENABLE_POCSAG */
