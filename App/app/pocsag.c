/* pocsag.c — décodeur POCSAG résident + scan SCANLIST « POCSAG »
 *
 * OPTION B : décodage en continu, changement de fréquence sans couper l'audio.
 *
 * STRATÉGIE DOUBLE TIMEOUT :
 *   - Fenêtre de découverte = POCSAG_SCAN_DWELL_MS (100 ms).
 *     En 100 ms à 1200 bps on a ~120 bits = ~3,7 codewords : assez pour
 *     capter un SYNC (26,7 ms) ou quelques codewords valides si le PLL
 *     a accroché, ou pour mesurer l'activité du signal (transitions).
 *   - Si à l'échéance rien n'a été vu (ni SYNC, ni codeword OK, ni
 *     suffisamment de transitions) → canal vide → on passe au suivant.
 *   - Sinon → canal ACTIF → on prolonge jusqu'à POCSAG_LISTEN_TIMEOUT_MS
 *     (3000 ms) ou jusqu'à réception d'un message complet (g.count > 0).
 *
 * AUDIO — AUCUN SON :
 *   - PA4 (entrée ADC du décodeur) reste TOUJOURS alimenté pendant le scan,
 *     sinon aucun décodage n'est possible.
 *   - Le haut-parleur est rendu TOTALEMENT muet en forçant le volume BK4819
 *     à 0 (pocsag_mute_speaker). Le signal sur PA4 est prélevé AVANT l'étage
 *     de volume, donc le décodeur n'est pas affecté.
 *   - AUCUNE touche ne réactive le son pendant le scan.
 *   - À la sortie du scan, le volume utilisateur est restauré.
 *
 * BACKLIGHT :
 *   - Rallumé à l'arrivée d'un message (REDRAW_FRAME).
 *   - Rallumé sur toute action utilisateur (touche valide).
 *   - Extinction auto gérée par BACKLIGHT_Update() selon le timeout
 *     configuré par l'utilisateur.
 *   - L'affichage reste permanent (pas de timeout de showMessages).
 *
 * Touches :
 *  - EXIT : quitte le scan (g_pocsagScanStop = true)
 *  - UP/DOWN : parcourt les messages
 *  - KEY_STAR : mode compact
 *  - KEY_2 : efface les messages → reprise du scan
 *  - KEY_3 : debug (compteurs)
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
#include "scheduler.h"
#include "helper/battery.h"
#include "py32f0xx.h"

#include "app/pocsag.h"

extern char gListName[MR_CHANNELS_LIST][10];

/* ============================================================
 * Constantes POCSAG
 * ============================================================ */
#define BIAS_CODE      2048u
#define FS             9600u
#define CYC_PER_SAMPLE (48000000u / FS)
#define HOUSE_EVERY    480u
#define BUSY_MAX       19200u
#define LED_STEP       960u
#define SYNC_SLOT      8u
#define REFRESH_SLOTS  100u
#define REDRAW_ON      1u
#define REDRAW_FRAME   2u

#define REG_2B      0x2B
#define REG_73      0x73
#define REG_48      0x48

#define NSL         3
#define STEP_1200   8192u
#define DUP_1200    128u
#define PLL_SHIFT   3
#define PRE_RUNS    32u
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
#define NO_MSG_STR      "NO MSG"
#define TONE_STR        "TONE"
#define NOTEXT_X        60
#define TITLE_X         2u
#define CAPS_END        127u
#define ROW_CHARS       18u
#define CW_COMPACT      14u
#define ROWS            3u
#define SPK_X           59u

/* ============================================================
 * Paramètres du scan — DOUBLE TIMEOUT
 * ============================================================ */
#define POCSAG_LIST_NAME        "POCSAG"

/* Fenêtre de découverte : si rien vu en 100 ms → canal vide. */
#define POCSAG_SCAN_DWELL_MS    100u

/* Délai de stabilisation après tune (ms) avant de compter les SYNC. */
#define POCSAG_SETTLE_MS        30u

/* Timeout total quand on est sur un canal "accroché" (ms). */
#define POCSAG_LISTEN_TIMEOUT_MS 20u

/* Nombre minimal de transitions slicer en 100 ms pour considérer
 * qu'il y a de la modulation (bruit seul ≈ quelques transitions ;
 * POCSAG 1200 bps ≈ 120 transitions max, on prend une marge basse). */
#define POCSAG_LIVE_BITS_MIN    20u

/* Anti-rebond : un canal qui a déclenché est ignoré pendant ce temps (ms). */
#define POCSAG_REBOND_MS        500u

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

static char     *g_pool = NULL;
static uint16_t *g_pool_head = NULL;
static page_t   *g_pages = NULL;

/* Buffers statiques : évitent 3,5 Ko sur la pile (HardFault sinon). */
static char     g_local_pool[POOL_SIZE];
static page_t   g_local_pages[MAX_MSGS];
static sl_t     g_sl[NSL];
static uint16_t g_local_pool_head = 0;

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
    uint16_t preRate;
    uint16_t rfCls;
    uint8_t  rfRun, rfStreak;
    uint16_t busyFor;
    uint32_t n;
    uint32_t current_freq;
    char     *text;
    sl_t     *sl;
    /* Transitions slicer (bits "live") pendant la fenêtre de découverte. */
    uint16_t liveBits;
} g;
#define str g.text

volatile bool g_pocsagScanStop = false;

static uint32_t last_triggered_freq = 0;
static uint32_t last_triggered_tick = 0;

/* Fréquence affichée quand g.count == 0 (scan en cours) */
static uint32_t scan_freq = 0;

/* Timeout total (prolongation) pour listen() — piloté par APP_RunPocsagScan */
uint32_t g_listen_timeout_ms = POCSAG_LISTEN_TIMEOUT_MS;

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
 * Contrôle audio : couper le HP sans couper PA4
 * ============================================================
 * Le signal POCSAG sur PA4 est prélevé AVANT l'étage de volume
 * du BK4819. Mettre le volume à 0 rend le HP totalement muet
 * sans affecter le décodeur.
 *
 * Aucune touche ne réactive le son pendant le scan : le scan
 * POCSAG est TOUJOURS silencieux.
 * ============================================================ */


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
 * Décodage
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
__attribute__((noinline))
static void draw(void){
    char s[32] __attribute__((aligned(4)));
    char *o;
    const page_t *p = &g_pages[g.cur];
    unsigned lim = 0;

    UI_DisplayClear();
    UI_StatusClear();

    {
        uint32_t f = (g.count > 0) ? p->freq : scan_freq;
        char hdr[16];
        snprintf(hdr, sizeof(hdr), "%u.%05u",
                 (unsigned)(f / 100000u),
                 (unsigned)(f % 100000u));
        GUI_DisplaySmallestInverse(hdr, TITLE_X, 0, true, true,
                                   (uint8_t)(TITLE_X + 10u * 4u));
    }

    {
        bool yielded = pocsag_afc_yield_adc();
        BOARD_ADC_GetBatteryInfo(&gBatteryVoltages[gBatteryCheckCounter++ % 4], &gBatteryCurrent);
        if (yielded) pocsag_afc_resume_adc();

        uint16_t voltage = (gBatteryVoltages[0] + gBatteryVoltages[1] +
                            gBatteryVoltages[2] + gBatteryVoltages[3]) / 4 * 760 / gBatteryCalibration[3];
        unsigned perc = BATTERY_VoltsToPercent(voltage);
        sprintf(s, "%d%%", perc);
        GUI_DisplaySmallest(s, 128, 0, true, true);
    }

    if(g.count){
        o = s;
        o += sprintf(o, "%lu", (unsigned long)p->ric);
        *o = '\0';
        UI_PrintStringSmallBold(s, 0, 0, 0);

        unsigned capsEnd = CAPS_END;
        if(g.count > 1u){
            o = s;
            o += sprintf(o, "%u/%u", g.cur + 1, g.count);
            *o = '\0';
            capsEnd -= 4u * (unsigned)(o - s);
            GUI_DisplaySmallestInverse(s, (uint8_t)capsEnd, 0, false, true, CAPS_END);
            capsEnd -= 4u;
        }
        o = s;
        o += sprintf(o, "%u", g.nOk);
        *o = '\0';
        GUI_DisplaySmallestInverse(s, (uint8_t)(capsEnd - 4u * (unsigned)(o - s)),
                                   0, false, true, (uint8_t)capsEnd);

        unsigned w = ROW_CHARS + g.cw, rows = 0;
        for(unsigned i = 0; i < p->len; i += w, rows++){
            unsigned n = p->len - i, r = rows - g.top;
            if(n > w) n = w;
            for(unsigned j = 0; j < n; j++){
                char ch = g_pool[(p->offset + i + j) % POOL_SIZE];
                s[j] = (uint8_t)ch == GAP ? '?' : ch;
            }
            s[n] = '\0';
            if(g.cw){ if(r < 8u) GUI_DisplaySmallest(s, 0, (uint8_t)(8u + r * 6u), false, true); }
            else if(r < ROWS) UI_PrintStringSmallBold(s, 0, 0, (uint8_t)(1u + r));
        }

        if(!p->len){
            char lost_val = g_pool[p->offset % POOL_SIZE];
            const char *msg = lost_val ? NO_MSG_STR : TONE_STR;
            GUI_DisplaySmallest(msg, NOTEXT_X, 0, 2, true);
        }
        if(!g.cw && rows > ROWS) lim = rows - ROWS;
    } else {
        GUI_DisplaySmallest("WAIT...", NOTEXT_X, 0, 2, true);
    }
    g.lim = (uint8_t)lim;
    
}

/* Traitement des touches */
static void handleKeys(void){
    uint8_t key = KEYBOARD_GetKey();
    if(key == KEY_INVALID || key == g.prevKey){ g.prevKey = key; return; }
    g.prevKey = key;
    BACKLIGHT_TurnOn();                /* toute touche rallume l'écran */
    g.redraw |= REDRAW_ON;
    int d = (key == KEY_DOWN) - (key == KEY_UP);
    if(d){
        unsigned top = (unsigned)(g.top + d);
        if(top <= g.lim) g.top = (uint8_t)top;
        else if((unsigned)(g.cur + d) < g.count){ g.cur = (uint8_t)(g.cur + d); g.top = 0; }
    }
    else if(key == KEY_EXIT) {
        g.running = false;
        g_pocsagScanStop = true;
    }
    else if(key == KEY_2){
        g.count = g.cur = 0; g.nOk = 0; g.top = 0;
        *g_pool_head = 0;
        pocsag_memset(g_pool, 0, POOL_SIZE);
        pocsag_memset(g_pages, 0, sizeof(page_t) * MAX_MSGS);
        g.nSync = 0; g.preRate = 0;
        for(unsigned k = 0; k < NSL; k++) g.sl[k].ok = 0;
    }
    else if(key == KEY_3) g.dbg ^= 1u;
    else if(key == KEY_STAR){ g.cw ^= CW_COMPACT; g.top = 0; }
}

static void house(void){
    handleKeys();
    if(!g.running) return;
    if(g.redraw & REDRAW_FRAME) BACKLIGHT_TurnOn();   /* nouveau message */
    BACKLIGHT_Update();                                /* extinction auto */
    if(++g.refresh >= REFRESH_SLOTS) g.redraw |= REDRAW_ON;
    if(!g.redraw) return;
    g.redraw = 0;
    g.refresh = 0;
    draw();
    ST7565_BlitStatusLine();
    ST7565_BlitFullScreen();
}

static void rateFind(bool flipped){
    if(g.rfRun < 255u) g.rfRun++;
    if(!flipped) return;
    unsigned r = g.rfRun, c = 0;
    if(r - 3u <= 2u) c = 2400u;
    else if(r - 6u <= 4u) c = 1200u;
    g.rfRun = 0;
    if(c && c == g.rfCls){
        if(g.rfStreak < PRE_RUNS && ++g.rfStreak == PRE_RUNS) g.preRate = (uint16_t)c;
    } else {
        g.rfCls = (uint16_t)c;
        g.rfStreak = 1;
    }
}

/* ============================================================
 * Boucle de décodage en continu — DOUBLE TIMEOUT
 *
 *  Phase 1 (découverte) : 0 .. POCSAG_SCAN_DWELL_MS
 *      → si nSync == 0 ET nOk == 0 ET liveBits < POCSAG_LIVE_BITS_MIN
 *        alors canal vide → sortie immédiate.
 *
 *  Phase 2 (prolongation) : POCSAG_SCAN_DWELL_MS .. g_listen_timeout_ms
 *      → on continue tant que rien n'est décodé ou timeout atteint.
 *
 *  AUDIO : GPIO_EnableAudioPath() est appelé INCONDITIONNELLEMENT
 *  pour garder PA4 alimenté. Le HP est déjà muet via pocsag_mute_speaker()
 *  (volume BK4819 = 0), donc aucun son n'est émis.
 * ============================================================ */
static void listen(void){
    /* Buffers statiques (pas sur la pile → évite HardFault) */
    g_pool = g_local_pool;
    g_pool_head = &g_local_pool_head;
    g_pages = g_local_pages;

    pocsag_memset(g_local_pool, 0, POOL_SIZE);
    pocsag_memset(g_local_pages, 0, sizeof(g_local_pages));
    g_local_pool_head = 0;   /* reset tête de pool à chaque canal */

    sl_t *sl = g_sl;
    int32_t box4[4], sum4 = 0, dc = (int32_t)(BIAS_CODE << 4);
    unsigned bi = 0;
    pocsag_memset(sl, 0, sizeof(g_sl));
    pocsag_memset(box4, 0, sizeof box4);
    for(unsigned k = 0; k < NSL; k++){
        sl_t *m = &sl[k];
        m->num = (uint8_t)(2u + k);
        m->word = HUNT;
        m->step = STEP_1200;
        m->dup = DUP_1200;
    }

    g.sl = sl;
    g.liveBits = 0;
    g.nSync = 0;
    g.nOk = 0;

    uint32_t next = pocsag_cycles_48mhz();
    uint16_t cnt = 0;
    g.redraw = REDRAW_ON;

    uint32_t listen_start = gGlobalSysTickCounter;
    bool dwell_expired = false;

    while(g.running){
        if (g_pocsagScanStop) { g.running = false; break; }

        /* --- Détection de fin de fenêtre (dwell puis prolongation) --- */
        uint32_t elapsed_ms = (gGlobalSysTickCounter - listen_start) * 10u;

        if (!dwell_expired && elapsed_ms >= POCSAG_SCAN_DWELL_MS) {
            dwell_expired = true;
            /* Canal vide ? Aucun SYNC, aucun codeword valide,
             * et pas assez de transitions → on abandonne. */
            if (g.nSync == 0 && g.nOk == 0 && g.liveBits < POCSAG_LIVE_BITS_MIN) {
                g.running = false;
                break;
            }
            /* Sinon on prolonge : on garde listen_start pour le timeout total. */
        }

        if (dwell_expired && elapsed_ms >= g_listen_timeout_ms) {
            g.running = false;
            break;
        }

        /* --- Échantillonnage temps réel --- */
        while((int32_t)(pocsag_cycles_48mhz() - next) < 0){}
        next += CYC_PER_SAMPLE;
        g.n++;
        int32_t adc = pocsag_af_capture_sample();
        dc += ((adc << 4) - dc) >> 8;
        int32_t x = adc - (dc >> 4);
        sum4 += x - box4[bi & 3u];
        box4[bi & 3u] = x;
        bi = (bi + 1u) & 3u;
        int32_t before = sl[0].rprev;
        for(unsigned k = 0; k < NSL; k++) slice(&sl[k], sum4);
        /* Détection de transition sur le slicer k=0 (=1200 bps nominal) */
        if (sl[0].rprev != before) {
            if (g.liveBits < 0xFFFFu) g.liveBits++;
        }
        rateFind(sl[0].rprev != before);
        if(++g.ledCnt >= LED_STEP){
            g.ledCnt = 0;
            if(g.led){ g.led--; BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, g.led & 1u); }
        }
        if(cnt < HOUSE_EVERY){ cnt++; continue; }
        bool wait = false;
        for(unsigned k = 0; k < NSL; k++){
            unsigned w = sl[k].word;
            if(w != HUNT && (w != BATCH_WORDS || sl[k].nb >= SYNC_SLOT)) wait = true;
        }
        if(wait && ++g.busyFor < BUSY_MAX) continue;
        g.busyFor = 0;
        cnt = 0;
        /* PA4 doit rester alimenté pour le décodeur : chemin audio
         * TOUJOURS actif. Le HP est déjà muet via volume BK4819 = 0. */
        GPIO_EnableAudioPath();
        house();

        if (!g.running) break;

        if (g.count > 0) {
            g.running = false;
            break;
        }

        while((int32_t)(pocsag_cycles_48mhz() - next) >= 0){
            next += CYC_PER_SAMPLE;
            g.n++;
            for(unsigned k = 0; k < NSL; k++){
                if(sl[k].word == BATCH_WORDS) sl[k].excuse = 1;
                sl[k].phase += sl[k].step;
                if(sl[k].phase >= 65536){ sl[k].phase -= 65536; seqBit(&sl[k], 0); }
            }
        }
    }
}

/* ============================================================
 * Affichage des messages — PERMANENT jusqu'au prochain message
 * ou action utilisateur (EXIT, KEY_2).
 *
 * Backlight : rallumé à l'arrivée du message et sur chaque touche.
 * BACKLIGHT_Update() gère l'extinction auto après le timeout configuré,
 * mais l'affichage reste permanent (pas de timeout de showMessages).
 *
 * Aucune touche ne réactive le son : le scan reste muet.
 * ============================================================ */
static bool showMessages(void)
{
    g.cur = 0;
    g.top = 0;
    g.redraw = REDRAW_ON;

    BACKLIGHT_TurnOn();   /* écran allumé à l'arrivée du message */

    for (;;) {
        if (g.redraw) {
            g.redraw = 0;
            draw();
            ST7565_BlitStatusLine();
            ST7565_BlitFullScreen();
        }

        uint8_t key = KEYBOARD_GetKey();
        if (key != KEY_INVALID && key != g.prevKey) {
            g.prevKey = key;
            BACKLIGHT_TurnOn();   /* toute touche rallume l'écran */
            g.redraw |= REDRAW_ON;

            if (key == KEY_EXIT) {
                g_pocsagScanStop = true;
                return true;
            }
            else if (key == KEY_DOWN) {
                if (g.cur + 1u < g.count) { g.cur++; g.top = 0; }
            }
            else if (key == KEY_UP) {
                if (g.cur > 0) { g.cur--; g.top = 0; }
            }
            else if (key == KEY_STAR) {
                g.cw ^= CW_COMPACT;
                g.top = 0;
            }
            else if (key == KEY_2) {
                g.count = 0;
                g.cur = 0;
                g.nOk = 0;
                return false;
            }
        }
        else {
            g.prevKey = key;
        }

        /* Gestion du timeout backlight (extinction auto configurable
         * par l'utilisateur). L'affichage reste permanent, lui. */
        BACKLIGHT_Update();

        /* PAS de timeout d'affichage : on reste affiché indéfiniment */
        SYSTEM_DelayMs(20);
    }
}

/* ============================================================
 * APP_RunPocsagScan — scan OPTION B
 * ============================================================ */
void APP_RunPocsagScan(void)
{
    int8_t listIdx = pocsag_find_scanlist(POCSAG_LIST_NAME);
    if (listIdx < 0) {
        UI_DisplayClear();
        UI_PrintStringSmallBold("NO " POCSAG_LIST_NAME " LIST", 0, 0, 2);
        ST7565_BlitFullScreen();
        SYSTEM_DelayMs(1500);
        return;
    }
    pocsag_af_capture_open();
    BK4819_WriteRegister((BK4819_REGISTER_t)REG_2B,
                         (uint16_t)((BK4819_ReadRegister((BK4819_REGISTER_t)REG_2B) | 0x0700u) & ~0x0007u));
    BK4819_WriteRegister((BK4819_REGISTER_t)REG_73,
                         (uint16_t)(BK4819_ReadRegister((BK4819_REGISTER_t)REG_73) | 0x0010u));
    BK4819_SetAF(BK4819_AF_MUTE);
    BK4819_WriteRegister(REG_48,0);
    BACKLIGHT_TurnOn();

    g_cyc_base = APP_CYC_DIAGNOSTIC_START - pocsag_cycles_raw();

    for (uint16_t ch = MR_CHANNEL_FIRST; ch <= MR_CHANNEL_LAST; ch++) {
        if (g_pocsagScanStop) break;

        uint32_t freq = pocsag_channel_freq(ch);
        if (freq == 0u) continue;
        if (!pocsag_channel_in_list(ch, listIdx)) continue;

        if (freq == last_triggered_freq &&
            (gGlobalSysTickCounter - last_triggered_tick) * 10u < POCSAG_REBOND_MS)
            continue;

        BK4819_SetFrequency(freq);
        BK4819_PickRXFilterPathBasedOnFrequency(freq);
        BK4819_SetFilterBandwidth(BK4819_FILTER_BW_WIDE, false);
        BK4819_RX_TurnOn();
        SYSTEM_DelayMs(POCSAG_SETTLE_MS);

        g.current_freq = freq;
        scan_freq = freq;

        /* Timeout total (prolongation) ; la fenêtre de découverte
         * de 100 ms est gérée en interne par listen(). */
        g_listen_timeout_ms = POCSAG_LISTEN_TIMEOUT_MS;

        g.nSync = 0;
        g.count = 0;
        g.nOk = 0;

        g.redraw |= REDRAW_ON;
        house();

        g.running = true;
        listen();

        if (g_pocsagScanStop) break;

        if (g.count) {
            last_triggered_freq = freq;
            last_triggered_tick = gGlobalSysTickCounter;

            bool exit_scan = showMessages();
            if (exit_scan) {
                g_pocsagScanStop = true;
                break;
            }
            /* Après KEY_2, on reprend le scan au canal suivant */
        }
    }

    BACKLIGHT_TurnOn();
    pocsag_af_capture_close();
    BK4819_SetAF(BK4819_AF_MUTE);
    GPIO_DisableAudioPath();
    BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false);
}

#endif /* ENABLE_POCSAG */