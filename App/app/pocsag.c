/* pocsag.c — scan POCSAG résident (scanlist « POCSAG »), décodage 1200/512 bauds
 *
 * Le décodeur (filtre, 3 slicers à hystérésis, DPLL, mot de synchro dans
 * les deux polarités, BCH avec correction d'un bit, assemblage des pages) est
 * celui de l'app POCSAG RX, inchangé, étendu au 512 bauds.
 *
 * BAUD
 *  - Le baudrate est FIXÉ par la modulation du canal dans la scanlist :
 *    AM → 512 bauds, FM → 1200 bauds. Le récepteur reste toujours en NFM.
 *    Plus de détection automatique : à chaque changement de canal, on lit
 *    la modulation en flash et on applique le step/dup correspondant.
 *
 * SCAN RAPIDE PAR SEUIL RSSI
 *  - Les canaux de la scanlist « POCSAG » sont relus à la volée.
 *  - Sur chaque canal : accord, POCSAG_SETTLE_MS (5 ms) de stabilisation,
 *    puis LECTURE DU RSSI. Si RSSI <= plancher + marge, on saute
 *    immédiatement au canal suivant (aucun dwell). Sinon, on lance le
 *    dwell (40 ms si signal marginal, 100 ms si signal franc).
 *  - Le plancher de bruit est calibré sur les 16 premiers canaux du
 *    premier tour, puis figé. La marge est POCSAG_RSSI_SKIP_MARGIN (6).
 *  - Rien : canal suivant. Trouvé : on décode jusqu'à la fin de l'émission
 *    (plus aucun lot pendant POCSAG_END_MS), puis on affiche la ou les pages
 *    reçues avec la fréquence en en-tête, et le scan reprend.
 *
 * LED
 *  - Clignote toutes les 5 s en scan normal, toutes les 1 s si au moins
 *    un message reçu n'a pas été lu (hasUnread). Le clignotement est
 *    cadencé par pocsag_ticks_ms() et fonctionne aussi pendant listen()
 *    via houseLight().
 *
 * AUDIO (touche 1)
 *  - La touche 1 bascule l'écoute du son RX (plus de hold) : toggle
 *    audioOn, active/désactive le chemin audio et l'AF du BK4819.
 *  - Le décodeur lit la sortie AF du BK4819 sur PA4 ; couper l'AF coupe
 *    aussi le décodeur. On garde donc AF en FM tant que audioOn est vrai ;
 *    sinon AF_MUTE et GPIO_DisableAudioPath().
 *
 * FILTRAGE DES MESSAGES (touche 4, actif par défaut)
 *  - Quand le filtre est actif :
 *      · rejet des messages de moins de POCSAG_MIN_MSG_LEN (10) caractères
 *        (les tone-only ont len == 0 et sont donc aussi ignorés) ;
 *      · rejet des messages numériques purs (func=0 sans alpha) ;
 *      · rejet des messages dont plus de la moitié des caractères sont des
 *        GAP (2*gaps > n) ;
 *      · rejet des pages dont trop de mots BCH ont été corrigés
 *        (nBad + nFixed > 8 et nFixed*2 > nBad+nFixed) ;
 *      · rejet des messages sans aucune lettre (bruit, fausses pages).
 *  - Quand le filtre est désactivé (indicateur "RAW" en scan) :
 *      · TOUT passe, y compris les numériques purs, les messages courts,
 *        les pages bruitées et les pages sans lettres.
 *  - Numérique (Option B) : preferAlpha() est conservée pour rattraper
 *    les messages alpha émis par erreur sur func=0.
 *  - BCH : les mots corrigés (1 bit) sont comptés (nFixed) ainsi que les
 *    mots non corrigeables (nBad).
 *
 * TAILLE DES MESSAGES
 *  - MSG_MAX = 320 caractères. page_t.len, sl_t.len et sl_t.alen sont
 *    uint16_t (sinon troncature à 255).
 *
 * STOCKAGE (flash externe, 0x012000–0x017FFF, 24 576 o, 6 secteurs de 4 Ko)
 *   [0x012000, 0x012018) : en-tête (24 o)
 *                          magic + crc + pool_head + count + head
 *   [0x012018, 0x012C98) : anneau pages (200 × 16 o = 3 200 o)
 *   [0x012C98, 0x018000) : pool froid (~21 384 o, circulaire)
 *
 * AFFICHAGE
 *  - Écran avant messages : touche '*' alterne noms des canaux / fréquences.
 *  - Messages reçus : nom du canal (ou fréquence) affiché en en-tête,
 *    fréquence affichée en négatif dans la barre de statut.
 *  - En affichage message, EXIT revient à l'écran SCAN (messages conservés).
 *  - En affichage SCAN, UP/DOWN ouvre l'affichage des messages.
 *  - En affichage SCAN, EXIT quitte POCSAG.
 *
 * Touches : EXIT quitte/revient · UP/DOWN pages ou ouvre les messages ·
 *            * alterne noms/fréquences en scan · 2 efface les pages ·
 *            1 bascule l'audio RX · 4 active/désactive le filtrage.
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
#define HOUSE_EVERY    192u      /* ~20 ms à 9600 Hz : réactivité clavier */
#define BUSY_MAX       19200u
#define SYNC_SLOT      8u
#define REFRESH_SLOTS  100u
#define REDRAW_ON      1u
#define REDRAW_FRAME   2u

#define REG_2B      0x2B
#define REG_73      0x73
#define REG_48      0x48
#define REG_67      0x67    /* RSSI */

#define NSL         3u
#define STEP_1200   8192u
#define DUP_1200    128u
#define STEP_512    3495u
#define DUP_512     300u
#define PLL_SHIFT   3
#define DIRTY_WORDS 8u

#define BAUD_1200   1200u
#define BAUD_512    512u

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
#define MSG_MAX     320u       /* plafond stockage/affichage (fusion incluse) */
#define GAP         0x7Fu
#define BAD_RUN     3u

/* Seuil de rejet des messages trop courts (tone-only inclus). */
#define POCSAG_MIN_MSG_LEN  10u

/* ============================================================
 * Stockage flash (0x012000–0x017FFF, 24 576 o, 6 secteurs de 4 Ko)
 * ============================================================ */
#define POCSAG_FLASH_BASE        0x012000u
#define POCSAG_FLASH_END         0x018000u

#define MAX_MSGS                 200u

#define POCSAG_META_FLASH_ADDR   (POCSAG_FLASH_BASE)                       /* 0x012000 */
#define POCSAG_META_SIZE         16u
#define POCSAG_PAGES_FLASH_ADDR  (POCSAG_META_FLASH_ADDR + POCSAG_META_SIZE)  /* 0x012010 */

/* Layout texte */
#define NOTEXT_X        28
#define TITLE_X         2u
#define CAPS_END        127u
#define ROW_CHARS       18u
#define CW_COMPACT      14u
#define ROWS            6u

/* Répétition touches UP/DOWN, en millisecondes réelles. */
#define KEY_REPEAT_DELAY_MS   320u
#define KEY_REPEAT_PERIOD_MS   85u
#define KEY_DEBOUNCE_MAX_MS   100u  /* tolérance KEY_INVALID (rebond) */

/* LED */
#define LED_ON_SCAN_MS        400u   /* 1 s allumée en scan */
#define LED_PERIOD_SCAN_MS    2000u   /* toutes les 5 s */
#define LED_ON_UNREAD_MS       100u   /* flash court si non lu */
#define LED_PERIOD_UNREAD_MS   300u   /* toutes les 1 s */

/* ============================================================
 * Paramètres du scan
 * ============================================================ */
#define POCSAG_LIST_NAME    "POCSAG"

#define POCSAG_SETTLE_MS        5u

#define POCSAG_DWELL_MIN_MS     40u
#define POCSAG_DWELL_MAX_MS    100u

#define POCSAG_RSSI_SKIP_MARGIN   6u
#define POCSAG_RSSI_CAL_CHANNELS 16u

#define POCSAG_LOCK_GOOD    24u
#define POCSAG_LOCK_RATIO   4u
#define POCSAG_SYNC_WAIT_MS 1500u
#define POCSAG_END_MS       100u
#define POCSAG_MAX_RX_MS    2000u

#define MS_SAMPLES(ms)  ((uint32_t)(ms) * (FS / 1000u))

/* ============================================================
 * Structures
 * ============================================================ */

typedef struct {
    uint32_t ric;      /* identifiant destinataire                */
    uint32_t freq;     /* fréquence en Hz                         */
    uint16_t offset;   /* offset du texte dans le pool froid      */
    uint16_t len;      /* longueur du texte (0..MSG_MAX)          */
    uint8_t  flags;    /* bits 1:0 func | 2 alpha | 3 baud        */
    uint8_t  _pad[3];  /* réservé / alignement                    */
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

typedef struct {
    int32_t  amp;
    int32_t  rprev, phase;
    uint32_t sr;
    uint32_t inv;
    uint32_t ric;
    uint32_t at;
    uint16_t len;      /* uint16_t : MSG_MAX > 255 */
    uint8_t  num;
    uint8_t  dirty;
    uint8_t  nb;
    uint8_t  word;
    uint8_t  has;
    uint8_t  func;
    uint8_t  alpha;
    uint16_t alen;     /* uint16_t : MSG_MAX > 255 */
    uint8_t  aacc, ana, ataint, actrl;
    uint8_t  acc, na;
    uint8_t  taint;
    uint8_t  badrun;
    uint8_t  excuse;
    uint16_t ok;
    uint16_t nFixed;   /* mots corrigés (1 bit) */
    uint16_t nBad;     /* mots non corrigeables */
    uint16_t step;
    uint16_t dup;
    uint8_t  live;
    char     text[MSG_MAX];
    char     atext[MSG_MAX];
} sl_t;

typedef struct {
    uint32_t magic;      /* 4  */
    uint32_t crc;        /* 4  */
    uint16_t pool_head;  /* 2  */
    uint16_t count;      /* 2  */
    uint16_t head;       /* 2  */
    uint16_t _pad;       /* 2  = 16 */
} pocsag_hdr_t;

_Static_assert(sizeof(pocsag_hdr_t) == 16u, "hdr doit faire 16 octets");

#define POCSAG_META_MAGIC  0x504F4353u     /* "POCS" */

_Static_assert(sizeof(page_t) == 16u, "page_t doit faire 16 octets");

#define POCSAG_POOL_FLASH_ADDR  (POCSAG_PAGES_FLASH_ADDR + MAX_MSGS * (uint32_t)sizeof(page_t))
#define POCSAG_POOL_SIZE        (POCSAG_FLASH_END - POCSAG_POOL_FLASH_ADDR)

_Static_assert(POCSAG_POOL_FLASH_ADDR < POCSAG_FLASH_END,
               "le pool froid doit être non vide");
_Static_assert(POCSAG_POOL_SIZE < 65536u,
               "offset pool en uint16_t : pool < 64 Ko");

/* ============================================================
 * État global (RAM minimale)
 * ============================================================ */
static sl_t          g_sl[NSL];

static pocsag_hdr_t  g_hdr;
static bool          g_hdr_dirty;

static page_t        g_cur_page;
static bool          g_cur_valid;

static char          g_pool_ram[MSG_MAX];
static char          s_merged[MSG_MAX];
static char          s_old[MSG_MAX];

struct {
    uint8_t  prevKey, redraw;
    uint8_t  repKey;
    uint32_t repNext;
    uint32_t invalidAt;
    bool     keyDown;
    bool     running;
    uint16_t cur;
    uint8_t  cw;          /* 0 = grande police, 1 = petite police */
    uint8_t  showNames;   /* 0 = fréquences, 1 = noms des canaux en scan */
    uint8_t  refresh;
    uint8_t  top, lim;
    uint16_t nOk;
    uint8_t  led;
    uint16_t nSync;
    uint16_t busyFor;
    uint32_t n;
    uint32_t current_freq;
    uint16_t nCh, nHits;
    uint16_t lastGood, lastBad, lastQ;
    uint16_t nWords;
    uint8_t  battOk;
    uint16_t baud;
    uint16_t lastRssi;
    uint8_t  rssiFloor;
    uint16_t rssiCalCount;
    bool     rssiFloorSet;
    bool     hasUnread;      /* au moins un message non lu */
    uint32_t ledNextMs;      /* prochain toggle LED */
    bool     audioOn;        /* touche 1 : audio HP activé */
    bool     showScan;       /* true = écran SCAN, false = écran messages */
    bool     filterOn;       /* touche 4 : filtrage des messages actif */
    sl_t     *sl;
} g = { .cw = 0, .showNames = 1, .baud = BAUD_1200, .showScan = true, .filterOn = true };

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
 * Lecture modulation / bande / step canal en flash
 * ============================================================ */
static uint8_t LookupChannelModulation(uint16_t ch)
{
    uint8_t tmp;
    uint8_t data[8];
    uint32_t address = (uint32_t)ADRESS_CHANNELS + ((uint32_t)ch * 16U) + 8U;

    PY25Q16_ReadBuffer(address, data, sizeof(data));

    tmp = data[3] >> 4;
#ifdef MODULATION_UKNOWN
    if (tmp >= MODULATION_UKNOWN)
        tmp = MODULATION_FM;
#endif

    uint8_t channelBandwidth;
    if (data[4] == 0xFFu) {
        channelBandwidth = BK4819_FILTER_BW_WIDE;
    } else {
        const uint8_t d4 = data[4];
        channelBandwidth = (uint8_t)!!((d4 >> 1) & 1u);
        if (channelBandwidth != BK4819_FILTER_BW_WIDE)
            channelBandwidth = (uint8_t)(((d4 >> 5) & 3u) + 1u);
    }
    (void)channelBandwidth;

    tmp = data[6];
#ifdef STEP_N_ELEM
    if (tmp >= STEP_N_ELEM)
        tmp = STEP_12_5kHz;
#endif
    (void)tmp;

    return (uint8_t)(data[3] >> 4);
}

/* ============================================================
 * Accès flash : en-tête et pages
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

/* ============================================================
 * Accès flash : pool froid (lecture/écriture contiguës, circulaire)
 * ============================================================ */
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
static void pocsag_afc_resume_adc(void) { pocsag_afc_select_pa4(); }

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
static inline uint32_t pocsag_ticks_ms(void)
{
    return gGlobalSysTickCounter * 10u;
}

static uint16_t pocsag_read_rssi(void)
{
    return (uint16_t)(BK4819_ReadRegister((BK4819_REGISTER_t)REG_67) & 0x01FFu);
}

/* ============================================================
 * Scan (canaux de la scanlist relus à la volée)
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

static uint8_t pocsag_channel_mod_at(int8_t listIdx, uint16_t idx)
{
    uint16_t n = 0;
    for (uint16_t ch = MR_CHANNEL_FIRST; ch <= MR_CHANNEL_LAST; ch++) {
        uint32_t freq = pocsag_channel_freq(ch);
        if (freq && pocsag_channel_in_list(ch, listIdx)) {
            if (n == idx)
                return LookupChannelModulation(ch);
            n++;
        }
    }
    return MODULATION_FM;
}

/* ============================================================
 * Décodage (app POCSAG RX, inchangé + comptage BCH)
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
    while(n && (t[n - 1u] == GAP || (m->func == FUNC_NUMERIC && t[n - 1u] == ' ')))
        lost |= t[--n] == GAP;
    for(unsigned i = 0; i < n; i++) gaps += t[i] == GAP;

    if (g.filterOn) {
        /* Numérique pur sans alpha : rejeté (uniquement si filtre actif). */
        if(m->func == FUNC_NUMERIC && n > 0u && !m->alpha)
            return;

        if(n < POCSAG_MIN_MSG_LEN) return;
        if(2u * gaps > n) return;

        /* Rejet si trop de mots BCH corrigés (page douteuse). */
        if(m->nBad + m->nFixed > 8u && m->nFixed * 2u > m->nBad + m->nFixed)
            return;

        /* Rejet des messages sans aucune lettre (bruit, fausses pages). */
        {
            unsigned letters = 0;
            for (unsigned i = 0; i < n; i++) {
                unsigned ch = (uint8_t)t[i];
                if (ch == GAP) continue;
                if ((ch | 32u) - 'a' < 26u) letters++;
            }
            if (letters == 0u) return;
        }
    }

    g.redraw |= REDRAW_FRAME;

    for(unsigned j = 0; j < g_hdr.count && j < 2u; j++){
        page_t q;
        pocsag_page_load(j, &q);
        if(q.ric != m->ric || PAGE_GET_FUNC(&q) != m->func) continue;

        for(unsigned i = 0; i < n; i++)
            s_merged[i] = ((uint8_t)t[i] == GAP) ? '?' : (char)t[i];

        if(PAGE_GET_ALPHA(&q) != m->alpha){
            if(!m->alpha) return;
            page_set_alpha(&q, 1);
            q.len = (uint16_t)n;
            pocsag_pool_write(q.offset, s_merged, n);
            pocsag_page_store_phys(pocsag_phys(j), &q);
            g_hdr_dirty = true;
            pocsag_hdr_flush();
            if (j == 0) g_cur_valid = false;
            return;
        }

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
    page_set_func(&p, m->func);
    page_set_alpha(&p, m->alpha);
    page_set_baud(&p, (g.baud == BAUD_512) ? 1u : 0u);
    pocsag_page_store_phys(g_hdr.head, &p);

    g.nOk++;
    g.cur = 0;
    g.top = 0;
    g_cur_valid = false;
    g.hasUnread = true;     /* LED clignotera toutes les 1 s */
    g_hdr_dirty = true;
    pocsag_hdr_flush();
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
    if(!good){
        m->nBad++;
        m->dirty = DIRTY_WORDS;
    } else {
        if(good == 2u) m->nFixed++;
        if(m->dirty){
            m->dirty--;
            if(good == 2u) good = 0;
        }
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
    m->nFixed = m->nBad = 0;
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

static void pocsag_reset_slicer(sl_t *m)
{
    m->amp = 0; m->rprev = 0; m->phase = 0;
    m->sr = 0; m->inv = 0;
    m->nb = 0; m->word = HUNT;
    m->has = 0;
    m->acc = m->na = m->len = 0;
    m->taint = m->badrun = 0;
    m->live = 0; m->excuse = 0;
    m->alen = m->aacc = m->ana = m->ataint = m->actrl = 0;
    m->alpha = 0; m->dirty = 0;
    m->ric = 0; m->at = 0;
    m->func = 0;
    m->ok = 0;
    m->nFixed = 0; m->nBad = 0;
}

static void pocsag_apply_baud(void)
{
    for (unsigned k = 0; k < NSL; k++) {
        sl_t *m = &g_sl[k];
        m->step = (g.baud == BAUD_512) ? STEP_512 : STEP_1200;
        m->dup  = (g.baud == BAUD_512) ? DUP_512  : DUP_1200;
        pocsag_reset_slicer(m);
    }
}

/* ============================================================
 * Affichage
 * ============================================================ */
static void printFreq(char *s, uint32_t f)
{
    sprintf(s, "%u.%05u", (unsigned)(f / 100000u), (unsigned)(f % 100000u));
}

/* LED : 1 s allumée / 5 s en scan, flash 250 ms / 1 s si non lu. */
static void ledTick(void)
{
    const uint32_t now = pocsag_ticks_ms();
    if ((int32_t)(now - g.ledNextMs) < 0) return;

    if (g.led) {
        g.led = 0;
        BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false);
        const uint32_t period = g.hasUnread ? LED_PERIOD_UNREAD_MS : LED_PERIOD_SCAN_MS;
        const uint32_t on     = g.hasUnread ? LED_ON_UNREAD_MS     : LED_ON_SCAN_MS;
        g.ledNextMs = now + (period - on);
    } else {
        g.led = 1;
        BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, true);
        g.ledNextMs = now + (g.hasUnread ? LED_ON_UNREAD_MS : LED_ON_SCAN_MS);
    }
}

extern void SETTINGS_FetchChannelName(char *s, const uint16_t channel);

static void pocsag_channel_name_at(int8_t listIdx, uint16_t idx, char *dst, size_t maxLen)
{
    uint16_t n = 0;
    char buf[12];
    for (uint16_t ch = MR_CHANNEL_FIRST; ch <= MR_CHANNEL_LAST; ch++) {
        uint32_t freq = pocsag_channel_freq(ch);
        if (freq && pocsag_channel_in_list(ch, listIdx)) {
            if (n == idx) {
                SETTINGS_FetchChannelName(buf, ch);
                buf[10] = '\0';
                strncpy(dst, buf, maxLen - 1u);
                dst[maxLen - 1u] = '\0';
                return;
            }
            n++;
        }
    }
    if (maxLen) dst[0] = '\0';
}

static void pocsag_channel_label_at(int8_t listIdx, uint16_t idx, char *dst, size_t maxLen)
{
    uint16_t n = 0;
    for (uint16_t ch = MR_CHANNEL_FIRST; ch <= MR_CHANNEL_LAST; ch++) {
        uint32_t freq = pocsag_channel_freq(ch);
        if (freq && pocsag_channel_in_list(ch, listIdx)) {
            if (n == idx) {
                if (g.showNames) {
                    pocsag_channel_name_at(listIdx, idx, dst, maxLen);
                } else {
                    sprintf(dst, "%u.%05u", (unsigned)(freq / 100000u),
                            (unsigned)(freq % 100000u));
                }
                return;
            }
            n++;
        }
    }
    if (maxLen) dst[0] = '\0';
}

/* Récupère le nom du canal (ou la fréquence) associé à une fréquence donnée. */
static void pocsag_get_channel_label_for_freq(uint32_t freq, char *dst, size_t maxLen)
{
    int8_t listIdx = pocsag_find_scanlist(POCSAG_LIST_NAME);
    if (listIdx >= 0) {
        for (uint16_t ch = MR_CHANNEL_FIRST; ch <= MR_CHANNEL_LAST; ch++) {
            uint32_t f = pocsag_channel_freq(ch);
            if (f && pocsag_channel_in_list(ch, listIdx)) {
                if (f == freq) {
                    char buf[12];
                    SETTINGS_FetchChannelName(buf, ch);
                    buf[10] = '\0';
                    if (buf[0] != '\0') {
                        strncpy(dst, buf, maxLen - 1u);
                        dst[maxLen - 1u] = '\0';
                        return;
                    }
                    break;
                }
            }
        }
    }
    /* Fallback : fréquence */
    sprintf(dst, "%u.%05u", (unsigned)(freq / 100000u), (unsigned)(freq % 100000u));
}

__attribute__((noinline))
static void draw(void)
{
    char s1[6]  __attribute__((aligned(4)));
    char s2[64] __attribute__((aligned(4)));
    unsigned lim = 0;

    UI_DisplayClear();
    UI_StatusClear();

    pocsag_cur_load();
    const page_t *p = &g_cur_page;

    /* ---- Barre de statut ---- */
    if (g_hdr.count && !g.showScan) {
        /* Fréquence affichée en négatif dans la barre de statut */
        char freqStr[16];
        printFreq(freqStr, p->freq);
        unsigned nFreq = (unsigned)strlen(freqStr);
        //GUI_DisplaySmallestInverse(freqStr, 0, 0, true, true,(uint8_t)(4u * nFreq));
        UI_PrintStringSmallBold(freqStr,0,0,0xFF);
    } else if (!g.showScan) {
        char r[20];
        sprintf(r, "R%u/%u", g.lastRssi, (unsigned)g.rssiFloor + POCSAG_RSSI_SKIP_MARGIN);
        GUI_DisplaySmallest(r, 60, 0, true, true);
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
    
    //GUI_DisplaySmallestInverse(s1, 112, 0, true, true, nBatt);
    UI_PrintStringSmallBoldRight(s1, 127, 0xFF);


    /* ---- Corps : page courante ou mode scan ---- */
    if (!g.showScan && g_hdr.count) {
        /* En-tête : nom du canal (ou fréquence si nom absent) au lieu du RIC */
        char label[16];
        pocsag_get_channel_label_for_freq(p->freq, label, sizeof(label));
        UI_PrintStringSmallBold(label, 0, 0, 0);

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
    } else if (g.showScan) {
        /* Écran SCAN */
        sprintf(s2, "SCAN %uCH %s", g.nCh, g.filterOn ? "FIL" : "RAW");
        //GUI_DisplaySmallestInverse(s2, 0, 0, true, true, 4u * (unsigned)strlen(s2));
        UI_PrintStringSmallBold(s2, 0, 0, 0xFF);
        

        /* Noms des canaux ou fréquences sur 3 colonnes (x = 0, 43, 86) */
        int8_t listIdx = pocsag_find_scanlist(POCSAG_LIST_NAME);
        if (listIdx >= 0) {
            static const uint8_t colX[3] = { 0u, 43u, 86u };
            const uint16_t maxCells = 24u; /* 3 colonnes × 8 lignes */
            uint16_t startIdx = 0;

            if (g.nCh > maxCells) {
                uint16_t curPos = 0;
                for (uint16_t k = 0; k < g.nCh; k++) {
                    if (pocsag_channel_freq_at(listIdx, k) == g.current_freq) {
                        curPos = k;
                        break;
                    }
                }
                if (curPos >= maxCells / 2u) {
                    startIdx = (uint16_t)(curPos - maxCells / 2u);
                    if (startIdx + maxCells > g.nCh)
                        startIdx = (uint16_t)(g.nCh - maxCells);
                }
            }

            char label[12];
            uint16_t idx = startIdx;
            for (uint16_t cell = 0; cell < maxCells && idx < g.nCh; idx++) {
                uint32_t fr = pocsag_channel_freq_at(listIdx, idx);
                if (!fr) continue;
                pocsag_channel_label_at(listIdx, idx, label, sizeof(label));
                uint8_t col = (uint8_t)(cell / 8u);
                uint8_t row = (uint8_t)(cell % 8u);
                uint8_t y   = (uint8_t)(2u + row * 7u);
                GUI_DisplaySmallest(label, colX[col], y, false, true);
                cell++;
            }
        }
    } else {
        /* g_hdr.count == 0 : pas de messages, on force l'écran SCAN */
        g.showScan = true;
        g.redraw |= REDRAW_ON;
    }

    g.lim = (uint8_t)lim;
}

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
    g.redraw |= REDRAW_ON;

    int d = (key == KEY_DOWN) - (key == KEY_UP);

    /* ---- Touche EXIT ---- */
    if (key == KEY_EXIT) {
        if (!g.showScan) {
            /* Depuis l'affichage message : retour à l'écran SCAN */
            g.showScan = true;
            g.redraw |= REDRAW_ON;
        } else {
            /* Depuis l'écran SCAN : quitter POCSAG */
            g.running = false;
            g_pocsagScanStop = true;
        }
        return;
    }

    /* ---- Touche UP/DOWN ---- */
    if (d) {
        if (g.showScan) {
            /* En mode SCAN, UP/DOWN ouvre l'affichage des messages */
            if (g_hdr.count) {
                g.showScan = false;
                g.hasUnread = false;
                g.redraw |= REDRAW_ON;
            }
        } else {
            /* En mode messages, UP/DOWN navigue */
            if (!g_hdr.count) return;
            g.hasUnread = false;     /* navigation => messages lus */
            int nextTop = (int)g.top + d;
            if (nextTop >= 0 && nextTop <= (int)g.lim) {
                g.top = (uint8_t)nextTop;
            } else {
                int count = (int)g_hdr.count;
                int nc = ((int)g.cur + d) % count;
                if (nc < 0) nc += count;
                g.cur = (uint16_t)nc;
                g.top = 0;
                g_cur_valid = false;
            }
        }
        return;
    }

    /* ---- Touche 2 : effacer les pages ---- */
    if (key == KEY_2) {
        g.cur = 0; g.nOk = 0; g.top = 0;
        g.nSync = 0; g.nHits = 0; g.nWords = 0;
        for (unsigned k = 0; k < NSL; k++) g_sl[k].ok = 0;
        g.hasUnread = false;
        pocsag_flash_wipe();
        g.showScan = true;
        g.redraw |= REDRAW_ON;
        return;
    }

    /* ---- Touche * ---- */
    if (key == KEY_STAR) {
        if (g.showScan) {
            g.showNames ^= 1u;
        } else {
            g.cw ^= 1u;
            g.top = 0;
            g_cur_valid = false;
        }
        g.redraw |= REDRAW_ON;
        return;
    }

    /* ---- Touche 1 : audio ---- */
    if (key == KEY_1) {
        g.audioOn ^= 1u;
        if (g.audioOn) {
            GPIO_EnableAudioPath();
            BK4819_SetAF(BK4819_AF_FM);
        } else {
            GPIO_DisableAudioPath();
            BK4819_SetAF(BK4819_AF_MUTE);
        }
        g.redraw |= REDRAW_ON;
        return;
    }

    /* ---- Touche 4 : filtrage des messages ---- */
    if (key == KEY_4) {
        g.filterOn ^= 1u;
        g.redraw |= REDRAW_ON;
        return;
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
    ledTick();
    backlightTick();
}

static void house(void){
    handleKeys();
    ledTick();
    if (g_pocsagScanStop) return;
    if (g.redraw & REDRAW_FRAME) BACKLIGHT_TurnOn();
    backlightTick();
    if (++g.refresh >= REFRESH_SLOTS) g.redraw |= REDRAW_ON;
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
    for (unsigned k = 0; k < NSL; k++) {
        sl_t *m = &sl[k];
        m->num = (uint8_t)(2u + k);
        m->word = HUNT;
        m->step = (g.baud == BAUD_512) ? STEP_512 : STEP_1200;
        m->dup  = (g.baud == BAUD_512) ? DUP_512  : DUP_1200;
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

    g.busyFor = 0;

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

        if (cnt < HOUSE_EVERY) {
            cnt++;
        } else {
            cnt = 0;
            houseLight();
            if (g_pocsagScanStop) break;
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
            } else if (elapsed >= MS_SAMPLES(dwell_ms)) {
                g.lastGood = good; g.lastBad = bad; g.lastQ = q;
                return;
            }
            continue;
        }
        if (g.nSync == sync0) {
            if (elapsed >= MS_SAMPLES(POCSAG_SYNC_WAIT_MS)) break;
        } else if (!inBatch && g.n - lastBatch >= MS_SAMPLES(POCSAG_END_MS)) {
            break;
        }
        if (elapsed >= MS_SAMPLES(POCSAG_MAX_RX_MS)) break;

        if (cnt == 0) {
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
    }
    for (unsigned k = 0; k < NSL; k++) {
        closePage(&sl[k]);
        g.nWords += sl[k].ok;
    }
}

/* ============================================================
 * Accord rapide
 * ============================================================ */
static uint16_t g_rx30;

static void pocsag_tune(uint32_t f)
{
    BK4819_SetFrequency(f);
    BK4819_PickRXFilterPathBasedOnFrequency(f);
    BK4819_WriteRegister(BK4819_REG_30, 0x200);
    BK4819_WriteRegister(BK4819_REG_30, g_rx30);
}

/* ============================================================
 * APP_RunPocsagScan
 * ============================================================ */
void APP_RunPocsagScan(void)
{
    int8_t listIdx = pocsag_find_scanlist(POCSAG_LIST_NAME);
    if (listIdx < 0) {
        UI_DisplayClear();
        UI_PrintStringSmallBold("NO " POCSAG_LIST_NAME " LIST", 0, 0, 2);
        ST7565_BlitFullScreen();
        SYSTEM_DelayMs(1500);
        g_pocsagScanStop = true;
        return;
    }

    pocsag_hdr_load();
    g.nCh = pocsag_load_channels(listIdx);
    if (!g.nCh) {
        UI_DisplayClear();
        UI_PrintStringSmallBold("NO CH IN LIST", 0, 0, 2);
        ST7565_BlitFullScreen();
        SYSTEM_DelayMs(1500);
        g_pocsagScanStop = true;
        return;
    }

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
    g.redraw = REDRAW_ON;

    g.baud = BAUD_1200;
    g.rssiFloor = 0;
    g.rssiCalCount = 0;
    g.rssiFloorSet = false;
    g.lastRssi = 0;
    g.repKey = 0;
    g.repNext = 0;
    g.invalidAt = 0;
    g.keyDown = false;
    g.hasUnread = false;
    g.audioOn = false;
    g.filterOn = true;   /* filtrage actif par défaut */
    g.led = 0;
    g.ledNextMs = pocsag_ticks_ms() + LED_ON_SCAN_MS;
    g.showScan = true;   /* démarrer sur l'écran SCAN */

    uint32_t tuned = first_freq;
    SYSTEM_DelayMs(POCSAG_SETTLE_MS);
    for (uint16_t i = 0; !g_pocsagScanStop; i = (uint16_t)((i + 1u) % g.nCh)) {
        g.nCh = pocsag_load_channels(listIdx);
        if (!g.nCh) { i = 0; SYSTEM_DelayMs(50); continue; }
        if (i >= g.nCh) i = 0;

        uint32_t freq = pocsag_channel_freq_at(listIdx, i);
        if (!freq) continue;

        /* Baud fixé par la modulation du canal (AM=512, FM=1200). */
        uint8_t mod = pocsag_channel_mod_at(listIdx, i);
        g.baud = (mod == MODULATION_AM) ? BAUD_512 : BAUD_1200;
        pocsag_apply_baud();

        g.current_freq = freq;
        if (g.current_freq != tuned) {
            tuned = g.current_freq;
            pocsag_tune(tuned);
            SYSTEM_DelayMs(POCSAG_SETTLE_MS);
        }

        const uint16_t rssi = pocsag_read_rssi();
        g.lastRssi = rssi;

        if (g.rssiCalCount < POCSAG_RSSI_CAL_CHANNELS) {
            if (g.rssiCalCount == 0 || rssi < g.rssiFloor)
                g.rssiFloor = (uint8_t)rssi;
            g.rssiCalCount++;
            g.rssiFloorSet = true;
        }

        if (g.rssiFloorSet &&
            rssi <= (uint16_t)g.rssiFloor + POCSAG_RSSI_SKIP_MARGIN) {
            g.redraw |= REDRAW_ON;
            continue;
        }

        uint32_t dwell = POCSAG_DWELL_MIN_MS;
        if (rssi > (uint16_t)g.rssiFloor + 2u * POCSAG_RSSI_SKIP_MARGIN)
            dwell = POCSAG_DWELL_MAX_MS;

        const uint16_t before = g.nOk;
        listen(dwell);
        if (g.nOk != before) {
            g.cur = 0; g.top = 0;
            g_cur_valid = false;
            g.led = 0;
            g.showScan = false;  /* afficher le message reçu */
            g.redraw |= REDRAW_FRAME;
        }
        house();
    }

    if (!g.audioOn) GPIO_DisableAudioPath();
    BK4819_SetAF(g.audioOn ? BK4819_AF_FM : BK4819_AF_MUTE);
    pocsag_af_capture_close();
    BK4819_WriteRegister((BK4819_REGISTER_t)REG_2B, r2b);
    BK4819_WriteRegister((BK4819_REGISTER_t)REG_73, r73);
    BK4819_WriteRegister((BK4819_REGISTER_t)REG_48, r48);
    BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false);
    BACKLIGHT_TurnOn();
}

#endif /* ENABLE_POCSAG */