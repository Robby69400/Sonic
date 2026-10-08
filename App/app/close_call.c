//Close Call
/* Copyright 2026 Robby
 * https://github.com/Robby69400
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 */

// Dedicated Close Call screen.
//
// The BK4819 "frequency scan" block is a direct RF frequency counter at the
// LNAIN pin (not an RSSI sweep). It reports the actual frequency of a strong
// nearby transmitter (roughly > -40 dBm) with 10 Hz resolution. Close Call
// arms that counter, waits for a capture, then re-tunes to the reported spot
// and verifies it with a real RSSI read before opening the audio. This mirrors
// the Uniden analog "Close Call" behaviour: grab a strong nearby signal and
// hold on it.

#include "close_call.h"

#include <string.h>
#include <stdio.h>

#ifdef ENABLE_AM_FIX
#include "driver/am_fix.h"
#endif
#include "driver/backlight.h"
#include "driver/bk4819.h"
#include "driver/gpio.h"
#include "driver/keyboard.h"
#include "driver/py25q16.h"
#include "driver/st7565.h"
#include "driver/system.h"
#include "driver/systick.h"
#include "dcs.h"
#include "external/printf/printf.h"
#include "functions.h"
#include "misc.h"
#include "radio.h"
#include "settings.h"
#include "ui/helper.h"
#include "ui/inputbox.h"
#include "ui/main.h"
#include "ui/ui.h"

// Right-aligned small-bold text helper (the F4HWN tree only ships the
// left/centered variants).
static void CC_PrintSmallBoldRight(const char *pString, uint8_t End)
{
    uint8_t len = (uint8_t)strlen(pString);
    const uint8_t charWidth = 6;
    uint8_t width = (uint8_t)(len * (charWidth + 1));
    uint8_t start = (End > width) ? (uint8_t)(End - width) : 0;
    UI_PrintStringSmallBold(pString, start, End, 0);
}

// Arm the BK4819 frequency-scan integration gate. The F4HWN driver only exposes
// a fixed 0.2 s scan, so write REG_32 directly: <15:14> gate (0=0.2 s, 1=0.4 s,
// 2=0.8 s, 3=1.6 s), <13:1> fixed window, <0> enable.
static void CC_SetFrequencyScanGate(uint8_t gate)
{
    BK4819_WriteRegister(BK4819_REG_32,
        ((uint16_t)(gate & 0x3u) << 14) | (290u << 1) | 1u);
}

// Persisted Close Call settings ride in spare EEPROM bytes so the layout (and
// everyone's existing settings) is untouched. field77 = threshold+120, field78
// = preset. Defined after the state block below.

// ============================================================
// Configuration
// ============================================================
#define CC_MIN_LISTEN_MS        500     // minimum hold once locked
#define CC_MAX_LISTEN_MS        5000    // never stay locked longer than this
#define CC_UNLOCK_FADE          5       // weak 20 ms slices before releasing
#define CC_RELEASE_DWELL_MS     700     // carrier must be gone this long before release
#define CC_RELEASE_MARGIN       6       // dBm below threshold counts as "gone"
#define CC_SETTLE_US            4000    // re-tune settle before verifying RSSI
#define CC_SHOWN_TIMEOUT_MS     5000    // blank the shown freq if no fresh hit for this long
#define CC_MAX_BLOCKED          32      // temporary skip list
#define CC_MAX_DISCOVERY        48      // newest-first discovery history
#define CC_CONSISTENT_HITS      1       // capture count required before locking
#define CC_CONSISTENT_TOL       100     // 100 * 10 Hz = 1 kHz
#define CC_FM_LO                8750000u// 87.5 MHz (10 Hz units)
#define CC_FM_HI                10800000u//108.0 MHz (10 Hz units)
#define CC_MAX_GLITCH           255     // glitch gate effectively disabled (accept hits)
#define CC_DBM_MIN              (-120)  // limits of the user signal window
#define CC_DBM_MAX              (-20)
#define CC_DBM_DEFAULT          (-78)   // factory threshold (no stored setting)
#define CC_DBM_STEP             2       // dBm per UP/DOWN press
#define CC_SQUELCH_HYST         3       // dBm of close/open hysteresis
#define CC_KEY_REPEAT_DELAY     15      // 20 ms ticks before UP/DOWN auto-repeat (300 ms)
#define CC_KEY_REPEAT_INTERVAL  6       // 20 ms ticks between repeats (120 ms)
#define CC_VERIFY_COOLDOWN_MS   40      // pause after a failed verify before retrying
    #define CC_VERIFY_MARGIN        20      // REG_67 LSB (0.5 dB each): candidates
                                                // down to threshold-10 dBm are
                                                // fine-tuned and judged (user request)
                                        // below the user floor before rejecting
#define CC_ARM_SETTLE_MS_FAST   260     // cold window after arming the fast 0.2 s gate
#define CC_ARM_SETTLE_MS_SLOW   460     // cold window after arming the slow 0.4 s gate
#define CC_ARM_SETTLE_MS_MAX    1800    // cold window after arming the longest 1.6 s gate
#define CC_SKIP_QUIET_MS        1500    // suppress a SKIPped freq so a live carrier
                                        // does not immediately re-lock
#define CC_SILENT_RELEASE_MS    2000    // inaudible hold: release and keep scanning
#define CC_WIDE_SEARCH_SPAN     2000    // fine peak below floor: probe +/-200 kHz
#define CC_WIDE_SEARCH_STEP     50      // wide-search step: 5 kHz (10 Hz units)
#define CC_REFINE_SPAN          600     // centring pass: +/-6 kHz around the candidate
#define CC_REFINE_STEP          20      // centring pass step: 200 Hz -> <=100 Hz residual
#define CC_FAIL_PATH_TOGGLE     2       // consecutive verify fails -> switch LNA path
#define CC_FAIL_AUTOBLOCK       3       // consecutive verify fails -> auto-block
#define CC_CAP_ESCALATE_MS      5000    // 2nd cap of same freq: longer suppression
#define CC_CAP_FINAL_MS         15000   // 3rd cap of same freq: longest suppression
#define CC_CAP_AUTOBLOCK        4       // 4th cap of same freq -> auto-block
#define CC_DEFER_FRESH_MS       1500    // strongest-first candidate stays fresh
#define CC_DEFER_MARGIN         12      // 6 dBm advantage needed to defer a weaker hit
#define CC_FAIL_LEDGER_N        6       // per-freq verify-fail ledger entries
#define CC_FAIL_LEDGER_MS       10000   // ledger entry lifetime (ms)
#define CC_SKIP_ESCALATE_MS     5000    // 2nd consecutive SKIP of same freq
#define CC_SKIP_FINAL_MS        15000   // 3rd+ consecutive SKIP of same freq
#define CC_PATH_DWELL_MS        3000    // max ms on one LNA path while scanning

// ============================================================
// Tone history (Uniden-style): record sustained, tone-qualified hits
// ============================================================
// Close Call qualifies a hit only if the signal stays present for at least this
// long, so the log holds real "someone was talking" catches, not noise/spurs.
#define CC_LOG_TIME_MS          3000    // signal must be continuously present this long
#define CC_LOG_MAX              100     // ring buffer, newest first (bounded flash)
#define CC_LOG_FLASH_ADDR       0x00E000u // free 4 KiB sector above the settings block
#define CC_LOG_HDR_ADDR         0x00F000u // separate free 4 KiB sector: header journal
#define CC_LOG_HDR_REC          8         // bytes per appended header record
#define CC_LOG_HDR_MAX          (0x1000u / CC_LOG_HDR_REC)
#define CC_LOG_FLASH_MAGIC      0x314C4343u // "CCL1"
#define CC_LOG_TONE_NONE        0
#define CC_LOG_TONE_CTCSS       1
#define CC_LOG_TONE_DCS         2

// One stored entry. Kept tiny (16 B) so 100 entries fit in one 1600-byte block.
typedef struct __attribute__((packed)) {
    uint32_t freq;      // caught RX frequency (10 Hz units)
    uint32_t talkback;  // repeater input / talk-back frequency (10 Hz units)
    uint16_t rssi;      // REG_67 units at capture
    uint8_t  toneType;  // CC_LOG_TONE_*
    uint8_t  toneCode;  // CTCSS index (1..50) or DCS index (0..103)
    uint8_t  shiftDir;  // 0 none, 1 plus, 2 minus
    uint8_t  reserved;  // pad to 16 B
} cc_log_entry_t;

// Repeater input offset applied to derive the talk-back frequency (Uniden-style
// standard band splits). Values are 10 Hz units; 0 = no split for that band.
typedef struct {
    uint32_t lo;        // inclusive lower bound (10 Hz units)
    uint32_t hi;        // inclusive upper bound (10 Hz units)
    uint32_t offset;    // split magnitude
    uint8_t  dir;       // 1 = +offset, 2 = -offset
} cc_repeater_shift_t;

static const cc_repeater_shift_t ccRepeaterShifts[] = {
    {  2800000u,  2970000u,  100000u, 2 },  // 10 m:   -1.0 MHz
    {  5000000u,  5400000u,  100000u, 2 },  // 6 m:    -1.0 MHz
    { 14400000u, 14800000u,   60000u, 2 },  // 2 m:    -600 kHz
    { 22200000u, 22500000u,  160000u, 2 },  // 1.25 m: -1.6 MHz
    { 43000000u, 45000000u,  500000u, 2 },  // 70 cm:  -5.0 MHz
};
#define CC_REPEATER_SHIFT_COUNT (sizeof(ccRepeaterShifts) / sizeof(ccRepeaterShifts[0]))

// BK4819 RSSI register (REG_67) is 0.5 dB/LSB and dBm = rssi/2 - 160, so
// rssi = (dBm + 160) * 2. REG_67 is 9-bit (0..511). The hardware squelch
// thresholds are 8-bit, so they are additionally clamped to 255.
static inline uint16_t CC_Dbm2Rssi(int8_t dbm)
{
    int16_t v = ((int16_t)dbm + 160) * 2;
    if (v < 0) v = 0;
    if (v > 511) v = 511;
    return (uint16_t)v;
}

static inline uint8_t CC_Dbm2Rssi8(int8_t dbm)
{
    uint16_t v = CC_Dbm2Rssi(dbm);
    return (v > 255) ? 255 : (uint8_t)v;
}

// Preset frequency ranges, in 10 Hz units. Sentinel frequencies select the
// front-end LNA path (VHF < 28 MHz, UHF otherwise). Presets whose name is
// marked "spans both" alternate the LNA path on each sweep.
enum {
    CC_PRESET_AIR = 0,
    CC_PRESET_VHF,
    CC_PRESET_2M,
    CC_PRESET_MARINE,
    CC_PRESET_MILAIR,
    CC_PRESET_UHF,
    CC_PRESET_SATCOM,
    CC_PRESET_VUHF,
    CC_PRESET_ALL,
    CC_PRESET_10M,
    CC_PRESET_6M,
    CC_PRESET_COUNT
};

typedef struct {
    uint32_t start;
    uint32_t stop;
    uint32_t start2;    // optional second sub-range (0 = unused)
    uint32_t stop2;
    const char *name;
} cc_preset_t;

static const cc_preset_t ccPresets[CC_PRESET_COUNT] = {
    { 11800000u, 13700000u, 0u,         0u,         "AIR"     },   // 118-137 MHz civil air
    { 13700000u, 17400000u, 0u,         0u,         "VHF"     },   // 137-174 MHz VHF
    { 14400000u, 14800000u, 0u,         0u,         "2M"      },   // 144-148 MHz 2m amateur
    { 15600000u, 16300000u, 0u,         0u,         "MARINE"  },   // 156-163 MHz maritime VHF
    { 22500000u, 40000000u, 0u,         0u,         "MIL-AIR" },   // 225-400 MHz military air (UHF)
    { 40000000u, 47000000u, 0u,         0u,         "UHF"     },   // 400-470 MHz UHF
    { 24000000u, 27000000u, 0u,         0u,         "SATCOM"  },   // 240-270 MHz SATCOM
    { 11800000u, 17400000u, 40000000u,  47000000u,  "VHF+UHF" },   // 118-174 and 400-470 MHz
    { 11800000u, 47000000u, 0u,         0u,         "ALL"     },   // 118-470 MHz, alternates VHF/UHF
    { 28000000u, 29700000u, 0u,         0u,         "10M"     },   // 28.0-29.7 MHz 10m amateur
    { 50000000u, 54000000u, 0u,         0u,         "6M"      },   // 50-54 MHz 6m amateur
};

// Presets that span the VHF/UHF LNA split and therefore alternate the front-end
// path each sweep.
static bool CC_PresetSpansBoth(uint8_t p)
{
    return (p == CC_PRESET_VUHF || p == CC_PRESET_ALL);
}

// True when a captured frequency is inside a preset's range(s). VHF+UHF has two
// disjoint sub-ranges so the gap between them is never scanned or accepted.
static bool CC_PresetInRange(uint8_t p, uint32_t f)
{
    if (f >= ccPresets[p].start && f <= ccPresets[p].stop) return true;
    if (ccPresets[p].start2 != 0u && f >= ccPresets[p].start2 && f <= ccPresets[p].stop2) return true;
    return false;
}

// Counter integration gate per preset. The BK4819 frequency counter is a
// strong-signal detector, so faint bands need a longer gate to accumulate enough
// energy to be counted. Airband AM carriers are the weakest and most marginal,
// so AIR uses the longest 1.6 s gate; SATCOM uses 0.4 s. VHF/UHF and the wide
// presets keep the fast 0.2 s gate.
static uint8_t CC_PresetSlowGate(uint8_t p)
{
    if (p == CC_PRESET_AIR) return 3u;                              // 1.6 s
    if (p == CC_PRESET_SATCOM) return 1u;                           // 0.4 s
    return 0u;                                                      // 0.2 s
}

// Front-end LNA path sentinels. Frequencies are in 10 Hz units, matching the
// BK4819 driver and spectrum code. The driver switches to the UHF LNA at
// 28000000 (280 MHz in 10 Hz units); below that it selects the VHF LNA.
// (Mirrors spectrum.c which uses 10000000 for VHF and 40000000 for UHF.)
#define CC_SENTINEL_VHF         10000000u   // 100 MHz -> below 280 MHz -> VHF LNA
#define CC_SENTINEL_UHF         40000000u   // 400 MHz -> at/above 280 MHz -> UHF LNA

// ============================================================
// State
// ============================================================
static uint8_t  ccPreset      = CC_PRESET_VHF;
static bool     ccRunning     = false;
static bool     ccFirstUHF    = false;  // when a preset spans both paths

static uint32_t ccFoundFreq   = 0;      // currently held frequency
static uint32_t ccKnownGood   = 0;      // last verified frequency
static uint16_t ccRssi        = 0;
static int8_t   ccThresholdDbm = CC_DBM_DEFAULT;  // ignore signals weaker than this
static uint16_t ccSquelchRssi = 0;      // ccThresholdDbm as REG_67 units (9-bit)
static bool     ccLocked      = false;
static uint16_t ccLockMs      = 0;      // how long we have been holding
static uint8_t  ccWeakSlices  = 0;
static bool     ccPaused      = false;  // user pressed UP/DOWN to browse

static uint32_t ccBlocked[CC_MAX_BLOCKED];
static uint8_t  ccBlockedCount = 0;

static uint32_t ccDiscovery[CC_MAX_DISCOVERY];
static uint16_t ccDiscoveryRssi[CC_MAX_DISCOVERY];  // REG_67 units at capture
static uint8_t  ccDiscoveryCount = 0;
static uint8_t  ccDiscoveryView  = 0;   // 0 = live, else 1..count

static uint8_t  ccLastHitCount = 0;
static uint32_t ccLastHitFreq  = 0;
static uint32_t ccSeenFreq     = 0;     // last freq detected by the counter but
                                        // NOT strong enough to tune/listen to
static uint16_t ccGoneMs       = 0;     // sustained signal-absent time while locked
static uint16_t ccAgeMs        = 0;     // time since the last counter hit / lock;
                                        // the shown frequency is blanked once stale
static uint16_t ccThrHintMs    = 0;     // transient "SQL -xx dBm" overlay timer
static const char *ccHintMsg   = 0;     // transient message shown on row 6 (SKIP/BLOCK)
static uint16_t ccHintMs       = 0;     // transient message timer
static char     ccTuneMsg[20]  = "";    // live fine-tune readout ("TUNE +2.0k -76dBm")
static uint16_t ccSilentMs     = 0;     // sustained inaudible time while locked
static uint16_t ccLiveRssi     = 0;     // raw REG_67 sample for the live dBm row
static uint16_t ccCooldownMs   = 0;     // pause after a failed verify (anti-retry-storm)
static uint16_t ccArmSettleMs  = 0;     // block counter reads until the gate has re-integrated
static uint32_t ccSkipFreq     = 0;     // last SKIPped frequency
static uint16_t ccSkipMs       = 0;     // SKIP suppression timer (lets scanning move on)
static uint16_t ccPathDwellMs  = 0;     // time on the current LNA path while scanning
static uint32_t ccCapFreq      = 0;     // last hard-cap/silence-released frequency
static uint8_t  ccCapStreak    = 0;     // consecutive caps of that frequency
static uint8_t  ccStaleFires   = 0;     // consecutive stale-latch watchdog fires
static uint8_t  ccSkipStreak   = 0;     // consecutive SKIPs of the same frequency
static uint8_t  ccStaleMatchStreak = 0; // consecutive reads matching the stale latch
static struct {
    uint32_t freq;                       // verify-failed frequency (10 Hz units)
    uint8_t  count;                      // fails on that frequency
    uint16_t ms;                         // entry lifetime (0 = dead)
} ccFailLedger[CC_FAIL_LEDGER_N];
static uint32_t ccDefFreq      = 0;     // strongest-first deferred candidate
static uint16_t ccDefRssi      = 0;     // its measured RSSI
static uint16_t ccDefMs        = 0;     // freshness timer

// The hardware counter latches its last completed capture in REG_0D/0E and has
// no "consumed" flag, so right after (re)arming the counter (and on first open,
// when the registers still hold power-on garbage or the previous session's
// capture) BK4819_GetFrequencyScanResult() keeps returning that OLD value
// forever. Re-reading it every loop either freezes the display on a dead
// "frequency" (it may even pass the threshold and lock) or keeps failing the
// threshold with the cooldown re-reading the same latch - the user sees the
// scan as completely stuck until a SKIP re-arms the counter.
// Snapshot the stale value at (re)arm time and ignore any read that matches it;
// once a different (fresh) capture shows up the block lifts automatically.
static uint32_t ccStaleLatchFreq  = 0;  // REG_0D/0E value latched before re-arm
static bool     ccStaleLatchValid = false;
static uint16_t ccStaleLatchAge   = 0;  // loop iterations spent ignoring it
#define CC_STALE_LATCH_REARM   100      // ~2 s of 20 ms loop ticks: re-capture

// Stay-on-screen hold: a verified signal is programmed into the RX VFO (RAM)
// and the full normal-receive chain (modulation/AM profile/AGC/gain - the same
// mechanics the MAIN screen uses) is opened right here on the CC screen. The
// loop never exits for a lock; scanning resumes via CC_Release()/CC_ArmScan()
// once the carrier is sustainedly gone. Flash is only written once, on exit,
// so a power loss mid-hold simply loses the tune.
static bool ccTuned = false;    // a verified signal was programmed into the VFO

// Fine-tune while locked: when the held carrier fades toward the user's
// threshold, re-sweep the verify probe offsets around the tuned frequency and
// slide to the strongest reading (catches drifting/off-center captures without
// leaving the CC screen). Engagement margin is -10 dBm from the squelch floor
// (user request), re-swept at most once per second while weak.
#define CC_FINETUNE_MARGIN    20        // 10 dBm below the floor (0.5 dB/LSB)
#define CC_FINETUNE_RETRY_MS  1000
static uint16_t ccFineTuneMs  = 0;

// Backlight dim: 4 levels cycled with the F key, plus a two-stage auto-dim
// (full -> half after CC_DIM1_MS idle, half -> off after another CC_DIM2_MS).
// Level 3 keeps the screen bright all the time and never auto-dims.
#define CC_HALF_BRIGHTNESS  5           // index into value[11]; value[5] = 48
#define CC_DIM1_MS          30000u      // full -> half after 30 s idle
#define CC_DIM2_MS          30000u      // half -> off after another 30 s idle
static uint8_t  ccDimLevel = 0;         // 0 = full, 1 = half, 2 = off, 3 = always-on
static uint32_t ccDimMs    = 0;         // idle accumulator

// Band picker (opened with MENU): a full-screen scrollable list of bands.
static bool     ccMenuOpen = false;
static uint8_t  ccMenuSel  = 0;         // highlighted band index
#define CC_MENU_VISIBLE     6           // content rows 1..6 below the title (fb rows 0..6)

// Tone history (Uniden-style hit log): a flash-backed circular ring of
// qualified hits. Entries live ONLY in flash (RAM is nearly full); adding a hit
// writes just the 8-byte header and one 16-byte slot, so no large buffer is
// needed. ccLogHead is the slot holding the newest entry.
static uint8_t  ccLogCount   = 0;       // valid entries (0..CC_LOG_MAX)
static uint8_t  ccLogHead    = 0;       // flash slot of the newest entry
static bool     ccLogView    = false;   // true while the history list is on screen
static uint8_t  ccLogSel     = 0;       // highlighted entry (0 = newest)
static uint16_t ccLogHdrNext = 0;       // next free slot in the header journal

// Qualifying timer for the current lock: a hit is logged only once the signal
// has been continuously present for CC_LOG_TIME_MS.
static uint16_t ccQualifyMs  = 0;
static bool     ccLogged     = false;   // this lock already produced a log entry

// Tone detected on the current lock (filled by the CTCSS/DCS scan).
static uint8_t  ccToneType   = CC_LOG_TONE_NONE;
static uint8_t  ccToneCode   = 0;
static uint16_t ccTonePeekMs = 0;       // remaining settle before reading the tone

// ============================================================
// Helpers
// ============================================================
static bool CC_IsAirband(void);
static void CC_Render(void);
static void CC_SquelchFromConfig(void);
static void CC_ApplyDim(void);

static void CC_LoadPersisted(void)
{
    if (gEeprom.field8_0xb < CC_PRESET_COUNT) {
        ccPreset = gEeprom.field8_0xb;
    }
    // Stored value = threshold + 120. The old default (-120) is the only
    // setting that stores 0, which is indistinguishable from "never set";
    // treat field7 == 0 as unset and fall back to the factory default.
    if (gEeprom.field7_0xa == 0) {
        ccThresholdDbm = CC_DBM_DEFAULT;
    } else {
        int16_t dbm = (int16_t)gEeprom.field7_0xa - 120;
        if (dbm < CC_DBM_MIN) dbm = CC_DBM_MIN;
        if (dbm > CC_DBM_MAX) dbm = CC_DBM_MAX;
        ccThresholdDbm = (int8_t)dbm;
    }
}

static void CC_SavePersisted(void)
{
    gEeprom.field7_0xa = (uint8_t)(ccThresholdDbm + 120);
    gEeprom.field8_0xb = ccPreset;
}

static bool CC_IsBlocked(uint32_t f)
{
    for (uint8_t i = 0; i < ccBlockedCount; i++) {
        int32_t d = (int32_t)ccBlocked[i] - (int32_t)f;
        if (d < 0) d = -d;
        if (d <= CC_CONSISTENT_TOL) return true;    // tolerant: counter jitters
    }
    return false;
}

static void CC_Block(uint32_t f)
{
    if (CC_IsBlocked(f)) return;
    if (ccBlockedCount < CC_MAX_BLOCKED) {
        ccBlocked[ccBlockedCount++] = f;
    } else {
        // Shift out the oldest entry to keep accepting new blocks.
        memmove(&ccBlocked[0], &ccBlocked[1], (CC_MAX_BLOCKED - 1) * sizeof(ccBlocked[0]));
        ccBlocked[CC_MAX_BLOCKED - 1] = f;
    }
}

static void CC_Discover(uint32_t f, uint16_t rssi)
{
    // Newest first, no duplicates. The capture RSSI (REG_67 units) is stored
    // alongside so the screen can show how strong the signal was.
    for (uint8_t i = 0; i < ccDiscoveryCount; i++) {
        if (ccDiscovery[i] == f) {
            // Move to front and refresh the RSSI.
            for (uint8_t j = i; j > 0; j--) {
                ccDiscovery[j]     = ccDiscovery[j - 1];
                ccDiscoveryRssi[j] = ccDiscoveryRssi[j - 1];
            }
            ccDiscovery[0]     = f;
            ccDiscoveryRssi[0] = rssi;
            ccDiscoveryView    = 0;
            return;
        }
    }
    if (ccDiscoveryCount < CC_MAX_DISCOVERY) ccDiscoveryCount++;
    for (uint8_t j = ccDiscoveryCount - 1; j > 0; j--) {
        ccDiscovery[j]     = ccDiscovery[j - 1];
        ccDiscoveryRssi[j] = ccDiscoveryRssi[j - 1];
    }
    ccDiscovery[0]     = f;
    ccDiscoveryRssi[0] = rssi;
    ccDiscoveryView    = 0;
}

static bool CC_IsFmBroadcast(uint32_t f)
{
    return (f >= CC_FM_LO && f <= CC_FM_HI);
}

// ============================================================
// Tone history: repeater offset, persistence, tone scan
// ============================================================

// Derive the talk-back (repeater input) frequency from the caught RX frequency
// using standard band splits, exactly like a Uniden scanner's automatic offset.
// Sets *pDir to the shift direction used (0 = none).
static uint32_t CC_TalkbackFreq(uint32_t f, uint8_t *pDir)
{
    for (uint8_t i = 0; i < CC_REPEATER_SHIFT_COUNT; i++) {
        const cc_repeater_shift_t *s = &ccRepeaterShifts[i];
        if (f >= s->lo && f <= s->hi) {
            if (pDir) *pDir = s->dir;
            return (s->dir == 1) ? (f + s->offset) : (f - s->offset);
        }
    }
    if (pDir) *pDir = 0;
    return f;
}

// Append the 8-byte ring header (magic, count, head) to the journal. The header
// lives in its own sector so appending never erases the entry sector; the sector
// is only erased once the journal is full, which is far rarer than a hit.
static void CC_LogWriteHeader(void)
{
    uint8_t hdr[8];
    hdr[0] = (uint8_t)(CC_LOG_FLASH_MAGIC & 0xFF);
    hdr[1] = (uint8_t)((CC_LOG_FLASH_MAGIC >> 8) & 0xFF);
    hdr[2] = (uint8_t)((CC_LOG_FLASH_MAGIC >> 16) & 0xFF);
    hdr[3] = (uint8_t)((CC_LOG_FLASH_MAGIC >> 24) & 0xFF);
    hdr[4] = ccLogCount;
    hdr[5] = ccLogHead;
    hdr[6] = 0;
    hdr[7] = 0;
    if (ccLogHdrNext >= CC_LOG_HDR_MAX) {
        PY25Q16_SectorErase(CC_LOG_HDR_ADDR);
        ccLogHdrNext = 0;
    }
    PY25Q16_WriteBuffer(CC_LOG_HDR_ADDR + (uint32_t)ccLogHdrNext * CC_LOG_HDR_REC,
                        hdr, sizeof(hdr), false);
    ccLogHdrNext++;
}

// Scan the header journal for the newest valid record. A blank or torn record
// (magic mismatch) ends the scan; the last valid record wins.
static void CC_LogLoad(void)
{
    ccLogCount   = 0;
    ccLogHead    = 0;
    ccLogHdrNext = 0;
    for (uint16_t i = 0; i < CC_LOG_HDR_MAX; i++) {
        uint8_t hdr[8];
        PY25Q16_ReadBuffer(CC_LOG_HDR_ADDR + (uint32_t)i * CC_LOG_HDR_REC,
                           hdr, sizeof(hdr));
        uint32_t magic = (uint32_t)hdr[0] | ((uint32_t)hdr[1] << 8) |
                         ((uint32_t)hdr[2] << 16) | ((uint32_t)hdr[3] << 24);
        if (magic != CC_LOG_FLASH_MAGIC) break;
        uint8_t count = hdr[4];
        if (count > CC_LOG_MAX) count = CC_LOG_MAX;
        ccLogCount   = count;
        ccLogHead    = (count > 0) ? (uint8_t)(hdr[5] % CC_LOG_MAX) : 0;
        ccLogHdrNext = (uint16_t)(i + 1);
    }
}

// Read the idx-th entry (0 = newest) from flash into *e.
static void CC_LogRead(uint8_t idx, cc_log_entry_t *e)
{
    if (idx >= ccLogCount) { memset(e, 0, sizeof(*e)); return; }
    uint8_t slot = (uint8_t)((ccLogHead + CC_LOG_MAX - idx) % CC_LOG_MAX);
    PY25Q16_ReadBuffer(CC_LOG_FLASH_ADDR + 8 + (uint32_t)slot * sizeof(cc_log_entry_t),
                       e, sizeof(*e));
}

// Add a new entry as the newest hit. Only one slot and the header are written;
// the sector is erased first (a real qualified hit is rare, so this is bounded).
static void CC_LogAdd(uint32_t freq, uint16_t rssi, uint8_t toneType, uint8_t toneCode)
{
    cc_log_entry_t e;
    e.freq     = freq;
    e.talkback = CC_TalkbackFreq(freq, &e.shiftDir);
    e.rssi     = rssi;
    e.toneType = toneType;
    e.toneCode = toneCode;
    e.reserved = 0;

    uint8_t slot;
    if (ccLogCount == 0) {
        // Very first entry: start the ring at slot 0.
        slot = 0;
        ccLogCount = 1;
        PY25Q16_SectorErase(CC_LOG_FLASH_ADDR);
    } else if (ccLogCount < CC_LOG_MAX) {
        slot = ccLogCount;      // append
        ccLogCount++;
    } else {
        slot = (uint8_t)((ccLogHead + 1) % CC_LOG_MAX); // wrap: overwrite oldest
    }
    ccLogHead = slot;

    PY25Q16_WriteBuffer(CC_LOG_FLASH_ADDR + 8 + (uint32_t)slot * sizeof(cc_log_entry_t),
                        &e, sizeof(e), false);
    CC_LogWriteHeader();

    ccLogSel  = 0;
    ccLogView = false;
}

// Format the tone field into a small string ("88.5" or "D023N"); empty if none.
static void CC_ToneText(char *buf, size_t len, uint8_t toneType, uint8_t toneCode)
{
    if (toneType == CC_LOG_TONE_CTCSS && toneCode != 0xFF && toneCode < 50) {
        snprintf(buf, len, "%u.%u", CTCSS_Options[toneCode] / 10, CTCSS_Options[toneCode] % 10);
    } else if (toneType == CC_LOG_TONE_DCS && toneCode != 0xFF && toneCode < 104) {
        snprintf(buf, len, "D%03oN", DCS_Options[toneCode]);
    } else {
        buf[0] = '\0';
    }
}

// Poll the CTCSS/DCS detector once. Called repeatedly while locked; the first
// non-zero result is remembered for the log entry.
static void CC_TonePoll(void)
{
    if (CC_IsAirband()) return;
    if (ccToneType != CC_LOG_TONE_NONE) return;

    uint32_t cdcssFreq = 0;
    uint16_t ctcssFreq = 0;
    BK4819_CssScanResult_t r = BK4819_GetCxCSSScanResult(&cdcssFreq, &ctcssFreq);
    if (r == BK4819_CSS_RESULT_CTCSS) {
        uint8_t code = DCS_GetCtcssCode(ctcssFreq);
        if (code != 0xFF && code < 50) {
            ccToneType = CC_LOG_TONE_CTCSS;
            ccToneCode = code;
        }
    } else if (r == BK4819_CSS_RESULT_CDCSS) {
        uint8_t code = DCS_GetCdcssCode(cdcssFreq);
        if (code != 0xFF) {
            ccToneType = CC_LOG_TONE_DCS;
            ccToneCode = code;
        }
    }
}

// Select the front-end LNA path for the current sweep. Frequencies are in
// 10 Hz units; the driver selects the UHF LNA at/above 28000000 (280 MHz) and
// the VHF LNA below that.
static void CC_SelectPath(bool uhf)
{
    BK4819_PickRXFilterPathBasedOnFrequency(uhf ? CC_SENTINEL_UHF : CC_SENTINEL_VHF);
}

// ============================================================
// Audio / squelch
// ============================================================
// Program the BK4819 hardware squelch (REG_4D/4E/4F/78) from the user's single
// signal threshold. The chip gates the AF demodulator: it stays muted until
// RSSI rises above the threshold, so anything weaker is ignored and empty
// channels/noise stay silent. A small hysteresis keeps it from chattering.
// Noise/glitch thresholds are left permissive so RSSI is the deciding gate.
static void CC_SquelchFromConfig(void)
{
    uint8_t open  = CC_Dbm2Rssi8(ccThresholdDbm);
    int16_t closeDbm = (int16_t)ccThresholdDbm - CC_SQUELCH_HYST;
    if (closeDbm < CC_DBM_MIN) closeDbm = CC_DBM_MIN;
    uint8_t close = CC_Dbm2Rssi8((int8_t)closeDbm);
    if (close > open) close = open;

    ccSquelchRssi = CC_Dbm2Rssi(ccThresholdDbm);

    BK4819_SetupSquelch(
        open,                       // SquelchOpenRSSIThresh
        close,                      // SquelchCloseRSSIThresh
        127,                        // SquelchOpenNoiseThresh  (permissive)
        127,                        // SquelchCloseNoiseThresh (permissive)
        255,                        // SquelchCloseGlitchThresh (permissive)
        255);                       // SquelchOpenGlitchThresh  (permissive)
}

// Open/close the speaker path. The chip owns the actual AF mute via the
// hardware squelch armed above; here we only gate the external audio path and
// select the AF mode (matching the normal receive path). The previous version
// left the path open with no squelch programmed, so noise was always audible.
static void CC_OpenAudio(bool on)
{
    if (on) {
        // The physical RF front end must be enabled or nothing reaches the
        // demodulator, even when the chip-internal RX link is running. The
        // normal RX path (radio.c) and the spectrum lock path both assert this
        // GPIO; without it a verified lock produced no signal/audio.
        BK4819_ToggleGpioOut(BK4819_GPIO0_PIN28_RX_ENABLE, true);
        BK4819_RX_TurnOn();
        SYSTEM_DelayMs(20);
        // Restore the normal chip AGC (CC_ArmScan forced max/manual gain for the
        // counter). Do this explicitly rather than via RADIO_SetupAGC, whose
        // internal cache may already believe AGC is enabled. In AM the software
        // servo (am_fix) owns the gain: keep the chip AGC OFF and hand REG_13
        // back to it, then let RADIO_SetModulation enable the servo again.
        if (CC_IsAirband()) {
            BK4819_SetAGC(false);
            BK4819_InitAGC(MODULATION_AM);
        } else {
            BK4819_SetAGC(true);
            BK4819_InitAGC(MODULATION_FM);
        }
        CC_SquelchFromConfig();     // (re)arm the squelch for the current setting
        RADIO_SetModulation(CC_IsAirband() ? MODULATION_AM : MODULATION_FM);
        // Filter bandwidth follows RADIO_SetModulation (RADIO_GetAMFilterBandwidth
        // -> WIDE for airband); the old narrow-AM override here cut the passband
        // and turned faint air carriers into static.
        BK4819_SetRxAudioGain();
        gEnableSpeaker = true;
        GPIO_EnableAudioPath();
    } else {
        gEnableSpeaker = false;
        BK4819_SetAF(BK4819_AF_MUTE);
        GPIO_DisableAudioPath();
        BK4819_ToggleGpioOut(BK4819_GPIO0_PIN28_RX_ENABLE, false);
    }
}

static bool CC_IsAirbandForFreq(uint32_t f)
{
    // Airband is 108..137 MHz civil air (10 Hz units).
    return (f >= 10800000u && f <= 13700000u);
}

static bool CC_IsAirband(void)
{
    uint32_t f = ccLocked ? ccFoundFreq : ccKnownGood;
    return CC_IsAirbandForFreq(f);
}

// ============================================================
// Capture / verification
// ============================================================

// The RF frequency counter samples the signal present at LNAIN, so the analog
// front-end gain directly sets how faint a carrier it can see. Force the RX
// gain table to maximum while the counter runs so weak, distant transmissions
// still reach the counter. REG_7E<15>=1 selects AGC fix (manual) mode and
// bits<14:12> the gain-table index (011 = index 3 -> REG_13, the max table).
// The gain registers are bitfields (LNA-short/LNA/mixer/PGA) where all-ones is
// 0 dB and all-zeroes is minimum: 0x03FF is true max gain, NOT 0x0000. The
// normal AGC is restored in CC_OpenAudio(true).
static void CC_SetMaxGain(void)
{
    uint16_t regVal = BK4819_ReadRegister(BK4819_REG_7E);
    regVal |= (1u << 15);           // 1 = AGC fix (manual) mode
    regVal &= ~(0b111u << 12);      // clear gain index
    regVal |= (3u << 12);           // 3 = gain-table index 3 (REG_13)
    BK4819_WriteRegister(BK4819_REG_7E, regVal);

    BK4819_WriteRegister(BK4819_REG_10, 0x03FF); // max gain tables (all fields 0 dB)
    BK4819_WriteRegister(BK4819_REG_11, 0x03FF);
    BK4819_WriteRegister(BK4819_REG_12, 0x03FF);
    BK4819_WriteRegister(BK4819_REG_13, 0x03FF);
}

// Fully (re)initialise the receiver and arm the hardware counter.
// This must be used everywhere the scan is (re)started: after CC_Verify the
// chip is left tuned to a specific frequency with the scan stopped and a
// frequency/AGC loaded, and a bare SetFrequencyScan(true) does not reliably
// restart the counter from that state. Doing the full sequence here is what
// makes band changes and post-verify resumes actually keep scanning.
static void CC_RestartCounter(void)
{
    // Stop any pending hardware capture before reconfiguring the front end.
    BK4819_SetFrequencyScan(false);

    // Re-establish the RX link/DSP so the counter sees a live front end, then
    // force maximum analog gain for faint-carrier pickup. RX_ENABLE is asserted
    // last, once the path and gain are configured.
    CC_SelectPath(ccFirstUHF);
    BK4819_RX_TurnOn();
    CC_SetMaxGain();
    BK4819_ToggleGpioOut(BK4819_GPIO0_PIN28_RX_ENABLE, true);

    // Longer integration gate for the weak/high bands (AIR/SATCOM) so faint
    // carriers accumulate enough energy for the counter; keep the fast 0.2 s
    // gate for VHF/UHF where there is more signal to see. For a dual-span
    // preset the VHF pass gets the long gate too: airband/MARINE carriers
    // are much fainter than typical UHF PMR traffic and would otherwise be
    // invisible under ALL/VHF+UHF.
    uint8_t gateIndex = CC_PresetSlowGate(ccPreset);
    if (CC_PresetSpansBoth(ccPreset) && !ccFirstUHF)
        gateIndex = 3u; // 1.6 s on the VHF pass
    CC_SetFrequencyScanGate(gateIndex);

    SYSTEM_DelayMs(10);

    // Snapshot whatever is latched in REG_0D/0E right now: this is the value
    // that would be re-reported as "Finished" while the counter integrates its
    // first fresh capture (and possibly forever if the latch never updates).
    {
        const uint16_t reg0D = BK4819_ReadRegister(BK4819_REG_0D);
        if ((reg0D & 0x8000u) == 0) {
            ccStaleLatchFreq = ((uint32_t)(reg0D & 0x7FFu) << 16) |
                               BK4819_ReadRegister(BK4819_REG_0E);
            ccStaleLatchValid = true;
            ccStaleLatchAge = 0;
        } else {
            ccStaleLatchValid = false;
            ccStaleLatchAge = 0;
        }
    }

    BK4819_SetFrequencyScan(true);

    // The counter latches its last completed capture in REG_0D/0E and has no
    // "consumed" flag, so right after (re)arming it still holds the result from
    // before the lock (typically the frequency we just released). That stale
    // value re-verifies and re-locks the same dead channel forever. Block reads
    // until a whole fresh integration gate has elapsed, otherwise a single
    // skipped loop iteration still returns the stale capture. The window must
    // cover the full gate (0.2/0.4/0.8/1.6 s), so map the gate index to it.
    switch (gateIndex) {
        case 1u:  ccArmSettleMs = CC_ARM_SETTLE_MS_SLOW; break;  // 0.4 s
        case 2u:  ccArmSettleMs = 1000;                  break;  // 0.8 s
        case 3u:  ccArmSettleMs = CC_ARM_SETTLE_MS_MAX;   break;  // 1.6 s
        default:  ccArmSettleMs = CC_ARM_SETTLE_MS_FAST;  break;  // 0.2 s
    }
}

static void CC_ArmScan(void)
{
    // Keep the audio path muted so nothing leaks to the speaker while the
    // counter runs.
    gEnableSpeaker = false;
    BK4819_SetAF(BK4819_AF_MUTE);
    GPIO_DisableAudioPath();

    ccCooldownMs = 0;

    // The hardware counter only sees one LNA path at a time, so for a preset
    // that spans VHF and UHF (VHF+UHF / ALL) alternate the path on each sweep;
    // otherwise lock to the path the preset lives in.
    if (CC_PresetSpansBoth(ccPreset)) {
        ccFirstUHF = !ccFirstUHF;
    } else {
        ccFirstUHF = (ccPresets[ccPreset].start >= 28000000u);
    }

    CC_RestartCounter();
}

// Live fine-tune readout helper: offset in kHz (10 Hz units) + RSSI in dBm.
static void CC_TuneShow(int32_t off, uint16_t rssi)
{
    const char *sign = (off < 0) ? "-" : "+";
    uint32_t a = (uint32_t)((off < 0) ? -off : off);
    snprintf(ccTuneMsg, sizeof(ccTuneMsg), "TUNE %s%u.%01uk %ddBm",
             sign, (unsigned)(a / 100u), (unsigned)((a / 10u) % 10u),
             (int)((rssi >> 1) - 160));
}

// Quick tune-and-measure: retunes the RX chain to f exactly like the verify
// head and returns the settled RSSI. Used by the strongest-first arbitration
// so candidate strength can be compared before committing to a full sweep.
static uint16_t CC_ProbeRssi(uint32_t f)
{
    const bool air = CC_IsAirbandForFreq(f);

    BK4819_SetFrequencyScan(false);
    BK4819_SetFrequency(f);
    BK4819_PickRXFilterPathBasedOnFrequency(f);
    BK4819_RX_TurnOn();
    BK4819_ToggleGpioOut(BK4819_GPIO0_PIN28_RX_ENABLE, true);
    BK4819_SetAGC(true);
    BK4819_InitAGC(air ? MODULATION_AM : MODULATION_FM);
    RADIO_SetModulation(air ? MODULATION_AM : MODULATION_FM);

    SYSTICK_DelayUs(air ? 20000 : 12000);
    uint16_t rssi = BK4819_GetRSSI();
    SYSTICK_DelayUs(2000);
    { const uint16_t r2 = BK4819_GetRSSI(); if (r2 > rssi) rssi = r2; }
    return rssi;
}

static bool CC_Verify(uint32_t *pf, uint16_t preRssi)
{
    // Live fine-tune readout is only meaningful during a sweep; start clean so
    // a stale "TUNE" line never lingers over a SEEN/lock display.
    ccTuneMsg[0] = '\0';

    // Measure the carrier at the reported frequency. The hardware counter
    // shares the front end with the RX chain, so scanning must be disabled and
    // the normal RX link re-powered before REG_67 is meaningful. Use the NORMAL
    // AGC here (not the forced max gain used for the counter): a strong local
    // signal would otherwise compress the front end and the RSSI read becomes
    // unreliable, which is exactly why a strong carrier often failed
    // verification. When preRssi is non-zero the caller just probed this exact
    // frequency with CC_ProbeRssi and the chain is already configured.
    const bool air = CC_IsAirbandForFreq(*pf);

    uint16_t rssi;
    if (preRssi != 0) {
        rssi = preRssi;
    } else {
        BK4819_SetFrequencyScan(false);
        BK4819_SetFrequency(*pf);
        BK4819_PickRXFilterPathBasedOnFrequency(*pf);
        BK4819_RX_TurnOn();
        BK4819_ToggleGpioOut(BK4819_GPIO0_PIN28_RX_ENABLE, true);
        BK4819_SetAGC(true);
        BK4819_InitAGC(air ? MODULATION_AM : MODULATION_FM);

        // Apply the same demodulator chain the normal receive path uses so the
        // RSSI read reflects a living receiver: airband must be AM (REG_31 AM
        // demod + AM filter/AF registers), everything else FM/narrow.
        // SetModulation also picks the filter bandwidth
        // (RADIO_GetAMFilterBandwidth -> WIDE for airband); do NOT override it
        // here with the narrow AM preset, which cut the passband down and
        // turned faint air carriers into static.
        RADIO_SetModulation(air ? MODULATION_AM : MODULATION_FM);

        SYSTICK_DelayUs(air ? 20000 : 12000);
        rssi = BK4819_GetRSSI();
        SYSTICK_DelayUs(2000);
        { const uint16_t r2 = BK4819_GetRSSI(); if (r2 > rssi) rssi = r2; }
    }

    // Candidate gate (user request): every counter hit within -10 dBm of the
    // threshold is fine-tuned below (offset sweep) and then judged again after
    // the sweep; anything weaker is reported as a merely "SEEN" frequency and
    // scanning continues.
    if (rssi + CC_VERIFY_MARGIN < ccSquelchRssi) return false;

    // Off-by-a-few-kHz correction: the frequency counter resolution leaves the
    // reported value beside the true carrier centre, which both misnames the
    // lock and reads several dB low at the probe point. Sweep a ±2/±4 kHz
    // window with short settling, keep the strongest reading, and lock to that
    // offset. Cost: 4 x ~5 ms, only during the initial verify of a capture.
    static const int32_t ccVerifyOffsets[4] = { -400, -200, 200, 400 };   // 10 Hz units
    int32_t bestOff = 0;
    uint16_t bestRssi = rssi;
    for (unsigned i = 0; i < 4; i++) {
        BK4819_SetFrequency(*pf + ccVerifyOffsets[i]);
        SYSTICK_DelayUs(air ? 5000 : 4000);
        uint16_t r = BK4819_GetRSSI();
        if (r > bestRssi) { bestRssi = r; bestOff = ccVerifyOffsets[i]; }
        // Live fine-tune display: show every probe offset + its RSSI as it is
        // measured (user request).
        CC_TuneShow(ccVerifyOffsets[i], r);
        CC_Render();
    }
    if (bestOff != 0) {
        // Confirm at the chosen offset with a fresh settle instead of trusting a
        // possibly-transient sweep read.
        BK4819_SetFrequency(*pf + bestOff);
        SYSTICK_DelayUs(air ? 9000 : 7000);
        uint16_t r = BK4819_GetRSSI();
        SYSTICK_DelayUs(2000);
        { const uint16_t r4 = BK4819_GetRSSI(); if (r4 > r) r = r4; }
        if (r < bestRssi) { r = bestRssi; }   // never lose the sweep's peak to a fade
        *pf += bestOff;
        rssi = r;
    }
    ccRssi = rssi;
    CC_TuneShow(bestOff, rssi);
    CC_Render();

    // Wide re-search: the counter resolution can park beside the carrier by
    // more than the fine window (user case: "sees 119.4500, signal really at
    // 119.5000"). If the fine-sweep peak is still below the floor, walk the
    // +/-200 kHz neighbourhood in 5 kHz steps with a live on-screen readout.
    // Locking still requires a probe at/above the threshold (gate below); a
    // miss is a normal verify fail (SEEN + auto-suppression), and the sweep
    // runs once per capture so it cannot loop.
    if (rssi < ccSquelchRssi) {
        const bool airW = CC_IsAirbandForFreq(*pf);
        int32_t  wideOff  = 0;
        uint16_t wideRssi = rssi;
        for (int32_t off = -CC_WIDE_SEARCH_SPAN; off <= CC_WIDE_SEARCH_SPAN; off += CC_WIDE_SEARCH_STEP) {
            BK4819_SetFrequency(*pf + off);
            SYSTICK_DelayUs(airW ? 5000 : 4000);
            const uint16_t r = BK4819_GetRSSI();
            if (r > wideRssi) { wideRssi = r; wideOff = off; }
            CC_TuneShow(off, r);
            CC_Render();
        }
        if (wideOff != 0) {
            BK4819_SetFrequency(*pf + wideOff);
            SYSTICK_DelayUs(airW ? 9000 : 7000);
            uint16_t r = BK4819_GetRSSI();
            SYSTICK_DelayUs(2000);
            { const uint16_t r4 = BK4819_GetRSSI(); if (r4 > r) r = r4; }
            if (r > wideRssi) { wideRssi = r; }
            *pf += wideOff;
        }
        rssi   = wideRssi;
        ccRssi = rssi;
        CC_TuneShow(wideOff, rssi);
        CC_Render();
    }

    // High-resolution centring pass (user request: "as accurate as possible").
    // The counter + the coarse sweeps still carry the crystal's ppm error and
    // the lock gate may still sit a few kHz beside the true carrier centre.
    // Attach a NARROW receive filter, which sharpens the RSSI-vs-offset curve
    // dramatically, walk +/-6 kHz in 200 Hz steps (61 probes, ~0.4 s - the
    // user explicitly accepted a slower check), and centre on the peak. The
    // narrow bandwidth is only a measurement tool here: the audio path is
    // reconfigured right after (and again by CC_LockOn), and the wide filter
    // is restored before returning so nothing outside this pass sees it.
    {
        const bool airR = CC_IsAirbandForFreq(*pf);
        int32_t  refOff  = 0;
        uint16_t baseline = 0;
        BK4819_SetFilterBandwidth(airR ? BK4819_FILTER_BW_AM : BK4819_FILTER_BW_NARROW, true);
        // Baseline: a NARROW-filter reading at the current (pre-centring)
        // frequency. Seeding from the wide-filter 'rssi' made every narrow
        // probe look worse, silently skipping the centring altogether.
        BK4819_SetFrequency(*pf);
        SYSTICK_DelayUs(airR ? 6000 : 5000);
        uint16_t refRssi = BK4819_GetRSSI();
        baseline = refRssi;
        for (int32_t off = -CC_REFINE_SPAN; off <= CC_REFINE_SPAN; off += CC_REFINE_STEP) {
            BK4819_SetFrequency(*pf + off);
            SYSTICK_DelayUs(airR ? 6000 : 5000);
            const uint16_t r = BK4819_GetRSSI();
            if (r > refRssi) { refRssi = r; refOff = off; }
            CC_TuneShow(off, r);
            CC_Render();
        }
        // Only shift when a probe beats the narrow baseline CLEARLY (>= 2 LSB
        // = 1 dBm) - otherwise the ppm/coarse residual beat the quantization
        // noise and we keep the counter's frequency.
        if (refOff != 0 && refRssi >= baseline + 2) {
            BK4819_SetFrequency(*pf + refOff);
            SYSTICK_DelayUs(airR ? 10000 : 8000);
            uint16_t r = BK4819_GetRSSI();
            SYSTICK_DelayUs(2000);
            { const uint16_t r4 = BK4819_GetRSSI(); if (r4 > r) r = r4; }
            if (r < refRssi) r = refRssi;   // never lose the sweep peak to a fade
            *pf += refOff;
            rssi = r;
        }
        // Back to the normal audio-passband filter at the centred frequency.
        BK4819_SetFilterBandwidth(airR ? BK4819_FILTER_BW_WIDE : BK4819_FILTER_BW_NARROW, true);
        BK4819_SetFrequency(*pf);
        SYSTICK_DelayUs(4000);
        ccRssi = rssi;
        CC_TuneShow(refOff, rssi);
        CC_Render();
    }

    // Lock gate: the -10 dBm candidate window only qualifies a hit for the
    // fine-tune sweep; the lock itself still requires the carrier to be at or
    // above the user's threshold. Without this a noise floor a couple of dBm
    // below the threshold passes the pre-gate, locks, and holds until the 8 s
    // cap - the "stuck on a dead frequency" pause. Weaker than that: report as
    // SEEN and keep scanning.
    if (rssi < ccSquelchRssi) return false;

    // The user wants to open on any counter hit above the floor, accepting false
    // positives (they can SKIP/BLOCK). Images of broadcasters are handled by
    // blocking the broadcast band outright rather than a fragile f/2 compare.
    return true;
}

// Shared lock path used by both discovery engines (hardware counter and the
// tuned RSSI sweep) once a candidate frequency has passed CC_Verify.
static void CC_LockOn(uint32_t freq)
{
    ccFoundFreq = freq;
    ccKnownGood = freq;
    ccSeenFreq  = 0;
    ccTuneMsg[0] = '\0';
    ccLocked    = true;
    ccPaused    = false;
    ccLockMs    = 0;
    ccWeakSlices = 0;
    ccQualifyMs = 0;
    ccLogged    = false;
    ccAgeMs     = 0;
    ccToneType  = CC_LOG_TONE_NONE;
    ccToneCode  = 0;
    // A captured signal is an event: wake the screen to full and restart the
    // idle timer, exactly like a key press. After it ends the 30 s auto-dim runs.
    // Always-on (level 3) is the user's explicit choice and is left untouched.
    if (ccDimLevel != 3) {
        ccDimLevel = 0;
        CC_ApplyDim();
    }
    CC_Discover(freq, ccRssi);

    // Program the verified carrier into the RX VFO (RAM only) exactly like a
    // manual tune, then open the full normal-receive audio path here on the CC
    // screen. CC_OpenAudio(true) applies the same chain the main screen uses:
    // RX front end, modulation (AM profile for airband incl. the WIDE filter
    // RADIO_GetAMFilterBandwidth picks), chip AGC/am_fix split, squelch and
    // AF gain. The loop keeps running and owns the hold logic.
    const bool air = CC_IsAirbandForFreq(freq);
    VFO_Info_t *vfo = &gEeprom.VfoInfo[0];
    RADIO_InitInfo(vfo, FREQ_CHANNEL, freq);
    vfo->Modulation        = air ? MODULATION_AM : MODULATION_FM;
    vfo->CHANNEL_BANDWIDTH = air ? BK4819_FILTER_BW_WIDE : BK4819_FILTER_BW_NARROW;
    gEeprom.ScreenChannel = FREQ_CHANNEL;
    gEeprom.MrChannel     = FREQ_CHANNEL;
    gEeprom.TX_VFO        = 0;
    ccTuned = true;

    CC_OpenAudio(true);
    ccFineTuneMs = 0;
    ccSilentMs   = 0;
}

// Fine-tune a held lock: sweep the verify probe offsets around the tuned
// frequency and re-tune to a meaningfully stronger reading. Called only while
// the carrier is fading (rssi < floor - 10 dBm) and at most once per second.
// Returns true when the tune actually moved.
static bool CC_FineTuneHold(void)
{
    static const int32_t offs[4] = { -400, -200, 200, 400 };   // 10 Hz units
    const bool air = CC_IsAirbandForFreq(ccFoundFreq);

    uint16_t baseRssi = BK4819_GetRSSI();
    uint16_t bestRssi = baseRssi;
    int32_t  bestOff  = 0;

    // Sharpen the RSSI-vs-offset curve with a narrow filter for this pass and
    // seed the sliding bar with a narrow-baseline reading (same reasoning as
    // the refine pass in CC_Verify: wide-filter readings are not comparable).
    BK4819_SetFilterBandwidth(air ? BK4819_FILTER_BW_AM : BK4819_FILTER_BW_NARROW, true);
    BK4819_SetFrequency(ccFoundFreq);
    SYSTICK_DelayUs(air ? 6000 : 5000);
    baseRssi = BK4819_GetRSSI();
    bestRssi = baseRssi;

    for (unsigned i = 0; i < 4; i++) {
        BK4819_SetFrequency(ccFoundFreq + offs[i]);
        SYSTICK_DelayUs(air ? 5000 : 4000);
        const uint16_t r = BK4819_GetRSSI();
        // Only slide when a neighbor is clearly better (>= 2 dBm).
        if (r > bestRssi + 4) { bestRssi = r; bestOff = offs[i]; }
        // Live fine-tune display while holding (user request).
        CC_TuneShow(offs[i], r);
        CC_Render();
    }

    if (bestOff == 0 || bestRssi < baseRssi + 2) {
        // No probe clearly beat the current tune - restore and stay put.
        BK4819_SetFilterBandwidth(air ? BK4819_FILTER_BW_WIDE : BK4819_FILTER_BW_NARROW, true);
        BK4819_SetFrequency(ccFoundFreq);
        SYSTICK_DelayUs(4000);
        return false;
    }

    BK4819_SetFrequency(ccFoundFreq + bestOff);
    SYSTICK_DelayUs(air ? 9000 : 7000);
    { uint16_t c = BK4819_GetRSSI(); SYSTICK_DelayUs(2000); const uint16_t c4 = BK4819_GetRSSI(); if (c4 > c) c = c4; if (c > bestRssi) bestRssi = c; }
    ccFoundFreq += bestOff;
    ccKnownGood = ccFoundFreq;
    // Back to the normal audio-passband filter at the centred frequency.
    BK4819_SetFilterBandwidth(air ? BK4819_FILTER_BW_WIDE : BK4819_FILTER_BW_NARROW, true);
    BK4819_SetFrequency(ccFoundFreq);
    SYSTICK_DelayUs(4000);
    ccRssi = bestRssi;

    // Keep the RAM VFO in sync so a later exit persists the corrected tune.
    gEeprom.VfoInfo[0].freq_config_RX.Frequency = ccFoundFreq;
    return true;
}

// Ledger over verify-failed frequencies, so two spurs on two LNA paths can
// escalate independently instead of resetting each other's streak.
static int CC_FailFind(uint32_t f)
{
    for (unsigned i = 0; i < CC_FAIL_LEDGER_N; i++) {
        const int32_t d = (int32_t)f - (int32_t)ccFailLedger[i].freq;
        if (ccFailLedger[i].ms != 0 &&
            (d < 0 ? -d : d) <= CC_CONSISTENT_TOL) return (int)i;
    }
    return -1;
}

static void CC_FailRemove(int idx)
{
    if (idx >= 0 && idx < CC_FAIL_LEDGER_N)
        memset(&ccFailLedger[idx], 0, sizeof(ccFailLedger[0]));
}

static void CC_ProcessResult(void)
{
    // Back off after a failed verify: without this a steady carrier on a
    // borderline/unverifiable frequency makes the loop retry every ~20 ms,
    // starving key/render handling and looking like a freeze.
    if (ccCooldownMs > 0) return;

    // Do not read the counter during the cold window after (re)arming: REG_0D
    // still holds the previous sweep's latched result, and acting on it re-locks
    // the frequency we just released. This must span the whole integration gate,
    // not one loop iteration, or the stale capture still gets through.
    if (ccArmSettleMs > 0) return;

    uint32_t resultFreq = 0;
    if (!BK4819_GetFrequencyScanResult(&resultFreq)) return;

    // Round to the nearest 1 kHz (100 * 10 Hz).
    resultFreq = ((resultFreq + 50u) / 100u) * 100u;

    // The stale-latch block: while the counter keeps re-reporting the value
    // latched at (re)arm time, sit and wait for a genuinely fresh capture
    // instead of restarting (restarting would just re-latch the same stale
    // value). Doing nothing here is what keeps the sweep alive on first open
    // and after SKIP. If that never lifts (faulty head-end that never renews
    // the capture), force one full re-arm after ~2 s so the baseline can be
    // re-snapshotted.
    if (ccStaleLatchValid) {
        int32_t staleDelta = (int32_t)resultFreq - (int32_t)((ccStaleLatchFreq + 50u) / 100u * 100u);
        if (staleDelta < 0) staleDelta = -staleDelta;
        bool staleLive = false;
        if (staleDelta <= CC_CONSISTENT_TOL) {
            ccAgeMs = 0;
            // Consecutive reads of the same value are NOT a latch artifact: real
            // carriers persist across reads. After a few identical reads let it
            // fall through to normal processing so a live-carrier stale entry
            // cannot eat the very capture it looks like.
            if (++ccStaleMatchStreak >= 3) {
                // Live carrier: lift the latch AND the mask so every following
                // read is processed normally until the next re-arm snapshot.
                ccStaleMatchStreak = 0;
                ccStaleLatchValid  = false;
                ccStaleMatchStreak = 0;
            } else {
                if (++ccStaleLatchAge >= CC_STALE_LATCH_REARM) {
                    ccStaleLatchAge = 0;
                    // Two watchdog fires in a row mean the same persistent capture
                    // survives a full re-arm: switch the LNA path (spans-both
                    // presets) so a spur on one path cannot pin the sweep forever.
                    if (++ccStaleFires >= 2) {
                        ccStaleFires = 0;
                        if (CC_PresetSpansBoth(ccPreset)) ccFirstUHF = !ccFirstUHF;
                    }
                    CC_RestartCounter();
                }
                return;
            }
        } else {
            ccStaleMatchStreak = 0;
        }
        if (!staleLive) {
            ccStaleLatchValid = false;   // fresh capture: the block lifts
            ccStaleLatchAge = 0;
            ccStaleFires = 0;
            ccStaleMatchStreak = 0;
        }
    }
    // A SKIPped frequency is suppressed as long as its capture shows up with the
    // carrier still keyed: previously the counter would read it again after the
    // quiet-window expired, re-lock the same channel and freeze the sweep until
    // the next SKIP. Refresh the quiet window while the capture persists and do
    // NOT restart the counter here - a restart costs a full settle window (the
    // "paused until you press SKIP" symptom). A fresh capture elsewhere simply
    // stops matching and scanning carries on.
    if (ccSkipFreq != 0) {
        int32_t skDelta = (int32_t)resultFreq - (int32_t)ccSkipFreq;
        if (skDelta < 0) skDelta = -skDelta;
        if (skDelta > CC_CONSISTENT_TOL) {
            ccSkipFreq = 0;
            ccSkipStreak = 0;
        }
    }
    if (ccSkipFreq != 0) {
        ccLastHitCount = 0;
        ccLastHitFreq = 0;
        // Final escalation reached: a carrier that STILL keys up on the skipped
        // frequency after the 15 s window is a persistent local carrier - block
        // it outright (KEY_9 clears blocks) instead of refreshing the quiet
        // window forever (that was the infinite/random SKIP pause).
        if (ccSkipStreak >= 3) {
            CC_Block(ccSkipFreq);
            ccHintMsg = "BLOCKED";
            ccHintMs  = 1000;
            ccSkipFreq = 0;
            ccSkipStreak = 0;
            CC_RestartCounter();
            return;
        }
        ccSkipStreak++;
        ccSkipMs = (ccSkipStreak <= 1)
                       ? CC_SKIP_QUIET_MS
                       : (ccSkipStreak == 2) ? CC_SKIP_ESCALATE_MS
                                             : CC_SKIP_FINAL_MS;
        return;
    }

    // A fresh counter capture: keep the shown frequency alive.
    ccAgeMs = 0;
    ccPathDwellMs = 0;

    // A capture elsewhere ends the hard-cap escalation streak (Bug 2 fix).
    if (ccCapFreq != 0) {
        int32_t capDelta = (int32_t)resultFreq - (int32_t)ccCapFreq;
        if (capDelta < 0) capDelta = -capDelta;
        if (capDelta > CC_CONSISTENT_TOL) {
            ccCapFreq = 0;
            ccCapStreak = 0;
        }
    }

    if (resultFreq < 1400000u || resultFreq > 116000000u) {
        ccLastHitCount = 0;
        ccLastHitFreq = 0;
        CC_RestartCounter();
        return;
    }

    // Band filter: the RF frequency counter has no band of its own, so a strong
    // out-of-band signal would still be captured even with the LNA path set.
    // Reject anything outside the selected preset's range(s) so AIR/VHF/UHF/etc.
    // actually constrain what we lock onto. VHF+UHF has two disjoint sub-ranges.
    if (!CC_PresetInRange(ccPreset, resultFreq)) {
        ccLastHitCount = 0;
        ccLastHitFreq = 0;
        CC_RestartCounter();
        return;
    }

    if (CC_IsBlocked(resultFreq) || CC_IsFmBroadcast(resultFreq) ||
        (resultFreq % 1300000u) == 0) {
        ccLastHitCount = 0;
        ccLastHitFreq = 0;
        // Quarantined frequency on this LNA path: alternate the path so the
        // sweep cannot stay stranded on one side of a dual-span preset.
        if (CC_PresetSpansBoth(ccPreset))
            ccFirstUHF = !ccFirstUHF;
        CC_RestartCounter();
        return;
    }

    // Require a couple of consistent captures to reject one-off spurs.
    int32_t delta = (int32_t)resultFreq - (int32_t)ccLastHitFreq;
    if (delta < 0) delta = -delta;
    if (delta <= CC_CONSISTENT_TOL) {
        ccLastHitCount++;
    } else {
        ccLastHitCount = 1;
    }
    ccLastHitFreq = resultFreq;

    if (ccLastHitCount < CC_CONSISTENT_HITS) {
        CC_RestartCounter();
        return;
    }

    ccLastHitCount = 0;

    // A frequency freshly recorded in the fail ledger is not worth re-verifying
    // every capture - park it as SEEN until its entry expires (10 s).
    if (CC_FailFind(resultFreq) >= 0) {
        ccCooldownMs = CC_VERIFY_COOLDOWN_MS;
        ccSeenFreq = resultFreq;
        ccLastHitFreq = 0;
        CC_RestartCounter();
        ccArmSettleMs = CC_ARM_SETTLE_MS_FAST;
        return;
    }

    // Strongest-first arbitration (user request): probe the capture cheaply and
    // defer it when a fresh, clearly stronger candidate is already waiting - the
    // strong carrier gets verified and locked first on its next capture.
    const uint16_t probeRssi = CC_ProbeRssi(resultFreq);
    if (ccDefMs > 0 && ccDefFreq != 0 && ccDefFreq != resultFreq &&
        ccDefRssi >= (uint16_t)(probeRssi + CC_DEFER_MARGIN)) {
        // Weaker hit: park as SEEN and let the counter keep alternating.
        ccCooldownMs = CC_VERIFY_COOLDOWN_MS;
        ccSeenFreq = resultFreq;
        ccLastHitFreq = 0;
        CC_RestartCounter();
        ccArmSettleMs = CC_ARM_SETTLE_MS_FAST;
        return;
    }
    if (ccDefFreq == 0 || ccDefFreq == resultFreq ||
        probeRssi + CC_DEFER_MARGIN >= ccDefRssi) {
        ccDefFreq = resultFreq;
        ccDefRssi = probeRssi;
        ccDefMs   = CC_DEFER_FRESH_MS;
    }

    if (!CC_Verify(&resultFreq, probeRssi)) {
        // Failed to confirm: back off briefly and restart the counter. Do not
        // stay pinned on this frequency. Remember it as a merely "SEEN"
        // frequency (detected by the counter but not strong enough to tune
        // into and listen to), which the UI shows distinctly.
        ccCooldownMs = CC_VERIFY_COOLDOWN_MS;
        ccSeenFreq = resultFreq;
        ccLastHitFreq = 0;

        // Escape logic: the same frequency failing over and over is either a
        // spur on the current LNA path (spans-both presets) or noise at the
        // capture gate. Ledger per frequency: toggle the path on the 2nd fail
        // of an entry, auto-block on the 3rd - two spurs on two paths escalate
        // independently instead of resetting each other's streak.
        int fi = CC_FailFind(resultFreq);
        if (fi >= 0) {
            ccFailLedger[fi].count++;
        } else {
            int slot = -1;
            for (unsigned i = 0; i < CC_FAIL_LEDGER_N; i++) {
                if (ccFailLedger[i].ms == 0) { slot = (int)i; break; }
            }
            if (slot < 0)
                slot = 0; // ledger full: overwrite the oldest slot
            ccFailLedger[slot].freq  = resultFreq;
            ccFailLedger[slot].count = 1;
            ccFailLedger[slot].ms    = CC_FAIL_LEDGER_MS;
            fi = slot;
        }
        if (ccFailLedger[fi].count >= CC_FAIL_AUTOBLOCK) {
            CC_Block(resultFreq);
            ccHintMsg = "BLOCKED";
            ccHintMs  = 1000;
            CC_FailRemove(fi);
        } else if (ccFailLedger[fi].count >= CC_FAIL_PATH_TOGGLE &&
                   CC_PresetSpansBoth(ccPreset)) {
            ccFirstUHF = !ccFirstUHF;
        }

        // The deferral pointed at this failed frequency: drop it so a fresh
        // candidate can win instead of a stale ghost.
        if (ccDefFreq == resultFreq) {
            ccDefFreq = 0;
            ccDefRssi = 0;
            ccDefMs = 0;
        }

        CC_RestartCounter();
        // The stale-latch snapshot taken by the restart is exactly this failed
        // frequency, so re-reads of it are ignored anyway: use only the fast
        // cold window instead of the full gate settle. A persistent noise
        // carrier sitting on the capture gate no longer costs a 0.4-1.6 s
        // pause per verify attempt - the sweep stays responsive.
        ccArmSettleMs = CC_ARM_SETTLE_MS_FAST;
        return;
    }

    // Confirmed. Hold on it. (Streaks and the deferral were for candidates
    // that failed - a real lock clears them.)
    CC_FailRemove(CC_FailFind(resultFreq));
    ccDefFreq = 0;
    ccDefRssi = 0;
    ccDefMs = 0;
    CC_LockOn(resultFreq);
}

static void CC_Release(void)
{
    CC_OpenAudio(false);
    ccTuneMsg[0] = '\0';
    // The stale counter latch that used to re-lock the just-released frequency
    // is already handled by the ccArmSettleMs cold window after every re-arm, so
    // no per-frequency cooldown is kept here: back-and-forth traffic on the same
    // channel must re-open immediately when the carrier keys up again.
    ccLocked = false;
    ccLockMs = 0;
    ccWeakSlices = 0;
    ccGoneMs = 0;
    ccSilentMs = 0;
    ccPathDwellMs = 0;
    ccQualifyMs = 0;
    ccLogged = false;
}

static void CC_SaveConfig(void)
{
    CC_SavePersisted();
    SETTINGS_SaveSettings();
}

// Live threshold adjustment from the UP/DOWN keys (no menu). Raising it makes
// the radio ignore weaker signals; lowering it lets weaker signals through.
// The value is persisted so it survives a power cycle.
static void CC_AdjustThreshold(int8_t dir)
{
    if (dir > 0) {
        if (ccThresholdDbm < CC_DBM_MAX) ccThresholdDbm += CC_DBM_STEP;
    } else {
        if (ccThresholdDbm > CC_DBM_MIN) ccThresholdDbm -= CC_DBM_STEP;
    }

    CC_SquelchFromConfig();
    if (ccLocked) CC_OpenAudio(true);
    CC_SaveConfig();

    ccThrHintMs = 1200;     // show the transient threshold overlay for ~1.2 s
}

// ============================================================
// Rendering
// ============================================================
static void CC_FormatFreq(char *buf, size_t len, uint32_t f)
{
    // f is in 10 Hz units: show as MHz with 4 decimals (10 Hz resolution).
    snprintf(buf, len, "%u.%04u", (unsigned)(f / 100000u), (unsigned)(f % 100000u));
}

static void CC_Render(void)
{
    char line[24];
    UI_DisplayClear();

    if (ccDimLevel == 3) CC_PrintSmallBoldRight("ON", 128);

    const char *presetName = ccPresets[ccPreset].name;
    if (ccDiscoveryView > 0 && ccDiscoveryView <= ccDiscoveryCount) {
        CC_FormatFreq(line, sizeof(line), ccDiscovery[ccDiscoveryView - 1]);
        snprintf(line + strlen(line), sizeof(line) - strlen(line), "  %u/%u",
                 (unsigned)ccDiscoveryView, (unsigned)ccDiscoveryCount);
        UI_PrintStringSmallNormal(line, 0, 0, 0);
    } else {
        // "RX NOW" while actually tuned/listening; "SEEN" when we are only
        // showing a detected-but-not-listenable carrier.
        const char *st;
        if (ccLocked)            st = "RX NOW";
        else if (ccSeenFreq != 0) st = "SEEN";
        else                     st = "SCANNING";
        snprintf(line, sizeof(line), "%s  %s", presetName, st);
        UI_PrintStringSmallNormal(line, 0, 0, 0);
    }

    // Live frequency line: only show a frequency that is current. A lock shows
    // its tuned frequency; otherwise a counter hit (last-hit or seen) is shown
    // only while it is fresh, then blanked after CC_SHOWN_TIMEOUT_MS so a stale
    // MHz never lingers when nothing is present.
    uint32_t shown;
    if (ccDiscoveryView > 0 && ccDiscoveryView <= ccDiscoveryCount) {
        shown = ccDiscovery[ccDiscoveryView - 1];
    } else if (ccLocked) {
        shown = ccFoundFreq;
    } else if (ccAgeMs < CC_SHOWN_TIMEOUT_MS) {
        shown = (ccLastHitFreq != 0) ? ccLastHitFreq : ccSeenFreq;
    } else {
        shown = 0;
    }
    // Frequency line is left BLANK when nothing is current: show what the
    // blank means, so "-80 dBm with no MHz" is not confusing.
    if (shown != 0) {
        char freqLine[16];
        CC_FormatFreq(freqLine, sizeof(freqLine), shown);
        UI_DisplayFrequency(freqLine, 0, 2, false);   // big digits on rows 2 and 3
    } else {
        const char *waitMsg = (ccLiveRssi != 0 &&
                               ccLiveRssi > ccSquelchRssi - CC_VERIFY_MARGIN)
                                  ? "NOISE FLOOR"
                                  : "SCANNING...";
        UI_PrintStringSmallNormal(waitMsg, 0, 0, 2);
    }

    // Row 4 replaces the old "MHz" label with the ACTUAL dBm the radio sees
    // right now (live REG_67 sample; the discovery list keeps its own value).
    uint16_t shownRssi = 0;
    if (ccDiscoveryView > 0 && ccDiscoveryView <= ccDiscoveryCount) {
        shownRssi = ccDiscoveryRssi[ccDiscoveryView - 1];
    } else if (ccLocked) {
        shownRssi = ccRssi;
    } else {
        shownRssi = ccLiveRssi;
    }
    if (shownRssi != 0) {
        snprintf(line, sizeof(line), "%ddBm", (int)((shownRssi >> 1) - 160));
        UI_PrintStringSmallNormal(line, 0, 0, 4);     // signal strength on row 4
    }

    // Row 5 shows the live fine-tune readout while a sweep is running (or its
    // last result), otherwise a transient SKIP/BLOCK/LOGGED hint.
    if (ccTuneMsg[0] != '\0') {
        UI_PrintStringSmallNormal(ccTuneMsg, 0, 0, 5);
    } else if (ccHintMs > 0 && ccHintMsg) {
        UI_PrintStringSmallNormal((char *)ccHintMsg, 0, 0, 5);
    }

    // Row 6 (bottom) always shows the current squelch floor so the user can see
    // the dBm threshold that is set. It brightens (brackets) briefly while adjusted.
    if (ccThrHintMs > 0) {
        snprintf(line, sizeof(line), "SQL %ddBm<", (int)ccThresholdDbm);
    } else {
        snprintf(line, sizeof(line), "SQL %ddBm", (int)ccThresholdDbm);
    }
    UI_PrintStringSmallNormal(line, 0, 0, 6);

    ST7565_BlitFullScreen();
}

// Full-screen band picker, opened with MENU. Shows a scrollable list with the
// highlighted row selected. UP/DOWN move, MENU or PTT confirm, EXIT cancels.
static void CC_RenderMenu(void)
{
    char line[24];
    UI_DisplayClear();
    // Title doubles as a scroll/position indicator: "SELECT BAND n/N".
    snprintf(line, sizeof(line), "SELECT BAND %u/%u",
             (unsigned)(ccMenuSel + 1), (unsigned)CC_PRESET_COUNT);
    UI_PrintStringSmallBold(line, 0, 0, 0);

    // Center the visible window on the highlighted entry. Only 6 content rows
    // (framebuffer rows 1..6) exist below the title; row 7 is out of bounds.
    uint8_t first = 0;
    if (ccMenuSel >= CC_MENU_VISIBLE) first = ccMenuSel - (CC_MENU_VISIBLE - 1);

    for (uint8_t row = 0; row < CC_MENU_VISIBLE; row++) {
        uint8_t p = first + row;
        if (p >= CC_PRESET_COUNT) break;
        if (p == ccMenuSel) {
            snprintf(line, sizeof(line), ">%s", ccPresets[p].name);
            UI_PrintStringSmallBold(line, 0, 0, (uint8_t)(row + 1));
        } else {
            snprintf(line, sizeof(line), " %s", ccPresets[p].name);
            UI_PrintStringSmallNormal(line, 0, 0, (uint8_t)(row + 1));
        }
    }

    ST7565_BlitFullScreen();
}

// Tone-history list (opened with KEY_1). Newest first. Each row shows the MHz,
// the CTCSS/DCS tone and the talk-back frequency. UP/DOWN scroll, EXIT back.
static void CC_RenderLog(void)
{
    char line[24];
    UI_DisplayClear();

    snprintf(line, sizeof(line), "HITS %u/%u", (unsigned)(ccLogCount == 0 ? 0 : ccLogSel + 1),
             (unsigned)ccLogCount);
    UI_PrintStringSmallBold(line, 0, 0, 0);

    if (ccLogCount == 0) {
        UI_PrintStringSmallNormal("NO HITS YET", 0, 0, 3);
        ST7565_BlitFullScreen();
        return;
    }

    // Three entries per screen (2 small rows each), newest at the top window.
    uint8_t first = 0;
    if (ccLogSel >= 3) first = ccLogSel - 2;

    for (uint8_t row = 0; row < 3; row++) {
        uint8_t idx = first + row;
        if (idx >= ccLogCount) break;
        cc_log_entry_t e;
        CC_LogRead(idx, &e);

        char freq[16];
        CC_FormatFreq(freq, sizeof(freq), e.freq);
        snprintf(line, sizeof(line), "%c%s", (idx == ccLogSel) ? '>' : ' ', freq);
        if (idx == ccLogSel) UI_PrintStringSmallBold(line, 0, 0, (uint8_t)(1 + row * 2));
        else                 UI_PrintStringSmallNormal(line, 0, 0, (uint8_t)(1 + row * 2));

        char tone[8];
        CC_ToneText(tone, sizeof(tone), e.toneType, e.toneCode);
        char tb[16];
        CC_FormatFreq(tb, sizeof(tb), e.talkback);
        if (tone[0]) {
            snprintf(line, sizeof(line), " %s  TX %s", tone, tb);
        } else {
            snprintf(line, sizeof(line), " TX %s", tb);
        }
        UI_PrintStringSmallNormal(line, 0, 0, (uint8_t)(2 + row * 2));
    }

    ST7565_BlitFullScreen();
}

// ============================================================
// Key handling
// ============================================================
static void CC_SetPreset(uint8_t p)
{
    if (p >= CC_PRESET_COUNT) p = 0;
    ccPreset = p;
    // Persist the chosen band (and current threshold) so it survives a power cycle.
    CC_SaveConfig();
    // Lock the LNA path to the selected band; combined presets alternate each sweep.
    ccFirstUHF = CC_PresetSpansBoth(ccPreset) ? !ccFirstUHF : (ccPresets[ccPreset].start >= 28000000u);
    CC_Release();
    ccLastHitCount = 0;
    ccLastHitFreq = 0;   // don't let a stale old-band frequency block the first new-band hit
    CC_ArmScan();
}

static void CC_HandleDiscovery(int8_t dir)
{
    if (ccDiscoveryCount == 0) return;
    if (ccDiscoveryView == 0) {
        ccDiscoveryView = (dir > 0) ? 1 : ccDiscoveryCount;
    } else {
        int8_t v = (int8_t)ccDiscoveryView + dir;
        if (v < 1) v = ccDiscoveryCount;
        if (v > ccDiscoveryCount) v = 1;
        ccDiscoveryView = (uint8_t)v;
    }
    // Show the selected entry briefly.
    ccFoundFreq = ccDiscovery[ccDiscoveryView - 1];
}

// Apply the current dim level to the backlight and restart the idle timer.
static void CC_ApplyDim(void)
{
    switch (ccDimLevel) {
        case 0:
        case 3:  BACKLIGHT_SetBrightness(gEeprom.BACKLIGHT_MAX); break;
        case 1:  BACKLIGHT_SetBrightness(CC_HALF_BRIGHTNESS);    break;
        default: BACKLIGHT_SetBrightness(gEeprom.BACKLIGHT_MIN); break;
    }
    ccDimMs = 0;
}

// F key: cycle full -> half -> off -> always-on -> full.
static void CC_CycleBacklight(void)
{
    ccDimLevel = (uint8_t)((ccDimLevel + 1) % 4);
    CC_ApplyDim();
}

// Returns false when the screen should exit.
static bool CC_HandleKey(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld)
{
    (void)bKeyHeld;
    if (!bKeyPressed) return true;

    // Tone-history list has its own key map while open.
    if (ccLogView) {
        switch (Key) {
            case KEY_UP:
                if (ccLogCount > 0) {
                    ccLogSel = (uint8_t)((ccLogSel + ccLogCount - 1) % ccLogCount);
                }
                break;
            case KEY_DOWN:
                if (ccLogCount > 0) {
                    ccLogSel = (uint8_t)((ccLogSel + 1) % ccLogCount);
                }
                break;
            case KEY_EXIT:
            case KEY_STAR:
            case KEY_1:
                ccLogView = false;
                break;
            default:
                break;
        }
        return true;
    }

    // Band picker has its own key map while open.
    if (ccMenuOpen) {        switch (Key) {
            case KEY_UP:
                ccMenuSel = (uint8_t)((ccMenuSel + CC_PRESET_COUNT - 1) % CC_PRESET_COUNT);
                break;
            case KEY_DOWN:
                ccMenuSel = (uint8_t)((ccMenuSel + 1) % CC_PRESET_COUNT);
                break;
            case KEY_MENU:
            case KEY_PTT:
                ccMenuOpen = false;
                CC_SetPreset(ccMenuSel);
                break;
            case KEY_EXIT:
                ccMenuOpen = false;
                break;
            default:
                break;
        }
        return true;
    }

    switch (Key) {
        case KEY_EXIT:
            CC_Release();
            ccRunning = false;
            return false;

        case KEY_MENU:
            // Open the full-screen band picker, starting on the current band.
            ccMenuOpen = true;
            ccMenuSel = ccPreset;
            break;

        case KEY_STAR:
            // Jump straight back to live scanning and clear the browse view.
            ccDiscoveryView = 0;
            CC_Release();
            ccLastHitCount = 0;
            CC_ArmScan();
            break;

        case KEY_UP:
            // Live adjust the squelch threshold: raise it (ignore weaker signals).
            CC_AdjustThreshold(+1);
            break;

        case KEY_DOWN:
            // Live adjust the squelch threshold: lower it (allow weaker signals).
            CC_AdjustThreshold(-1);
            break;

        case KEY_2:
            CC_HandleDiscovery(+1);
            break;

        case KEY_8:
            CC_HandleDiscovery(-1);
            break;

        case KEY_1:
            // Open the tone-history list (newest first).
            if (ccLogCount > 0) {
                ccLogView = true;
                ccLogSel  = 0;
            } else {
                ccHintMsg = "NO HITS";
                ccHintMs  = 800;
            }
            break;

        case KEY_3:
        case KEY_PTT:
            // SKIP: not this one - release the current hold and immediately
            // resume scanning without adding it to any list. PTT does the same
            // (TX is disabled in this build, so PTT is repurposed to skip).
            ccDiscoveryView = 0;
            ccSkipFreq = ccLocked ? ccFoundFreq : ccLastHitFreq;
            ccSkipMs = CC_SKIP_QUIET_MS;
            CC_Release();
            ccLastHitCount = 0;
            ccHintMsg = "SKIP";
            ccHintMs = 800;
            CC_ArmScan();
            break;

        case KEY_9:
            // Clear the temporary block list.
            ccBlockedCount = 0;
            ccHintMsg = "BLK CLEAR";
            ccHintMs = 800;
            break;

        case KEY_SIDE2:
            // BLOCK: never re-capture the shown/locked frequency this session,
            // then resume scanning.
            {
                uint32_t f = (ccDiscoveryView > 0 && ccDiscoveryView <= ccDiscoveryCount)
                                 ? ccDiscovery[ccDiscoveryView - 1]
                                 : (ccLocked ? ccFoundFreq : ccLastHitFreq);
                if (f != 0) {
                    CC_Block(f);
                    ccHintMsg = "BLOCKED";
                    ccHintMs = 800;
                    if (ccLocked && f == ccFoundFreq) CC_Release();
                    ccDiscoveryView = 0;
                    ccLastHitCount = 0;
                    CC_ArmScan();
                }
            }
            break;

        case KEY_F:
            // Cycle the screen backlight: full -> half -> off -> full.
            CC_CycleBacklight();
            break;

        default:
            break;
    }
    return true;
}

// ============================================================
// Main entry
// ============================================================
void APP_RunCloseCall(void)
{
    if (ccRunning) return;
    ccRunning = true;

    // Preserve whatever the main app was doing.
    GUI_DisplayType_t previousScreen = gScreenToDisplay;

    // Restore the persisted band preset and threshold.
    CC_LoadPersisted();

    // Enter a deterministic receive state.
    ccFirstUHF = CC_PresetSpansBoth(ccPreset) ? true : (ccPresets[ccPreset].start >= 28000000u);

    // Full reset of the scan state. There is no resume entry anymore: the lock
    // hold now lives entirely inside the loop below, so every entry starts a
    // fresh scanning session (discovery/log/blocked RAM state is kept where it
    // makes sense; discovery view and menu start closed).
    ccLocked = false;
    ccPaused = false;
    ccWeakSlices = 0;
    ccLockMs = 0;
    ccDiscoveryView = 0;
    ccBlockedCount = 0;
    ccDiscoveryCount = 0;
    ccLastHitCount = 0;
    ccLastHitFreq = 0;
    ccSeenFreq = 0;
    ccGoneMs = 0;
    ccAgeMs = CC_SHOWN_TIMEOUT_MS;   // start blank until the first real hit
    ccTuneMsg[0] = '\0';
    ccThrHintMs = 0;
    ccHintMsg = 0;
    ccHintMs = 0;
    ccCooldownMs = 0;
    ccFineTuneMs = 0;
    ccSilentMs = 0;
    ccLiveRssi = 0;
    memset(ccFailLedger, 0, sizeof(ccFailLedger));
    ccSkipStreak = 0;
    ccStaleMatchStreak = 0;
    ccCapFreq = 0;
    ccCapStreak = 0;
    ccStaleFires = 0;
    ccPathDwellMs = 0;
    ccDefFreq = 0;
    ccDefRssi = 0;
    ccDefMs = 0;
    ccTuned = false;
    ccMenuOpen = false;
    ccMenuSel = ccPreset;

    // Tone history: load the persisted ring and reset per-lock tone state.
    ccLogView = false;
    ccLogSel = 0;
    ccQualifyMs = 0;
    ccLogged = false;
    ccToneType = CC_LOG_TONE_NONE;
    ccToneCode = 0;
    ccTonePeekMs = 0;
    CC_LogLoad();

    // Start at full brightness and arm the two-stage auto-dim timer.
    ccDimLevel = 0;
    CC_ApplyDim();
    BACKLIGHT_Update();

    // Load the user's squelch threshold (persisted). Anything outside the valid
    // window is treated as "no valid setting" and reset to the safe default so
    // the UP key can never get stuck against a ceiling.
    if (ccThresholdDbm < CC_DBM_MIN || ccThresholdDbm > CC_DBM_MAX) {
        ccThresholdDbm = CC_DBM_DEFAULT;
    }
    CC_SquelchFromConfig();

    CC_OpenAudio(false);
    CC_ArmScan();

    uint32_t displayTimer = 0;
    KEY_Code_t lastKey = KEY_INVALID;
    uint16_t   holdCounter = 0;

    while (ccRunning) {
        KEY_Code_t Key = KEYBOARD_GetKey();

        if (Key == KEY_INVALID) {
            // Release: reset the edge detector so the next press is a new event.
            lastKey = KEY_INVALID;
            holdCounter = 0;
        } else if (Key != lastKey) {
            // New press. Fire once. UP/DOWN may auto-repeat after a hold delay.
            lastKey = Key;
            holdCounter = 0;
            if (!CC_HandleKey(Key, true, false)) break;
        } else {
            // Same key still held. Only UP/DOWN auto-repeat, and only after the
            // initial hold delay, then at a fixed interval - this prevents a
            // single tap from being seen as many rapid presses.
            if (Key == KEY_UP || Key == KEY_DOWN) {
                if (holdCounter < CC_KEY_REPEAT_DELAY) {
                    holdCounter++;
                } else {
                    holdCounter = CC_KEY_REPEAT_DELAY - CC_KEY_REPEAT_INTERVAL;
                    if (!CC_HandleKey(Key, true, true)) break;
                }
            }
        }

        if (ccCooldownMs >= 20) ccCooldownMs -= 20; else ccCooldownMs = 0;
        if (ccArmSettleMs >= 20) ccArmSettleMs -= 20; else ccArmSettleMs = 0;
        if (ccSkipMs >= 20) ccSkipMs -= 20; else ccSkipMs = 0;
        if (ccFineTuneMs >= 20) ccFineTuneMs -= 20; else ccFineTuneMs = 0;
        if (ccDefMs >= 20) ccDefMs -= 20; else ccDefMs = 0;
        // Band rotation guarantee for dual-span presets: left alone, every
        // restart path keeps the current LNA path, so one busy band can circle
        // forever while the other side never gets a pass. Force a path switch
        // when the last capture is this old (capture-safe: not while locked,
        // settled windows restart the dwell via CC_RestartCounter's arming).
        if (!ccLocked && CC_PresetSpansBoth(ccPreset)) {
            if (ccArmSettleMs > 0) {
                ccPathDwellMs = 0;
            } else {
                ccPathDwellMs += 20;
                if (ccPathDwellMs >= CC_PATH_DWELL_MS) {
                    ccPathDwellMs = 0;
                    ccFirstUHF = !ccFirstUHF;
                    CC_RestartCounter();
                }
            }
        }
        for (unsigned li = 0; li < CC_FAIL_LEDGER_N; li++) {
            if (ccFailLedger[li].ms >= 20) {
                ccFailLedger[li].ms -= 20;
            } else if (ccFailLedger[li].ms != 0) {
                ccFailLedger[li].ms = 0;
                ccFailLedger[li].freq = 0;
                ccFailLedger[li].count = 0;
            }
        }

        // Live signal sample for the dBm row: the raw REG_67 reading of whatever
        // the radio is currently tuned to (counter freq while scanning, carrier
        // while holding).
        ccLiveRssi = BK4819_GetRSSI();

        // Age the displayed frequency. Any fresh counter hit or an active lock
        // resets it (see CC_ProcessResult); once it exceeds CC_SHOWN_TIMEOUT_MS
        // with no signal the shown MHz is blanked.
        if (ccAgeMs < 0xFFFFu) ccAgeMs += 20;

        // Backlight: any key activity (a new press or a key still held) wakes the
        // screen to full brightness and restarts the idle timer, so the display
        // never dims while the user is pressing buttons. Auto-dim only advances
        // during true idle, then two-stage: full -> half -> off.
        BACKLIGHT_Update();
        if (Key != KEY_INVALID) {
            if (ccDimLevel != 0) CC_ApplyDim();
            else ccDimMs = 0;
        } else {
            ccDimMs += 20;
            if (ccDimLevel == 0 && ccDimMs >= CC_DIM1_MS) {
                ccDimLevel = 1;
                CC_ApplyDim();
            } else if (ccDimLevel == 1 && ccDimMs >= CC_DIM2_MS) {
                ccDimLevel = 2;
                CC_ApplyDim();
            }
        }

        // While the band picker is open, pause capture/hold and just draw the list.
        if (ccLogView) {
            displayTimer += 20;
            if (displayTimer >= 150) {
                displayTimer = 0;
                CC_RenderLog();
            }
            SYSTEM_DelayMs(20);
            continue;
        }

        // While the band picker is open, pause capture/hold and just draw the list.
        if (ccMenuOpen) {
            displayTimer += 20;
            if (displayTimer >= 150) {
                displayTimer = 0;
                CC_RenderMenu();
            }
            SYSTEM_DelayMs(20);
            continue;
        }

        if (!ccLocked) {
            // Poll the hardware counter (chip gates a capture over 0.2/0.4 s).
            CC_ProcessResult();
        } else {
            // Holding: stay on the signal through speech gaps. The carrier must be
            // sustainedly absent (CC_RELEASE_DWELL_MS below the threshold) before
            // we release, so normal talking (syllable pauses) never tears the lock
            // down. Logging never triggers a release/re-arm.
            uint16_t rssi = BK4819_GetRSSI();
            ccRssi = rssi;
            ccLockMs += 20;

#ifdef ENABLE_AM_FIX
            // The normal 10 ms AM servo never runs here (this loop blocks the
            // main timeslice), so drive it directly for an airband lock.
            if (CC_IsAirbandForFreq(ccFoundFreq)) AM_Fix_Process(ccFoundFreq);
#endif

            // Poll the CTCSS/DCS detector (only meaningful while the carrier is
            // present); it needs a short settle before the first read.
            if (ccTonePeekMs > 0) {
                if (ccTonePeekMs >= 20) ccTonePeekMs -= 20; else ccTonePeekMs = 0;
            } else {
                CC_TonePoll();
            }

            // "Gone" = the carrier has dropped CC_RELEASE_MARGIN dB below the
            // user's threshold.
            uint16_t goneFloor = (ccSquelchRssi > (uint16_t)(CC_RELEASE_MARGIN * 2))
                                     ? (uint16_t)(ccSquelchRssi - (CC_RELEASE_MARGIN * 2))
                                     : 0;
            bool gone = (rssi < goneFloor);

            // "Silent" = carrier gone OR (airband: the AM noise gate has muted
            // the audio). An inaudible hold is worthless: after
            // CC_SILENT_RELEASE_MS of silence the lock is dropped and scanning
            // resumes (same skip-suppression as the hard cap so a persistently
            // weak carrier cannot instantly re-lock and stall the screen).
            bool audible = !gone;
#ifdef ENABLE_AM_FIX
            if (audible && CC_IsAirbandForFreq(ccFoundFreq) && !AM_fix_GateIsOpen()) {
                audible = false;
            }
#endif
            if (audible) {
                ccSilentMs = 0;
            } else if (ccSilentMs < 0xFFFFu) {
                ccSilentMs += 20;
            }

            if (!gone) {
                ccWeakSlices = 0;
                ccGoneMs     = 0;           // any recovery cancels the dwell
                // Qualify: only log once the signal has been continuously present
                // for CC_LOG_TIME_MS, so brief blips are never stored.
                if (!ccLogged) {
                    ccQualifyMs += 20;
                    if (ccQualifyMs >= CC_LOG_TIME_MS) {
                        ccLogged = true;
                        CC_LogAdd(ccFoundFreq, ccRssi, ccToneType, ccToneCode);
                        ccHintMsg = "LOGGED";
                        ccHintMs  = 800;
                    }
                }
            } else {
                ccWeakSlices++;
                ccQualifyMs = 0;
                // Accumulate sustained-absence time; only release once the signal
                // has really gone (dwell), not on a momentary fade.
                ccGoneMs += 20;

                // Fine-tune: within -10 dBm of the floor, keep re-tuning to the
                // strongest nearby carrier (drifting / off-center captures). A
                // successful re-tune resets the dwell so the lock survives;
                // otherwise the dwell release below proceeds normally.
                if (rssi < (uint16_t)(ccSquelchRssi - CC_FINETUNE_MARGIN) &&
                    ccFineTuneMs == 0) {
                    ccFineTuneMs = CC_FINETUNE_RETRY_MS;
                    if (CC_FineTuneHold()) {
                        ccWeakSlices = 0;
                        ccGoneMs     = 0;
                    }
                }

                if (ccGoneMs >= CC_RELEASE_DWELL_MS &&
                    ccWeakSlices >= CC_UNLOCK_FADE &&
                    ccLockMs >= CC_MIN_LISTEN_MS) {
                    CC_Release();
                    ccLastHitCount = 0;
                    ccLastHitFreq = 0;
                    ccQualifyMs = 0;
                    ccGoneMs = 0;
                    ccLogged = false;
                    CC_ArmScan();
                    goto cc_after_hold;
                }
            }

            // Hard cap / silence release: resume scanning regardless, so it can
            // never sit locked on a dead frequency (too long) or an inaudible
            // carrier (no sound at all). Repeated caps of the SAME carrier
            // escalate the quiet window (1.5 s -> 5 s -> 15 s) and finally
            // auto-block it, so a persistent strong carrier cannot pin the
            // sweep in a re-lock loop that only SKIP could escape.
            if (ccLocked && (ccLockMs >= CC_MAX_LISTEN_MS ||
                             ccSilentMs >= CC_SILENT_RELEASE_MS)) {
                if (ccCapFreq == ccFoundFreq) {
                    ccCapStreak++;
                } else {
                    ccCapFreq = ccFoundFreq;
                    ccCapStreak = 1;
                }
                if (ccCapStreak >= CC_CAP_AUTOBLOCK) {
                    CC_Block(ccFoundFreq);
                    ccHintMsg = "BLOCKED";
                    ccHintMs  = 1000;
                    ccCapFreq = 0;
                    ccCapStreak = 0;
                    ccSkipFreq = 0;
                    ccSkipMs = 0;
                } else {
                    ccSkipFreq = ccFoundFreq;
                    ccSkipMs = (ccCapStreak == 1) ? CC_SKIP_QUIET_MS :
                               (ccCapStreak == 2) ? CC_CAP_ESCALATE_MS :
                                                    CC_CAP_FINAL_MS;
                }
                CC_Release();
                // Suppress this frequency briefly so an endlessly-present weak
                // carrier that never trips the dwell release cannot instantly
                // re-lock and loop; a genuine re-key after it drops still opens.
                ccSkipFreq = ccFoundFreq;
                ccSkipMs   = CC_SKIP_QUIET_MS;
                CC_Release();
                ccLastHitCount = 0;
                ccLastHitFreq = 0;
                ccQualifyMs = 0;
                ccGoneMs = 0;
                ccLogged = false;
                CC_ArmScan();
            }
        }
    cc_after_hold:
        ;

        // Re-render a few times a second, and immediately while a live overlay
        // (threshold nudge or SKIP/BLOCK feedback) is showing so it is visible.
        if (ccHintMs > 0 && !ccHintMsg) ccHintMs = 0;
        if (ccThrHintMs > 0 || ccHintMs > 0) {
            if (ccThrHintMs >= 20) ccThrHintMs -= 20; else ccThrHintMs = 0;
            if (ccHintMs >= 20) ccHintMs -= 20; else ccHintMs = 0;
            displayTimer = 0;
            CC_Render();
        } else {
            displayTimer += 20;
            if (displayTimer >= 150) {
                displayTimer = 0;
                CC_Render();
            }
        }

        SYSTEM_DelayMs(20);
    }

    BK4819_SetFrequencyScan(false);

    if (ccTuned) {
        // Leaving with a verified tune in the VFO: persist it like a manual
        // tune (this is the ONLY flash write of the session, so frequent
        // locks/re-locks cannot wear the flash), then let the normal app
        // reconfigure the receiver onto it. The user exits to the main screen
        // listening to the last found frequency.
        VFO_Info_t *vfo = &gEeprom.VfoInfo[0];
        const bool air = CC_IsAirbandForFreq(ccFoundFreq);
        vfo->freq_config_RX.Frequency = ccFoundFreq;
        vfo->Modulation               = air ? MODULATION_AM : MODULATION_FM;
        vfo->CHANNEL_BANDWIDTH        = air ? BK4819_FILTER_BW_WIDE : BK4819_FILTER_BW_NARROW;
        SETTINGS_SaveChannel(FREQ_CHANNEL, 0, vfo, 2);
        SETTINGS_SaveVfoIndices();
        gVfoConfigureMode = VFO_CONFIGURE_RELOAD;
    }

    // Mute and restore whatever screen the caller had. When a lock was active
    // the app's reconfigure pass (above) rebuilds the full normal RX path on
    // the same carrier.
    CC_OpenAudio(false);
    gRequestDisplayScreen = previousScreen;
    gUpdateDisplay = true;
}
