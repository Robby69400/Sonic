/* Copyright 2026
 *
 * Licensed under the Apache License, Version 2.0.
 */

#ifdef ENABLE_FEAT_F4HWN_HISTORY_LOG

#include <assert.h>
#include <stddef.h>
#include <string.h>

#include "app/common.h"
#include "app/generic.h"
#include "app/HISTORY_log.h"
#include "audio.h"
#include "driver/bk4819.h"
#include "driver/py25q16.h"
#include "driver/st7565.h"
#include "external/printf/printf.h"
#include "helper/battery.h"
#include "misc.h"
#include "settings.h"
#include "ui/helper.h"
#include "ui/menu.h"
#include "ui/ui.h"

#define HISTORY_LOG_FLASH_BASE          0x1E0000u
#define HISTORY_LOG_FLASH_SECTOR_SIZE   0x1000u
#define HISTORY_LOG_FLASH_SECTOR_COUNT  8u
#define HISTORY_LOG_FLASH_SIZE          (HISTORY_LOG_FLASH_SECTOR_SIZE * HISTORY_LOG_FLASH_SECTOR_COUNT)
#define HISTORY_LOG_FLASH_END           (HISTORY_LOG_FLASH_BASE + HISTORY_LOG_FLASH_SIZE)
#define HISTORY_LOG_SLOT_COUNT          (HISTORY_LOG_FLASH_SIZE / sizeof(HISTORY_LogFlashEntry_t))
#define HISTORY_LOG_VIEW_CACHE_COUNT    7u
#define HISTORY_LOG_VIEW_SCAN_BUDGET    8u
#define HISTORY_LOG_VIEW_ANCHOR_STRIDE  32u
#define HISTORY_LOG_VIEW_ANCHOR_COUNT   ((HISTORY_LOG_SLOT_COUNT + HISTORY_LOG_VIEW_ANCHOR_STRIDE - 1u) / HISTORY_LOG_VIEW_ANCHOR_STRIDE)
#define HISTORY_LOG_ENTRY_COMMIT        0xA5u
#define HISTORY_LOG_CHANNEL_NONE        0xFFFFu
#define HISTORY_LOG_FLAG_TX             (1u << 0)
// (1u << 1) was FLAG_NAMED, retired: names are resolved from the channel.
#define HISTORY_LOG_FLAG_MONITOR        (1u << 2)
#define HISTORY_LOG_FLAG_SESSION        (1u << 3)
#define HISTORY_LOG_FILTER_ALL          0u
#define HISTORY_LOG_FILTER_RX           1u
#define HISTORY_LOG_FILTER_TX           2u
#define HISTORY_LOG_SMETER_UNKNOWN      0xFFu
// battVolt stores centivolts above 6.00 V, saturating at 8.54 V (254).
#define HISTORY_LOG_BATT_UNKNOWN        0xFFu
#define HISTORY_LOG_BATT_OFFSET         600u
#define HISTORY_LOG_DETAIL_DURATION     0u
#define HISTORY_LOG_DETAIL_SMETER       1u
#define HISTORY_LOG_DETAIL_BATT         2u

typedef struct __attribute__((packed)) {
    // Display fields come first: they form the prefix mirrored by
    // HISTORY_LogEntry_t and copied to the view cache in one pass. Scan-only
    // fields (sequence) sit past the prefix so RAM does not carry them.
    uint32_t HFreqs :31;
    uint32_t HBlacklisted : 1;
    uint16_t HTimeS;
    uint8_t  HCode;
    uint8_t  crc;
} HISTORY_LogFlashEntry_t;

static_assert(sizeof(HISTORY_LogFlashEntry_t) == 32);
static_assert(offsetof(HISTORY_LogFlashEntry_t, sequence) % 4 == 0);
static_assert(HISTORY_LOG_VIEW_ANCHOR_COUNT <= 32);

#define HISTORY_LOG_ENTRY_COPY_SIZE offsetof(HISTORY_LogFlashEntry_t, pad)

// HISTORY_LogEntry_t (RAM) and HISTORY_LogFlashEntry_t must stay byte-identical for
// the copied prefix.
static_assert(HISTORY_LOG_ENTRY_COPY_SIZE == offsetof(HISTORY_LogEntry_t, battVolt) + sizeof(((HISTORY_LogEntry_t *)0)->battVolt));
static_assert(sizeof(HISTORY_LogEntry_t) >= HISTORY_LOG_ENTRY_COPY_SIZE);

static HISTORY_LogEntry_t gViewCache[HISTORY_LOG_VIEW_CACHE_COUNT];
static uint16_t        gViewCacheStart;
static uint8_t         gViewCacheCount;
static uint8_t         gViewCacheFilter;
static bool            gViewCacheHasOlder;
static bool            gViewCacheComplete;
static bool            gViewScanActive;
static uint16_t        gViewScanSlot;
static uint16_t        gViewScanScanned;
static uint16_t        gViewScanSkip;
static uint16_t        gViewScanIndex;
static uint16_t        gViewAnchorSlots[HISTORY_LOG_VIEW_ANCHOR_COUNT];
static uint32_t        gViewAnchorMask;
static uint8_t         gViewAnchorFilter;
static bool            gClearActive;
static bool            gClearConfirmActive;
static uint8_t         gClearSector;
static bool            gMenuClearHandled;
static bool            gLogHasTraffic;
static uint32_t        gNextSequence;
static uint32_t        gNextTrafficSequence;
static uint32_t        gNextFlashAddress;

static bool            gSessionActive;
static uint8_t         gSessionFlags;
static uint32_t        gSessionFrequency;
static uint16_t        gSessionChannel;
static uint16_t        gSessionTicks500ms;
static uint8_t         gSessionSMeter;
static uint8_t         gSessionBattVolt;

static uint16_t        gLogCursor;
static uint8_t         gLogFilter;
static uint8_t         gLogDetailMode;

static uint8_t HISTORY_LOG_Crc8(const void *data, uint16_t size)
{
    const uint8_t *p = (const uint8_t *)data;
    uint8_t crc = 0x5Au;

    while (size-- > 0) {
        crc ^= *p++;
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x31u) : (uint8_t)(crc << 1);
        }
    }

    return crc;
}

static bool HISTORY_LOG_IsValidFlashEntry(const HISTORY_LogFlashEntry_t *entry)
{
    if (entry->commit != HISTORY_LOG_ENTRY_COMMIT ||
        entry->sequence == 0xFFFFFFFFu ||
        entry->frequency == 0xFFFFFFFFu)
        return false;

    return entry->crc == HISTORY_LOG_Crc8(entry, sizeof(*entry) - 2);
}

static bool HISTORY_LOG_IsBlankFlashEntry(const HISTORY_LogFlashEntry_t *entry)
{
    const uint8_t *p = (const uint8_t *)entry;

    for (uint8_t i = 0; i < sizeof(*entry); i++) {
        if (p[i] != 0xFFu)
            return false;
    }

    return true;
}

static uint32_t HISTORY_LOG_SlotToAddress(uint16_t slot)
{
    return HISTORY_LOG_FLASH_BASE + ((uint32_t)slot * sizeof(HISTORY_LogFlashEntry_t));
}

static uint16_t HISTORY_LOG_AddressToSlot(uint32_t address)
{
    return (uint16_t)((address - HISTORY_LOG_FLASH_BASE) / sizeof(HISTORY_LogFlashEntry_t));
}

static uint16_t HISTORY_LOG_PreviousSlot(uint16_t slot)
{
    return slot == 0 ? (uint16_t)(HISTORY_LOG_SLOT_COUNT - 1u) : (uint16_t)(slot - 1u);
}

static uint16_t HISTORY_LOG_NextSlot(uint16_t slot)
{
    return (uint16_t)(slot + 1u) >= HISTORY_LOG_SLOT_COUNT ? 0 : (uint16_t)(slot + 1u);
}


static void HISTORY_LOG_InvalidateViewCache(void)
{
    gViewCacheStart    = 0xFFFFu;
    gViewCacheCount    = 0;
    gViewCacheFilter   = 0xFFu;
    gViewCacheHasOlder = false;
    gViewCacheComplete = false;
    gViewScanActive    = false;    
}

static void HISTORY_LOG_ResetLogCounters(void)
{
    gClearSector         = 0;
    gLogHasTraffic       = false;
    gNextSequence        = 0;
    gNextTrafficSequence = 0;
    gNextFlashAddress    = HISTORY_LOG_FLASH_BASE;
    HISTORY_LOG_InvalidateViewCache();
}

static void HISTORY_LOG_StartClear(void)
{
    if (gClearActive)
        return;

    gClearActive        = true;
    gClearConfirmActive = false;
    gSessionActive      = false;
    gLogCursor          = 0;
    HISTORY_LOG_ResetLogCounters();
}

static void HISTORY_LOG_CancelClearConfirm(void)
{
    if (!gClearConfirmActive)
        return;

    gClearConfirmActive = false;
    gUpdateDisplay = true;
}

static void HISTORY_LOG_StepClear(void)
{
    if (!gClearActive)
        return;

    PY25Q16_SectorErase(HISTORY_LOG_FLASH_BASE + ((uint32_t)gClearSector * HISTORY_LOG_FLASH_SECTOR_SIZE));

    gClearSector++;
    if (gClearSector >= HISTORY_LOG_FLASH_SECTOR_COUNT) {
        gClearActive = false;
        HISTORY_LOG_ResetLogCounters();
    }
}

static uint16_t HISTORY_LOG_PageStart(uint16_t indexFromNewest)
{
    if (indexFromNewest >= HISTORY_LOG_SLOT_COUNT)
        indexFromNewest = HISTORY_LOG_SLOT_COUNT - 1u;

    return indexFromNewest;
}

static bool HISTORY_LOG_AlignLastViewPage(void)
{
    if (gViewScanActive ||
        !gViewCacheComplete ||
        gViewCacheHasOlder ||
        gViewCacheCount == 0)
        return false;

    if (gViewCacheCount >= HISTORY_LOG_VIEW_CACHE_COUNT) {
        if (gLogCursor == gViewCacheStart)
            return false;
        gLogCursor = gViewCacheStart;
        return true;
    }

    const uint16_t missingRows = HISTORY_LOG_VIEW_CACHE_COUNT - gViewCacheCount;
    const uint16_t start = gViewCacheStart > missingRows ? (uint16_t)(gViewCacheStart - missingRows) : 0;
    if (start == gViewCacheStart)
        return false;

    gLogCursor = start;
    HISTORY_LOG_StartViewCacheScan(start, false, false);
    return true;
}

static void HISTORY_LOG_StopViewScan(void)
{
    gViewScanActive    = false;
    gViewCacheComplete = true;
}

static void HISTORY_LOG_StepViewCacheScan(void)
{
    uint8_t budget = HISTORY_LOG_VIEW_SCAN_BUDGET;
    bool capReached = false;

    while (gViewScanActive && budget-- > 0 && gViewScanScanned < HISTORY_LOG_SLOT_COUNT) {
        HISTORY_LogFlashEntry_t flashEntry;

        gViewScanSlot = HISTORY_LOG_PreviousSlot(gViewScanSlot);
        gViewScanScanned++;

        PY25Q16_ReadBuffer(HISTORY_LOG_SlotToAddress(gViewScanSlot), &flashEntry, sizeof(flashEntry));
        if (HISTORY_LOG_IsBlankFlashEntry(&flashEntry)) {
            gViewScanScanned = HISTORY_LOG_SLOT_COUNT;
            break;
        }

        if (!HISTORY_LOG_IsValidFlashEntry(&flashEntry) ||
            !HISTORY_LOG_MatchesFlags(flashEntry.flags))
            continue;

        if (HISTORY_LOG_IsTrafficFlags(flashEntry.flags) &&
            (gNextTrafficSequence - 1u - flashEntry.trafficSeq) >= HISTORY_LOG_VISIBLE_COUNT) {
            capReached = true;
            HISTORY_LOG_StopViewScan();
            break;
        }

        HISTORY_LOG_RecordViewAnchor(gViewScanIndex, gViewScanSlot);

        if (gViewScanSkip > 0) {
            gViewScanSkip--;
            gViewScanIndex++;
            continue;
        }

        if (gViewCacheCount < HISTORY_LOG_VIEW_CACHE_COUNT) {
            memcpy(&gViewCache[gViewCacheCount], &flashEntry, HISTORY_LOG_ENTRY_COPY_SIZE);
            gViewCacheCount++;
        } else {
            gViewCacheHasOlder = true;
            HISTORY_LOG_StopViewScan();
            break;
        }

        gViewScanIndex++;
    }

    if (gViewScanScanned >= HISTORY_LOG_SLOT_COUNT || capReached) {
        HISTORY_LOG_StopViewScan();
        // A jump-to-end aims past the last row on purpose: when the whole
        // scan was spent skipping, the leftover skip locates the actual
        // last row, so retarget the view there.
        if (gViewCacheCount == 0 &&
            gViewScanSkip > 0 &&
            gViewScanSkip < gViewCacheStart) {
            HISTORY_LOG_StartCursorView((uint16_t)(gViewCacheStart - gViewScanSkip - 1u));
            return;
        }
    }

    if (HISTORY_LOG_AlignLastViewPage())
        return;

}

static void HISTORY_LOG_StartViewCacheScan(uint16_t start, bool circular, bool discoverTotal)
{
    uint16_t anchorIndex;
    uint16_t anchorSlot;

    start = HISTORY_LOG_PageStart(start);
    HISTORY_LOG_EnsureViewAnchors();

    gViewCacheStart    = start;
    gViewCacheCount    = 0;
    gViewCacheFilter   = gLogFilter;
    gViewCacheHasOlder = false;
    gViewCacheComplete = false;
    gViewScanActive    = false;
    (void)circular;
    (void)discoverTotal;
    if (gNextSequence == 0) {
        gViewCacheComplete = true;
        return;
    }

    if (HISTORY_LOG_FindViewAnchor(start, &anchorIndex, &anchorSlot)) {
        gViewScanSlot  = HISTORY_LOG_NextSlot(anchorSlot);
        gViewScanSkip  = start - anchorIndex;
        gViewScanIndex = anchorIndex;
    } else {
        gViewScanSlot  = HISTORY_LOG_AddressToSlot(gNextFlashAddress);
        gViewScanSkip  = start;
        gViewScanIndex = 0;
    }

    gViewScanScanned = 0;
    gViewScanActive  = true;
}

static void HISTORY_LOG_GoToLastRow(void)
{
    // Aim past the end; the scan-completion retarget in StepViewCacheScan
    // snaps the cursor back to the last existing row.
    HISTORY_LOG_StartCursorView(HISTORY_LOG_SLOT_COUNT - 1u);
}

static bool HISTORY_LOG_ViewCacheCovers(uint16_t indexFromNewest)
{
    return gViewCacheFilter == gLogFilter &&
           indexFromNewest >= gViewCacheStart &&
           indexFromNewest < (uint16_t)(gViewCacheStart + gViewCacheCount);
}

static bool HISTORY_LOG_SlotIsBlank(uint32_t address)
{
    HISTORY_LogFlashEntry_t entry;

    PY25Q16_ReadBuffer(address, &entry, sizeof(entry));

    return HISTORY_LOG_IsBlankFlashEntry(&entry);
}

static void HISTORY_LOG_PrepareNextSlot(void)
{
    if (gNextFlashAddress >= HISTORY_LOG_FLASH_END)
        gNextFlashAddress = HISTORY_LOG_FLASH_BASE;

    if (!HISTORY_LOG_SlotIsBlank(gNextFlashAddress)) {
        if ((gNextFlashAddress % HISTORY_LOG_FLASH_SECTOR_SIZE) != 0) {
            gNextFlashAddress += HISTORY_LOG_FLASH_SECTOR_SIZE - (gNextFlashAddress % HISTORY_LOG_FLASH_SECTOR_SIZE);
            if (gNextFlashAddress >= HISTORY_LOG_FLASH_END)
                gNextFlashAddress = HISTORY_LOG_FLASH_BASE;
        }

        if (!HISTORY_LOG_SlotIsBlank(gNextFlashAddress)) {
            const uint32_t sector = gNextFlashAddress - (gNextFlashAddress % HISTORY_LOG_FLASH_SECTOR_SIZE);
            PY25Q16_SectorErase(sector);
        }
    }
}

static void HISTORY_LOG_AdvanceFlashAddress(void)
{
    gNextFlashAddress += sizeof(HISTORY_LogFlashEntry_t);
    if (gNextFlashAddress >= HISTORY_LOG_FLASH_END)
        gNextFlashAddress = HISTORY_LOG_FLASH_BASE;
}

static void HISTORY_LOG_WriteEntry(const HISTORY_LogEntry_t *src)
{
    HISTORY_LogFlashEntry_t entry;
    uint8_t commit = HISTORY_LOG_ENTRY_COMMIT;

    memset(&entry, 0xFF, sizeof(entry));
    memcpy(&entry, src, HISTORY_LOG_ENTRY_COPY_SIZE);
    entry.sequence = gNextSequence++;
    entry.crc = HISTORY_LOG_Crc8(&entry, sizeof(entry) - 2);

    HISTORY_LOG_PrepareNextSlot();

    PY25Q16_WriteBuffer(gNextFlashAddress, &entry, sizeof(entry), false);
    PY25Q16_WriteBuffer(gNextFlashAddress + sizeof(entry) - 1u, &commit, 1, false);
    HISTORY_LOG_AdvanceFlashAddress();
}

static void HISTORY_LOG_WriteSessionMarker(void)
{
    HISTORY_LogEntry_t entry;

    memset(&entry, 0, sizeof(entry));
    // Markers share the ordinal of the next traffic row. Their flash
    // sequence remains unique and is used to order them in K5Viewer.
    entry.trafficSeq = gNextTrafficSequence;
    entry.channel  = HISTORY_LOG_CHANNEL_NONE;
    entry.flags    = HISTORY_LOG_FLAG_SESSION;
    entry.sMeter   = HISTORY_LOG_SMETER_UNKNOWN;
    entry.battVolt = HISTORY_LOG_BATT_UNKNOWN;

    HISTORY_LOG_WriteEntry(&entry);
    HISTORY_LOG_InvalidateViewCache();
}

static void HISTORY_LOG_EnsureViewCache(void)
{
    const uint16_t pageStart = HISTORY_LOG_PageStart(gLogCursor);

    if (gViewCacheFilter == gLogFilter &&
        gViewCacheStart == pageStart &&
        (gViewScanActive ||
         gViewCacheComplete ||
         HISTORY_LOG_ViewCacheCovers(gLogCursor)))
        return;

    HISTORY_LOG_StartCursorView(gLogCursor);
}

static bool HISTORY_LOG_GetFilteredEntry(uint16_t indexFromNewest, HISTORY_LogEntry_t *entry)
{
    if (indexFromNewest >= HISTORY_LOG_SLOT_COUNT)
        return false;

    if (!HISTORY_LOG_ViewCacheCovers(indexFromNewest))
        return false;

    *entry = gViewCache[indexFromNewest - gViewCacheStart];
    return true;
}

static void HISTORY_LOG_CaptureSession(uint8_t flags, const VFO_Info_t *vfo)
{
    if (gClearActive)
        return;

    const uint32_t frequency = (flags & HISTORY_LOG_FLAG_TX) ? vfo->pTX->Frequency : vfo->pRX->Frequency;
    const bool isMemoryChannel = IS_MR_CHANNEL(vfo->CHANNEL_SAVE);
    const uint16_t channel = isMemoryChannel ? vfo->CHANNEL_SAVE : HISTORY_LOG_CHANNEL_NONE;

    if (gSessionActive &&
        gSessionFlags == flags &&
        gSessionFrequency == frequency &&
        gSessionChannel == channel)
        return;

    HISTORY_LOG_EndActive();

    gSessionActive     = true;
    gSessionFlags      = flags;
    gSessionFrequency  = frequency;
    gSessionChannel    = channel;
    gSessionTicks500ms = 0;
    // TX sessions repurpose the sMeter byte to store the TX power level
    // (OUTPUT_POWER, indexes gSubMenu_TXP); RX sessions track the S-meter.
    gSessionSMeter     = (flags & HISTORY_LOG_FLAG_TX) ? vfo->OUTPUT_POWER : HISTORY_LOG_SMETER_UNKNOWN;
    gSessionBattVolt   = HISTORY_LOG_BATT_UNKNOWN;
    HISTORY_LOG_UpdateSessionMeters();
}

void HISTORY_LOG_Init(void)
{
    uint32_t maxSequence = 0;
    uint32_t maxAddress = HISTORY_LOG_FLASH_BASE;
    uint32_t maxTrafficSeq = 0;
    uint8_t lastEntryFlags = 0;
    bool found = false;
    bool foundTraffic = false;

    gLogCursor        = 0;
    gLogFilter        = HISTORY_LOG_FILTER_ALL;
    gSessionActive    = false;
    gSessionSMeter    = HISTORY_LOG_SMETER_UNKNOWN;
    gSessionBattVolt  = HISTORY_LOG_BATT_UNKNOWN;
    gClearActive        = false;
    gClearConfirmActive = false;
    gClearSector        = 0;
    gMenuClearHandled   = false;
    gLogDetailMode      = HISTORY_LOG_DETAIL_DURATION;
    gLogHasTraffic      = false;
    gNextFlashAddress   = HISTORY_LOG_FLASH_BASE;
    HISTORY_LOG_InvalidateViewCache();

    for (uint32_t address = HISTORY_LOG_FLASH_BASE; address < HISTORY_LOG_FLASH_END; address += sizeof(HISTORY_LogFlashEntry_t)) {
        HISTORY_LogFlashEntry_t flashEntry;

        PY25Q16_ReadBuffer(address, &flashEntry, sizeof(flashEntry));
        if (!HISTORY_LOG_IsValidFlashEntry(&flashEntry))
            continue;

        if (HISTORY_LOG_IsTrafficFlags(flashEntry.flags)) {
            gLogHasTraffic = true;
            if (!foundTraffic || flashEntry.trafficSeq > maxTrafficSeq) {
                foundTraffic = true;
                maxTrafficSeq = flashEntry.trafficSeq;
            }
        }

        if (!found || flashEntry.sequence > maxSequence) {
            found = true;
            maxSequence = flashEntry.sequence;
            maxAddress = address;
            lastEntryFlags = flashEntry.flags;
        }
    }

    if (found) {
        gNextSequence = maxSequence + 1u;
        gNextFlashAddress = maxAddress + sizeof(HISTORY_LogFlashEntry_t);
        if (gNextFlashAddress >= HISTORY_LOG_FLASH_END)
            gNextFlashAddress = HISTORY_LOG_FLASH_BASE;
    } else {
        gNextSequence = 0;
    }

    gNextTrafficSequence = foundTraffic ? maxTrafficSeq + 1u : 0;

    // Skip the marker if the log already ends with one (e.g. repeated
    // reboots with no RX/TX in between) to avoid stacking empty separators.
    if (!found || (lastEntryFlags & HISTORY_LOG_FLAG_SESSION) == 0)
        HISTORY_LOG_WriteSessionMarker();
}

void HISTORY_LOG_BeginRx(const VFO_Info_t *vfo, FUNCTION_Type_t function)
{
    if (vfo == NULL)
        return;

    uint8_t flags = 0;
    if (function == FUNCTION_MONITOR)
        flags |= HISTORY_LOG_FLAG_MONITOR;

    HISTORY_LOG_CaptureSession(flags, vfo);
}

void HISTORY_LOG_BeginTx(const VFO_Info_t *vfo)
{
    if (vfo == NULL)
        return;

    HISTORY_LOG_CaptureSession(HISTORY_LOG_FLAG_TX, vfo);
}

void HISTORY_LOG_EndActive(void)
{
    if (!gSessionActive)
        return;

    if (gClearActive) {
        gSessionActive   = false;
        gSessionSMeter   = HISTORY_LOG_SMETER_UNKNOWN;
        gSessionBattVolt = HISTORY_LOG_BATT_UNKNOWN;
        return;
    }

    HISTORY_LOG_UpdateSessionMeters();

    HISTORY_LogEntry_t entry;
    memset(&entry, 0, sizeof(entry));

    entry.trafficSeq      = gNextTrafficSequence++;
    entry.frequency       = gSessionFrequency;
    entry.durationSeconds = (gSessionTicks500ms + 1u) / 2u;
    entry.channel         = gSessionChannel;
    entry.flags           = gSessionFlags;
    entry.sMeter          = gSessionSMeter;
    entry.battVolt        = gSessionBattVolt;

    if (entry.durationSeconds == 0)
        entry.durationSeconds = 1;

    HISTORY_LOG_WriteEntry(&entry);
    gLogHasTraffic = true;
    HISTORY_LOG_InvalidateViewCache();

    gSessionActive   = false;
    gSessionSMeter   = HISTORY_LOG_SMETER_UNKNOWN;
    gSessionBattVolt = HISTORY_LOG_BATT_UNKNOWN;
}

void HISTORY_LOG_Tick500ms(void)
{
    if (gSessionActive) {
        HISTORY_LOG_UpdateSessionMeters();
        if (gSessionTicks500ms < 0xFFFEu)
            gSessionTicks500ms++;
    }
}

void HISTORY_LOG_Task10ms(void)
{
    if (gClearActive) {
        HISTORY_LOG_StepClear();
    } else if (gViewScanActive) {
        HISTORY_LOG_StepViewCacheScan();
    } else {
        return;
    }

    if (gScreenToDisplay == DISPLAY_HISTORY_LOG)
        gUpdateDisplay = true;
}

void ACTION_RxTxLog(void)
{
    gLogCursor = 0;
    gLogDetailMode = HISTORY_LOG_DETAIL_DURATION;
    gClearConfirmActive = false;
    gMenuClearHandled = false;
    HISTORY_LOG_InvalidateViewCache();
    gUpdateStatus = true;
    GUI_SelectNextDisplay(DISPLAY_HISTORY_LOG);
}

void UI_DisplayRxTxLog(void)
{
    char detail[8];
    char title[16];
    HISTORY_LogEntry_t entry;

    UI_DisplayClear();

    if (gClearActive) {
        HISTORY_LOG_ShowEmpty(true);
        return;
    }

    if (gClearConfirmActive) {
        HISTORY_LOG_ShowClearConfirm();
        return;
    }

    HISTORY_LOG_EnsureViewCache();

    if (gLogFilter == HISTORY_LOG_FILTER_ALL && !gLogHasTraffic) {
        HISTORY_LOG_ShowEmpty(true);
        return;
    }

    if (gViewCacheCount == 0) {
        HISTORY_LOG_ShowEmpty(!gViewScanActive);
        return;
    }

    for (uint8_t row = 0; row < HISTORY_LOG_VIEW_CACHE_COUNT; row++) {
        {
            const uint16_t index = gLogCursor + row;
            if (!HISTORY_LOG_GetFilteredEntry(index, &entry))
                break;
        }

        if (HISTORY_LOG_IsSessionMarker(&entry)) {
            HISTORY_LOG_DrawSessionMarker(row);
            continue;
        }

        const bool isTx = HISTORY_LOG_IsTx(&entry);

        HISTORY_LOG_FormatTitle(&entry, title);
        HISTORY_LOG_DrawIndexBadge((uint16_t)(gNextTrafficSequence - 1u - entry.trafficSeq), row);

        if (isTx)
            UI_PrintStringSmallBold(title, 17, 0, row);
        else
            UI_PrintStringSmallNormal(title, 17, 0, row);

        GUI_DisplaySmallest(isTx ? "TX" : "RX", 95, (uint8_t)((row * 8u) + 1u), false, true);

        if (gLogDetailMode == HISTORY_LOG_DETAIL_SMETER) {
            if (isTx)
                strcpy(detail, gSubMenu_TXP[MIN(entry.sMeter, ARRAY_SIZE(gSubMenu_TXP) - 1u)]);
            else
                HISTORY_LOG_FormatSMeter(entry.sMeter, detail);
        } else if (gLogDetailMode == HISTORY_LOG_DETAIL_BATT) {
            const uint16_t volt = HISTORY_LOG_BATT_OFFSET + entry.battVolt;
            sprintf(detail, "%u.%02u", volt / 100u, volt % 100u);
        } else {
            sprintf(detail, "%02u:%02u", entry.durationSeconds / 60u, entry.durationSeconds % 60u);
        }
        // Draw the fixed-width badge first, then punch the text out of it
        // centered: text length varies (Sn vs S9+XX vs MM:SS), the badge
        // must not. Each glyph cell is 4 px wide, 5 cells fill the badge.
        GUI_DisplaySmallestInverse("", 107, row, false, true, 127);
        GUI_DisplaySmallest(detail, (uint8_t)(107u + (5u - strlen(detail)) * 2u),
                            (uint8_t)((row * 8u) + 1u), false, false);
    }

    ST7565_BlitFullScreen();
}

#endif
