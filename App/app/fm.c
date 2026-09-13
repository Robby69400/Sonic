/* Copyright 2023 Dual Tachyon
 * https://github.com/DualTachyon
 *
 * OURO_KA52: simple FM, auto-seek only, no memory, no band switching
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#ifdef ENABLE_FMRADIO

#include <string.h>
#include "nav_invert.h"
#include "app/action.h"
#include "app/fm.h"
#include "app/generic.h"
#include "driver/bk1080.h"
#include "driver/bk4819.h"
#include "driver/py25q16.h"
#include "driver/system.h"
#include "driver/gpio.h"
#include "functions.h"
#include "misc.h"
#include "settings.h"
#include "ui/ui.h"

#ifndef ARRAY_SIZE
    #define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))
#endif

// Fixed band: 87.5–108.0 MHz
#define FM_BAND  0

// ---- Globals (kept for settings.c / app.c / action.c) ----
uint16_t          gFM_Channels[20];
bool              gFmRadioMode;
uint8_t           gFmRadioCountdown_500ms;
volatile uint16_t gFmPlayCountdown_10ms;
volatile int8_t   gFM_ScanState;
bool              gFM_AutoScan;
uint8_t           gFM_ChannelPosition;
bool              gFM_FoundFrequency;
uint16_t          gFM_RestoreCountdown_10ms;
bool              gFM_ManualMode = false;
bool              gFM_Mute       = false;
bool              gFM_No_Rx      = false;
bool              gFmNameDisplay = true;

// ── FM nine-slot memory ──────────────────────────────────────────────────
// 0 = empty; otherwise, frequency (875..1080)
// EEPROM: 0xA070 (slots 0-3), 0xA078 (slots 4-7), and 0xA080 (slots 8-9)
// Stored in the unused part of the FM channels 0xA028..0xA0A7 (eeprom_compat)
#define FM_MEMORY_EEPROM_ADDR0  0xA070
#define FM_MEMORY_EEPROM_ADDR1  0xA078
#define FM_MEMORY_EEPROM_ADDR2  0xA080

uint16_t gFM_Memory[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

// ── FM radios list (50 × 15 bytes, written by CHIRP) ────────────────────
// Record: ul16 freq (0.1 MHz units, 0 = empty) + char name[15]
#define FM_RADIOS_MAX           50
#define FM_RADIOS_EEPROM_ADDR   0x008AF4
#define FM_RADIO_NAME_LEN       15
#define FM_RADIO_RECORD         (2 + FM_RADIO_NAME_LEN)

static char gFM_RadioName[FM_RADIO_NAME_LEN + 1];  // zero-terminated copy

const char *FM_FindRadioName(uint16_t freq)
{
    uint8_t buf[2];

    for (uint8_t i = 0; i < FM_RADIOS_MAX; i++) {
        PY25Q16_ReadBuffer(FM_RADIOS_EEPROM_ADDR + (uint16_t)i * FM_RADIO_RECORD,
                           buf, 2);

        uint16_t f = buf[0] | ((uint16_t)buf[1] << 8);
        if (f != freq)
            continue;

        // Frequency match — load only this station's name
        PY25Q16_ReadBuffer(FM_RADIOS_EEPROM_ADDR + (uint16_t)i * FM_RADIO_RECORD + 2,
                           (uint8_t *)gFM_RadioName, FM_RADIO_NAME_LEN);

        uint8_t j;
        for (j = 0; j < FM_RADIO_NAME_LEN; j++) {
            char c = gFM_RadioName[j];
            if (c == '\0' || c == (char)0xFF)
                break;
            if (c < 32 || c > 126)
                gFM_RadioName[j] = ' ';
        }
        gFM_RadioName[j] = '\0';

        while (j > 0 && gFM_RadioName[j - 1] == ' ')
            j--;
        gFM_RadioName[j] = '\0';

        if (gFM_RadioName[0] != '\0')
            return gFM_RadioName;
        return NULL;
    }
    return NULL;
}

void FM_Memory_Load(void)
{
    const uint16_t lo = BK1080_GetFreqLoLimit(FM_BAND);
    const uint16_t hi = BK1080_GetFreqHiLimit(FM_BAND);
    uint8_t buf[8];

    PY25Q16_ReadBuffer(FM_MEMORY_EEPROM_ADDR0, buf, 8);
    for (uint8_t i = 0; i < 4; i++) {
        uint16_t f = buf[i * 2] | ((uint16_t)buf[i * 2 + 1] << 8);
        gFM_Memory[i] = (f >= lo && f <= hi) ? f : 0;
    }

    PY25Q16_ReadBuffer(FM_MEMORY_EEPROM_ADDR1, buf, 8);
    for (uint8_t i = 0; i < 4; i++) {
        uint16_t f = buf[i * 2] | ((uint16_t)buf[i * 2 + 1] << 8);
        gFM_Memory[4 + i] = (f >= lo && f <= hi) ? f : 0;
    }

    PY25Q16_ReadBuffer(FM_MEMORY_EEPROM_ADDR2, buf, 8);
    for (uint8_t i = 0; i < 1; i++) {
        uint16_t f = buf[i * 2] | ((uint16_t)buf[i * 2 + 1] << 8);
        gFM_Memory[8 + i] = (f >= lo && f <= hi) ? f : 0;
    }
}

void FM_Memory_Save(uint8_t slot)
{
    uint8_t buf[8];
    if (slot < 4) {
        PY25Q16_ReadBuffer(FM_MEMORY_EEPROM_ADDR0, buf, 8);
        buf[slot * 2]     = (uint8_t)(gFM_Memory[slot] & 0xFF);
        buf[slot * 2 + 1] = (uint8_t)(gFM_Memory[slot] >> 8);
        PY25Q16_WriteBuffer(FM_MEMORY_EEPROM_ADDR0, buf, sizeof(buf), false);
    } else if (slot < 8) {
        uint8_t idx = slot - 4;
        PY25Q16_ReadBuffer(FM_MEMORY_EEPROM_ADDR1, buf, 8);
        buf[idx * 2]     = (uint8_t)(gFM_Memory[slot] & 0xFF);
        buf[idx * 2 + 1] = (uint8_t)(gFM_Memory[slot] >> 8);
        PY25Q16_WriteBuffer(FM_MEMORY_EEPROM_ADDR1, buf, sizeof(buf), false);
    } else if (slot < 9) {
        PY25Q16_ReadBuffer(FM_MEMORY_EEPROM_ADDR2, buf, 8);
        buf[0] = (uint8_t)(gFM_Memory[slot] & 0xFF);
        buf[1] = (uint8_t)(gFM_Memory[slot] >> 8);
        PY25Q16_WriteBuffer(FM_MEMORY_EEPROM_ADDR2, buf, sizeof(buf), false);
    }
}

const uint8_t BUTTON_STATE_PRESSED = 1 << 0;
const uint8_t BUTTON_STATE_HELD    = 1 << 1;
const uint8_t BUTTON_EVENT_PRESSED = BUTTON_STATE_PRESSED;
const uint8_t BUTTON_EVENT_HELD    = BUTTON_STATE_PRESSED | BUTTON_STATE_HELD;
const uint8_t BUTTON_EVENT_SHORT   = 0;
const uint8_t BUTTON_EVENT_LONG    = BUTTON_STATE_HELD;

// ---- Stubs for external callers ----

bool FM_CheckValidChannel(uint8_t Channel)
{
    return Channel < ARRAY_SIZE(gFM_Channels) &&
           gFM_Channels[Channel] >= BK1080_GetFreqLoLimit(FM_BAND) &&
           gFM_Channels[Channel] <  BK1080_GetFreqHiLimit(FM_BAND);
}

uint8_t FM_FindNextChannel(uint8_t Channel, uint8_t Direction)
{
    (void)Channel; (void)Direction;
    return 0xFF;
}

int FM_ConfigureChannelState(void)
{
    gEeprom.FM_IsMrMode         = false;
    gEeprom.FM_FrequencyPlaying = gEeprom.FM_SelectedFrequency;
    return 0;
}

void FM_EraseChannels(void)
{
    PY25Q16_SectorErase(0x003000);
    memset(gFM_Channels, 0xFF, sizeof(gFM_Channels));
}

// Stubs — not used, kept for app.c/scheduler
void FM_Tune(uint16_t Frequency, int8_t Step, bool bFlag)
{
    (void)Frequency; (void)Step; (void)bFlag;
}

void FM_PlayAndUpdate(void)
{
    gFM_ScanState = FM_SCAN_OFF;
    GPIO_EnableAudioPath();
    gEnableSpeaker = true;
}

int FM_CheckFrequencyLock(uint16_t Frequency, uint16_t LowerLimit)
{
    int ret = -1;
    const uint16_t Test2     = BK1080_ReadRegister(BK1080_REG_07);
    const uint16_t Deviation = BK1080_REG_07_GET_FREQD(Test2);

    if (BK1080_REG_07_GET_SNR(Test2) <= 2) {
        BK1080_FrequencyDeviation = Deviation;
        BK1080_BaseFrequency      = Frequency;
        return ret;
    }
    const uint16_t Status = BK1080_ReadRegister(BK1080_REG_10);
    if ((Status & BK1080_REG_10_MASK_AFCRL) != BK1080_REG_10_AFCRL_NOT_RAILED ||
        BK1080_REG_10_GET_RSSI(Status) < 10) {
        BK1080_FrequencyDeviation = Deviation;
        BK1080_BaseFrequency      = Frequency;
        return ret;
    }
    if (Deviation >= 280 && Deviation <= 3815) {
        BK1080_FrequencyDeviation = Deviation;
        BK1080_BaseFrequency      = Frequency;
        return ret;
    }
    if (Frequency > LowerLimit && (Frequency - BK1080_BaseFrequency) == 1) {
        if (BK1080_FrequencyDeviation & 0x800 || BK1080_FrequencyDeviation < 20) {
            BK1080_FrequencyDeviation = Deviation;
            BK1080_BaseFrequency      = Frequency;
            return ret;
        }
    }
    if (Frequency >= LowerLimit && (BK1080_BaseFrequency - Frequency) == 1) {
        if ((BK1080_FrequencyDeviation & 0x800) == 0 || BK1080_FrequencyDeviation > 4075) {
            BK1080_FrequencyDeviation = Deviation;
            BK1080_BaseFrequency      = Frequency;
            return ret;
        }
    }
    ret = 0;
    BK1080_FrequencyDeviation = Deviation;
    BK1080_BaseFrequency      = Frequency;
    return ret;
}

void FM_Play(void)
{
    GUI_SelectNextDisplay(DISPLAY_FM);
}

// ---- Synchronous seek: step through frequencies until station found ----

static void FM_SeekNext(int8_t direction)
{
    const uint16_t lo   = BK1080_GetFreqLoLimit(FM_BAND);
    const uint16_t hi   = BK1080_GetFreqHiLimit(FM_BAND);
    uint16_t       freq = gEeprom.FM_FrequencyPlaying;
    const uint16_t start = freq;

    // Mute audio while seeking
    GPIO_DisableAudioPath();
    gEnableSpeaker = false;

    do {
        freq += direction;
        if (freq < lo) freq = hi;
        if (freq > hi) freq = lo;

        BK1080_SetFrequency(freq, FM_BAND);
        SYSTEM_DelayMs(100);

        if (FM_CheckFrequencyLock(freq, lo) == 0) {
            // Station found — restore audio
            gEeprom.FM_FrequencyPlaying  = freq;
            gEeprom.FM_SelectedFrequency = freq;
            gRequestSaveFM = true;
            gRequestDisplayScreen = DISPLAY_FM;
            GPIO_EnableAudioPath();
            gEnableSpeaker = true;
            return;
        }

        // Full circle — nothing found, restore original
    } while (freq != start);

    // No station found — restore frequency, keep muted
    BK1080_SetFrequency(gEeprom.FM_FrequencyPlaying, FM_BAND);
    GPIO_EnableAudioPath();
    gEnableSpeaker = true;
}

// ---- Manual frequency step: 0.1 MHz ----

static void FM_StepFreq(int8_t direction)
{
    const uint16_t lo = BK1080_GetFreqLoLimit(FM_BAND);
    const uint16_t hi = BK1080_GetFreqHiLimit(FM_BAND);
    uint16_t freq = gEeprom.FM_FrequencyPlaying + direction;
    if (freq < lo) freq = hi;
    if (freq > hi) freq = lo;

    gEeprom.FM_FrequencyPlaying  = freq;
    gEeprom.FM_SelectedFrequency = freq;
    BK1080_SetFrequency(freq, FM_BAND);
    gRequestSaveFM = true;
    gRequestDisplayScreen = DISPLAY_FM;
}

// ---- Key handling ----

void FM_ProcessKeys(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld)
{
    uint8_t state = bKeyPressed + 2 * bKeyHeld;

    // ── Long press 1-9: save the frequency to a slot ───────────────────
    if (bKeyHeld && bKeyPressed) {
        uint8_t slot = 0xFF;
        switch (Key) {
            case KEY_1: slot = 0; break;
            case KEY_2: slot = 1; break;
            case KEY_3: slot = 2; break;
            case KEY_4: slot = 3; break;
            case KEY_5: slot = 4; break;
            case KEY_6: slot = 5; break;
            case KEY_7: slot = 6; break;
            case KEY_8: slot = 7; break;
            case KEY_9: slot = 8; break;
            default:    break;
        }
        if (slot != 0xFF) {
            gFM_Memory[slot] = gEeprom.FM_FrequencyPlaying;
            FM_Memory_Save(slot);
            gRequestDisplayScreen = DISPLAY_FM;
            return;
        }
    }

    // ── Short press 1-9: recall the frequency from a slot ──────────────
    if (state == BUTTON_EVENT_SHORT) {
        uint8_t slot = 0xFF;
        switch (Key) {
            case KEY_1: slot = 0; break;
            case KEY_2: slot = 1; break;
            case KEY_3: slot = 2; break;
            case KEY_4: slot = 3; break;
            case KEY_5: slot = 4; break;
            case KEY_6: slot = 5; break;
            case KEY_7: slot = 6; break;
            case KEY_8: slot = 7; break;
            case KEY_9: slot = 8; break;
            default:    break;
        }
        if (slot != 0xFF) {
            if (gFM_Memory[slot] != 0) {
                gEeprom.FM_FrequencyPlaying  = gFM_Memory[slot];
                gEeprom.FM_SelectedFrequency = gFM_Memory[slot];
                BK1080_SetFrequency(gEeprom.FM_FrequencyPlaying, FM_BAND);
                gRequestSaveFM        = true;
                gRequestDisplayScreen = DISPLAY_FM;
            }
            return;
        }
    }

    switch (Key) {
        case KEY_UP:
            if (state == BUTTON_EVENT_SHORT) {
                int8_t dir = NAV_DIR(-1); // UP direction (inverted if ENABLE_INVERT_NAV)
                if (gFM_ManualMode)
                    FM_StepFreq(dir);
                else
                    FM_SeekNext(dir);
            }
            break;
        case KEY_DOWN:
            if (state == BUTTON_EVENT_SHORT) {
                int8_t dir = NAV_DIR(1);  // DOWN direction (inverted if ENABLE_INVERT_NAV)
                if (gFM_ManualMode)
                    FM_StepFreq(dir);
                else
                    FM_SeekNext(dir);
            }
            break;
        case KEY_STAR:
            if (state == BUTTON_EVENT_HELD) {
                // Long press: toggle mute
                gFM_Mute = !gFM_Mute;
                gRequestDisplayScreen = DISPLAY_FM;
            } else if (state == BUTTON_EVENT_SHORT) {
                // Short press: toggle auto/manual
                gFM_ManualMode = !gFM_ManualMode;
                gRequestDisplayScreen = DISPLAY_FM;
            }
            break;
        case KEY_EXIT:
            if (state == BUTTON_EVENT_SHORT)
                ACTION_FM();
            break;
        case KEY_MENU:
            if (state == BUTTON_EVENT_SHORT) gFmNameDisplay = !gFmNameDisplay;
            break;
        case KEY_F:
            GENERIC_Key_F(bKeyPressed, bKeyHeld);
            break;
        case KEY_PTT:
            GENERIC_Key_PTT(bKeyPressed);
            break;
        case KEY_0:
            if (state == BUTTON_EVENT_SHORT) {
                gFM_No_Rx = !gFM_No_Rx;
                if (gFM_No_Rx) {
                    BK4819_Sleep(); //Locks Radio Rx in FM Radio
                } else {        
                    BK4819_RX_TurnOn(); //Wake up
                    SYSTEM_DelayMs(10);
                }
            }
            break;
        default:
            break;
    }
}

// ---- FM_TurnOff / FM_Start ----

void FM_TurnOff(void)
{
    gFmRadioMode              = false;
    gFM_ScanState             = FM_SCAN_OFF;
    gFM_RestoreCountdown_10ms = 0;
    gFM_Mute                  = false;
    gFM_ManualMode            = false;
    BK1080_Init0();
    BK4819_PickRXFilterPathBasedOnFrequency(gRxVfo->freq_config_RX.Frequency);
    GPIO_EnableAudioPath();
    gEnableSpeaker = true;
    gUpdateStatus = true;
    #ifdef ENABLE_FEAT_F4HWN_RESUME_STATE
        gEeprom.CURRENT_STATE = 0;
        SETTINGS_WriteCurrentState();
    #endif
}

void FM_Start(void)
{
    gFmRadioMode              = true;
    gFM_ScanState             = FM_SCAN_OFF;
    gFM_RestoreCountdown_10ms = 0;
    gFM_AutoScan              = false;
    gFM_ChannelPosition       = 0;
    gFM_ManualMode            = false;
    gFM_Mute                  = false;
    gEeprom.FM_Band     = FM_BAND;
    gEeprom.FM_IsMrMode = false;

    

    // Ensure FrequencyPlaying is valid before passing to BK1080_Init.
    // After a hard reset the RAM field may be 0 (or stale), which causes
    // BK1080_Init to enter powerdown mode instead of tuning, and makes
    // FM_SeekNext's do-while loop terminate immediately (freq == start == 0).
    {
        const uint16_t lo = BK1080_GetFreqLoLimit(FM_BAND);
        const uint16_t hi = BK1080_GetFreqHiLimit(FM_BAND);
        if (gEeprom.FM_FrequencyPlaying < lo || gEeprom.FM_FrequencyPlaying > hi) {
            gEeprom.FM_FrequencyPlaying =
                (gEeprom.FM_SelectedFrequency >= lo && gEeprom.FM_SelectedFrequency <= hi)
                ? gEeprom.FM_SelectedFrequency : lo;
        }
    }
    BK1080_Init(gEeprom.FM_FrequencyPlaying, FM_BAND);
    BK4819_PickRXFilterPathBasedOnFrequency(10320000);

    FM_Memory_Load();   // load memory slots from EEPROM
    GPIO_EnableAudioPath();
    gEnableSpeaker = true;
    gUpdateStatus  = true;

    #ifdef ENABLE_FEAT_F4HWN_RESUME_STATE
        gEeprom.CURRENT_STATE = 3;
        SETTINGS_WriteCurrentState();
    #endif
}

#endif
