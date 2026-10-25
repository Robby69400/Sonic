/* Copyright 2025 muzkr https://github.com/muzkr
 * Copyright 2023 Dual Tachyon
 * https://github.com/DualTachyon
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

#include <string.h>
#include "frequencies.h"
#include "radio.h"
#ifdef ENABLE_FMRADIO
    #include "app/fm.h"
#endif
#include "driver/bk1080.h"
#include "driver/bk4819.h"
#include "driver/py25q16.h"
#include "misc.h"
#include "settings.h"
#include "ui/menu.h"
#include <stdio.h>
#include "app/spectrum.h"
#include "app/common.h"

EEPROM_Config_t gEeprom = { 0 };

/**
 * @brief Initialize EEPROM configuration values from flash storage
 * @note All values are validated against expected ranges with safe defaults
 */
void SETTINGS_InitEEPROM(void)
{
    uint8_t Data[16] = {0};

    // ======================================================================
    // Block 1: 0x00A000 .. 0x00A007 (0E70..0E77)
    // Audio, squelch, timeout, scrambling, key lock, flashlight, mic sensitivity
    // ======================================================================
    PY25Q16_ReadBuffer(0x00A000, Data, 8);
    
    gSetting_set_audio_fm = ((Data[0] & 0x0F) < 5) ? (Data[0] & 0x0F) : 0;
    gSetting_set_audio_am = (((Data[0] >> 4) & 0x0F) < 3) ? ((Data[0] >> 4) & 0x0F) : 0;
    gEeprom.SQUELCH_LEVEL        = (Data[1] > 0 && Data[1] < 10) ? Data[1] : 1;
    gEeprom.TX_TIMEOUT_TIMER     = (Data[2] > 4 && Data[2] < 180) ? Data[2] : 11;
    gEeprom.SCRAMBLING_TYPE      = (Data[3] > 0 && Data[3] < 11) ? Data[3] : 0;
    gEeprom.KEY_LOCK             = (Data[4] <  2) ? Data[4] : false;
    gEeprom.FlashlightOnRX       = (Data[5] & 0x02) ? true : false;
    gEeprom.MIC_SENSITIVITY      = (Data[7] <  9) ? Data[7] : 4;

    // ======================================================================
    // Block 2: 0x00A008 .. 0x00A00F (0E78..0E7F)
    // Backlight levels, display mode, battery save, backlight timer
    // ======================================================================
    PY25Q16_ReadBuffer(0x00A008, Data, 8);
    gEeprom.BACKLIGHT_MAX         = (Data[0] & 0xF) <= 10 ? (Data[0] & 0xF) : 10;
    gEeprom.BACKLIGHT_MIN         = (Data[0] >> 4) < gEeprom.BACKLIGHT_MAX ? (Data[0] >> 4) : 0;
    gEeprom.CHANNEL_DISPLAY_MODE  = (Data[1] < 4) ? Data[1] : MDF_NAME_FREQ;
    gEeprom.BATTERY_SAVE          = (Data[3] < 6) ? Data[3] : 4;
    gEeprom.BACKLIGHT_TIME        = (Data[5] < 62) ? Data[5] : 12;
    
    #ifdef ENABLE_FEAT_F4HWN_NARROWER
        gEeprom.TAIL_TONE_ELIMINATION = Data[6] & 0x01;
        gSetting_set_nfm = (Data[6] >> 1) & 0x01;
        #ifdef ENABLE_FEAT_F4HWN_RESUME_STATE
            gEeprom.VFO_OPEN = ((Data[6] >> 2) & 0x01) != 0;
        #endif
    #else
        gEeprom.TAIL_TONE_ELIMINATION = (Data[6] < 2) ? Data[6] : false;
    #endif

    // ======================================================================
    // Block 3: 0x00A007 (0E7F byte 7)
    // Resume state or VFO open flag
    // ======================================================================
    #ifdef ENABLE_FEAT_F4HWN_RESUME_STATE
        gEeprom.CURRENT_STATE =  Data[7]        & 0x07;   // bits 0..2
        gEeprom.CURRENT_LIST  = (Data[7] >> 3)  & 0x1F;   // bits 3..7
    #else
        gEeprom.VFO_OPEN = (Data[7] < 2) ? Data[7] : true;
    #endif

    // ======================================================================
    // Block 4: 0x00A010 .. 0x00A013 (0E80..0E83)
    // Screen channel and MR channel
    // ======================================================================
    uint16_t Data16[2];
    PY25Q16_ReadBuffer(0x00A010, Data16, sizeof(Data16));
    gEeprom.ScreenChannel = Data16[0];
    gEeprom.MrChannel     = Data16[1];

    // ======================================================================
    // Block 5: FM Radio Configuration (0x00A020 .. 0x00A02F)
    // Only compiled when ENABLE_FMRADIO is defined
    // ======================================================================
    #ifdef ENABLE_FMRADIO
    {
        struct
        {
            uint16_t selFreq;
            uint8_t  selChn;
            uint8_t  isMrMode:1;
            uint8_t  band:2;
        } __attribute__((packed)) fmCfg;
        
        PY25Q16_ReadBuffer(0x00A020, &fmCfg, 4);

        gEeprom.FM_Band = fmCfg.band;
        
        // Validate selected frequency is within band limits
        uint16_t freqMin = BK1080_GetFreqLoLimit(gEeprom.FM_Band);
        uint16_t freqMax = BK1080_GetFreqHiLimit(gEeprom.FM_Band);
        gEeprom.FM_SelectedFrequency = 
            (fmCfg.selFreq >= freqMin && fmCfg.selFreq <= freqMax) ? 
                fmCfg.selFreq : freqMin;
            
        gEeprom.FM_SelectedChannel = fmCfg.selChn;
        gEeprom.FM_IsMrMode        = fmCfg.isMrMode;

        // Load FM channel table
        PY25Q16_ReadBuffer(0x00A028, gFM_Channels, sizeof(gFM_Channels));
        FM_ConfigureChannelState();
    }
    #endif

    // ======================================================================
    // Block 6: 0x00A0A8 .. 0x00A0AF (0E90..0E97)
    // Key actions, scan resume mode, keypad lock, power-on display
    // ======================================================================
    PY25Q16_ReadBuffer(0x00A0A8, Data, 8);
    gEeprom.KEY_M_LONG_PRESS_ACTION      = ((Data[0] >> 1) < ACTION_OPT_LEN) ? (Data[0] >> 1) : ACTION_OPT_FLASHLIGHT;
    gEeprom.KEY_1_SHORT_PRESS_ACTION     = (Data[1] < ACTION_OPT_LEN) ? Data[1] : ACTION_OPT_MONITOR;
    gEeprom.KEY_1_LONG_PRESS_ACTION      = (Data[2] < ACTION_OPT_LEN) ? Data[2] : ACTION_OPT_NONE;
    gEeprom.KEY_2_SHORT_PRESS_ACTION     = (Data[3] < ACTION_OPT_LEN) ? Data[3] : ACTION_OPT_NONE;
    gEeprom.KEY_2_LONG_PRESS_ACTION      = (Data[4] < ACTION_OPT_LEN) ? Data[4] : ACTION_OPT_NONE;
    gEeprom.SCAN_RESUME_MODE             = (Data[5] < 105) ? Data[5] : 14;
    gEeprom.AUTO_KEYPAD_LOCK             = (Data[6] < 41) ? Data[6] : 0;
    gEeprom.POWER_ON_DISPLAY_MODE        = (Data[7] < 3) ? Data[7] : POWER_ON_DISPLAY_MODE_LOGO;

    // ======================================================================
    // Block 7: 0x00A0B8 .. 0x00A0BF (0EA0..0EA7)
    // RSSI calibration bar (if enabled)
    // ======================================================================
    PY25Q16_ReadBuffer(0x00A0A8 + 0x10, Data, 8);
    #ifdef ENABLE_RSSI_BAR
        for (uint8_t i = 0; i < 7; i++) {
            int8_t val = (int8_t)Data[i + 1];
            if (val >= -64 && val <= 64)
                dBmCorrTable[i] = val;
        }
    #endif

    // ======================================================================
    // Block 8: 0x00A0C0 .. 0x00A0C7 (0EA8..0EAF)
    // Roger beep, repeater tail tone elimination, TX VFO, battery type
    // ======================================================================
    PY25Q16_ReadBuffer(0x00A0A8 + 0x18, Data, 8);
    gEeprom.ROGER                          = (Data[1] <  7) ? Data[1] : ROGER_MODE_OFF;
    gEeprom.REPEATER_TAIL_TONE_ELIMINATION = (Data[2] < 11) ? Data[2] : 0;
    gEeprom.TX_VFO                         = (Data[3] <  2) ? Data[3] : 0;
    gEeprom.BATTERY_TYPE                   = (Data[4] < BATTERY_TYPE_UNKNOWN) ? Data[4] : BATTERY_TYPE_1600_MAH;
    // NOTE: FlashlightOnRX was incorrectly overwritten here in original code
    // gEeprom.FlashlightOnRX = false;  // REMOVED - preserves EEPROM value

    // ======================================================================
    // Block 9: 0x00A130 .. 0x00A137 (0F18..0F1F)
    // Scan list configuration and priority channels
    // ======================================================================
    PY25Q16_ReadBuffer(0x00A130, Data, 8);

    gEeprom.SCAN_LIST_DEFAULT =
            (((Data[0] & 0x7F) >= 1) && ((Data[0] & 0x7F) <= (MR_CHANNELS_LIST + 1)))
                ? (Data[0] & 0x7F)
                : 1;
    gEeprom.SCAN_LIST_ENABLED = (Data[0] >> 7) & 0x01;

    gEeprom.SCANLIST_PRIORITY_CH[0] =
            (uint16_t)Data[1] |
            ((uint16_t)Data[2] << 8);

    gEeprom.SCANLIST_PRIORITY_CH[1] =
            (uint16_t)Data[3] |
            ((uint16_t)Data[4] << 8);

    // ======================================================================
    // Block 10: 0x00A150 .. 0x00A157 (0F40..0F47)
    // F-lock, battery text, mic bar, backlight on TX/RX
    // ======================================================================
    PY25Q16_ReadBuffer(0x00A150, Data, 8);
    gSetting_F_LOCK            = (Data[0] < F_LOCK_LEN) ? Data[0] : F_LOCK_136_500;
    gSetting_battery_text      = (((Data[7] >> 2) & 3u) <= 2) ? (Data[7] >> 2) & 3 : 2; // default PERCENT
    gSetting_backlight_on_tx_rx = (Data[7] >> 6) & 3u;

    // ======================================================================
    // Block 11: List names initialization
    // ======================================================================
    PY25Q16_ReadBuffer(0x008900, gListName, sizeof(gListName));

    // ======================================================================
    // Block 12: Channel attributes cache initialization
    // ======================================================================
    MR_InitChannelAttributesCache();

    // ======================================================================
    // Block 13: Channel data validation and repair
    // ======================================================================
    for (uint16_t i = 0; i < MR_CHANNELS_MAX + 7; i++) {
        ChannelAttributes_t *att = MR_GetChannelAttributes(i);
        
        if (att != NULL) {
            if (att->__val == 0xFFFF) {
                att->__val = 0;
                att->band = 0x7;
                MR_SetChannelAttributes(i, att);
            }
            else {
                att->exclude = 0;
                MR_SetChannelAttributes(i, att);
            }
        }
    }

    // ======================================================================
    // Block 14: Custom AES key (0x00A138 .. 0x00A13F)
    // ======================================================================
    PY25Q16_ReadBuffer(0x00A138, gCustomAesKey, sizeof(gCustomAesKey));
    bHasCustomAesKey = false;
    #ifndef ENABLE_FEAT_F4HWN
        for (unsigned int i = 0; i < ARRAY_SIZE(gCustomAesKey); i++)
        {
            if (gCustomAesKey[i] != 0xFFFFFFFFu)
            {
                bHasCustomAesKey = true;
                break;  // FIXED: was 'return' which exited function prematurely
            }
        }
    #endif

    // ======================================================================
    // Block 15: Feature flags (0x00A158 .. 0x00A15F)
    // PTT, TOT, EOT, INV, LCK, MET, GUI, TIMER, NAV_INVERT
    // ======================================================================
    PY25Q16_ReadBuffer(0x00A158, Data, 8);
    gSetting_set_ptt = (((Data[7] & 0x0F)) < 2) ? ((Data[7] & 0x0F)) : 0;
    gSetting_set_tot = (((Data[6] & 0xF0) >> 4) < 4) ? ((Data[6] & 0xF0) >> 4) : 0;
    gSetting_set_eot = (((Data[6] & 0x0F)) < 4) ? ((Data[6] & 0x0F)) : 0;
    
    int tmp = (Data[5] & 0xF0) >> 4;
    gSetting_set_inv = (tmp >> 0) & 0x01;
    gSetting_set_lck = (tmp >> 1) & 0x01;
    gSetting_set_met = (tmp >> 2) & 0x01;
    gSetting_set_gui = (tmp >> 3) & 0x01;
    gSetting_set_ctr = 10;
    gSetting_set_tmr = Data[4] & 0x01;
    
    // NAV_INVERT is disabled - always force off regardless of EEPROM
    gSetting_nav_invert = false;  // feature disabled — always off
    
    gSetting_set_ptt_session = gSetting_set_ptt;
    gEeprom.KEY_LOCK_PTT = gSetting_set_lck;
}

/**
 * @brief Load RSSI and battery calibration data from EEPROM
 * @note Validates calibration ranges and applies fallback defaults if out of spec
 */
void SETTINGS_LoadCalibration(void)
{
    // ======================================================================
    // RSSI calibration tables (multiple banks for redundancy)
    // ======================================================================
    // Bank 3 at 0x100C0 -> copied to banks 4, 5, 6
    PY25Q16_ReadBuffer(0x010000 + 0xC0, gEEPROM_RSSI_CALIB[3], 8);
    memcpy(gEEPROM_RSSI_CALIB[4], gEEPROM_RSSI_CALIB[3], 8);
    memcpy(gEEPROM_RSSI_CALIB[5], gEEPROM_RSSI_CALIB[3], 8);
    memcpy(gEEPROM_RSSI_CALIB[6], gEEPROM_RSSI_CALIB[3], 8);

    // Bank 0 at 0x100C8 -> copied to banks 1, 2
    PY25Q16_ReadBuffer(0x010000 + 0xC8, gEEPROM_RSSI_CALIB[0], 8);
    memcpy(gEEPROM_RSSI_CALIB[1], gEEPROM_RSSI_CALIB[0], 8);
    memcpy(gEEPROM_RSSI_CALIB[2], gEEPROM_RSSI_CALIB[0], 8);

    // Battery calibration at 0x1F40
    PY25Q16_ReadBuffer(0x010000 + 0x140, gBatteryCalibration, 12);
    if (gBatteryCalibration[0] >= 5000)  // Invalid value detected
    {
        gBatteryCalibration[0] = 1900;
        gBatteryCalibration[1] = 2000;
    }
    gBatteryCalibration[5] = 2300;

    // Microphone gain tuning based on sensitivity level
    gEeprom.MIC_SENSITIVITY_TUNING = gMicGain_dB2[gEeprom.MIC_SENSITIVITY];

    // ======================================================================
    // Miscellaneous calibration (crystal frequency, gains)
    // ======================================================================
    struct
    {
        int16_t  BK4819_XtalFreqLow;
        uint16_t EEPROM_1F8A;
        uint16_t EEPROM_1F8C;
        uint8_t  VOLUME_GAIN;
        uint8_t  DAC_GAIN;
    } __attribute__((packed)) Misc;

    // Read from 0x1F88
    PY25Q16_ReadBuffer(0x010000 + 0x188, &Misc, 8);

    // Validate crystal frequency offset
    gEeprom.BK4819_XTAL_FREQ_LOW = (Misc.BK4819_XtalFreqLow >= -1000 && 
                                    Misc.BK4819_XtalFreqLow <= 1000) ? 
                                   Misc.BK4819_XtalFreqLow : 0;
    
    // Mask reserved bits from EEPROM values
    gEEPROM_1F8A                 = Misc.EEPROM_1F8A & 0x01FF;
    gEEPROM_1F8C                 = Misc.EEPROM_1F8C & 0x01FF;
    
    // Validate volume gain
    gEeprom.VOLUME_GAIN          = (Misc.VOLUME_GAIN < 64) ? Misc.VOLUME_GAIN : 58;
    gEeprom.DAC_GAIN             = (Misc.DAC_GAIN < 16) ? Misc.DAC_GAIN : 8;

    #ifdef ENABLE_FEAT_F4HWN
        gEeprom.VOLUME_GAIN_BACKUP   = gEeprom.VOLUME_GAIN;
    #endif

    // Apply crystal frequency adjustment to BK4819 radio chip
    BK4819_WriteRegister(BK4819_REG_3B, 22656 + gEeprom.BK4819_XTAL_FREQ_LOW);
}

/**
 * @brief Fetch frequency for a specific channel
 * @param channel Channel number (0-based)
 * @return Frequency value or 0 if invalid
 */
uint32_t SETTINGS_FetchChannelFrequency(const uint16_t channel)
{
    struct
    {
        uint32_t frequency;
        uint32_t offset;
    } __attribute__((packed)) info;

    PY25Q16_ReadBuffer(ADRESS_CHANNELS + channel * 16, &info, sizeof(info));

    return info.frequency;
}

/**
 * @brief Fetch and validate channel name
 * @param s Output buffer for channel name (must be at least 11 bytes)
 * @param channel Channel number
 */
void SETTINGS_FetchChannelName(char *s, const uint16_t channel)
{
    if (s == NULL) return;
    s[0] = 0;
    if (!RADIO_CheckValidChannel(channel, false, 0)) return;
    
    PY25Q16_ReadBuffer(ADRESS_CHANNELS_NAMES + (channel * 16), s, 10);
    
    // Validate printable ASCII characters only
    int i;
    for (i = 0; i < 10; i++)
        if (s[i] < 32 || s[i] > 126)  // FIXED: was 127 (DEL char excluded)
            break;
    
    s[i] = '\0';  // Null terminate
    
    // Trim trailing spaces
    while (i > 0 && s[i - 1] == ' ')
        s[--i] = '\0';
    
    // Default name if empty
    if (s[0] == '\0') {
        snprintf(s, 11, "CH%u", channel + 1);  // FIXED: using snprintf for safety
    }
}

/**
 * @brief Perform factory reset
 * @param bIsAll If true, erase all sectors including channel data
 */
void SETTINGS_FactoryReset(bool bIsAll)
{    
    // Erase main settings sector
    PY25Q16_SectorErase(0x00A000);
    
    if (bIsAll)
    {
        // Erase all configuration sectors (0x000000 to 0x00E000)
        for (uint32_t addr = 0x000000; addr < 0x00E000; addr += 0x1000) {
            PY25Q16_SectorErase(addr);
        }
        
        // Erase channel data sectors based on feature flags
        #if defined(ENABLE_10K)
            for (uint32_t addr = 0x14C000; addr < 0x19F6C5; addr += 0x1000) {
                PY25Q16_SectorErase(addr);
            }
        #elif defined(ENABLE_4096)
            for (uint32_t addr = 0x1DE000; addr < 0x1FE000; addr += 0x1000) {
                PY25Q16_SectorErase(addr);
            }
        #endif
    }
    
    // Reset custom feature flags at 0x00A158
    uint8_t Data[8];
    PY25Q16_ReadBuffer(0x00A158, Data, 8);
    Data[5] = 0x00;
    PY25Q16_WriteBuffer(0x00A158, Data, 8, 0);

    // Reset VFO to default frequency
    RADIO_InitInfo(&gEeprom.VfoInfo[0], FREQ_CHANNEL, 44609775ULL);  // Added ULL suffix
    gEeprom.ScreenChannel = FREQ_CHANNEL;
    gEeprom.MrChannel     = MR_CHANNEL_FIRST;
    
    SETTINGS_SaveChannel(FREQ_CHANNEL, 0, &gEeprom.VfoInfo[0], 2);
    
    gVfoStateChanged = true;
    gScheduleVfoSave = true;
    
    SETTINGS_SaveVfoIndicesFlush();
    ClearSettings();
}

#ifdef ENABLE_FMRADIO
/**
 * @brief Save FM radio configuration to EEPROM
 */
void SETTINGS_SaveFM(void)
{
    union {
        struct {
            uint16_t selFreq;
            uint8_t  selChn;
            uint8_t  isMrMode:1;
            uint8_t  band:2;
        };
        uint8_t __raw[8];
    } __attribute__((packed)) fmCfg;

    // Initialize with 0xFF (valid erased state)
    memset(fmCfg.__raw, 0xFF, sizeof(fmCfg.__raw));
    
    fmCfg.selChn   = gEeprom.FM_SelectedChannel;
    fmCfg.selFreq  = gEeprom.FM_SelectedFrequency;
    fmCfg.isMrMode = gEeprom.FM_IsMrMode;
    fmCfg.band     = gEeprom.FM_Band;
    
    // Write configuration (4 bytes used, rest padded)
    PY25Q16_WriteBuffer(0x00A020, fmCfg.__raw, 4, false);  // FIXED: was 8, only 4 used

    // Write FM channel table
    PY25Q16_WriteBuffer(0x00A028, gFM_Channels, sizeof(gFM_Channels), false);
}
#endif

/**
 * @brief Request VFO indices to be saved (deferred write)
 */
void SETTINGS_SaveVfoIndices(void)
{
    gVfoStateChanged = true;
    gVfoSaveCountdown_10ms = 2;
}

/**
 * @brief Flush pending VFO indices write to EEPROM
 */
void SETTINGS_SaveVfoIndicesFlush(void)
{
    if (gScheduleVfoSave) {
        gScheduleVfoSave = false;
        
        if (gVfoStateChanged) {
            gVfoStateChanged = false;
            uint16_t Data16[2];
            Data16[0] = gEeprom.ScreenChannel;
            Data16[1] = gEeprom.MrChannel;
            PY25Q16_WriteBuffer(0x00A010, Data16, sizeof(Data16), false);
        }
    }
}

/**
 * @brief Save all EEPROM settings
 * @note Uses read-modify-write pattern to preserve unrelated bits
 */
void SETTINGS_SaveSettings(void)
{
    uint8_t *State;
    uint8_t SecBuf[0x50];

    // ======================================================================
    // Block 1: 0x0E70 - 0x0E7F (Main settings)
    // ======================================================================
    memset(SecBuf, 0xFF, 0x10);

    State = SecBuf;
    State[0] = (gSetting_set_audio_fm & 0x0F) | ((gSetting_set_audio_am & 0x0F) << 4);
    #ifdef ENABLE_FEAT_F4HWN
        if (gSquelchLevelOriginal < 10)
            State[1] = gSquelchLevelOriginal;
        else
    #endif
    State[1] = gEeprom.SQUELCH_LEVEL;
    State[2] = gEeprom.TX_TIMEOUT_TIMER;
    State[3] = gEeprom.SCRAMBLING_TYPE;
    State[4] = gEeprom.KEY_LOCK;
    State[5] = (gEeprom.FlashlightOnRX ? 0x02 : 0x00);
    State[6] = 0;
    State[7] = gEeprom.MIC_SENSITIVITY;

    State = SecBuf + 0x8;
    // FIXED: Properly combine BACKLIGHT_MIN and BACKLIGHT_MAX without overlap
    State[0] = (gEeprom.BACKLIGHT_MAX & 0x0F) | ((gEeprom.BACKLIGHT_MIN & 0x0F) << 4);
    State[1] = gEeprom.CHANNEL_DISPLAY_MODE;
    State[3] = gEeprom.BATTERY_SAVE;
    
    #ifdef ENABLE_FEAT_F4HWN
        State[5] = gBackLight ? gBacklightTimeOriginal : gEeprom.BACKLIGHT_TIME;
    #else
        State[5] = gEeprom.BACKLIGHT_TIME;
    #endif

    #ifdef ENABLE_FEAT_F4HWN_NARROWER
        State[6] = (gEeprom.TAIL_TONE_ELIMINATION & 0x01) |
                   ((gSetting_set_nfm & 0x01) << 1);
        #ifdef ENABLE_FEAT_F4HWN_RESUME_STATE
            State[6] |= ((gEeprom.VFO_OPEN & 0x01) << 2);
        #endif
    #else
        State[6] = gEeprom.TAIL_TONE_ELIMINATION;
    #endif

    #ifdef ENABLE_FEAT_F4HWN_RESUME_STATE
        State[7] = (gEeprom.CURRENT_STATE & 0x07) | 
                   ((gEeprom.SCAN_LIST_DEFAULT & 0x1F) << 3);
    #else
        State[7] = gEeprom.VFO_OPEN ? 1 : 0;
    #endif

    PY25Q16_WriteBuffer(0x00A000, SecBuf, 0x10, false);

    // ======================================================================
    // Block 2: 0x0E90 - 0x0EEF (Key actions, scan, roger, etc.)
    // Uses read-modify-write to preserve other bits
    // ======================================================================
    PY25Q16_ReadBuffer(0x00A0A8, SecBuf, 0x50);

    // 0x0E90 - Key actions
    State = SecBuf;
    // FIXED: Clear bits 1-7 first, then set new value
    State[0] = (State[0] & 0x01) | ((gEeprom.KEY_M_LONG_PRESS_ACTION & 0x7F) << 1);
    State[1] = gEeprom.KEY_1_SHORT_PRESS_ACTION;
    State[2] = gEeprom.KEY_1_LONG_PRESS_ACTION;
    State[3] = gEeprom.KEY_2_SHORT_PRESS_ACTION;
    State[4] = gEeprom.KEY_2_LONG_PRESS_ACTION;
    State[5] = gEeprom.SCAN_RESUME_MODE;
    State[6] = gEeprom.AUTO_KEYPAD_LOCK;
    State[7] = gEeprom.POWER_ON_DISPLAY_MODE;

    // 0x0EA0 - RSSI bar levels
    State = SecBuf + 0x10;
    #ifdef ENABLE_RSSI_BAR
        State[1] = gEeprom.S0_LEVEL;
        State[2] = gEeprom.S9_LEVEL;
    #endif

    // 0x0EA8 - Roger, tail tone, TX VFO, battery type
    State = SecBuf + 0x18;
    State[0] = 0x00;  // Reserved/clear
    State[1] = gEeprom.ROGER;
    State[2] = gEeprom.REPEATER_TAIL_TONE_ELIMINATION;
    State[3] = gEeprom.TX_VFO;
    State[4] = gEeprom.BATTERY_TYPE;
        
    PY25Q16_WriteBuffer(0x00A0A8, SecBuf, 0x50, false);

    // ======================================================================
    // Block 3: 0x0F18 - 0x0F1F (Scan list)
    // ======================================================================
    memset(SecBuf, 0xFF, 0x08);

    State = SecBuf;
    State[0] = (gEeprom.SCAN_LIST_DEFAULT & 0x7F) |
             ((gEeprom.SCAN_LIST_ENABLED & 0x01) << 7);

    State[1] = (uint8_t)(gEeprom.SCANLIST_PRIORITY_CH[0] & 0xFF);
    State[2] = (uint8_t)(gEeprom.SCANLIST_PRIORITY_CH[0] >> 8);
    State[3] = (uint8_t)(gEeprom.SCANLIST_PRIORITY_CH[1] & 0xFF);
    State[4] = (uint8_t)(gEeprom.SCANLIST_PRIORITY_CH[1] >> 8);

    PY25Q16_WriteBuffer(0x00A130, SecBuf, 0x08, false);

    // ======================================================================
    // Block 4: 0x0F40 - 0x0F47 (Feature flags)
    // ======================================================================
    memset(SecBuf, 0xFF, 8);

    State = SecBuf;
    
    // Preserve upper bits, update battery text (bits 2-3)
    PY25Q16_ReadBuffer(0x00A150, SecBuf, 8);
    State[0]  = gSetting_F_LOCK;
    State[7] = SecBuf[7];
    State[7] = (State[7] & ~(3u << 2)) | ((gSetting_battery_text & 3u) << 2);
    State[7] = (State[7] & ~(3u << 6)) | ((gSetting_backlight_on_tx_rx & 3u) << 6);

    PY25Q16_WriteBuffer(0x00A150, SecBuf, 8, false);

    // ======================================================================
    // Block 5: 0x0F50+ - Feature-specific flags (F4HWN)
    // ======================================================================
    #ifdef ENABLE_FEAT_F4HWN
    // 0x00A158 - Extended feature flags
    PY25Q16_ReadBuffer(0x00A158, State, 8);
    
    State[4] = (gSetting_set_tmr ? (1 << 0) : 0) | 
               (gSetting_nav_invert << 2);

    // Pack INV, LCK, MET, GUI into nibble
    uint8_t featureFlags = (gSetting_set_inv & 0x01) |
                           ((gSetting_set_lck & 0x01) << 1) |
                           ((gSetting_set_met & 0x01) << 2) |
                           ((gSetting_set_gui & 0x01) << 3);

    State[5] = ((featureFlags << 4) | (gSetting_set_ctr & 0x0F));
    State[6] = ((gSetting_set_tot & 0x0F) << 4) | (gSetting_set_eot & 0x0F);
    
    gEeprom.KEY_LOCK_PTT = gSetting_set_lck;

    PY25Q16_WriteBuffer(0x00A158, SecBuf, 8, false);
    #endif
}

/**
 * @brief Save channel data to EEPROM
 * @param Channel Channel number (0-based or special FREQ_CHANNEL/MR_CHANNEL constants)
 * @param VFO VFO identifier (0 or 1)
 * @param pVFO Pointer to VFO information structure
 * @param Mode Save mode (0-3): 0=none, 1=freq only, 2=full, 3=with name
 */
void SETTINGS_SaveChannel(uint16_t Channel, uint8_t VFO, const VFO_Info_t *pVFO, uint8_t Mode)
{
    // Calculate base offset for channel storage
    uint32_t OffsetVFO = (uint32_t)ADRESS_CHANNELS + ((uint32_t)Channel * 16U);

    // VFO/FREQ_CHANNEL uses different storage location
    if (IS_FREQ_CHANNEL(Channel)) {
        OffsetVFO = 0x009000;
    }

    // Only save if Mode >= 2 or it's a VFO channel
    if (Mode >= 2 || IS_FREQ_CHANNEL(Channel)) {
        typedef union {
            uint8_t _8[8];
            uint32_t _32[2];
        } State_t;
        
        uint8_t Buf[0x10];
        State_t *State = (State_t *)Buf;

        // RX Frequency
        State->_32[0] = pVFO->freq_config_RX.Frequency;
        // TX Offset
        State->_32[1] = pVFO->TX_OFFSET_FREQUENCY;

        State = (State_t *)(Buf + 0x8);
        State->_8[0] = pVFO->freq_config_RX.Code;
        State->_8[1] = pVFO->freq_config_TX.Code;
        State->_8[2] = (pVFO->freq_config_TX.CodeType << 4) | pVFO->freq_config_RX.CodeType;
        State->_8[3] = (pVFO->Modulation << 4) | pVFO->TX_OFFSET_FREQUENCY_DIRECTION;
        
        // Pack multiple boolean/bit fields into single byte
        State->_8[4] = 0
            | (pVFO->BUSY_CHANNEL_LOCK ? (1 << 5) : 0)
            | (pVFO->OUTPUT_POWER      << 2)
            | (pVFO->CHANNEL_BANDWIDTH << 1)
            | (pVFO->FrequencyReverse  ? (1 << 0) : 0);
        
        State->_8[6] = pVFO->STEP_SETTING;

        PY25Q16_WriteBuffer(OffsetVFO, Buf, 0x10, false);

        // Update channel attributes
        SETTINGS_UpdateChannel(Channel, pVFO, true);

        // Save channel name for MR channels
        if (IS_MR_CHANNEL(Channel)) {
            #ifndef ENABLE_KEEP_MEM_NAME
                SETTINGS_SaveChannelName(Channel, "");
            #else
                if (Mode >= 3) {
                    SETTINGS_SaveChannelName(Channel, pVFO->Name);
                }
            #endif
        }
    }
}

/**
 * @brief Save battery calibration data
 * @param batteryCalibration Pointer to 12-byte calibration array
 */
void SETTINGS_SaveBatteryCalibration(const uint16_t *batteryCalibration)
{
    PY25Q16_WriteBuffer(0x010000 + 0x140, batteryCalibration, 12, false);
}

/**
 * @brief Save channel name to EEPROM
 * @param channel Channel number
 * @param name Null-terminated string (max 10 chars)
 */
void SETTINGS_SaveChannelName(uint16_t channel, const char *name)
{
    uint32_t offset = ADRESS_CHANNELS_NAMES + ((uint32_t)channel * 16);
    uint8_t buf[16] = {0};
    
    // Copy up to 10 characters, null-terminate
    size_t len = name ? strlen(name) : 0;
    len = (len > 10) ? 10 : len;
    memcpy(buf, name, len);
    
    PY25Q16_WriteBuffer(offset, buf, 0x10, false);
}

/**
 * @brief Update channel attributes
 * @param channel Channel number
 * @param pVFO Source VFO info (can be NULL)
 * @param keep If true, preserve band/compander/scanlist from pVFO
 */
void SETTINGS_UpdateChannel(uint16_t channel, const VFO_Info_t *pVFO, bool keep)
{
    ChannelAttributes_t att = {
        .band = 0x7,
        .compander = 0,
        .unused_1 = 0,
        .unused_2 = 0,
        .exclude = 0,
        .scanlist = 0,
    };

    if (keep && pVFO != NULL) {
        att.band = pVFO->Band;
        att.compander = pVFO->Compander;
        att.scanlist = pVFO->SCANLIST_PARTICIPATION;
    }

    MR_SetChannelAttributes(channel, &att);

    // Clear name for MR channels when not keeping
    if (IS_MR_CHANNEL(channel) && !keep) {
        SETTINGS_SaveChannelName(channel, "");
    }
}

#ifdef ENABLE_FEAT_F4HWN_RESUME_STATE
/**
 * @brief Write current radio state to EEPROM for resume functionality
 * @note Saves current state, scan list, and priority channels
 */
void SETTINGS_WriteCurrentState(void)
{
    uint8_t State[8];
    
    // Save CURRENT_STATE and SCAN_LIST_DEFAULT (0x00A008)
    PY25Q16_ReadBuffer(0x00A008, State, sizeof(State));
    State[7] = (gEeprom.CURRENT_STATE & 0x07) | 
               ((gEeprom.SCAN_LIST_DEFAULT & 0x1F) << 3);
    PY25Q16_WriteBuffer(0x00A008, State, sizeof(State), false);
    
    // Save scan list configuration (0x00A130)
    PY25Q16_ReadBuffer(0x00A130, State, sizeof(State));
    State[0] = (gEeprom.SCAN_LIST_DEFAULT & 0x7F) |
               ((gEeprom.SCAN_LIST_ENABLED & 0x01) << 7);
    State[1] = (uint8_t)(gEeprom.SCANLIST_PRIORITY_CH[0] & 0xFF);
    State[2] = (uint8_t)(gEeprom.SCANLIST_PRIORITY_CH[0] >> 8);
    State[3] = (uint8_t)(gEeprom.SCANLIST_PRIORITY_CH[1] & 0xFF);
    State[4] = (uint8_t)(gEeprom.SCANLIST_PRIORITY_CH[1] >> 8);
    PY25Q16_WriteBuffer(0x00A130, State, sizeof(State), false);
}
#endif