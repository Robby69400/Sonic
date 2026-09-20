/* Copyright 2023 Dual Tachyon
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
#include <stdlib.h>  // abs()
#include "menu.h"
#include "bitmaps.h"
#include "board.h"
#include "driver/bk4819.h"
#include "driver/gpio.h"
#include "driver/st7565.h"
#include "external/printf/printf.h"
#include "functions.h"
#include "helper/battery.h"
#include "misc.h"
#include "radio.h"
#include "settings.h"
#include "ui/helper.h"
#include "ui/inputbox.h"
#include "ui/main.h"
#include "ui/ui.h"


#ifdef ENABLE_FEAT_F4HWN
    #include "driver/system.h"
#endif

center_line_t center_line = CENTER_LINE_NONE;

// Global S-level for the status bar (0..13, updated in DisplayRSSIBar)
int8_t gSmeterLevel = 0;

#ifdef ENABLE_FEAT_F4HWN
    // static int8_t RxBlink;
    static int8_t RxBlinkLed = 0;
    static int8_t RxBlinkLedCounter;
#endif

const char *VfoStateStr[] = {
       [VFO_STATE_NORMAL]="",
       [VFO_STATE_BUSY]="BUSY",
       [VFO_STATE_BAT_LOW]="BAT LOW",
       [VFO_STATE_TX_DISABLE]="TX DISABLE",
       [VFO_STATE_TIMEOUT]="TIMEOUT",
       [VFO_STATE_ALARM]="ALARM",
       [VFO_STATE_VOLTAGE_HIGH]="VOLT HIGH"
};

// ============================================================================
// DISPLAY HELPER FUNCTIONS
// ============================================================================

#if defined ENABLE_AUDIO_BAR || defined ENABLE_RSSI_BAR

/**
 * @brief Draw a horizontal level bar graph
 * @param xpos X coordinate of bar start
 * @param line Frame buffer line number
 * @param level Current level to display (0-bars)
 * @param bars Maximum bar count
 * 
 * Draws visual representation with filled bars for levels 0-(bars-4)
 * and hollow bars for levels (bars-4) to bars for fine detail
 */
static void DrawLevelBar(uint8_t xpos, uint8_t line, uint8_t level, uint8_t bars)
{
#ifndef ENABLE_FEAT_F4HWN
    const char hollowBar[] = {
        0b01111111,
        0b01000001,
        0b01000001,
        0b01111111
    };
#endif
    
    uint8_t *p_line = gFrameBuffer[line];
    level = MIN(level, bars);

    for(uint8_t i = 0; i < level; i++) {
#ifdef ENABLE_FEAT_F4HWN
        {   // flat solid squares for both MODERN and CLASSIC
            const char hollowBar[] = {
                0b00111110,
                0b00100010,
                0b00100010,
                0b00111110
            };
            const char simpleBar[] = {
                0b00111110,
                0b00111110,
                0b00111110,
                0b00111110
            };
            if(i < bars - 4) {
                memcpy(p_line + (xpos + i * 5), &simpleBar, ARRAY_SIZE(simpleBar));
            } else {
                memcpy(p_line + (xpos + i * 5), &hollowBar, ARRAY_SIZE(hollowBar));
            }
        }
#else
        if(i < bars - 4) {
            for(uint8_t j = 0; j < 4; j++)
                p_line[xpos + i * 5 + j] = (~(0x7F >> (i+1))) & 0x7F;
        }
        else {
            memcpy(p_line + (xpos + i * 5), &hollowBar, ARRAY_SIZE(hollowBar));
        }
#endif
    }
}
#endif

#ifdef ENABLE_AUDIO_BAR

void UI_DisplayAudioBar(void)
{
    if (!gSetting_mic_bar)
        return;
    if (gLowBattery && !gLowBatteryConfirmed)
        return;
    if (gCurrentFunction != FUNCTION_TRANSMIT || gScreenToDisplay != DISPLAY_MAIN
        )
        return;

#ifdef ENABLE_FEAT_F4HWN
    RxBlinkLed = 0;
    RxBlinkLedCounter = 0;
    BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false);
#endif
    UI_DisplayAudioScope();
}
#endif

// ============================================================================
// AUDIO SCOPE (living equalizer during TX)
// ============================================================================

#define SCOPE_SAMPLES        43
#define SCOPE_NOISE_GATE     50u
#define SCOPE_FLOOR_RISE     2u
#define SCOPE_FLOOR_DROP_SHR 3u
#define SCOPE_VOLUME_MIN     200u

void UI_DisplayAudioScope(void)
{
    static uint16_t g_scope_buf[SCOPE_SAMPLES];
    static uint8_t  g_scope_write  = 0;
    static uint16_t g_scope_floor  = SCOPE_VOLUME_MIN;
    static uint8_t  g_scope_ready  = 0;
    static bool     s_was_tx       = false;

    if (gCurrentFunction != FUNCTION_TRANSMIT) {
        s_was_tx = false;
        return;
    }

    if (!GPIO_IsPttPressed()
#ifdef ENABLE_FEAT_F4HWN
    && !gSetting_set_ptt_session
#endif
    )
        return;

    if (!s_was_tx) {
        for (uint8_t i = 0; i < SCOPE_SAMPLES; i++) g_scope_buf[i] = SCOPE_VOLUME_MIN;
        g_scope_write = 0u;
        g_scope_floor = SCOPE_VOLUME_MIN;
        g_scope_ready = 0;
        s_was_tx      = true;
    }

    if (g_scope_ready >= 7)
        g_scope_buf[g_scope_write] = BK4819_GetVoiceAmplitudeOut();
    else
        g_scope_ready++;

    if (g_scope_buf[g_scope_write] == 0)
        g_scope_buf[g_scope_write] = SCOPE_VOLUME_MIN;

    g_scope_write = (g_scope_write + 1u) % SCOPE_SAMPLES;

    if (gLowBattery && !gLowBatteryConfirmed)
        return;
    if (gScreenToDisplay != DISPLAY_MAIN
        )
        return;

    RxBlinkLed = 0;
    RxBlinkLedCounter = 0;
    BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false);
    const unsigned int line = 5;
    uint8_t *p_line = gFrameBuffer[line];
    memset(p_line, 0, LCD_WIDTH);

#ifdef ENABLE_FEAT_F4HWN
    // In main-only scope on line=5; clear ONLY one bottom pixel (bit 7)
    // of the lines above to avoid artifacts; clearing the entire line is unnecessary
    if (line > 0) {
        for (uint8_t x = 0; x < LCD_WIDTH; x++)
            gFrameBuffer[line - 1][x] &= ~0x80u;
        ST7565_BlitLine(line - 1);
    }
#endif

    uint16_t min_val = g_scope_buf[0];
    uint16_t max_val = g_scope_buf[0];
    for (uint8_t i = 1u; i < SCOPE_SAMPLES; i++) {
        if (g_scope_buf[i] < min_val) min_val = g_scope_buf[i];
        if (g_scope_buf[i] > max_val) max_val = g_scope_buf[i];
    }

    if (g_scope_floor > min_val)
        g_scope_floor -= ((g_scope_floor - min_val) >> SCOPE_FLOOR_DROP_SHR) + 1u;
    else
        g_scope_floor += SCOPE_FLOOR_RISE;

    const uint16_t range = (max_val > g_scope_floor) ? (max_val - g_scope_floor) : 0u;

    for (uint8_t i = 0u; i < SCOPE_SAMPLES; i++) {
        const uint8_t  idx = (g_scope_write + i) % SCOPE_SAMPLES;
        uint8_t        height = 0u;
        if (range >= SCOPE_NOISE_GATE) {
            const uint16_t v = (g_scope_buf[idx] > g_scope_floor) ? (g_scope_buf[idx] - g_scope_floor) : 0u;
            height = (uint8_t)((uint32_t)v * 7u / range);
        }
        const uint8_t mask = (height > 0u) ? (uint8_t)((0x7Fu << (7u - height)) & 0x7Fu) : 0x40u;
        uint8_t *p_col = &p_line[i * 3u];
        p_col[0] = mask;
        p_col[1] = mask;
    }

    ST7565_BlitLine(line);
}

// ============================================================================
// RSSI SIGNAL STRENGTH DISPLAY
// ============================================================================

/**
 * @brief Display RSSI bar
 * @param now Force immediate screen update
 */
void DisplayRSSIBar(const bool now)
{
    const unsigned int txt_width    = 7 * 8;                 // 8 text chars
    const unsigned int bar_x        = 2 + txt_width + 4;     // X coord of bar graph
    const unsigned int line = 5;
    uint8_t           *p_line        = gFrameBuffer[line];
    memset(p_line, 0, LCD_WIDTH);
    char               str[16];

#ifndef ENABLE_FEAT_F4HWN
    const char plus[] = {
        0b00011000,
        0b00011000,
        0b01111110,
        0b01111110,
        0b01111110,
        0b00011000,
        0b00011000,
    };
#endif

    if (gCurrentFunction == FUNCTION_TRANSMIT ||
        gScreenToDisplay != DISPLAY_MAIN
        )
        return;     // display is in use

    int16_t rssi_dBm = BK4819_GetRSSI_dBm() + dBmCorrTable[gRxVfo->Band];
    rssi_dBm = -rssi_dBm;
    if(rssi_dBm > 141) rssi_dBm = 141;
    if(rssi_dBm < 53) rssi_dBm = 53;

    uint8_t s_level = 0;
    uint8_t overS9dBm = 0;
    uint8_t overS9Bars = 0;

    if(rssi_dBm >= 93) {
        s_level = map(rssi_dBm, 141, 93, 1, 9);
    }
    else {
        s_level = 9;
        overS9dBm = map(rssi_dBm, 93, 53, 0, 40);
        overS9Bars = map(overS9dBm, 0, 40, 0, 4);
    }
    gSmeterLevel = (int8_t)(s_level + overS9Bars);
    sprintf(str, "%3d", -rssi_dBm);
    UI_PrintStringSmallNormal(str, LCD_WIDTH + 8, 0, line - 1);
    DrawLevelBar(bar_x, line, s_level + overS9Bars, 13);
    ST7565_BlitLine(line);
}

#ifdef ENABLE_AGC_SHOW_DATA
/**
 * @brief Debug display for AGC internal state
 * @param now Force immediate screen update
 * 
 * Shows AGC gain configuration for troubleshooting RX levels:
 * - AGC enable flag
 * - Gain index
 * - Calculated gain in dB
 * - Signal strength
 * - RSSI raw value
 */
void UI_MAIN_PrintAGC(bool now)
{
    char buf[20];
    memset(gFrameBuffer[3], 0, 128);
    union {
        struct {
            uint16_t _ : 5;
            uint16_t agcSigStrength : 7;
            int16_t gainIdx : 3;
            uint16_t agcEnab : 1;
        };
        uint16_t __raw;
    } reg7e;
    reg7e.__raw = BK4819_ReadRegister(0x7E);
    uint8_t gainAddr = reg7e.gainIdx < 0 ? 0x14 : 0x10 + reg7e.gainIdx;
    union {
        struct {
            uint16_t pga:3;
            uint16_t mixer:2;
            uint16_t lna:3;
            uint16_t lnaS:2;
        };
        uint16_t __raw;
    } agcGainReg;
    agcGainReg.__raw = BK4819_ReadRegister(gainAddr);
    int8_t lnaShortTab[] = {-28, -24, -19, 0};
    int8_t lnaTab[] = {-24, -19, -14, -9, -6, -4, -2, 0};
    int8_t mixerTab[] = {-8, -6, -3, 0};
    int8_t pgaTab[] = {-33, -27, -21, -15, -9, -6, -3, 0};
    int16_t agcGain = lnaShortTab[agcGainReg.lnaS] + lnaTab[agcGainReg.lna] + mixerTab[agcGainReg.mixer] + pgaTab[agcGainReg.pga];

    sprintf(buf, "%d%2d %2d %2d %3d", reg7e.agcEnab, reg7e.gainIdx, -agcGain, reg7e.agcSigStrength, BK4819_GetRSSI());
    UI_PrintStringSmallNormal(buf, 2, 0, 3);
    if(now)
        ST7565_BlitLine(3);
}
#endif

// ============================================================================
// DISPLAY UPDATE TIMESLICE
// ============================================================================

/**
 * @brief 500ms timeslice updates for main display
 * 
 * Called periodically to update display elements that change
 * at slower rates than the main loop (RSSI bar, audio bar, LEDs)
 * 
 * Handles:
 * - RSSI bar updates (every 500ms)
 * - Audio level bar updates
 * - RX LED blinking/indication
 * - End-of-transmission (EOT) visual/audio feedback
 */
void UI_MAIN_TimeSlice500ms(void)
{
    // Only update if main display is active
    if(gScreenToDisplay==DISPLAY_MAIN) {

#ifdef ENABLE_AGC_SHOW_DATA
        // Debug: show AGC data on center line
        UI_MAIN_PrintAGC(true);
        return;
#endif

        // Update RSSI bar during reception
        if(FUNCTION_IsRx()) {
            DisplayRSSIBar(true);
        }
#ifdef ENABLE_FEAT_F4HWN // Blink Green Led for white...
        else if(gSetting_set_eot > 0 && RxBlinkLed == 2)
        {
            if(RxBlinkLedCounter <= 8)
            {
                if(RxBlinkLedCounter % 2 == 0)
                {
                    if(gSetting_set_eot > 1 )
                    {
                        BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false);
                    }
                }
                else
                {
                    if(gSetting_set_eot > 1 )
                    {
                        BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, true);
                    }
                }
                RxBlinkLedCounter += 1;
            }
            else
            {
                RxBlinkLed = 0;
            }
        }
#endif
    }
}

// ============================================================================
// MAIN DISPLAY RENDERING
// ============================================================================

/**
 * @brief Render main frequency/channel display
 * 
 * This is the primary display showing:
 * - VFO A and VFO B with frequencies/channels
 * - TX/RX status and modulation
 * - Signal strength indicators
 * - TX power and offset information
 * - Scan list assignments
 * - Battery status
 * 
 * Layout (2 VFO mode):
 *  Line 0-1: VFO A frequency/channel + status
 *  Line 2-3: Center line (RSSI, audio, etc.)
 *  Line 4-5: VFO B frequency/channel + status
 *  Line 6-7: Status bar (battery, lock, etc.)
 * 
 * Layout (Main Only mode):
 *  Line 0-3: Active VFO with large frequency display
 *  Line 4-5: Center information (RSSI, audio, etc.)
 *  Line 6: Status/VFO indicator
 */
void UI_DisplayMain(void)
{
    char String[22];  // String buffer for formatted text

    center_line = CENTER_LINE_NONE;  // Center line initially available

    // Clear screen and prepare frame buffer
    UI_DisplayClear();

    // ================================================================
    // CHECK FOR LOW BATTERY ALERT
    // ================================================================
    
    if(gLowBattery && !gLowBatteryConfirmed) {
        // Display battery critical warning and block further operation
        UI_DisplayPopup("LOW BATTERY");
        ST7565_BlitFullScreen();
        return;
    }

    // ================================================================
    // CHECK FOR KEYPAD LOCK ALERT
    // ================================================================
    
    if (gEeprom.KEY_LOCK && gKeypadLocked > 0)
    {   // Display keypad lock message
        UI_PrintString("Long press #", 0, LCD_WIDTH, 1, 8);
        UI_PrintString("to unlock",    0, LCD_WIDTH, 3, 8);
        ST7565_BlitFullScreen();
        return;
    }

    // ================================================================
    // RENDER VFO
    // ================================================================
    
    // GUI style: CLASSIC in main-only, MODERN in dual-screen
    gSetting_set_gui = 1;
    const unsigned int vfo_num = 0;
    const VFO_Info_t  *vfoInfo = &gEeprom.VfoInfo[vfo_num];
    uint32_t frequency = vfoInfo->pRX->Frequency;
    if (gCurrentFunction == FUNCTION_TRANSMIT)
        frequency = vfoInfo->pTX->Frequency;
    const bool isMR = IS_MR_CHANNEL(gEeprom.ScreenChannel);
        
    if (isMR)
        {
            const bool inputting = (gInputBoxIndex != 0 && gEeprom.TX_VFO == vfo_num);
            if (!inputting)
                sprintf(String, "%u", gEeprom.ScreenChannel + 1);
            else
                sprintf(String, "M%.4s", INPUTBOX_GetAscii());
            UI_PrintStringSmallBold(String, 0, 0, 2);

            const ChannelAttributes_t* att = MR_GetChannelAttributes(gEeprom.ScreenChannel);
            if (att && att->scanlist > 0 && att->scanlist <= MR_CHANNELS_LIST) {
                sprintf(String, "%02d", att->scanlist);
                GUI_DisplaySmallestDark(String, 3, 25, false, false);
            }

            if (!inputting) {
                char dispName[22];
                SETTINGS_FetchChannelName(dispName, gEeprom.ScreenChannel);
                if (dispName[0] == 0)
                    sprintf(dispName, "%u.%05u", frequency / 100000, frequency % 100000);
                UI_PrintString(dispName, 40, 0, 1, 8);
            }
        }

        if (INPUTBOX_FrequencyIsActive())
        {
            const char *ascii = INPUTBOX_FrequencyGetString();
            UI_DisplayFrequency(ascii, 0, 2, false);
        }
        else
        {
            sprintf(String, "%3u.%05u", frequency / 100000, frequency % 100000);
            uint8_t small_y    = isMR ? 3 : 2;
            uint8_t small_x    = 105;
            UI_PrintString(String + 7, small_x - (strlen(String + 7) * 6 / 2), 0, small_y, 8);
            String[7] = 0;
            uint8_t big_y = isMR ? 3 : 2;
            uint8_t big_x = 40;
            UI_DisplayFrequency(String, big_x - (strlen(String) * 8 / 2), big_y, false);
        }

        
        {
            const char *s = "";
            const ModulationMode_t mod = vfoInfo->Modulation;
            switch (mod) {
                case MODULATION_FM: {
                    const FREQ_Config_t *pCfg = vfoInfo->pRX;
                    const char *code_list[] = {"FM", "CT", "DCS", "DCR"};
                    if (pCfg->CodeType < 4) s = code_list[pCfg->CodeType];
                    break;
                }
                default: s = gModulationStr[mod]; break;
            }
            if (s[0] != '\0') {
                uint8_t x_mr = 116, y_mr = 5;
                uint8_t x_vfo = 116, y_vfo = 5;
                uint8_t mod_x_base = isMR ? x_mr : x_vfo;
                uint8_t mod_y      = isMR ? y_mr : y_vfo;
                uint8_t mod_x = mod_x_base - (uint8_t)(strlen(s) * 7 / 2) - 1;
                UI_PrintStringSmallBold(s, LCD_WIDTH + mod_x, 0, mod_y);
            }
        }

        // ── PTT TOGGLE ────────────────────────────────────────────────
        if (gSetting_set_ptt_session) {
            uint8_t x_mr = 4, y_mr = 3;
            uint8_t x_vfo = 4, y_vfo = 1;
            uint8_t x = isMR ? x_mr : x_vfo;
            uint8_t y = isMR ? y_mr : y_vfo;
            UI_PrintStringSmallBold("T", LCD_WIDTH + x, 0, y);
        }

            uint8_t x_mr = 91, y_mr = 5;
            uint8_t x_vfo = 91, y_vfo = 5;
            uint8_t x = isMR ? x_mr : x_vfo;
            uint8_t y = isMR ? y_mr : y_vfo;
            
            const char pwr_base[][2] = {"L","M","H"}; // index 0 (X) → show "L" as base
            UI_PrintStringSmallBold(pwr_base[vfoInfo->OUTPUT_POWER], LCD_WIDTH + x, 0, y);

        if (vfoInfo->freq_config_RX.Frequency != vfoInfo->freq_config_TX.Frequency)
        {
            uint8_t x_mr = 4, y_mr = 3;
            uint8_t x_vfo = 4, y_vfo = 2;
            const char *dir[] = {"", "+", "-"};
            const char *d = dir[vfoInfo->TX_OFFSET_FREQUENCY_DIRECTION % 3];
            if (d[0] != '\0') {
                uint8_t x = isMR ? x_mr : x_vfo;
                uint8_t y = isMR ? y_mr : y_vfo;
                UI_PrintStringSmallBold(d, LCD_WIDTH + x, 0, y);
            }
        }

        
        {
            uint8_t x_mr = 62, y_mr = 5;
            uint8_t x_vfo = 62, y_vfo = 5;
            char stepStr[8];
            const uint16_t step = gStepFrequencyTable[vfoInfo->STEP_SETTING];
            if (step == 833) {
                strcpy(stepStr, "8.33");
            } else {
                uint32_t v = (uint32_t)step * 10;
                uint16_t integer = v / 1000;
                uint16_t decimal = (v % 1000) / 10;
                if (integer == 0)        sprintf(stepStr, "0.%02u", decimal);
                else if (integer >= 100) sprintf(stepStr, "%u", integer);
                else                     sprintf(stepStr, "%u.%02u", integer, decimal);
            }
            uint8_t x = isMR ? x_mr : x_vfo;
            uint8_t y = isMR ? y_mr : y_vfo;
            UI_PrintStringSmallBold(stepStr, LCD_WIDTH + x - (uint8_t)(strlen(stepStr) * 3), 0, y);
        }

        
        {
            uint8_t x_mr = 8, y_mr = 5;
            uint8_t x_vfo = 8, y_vfo = 5;
            char sqlStr[4];
            sprintf(sqlStr, "%u", gEeprom.SQUELCH_LEVEL);
            uint8_t x = isMR ? x_mr : x_vfo;
            uint8_t y = isMR ? y_mr : y_vfo;
            UI_PrintStringSmallBold(sqlStr, LCD_WIDTH + x, 0, y);
        }

        
        {
            uint8_t x_mr = 34, y_mr = 5;
            uint8_t x_vfo = 34, y_vfo = 5;
            const char *bwNames[] = {"W", "N"};
            const char *bw = bwNames[vfoInfo->CHANNEL_BANDWIDTH & 1];
            uint8_t x = isMR ? x_mr : x_vfo;
            uint8_t y = isMR ? y_mr : y_vfo;
            UI_PrintStringSmallBold(bw, LCD_WIDTH + x - (uint8_t)(strlen(bw) * 3), 0, y);
        }

        {
            enum VfoState_t state = VfoState[vfo_num];
            if (state != VFO_STATE_NORMAL) {
                const char *msg = (state < ARRAY_SIZE(VfoStateStr)) ? VfoStateStr[state] : "";
                uint8_t y_mr = 3, y_vfo = 2;
                uint8_t y_pos = isMR ? y_mr : y_vfo;
                memset(gFrameBuffer[y_pos],     0, LCD_WIDTH);
                memset(gFrameBuffer[y_pos + 1], 0, LCD_WIDTH);
                uint8_t tw = (uint8_t)(strlen(msg) * 8);
                UI_PrintString(msg, (LCD_WIDTH - tw) / 2, 0, y_pos, 8);
            }
        }

        // "VFO MODE" / "MR MODE" ─────────────────────────────
        char str[19];
        if (isMR) {
            if (gEeprom.SCRAMBLING_TYPE)
                sprintf(str, "CHANNEL SCR %d", gEeprom.SCRAMBLING_TYPE);
            else
                sprintf(str, "CHANNEL");
            UI_PrintStringSmallNormal(str, 0, 0, 0);
            GUI_DisplaySmallestDark("SQL",  6,  42, false, false);
            GUI_DisplaySmallestDark("BAND", 28, 42, false, false);
            GUI_DisplaySmallestDark("STEP", 58, 42, false, false);
            GUI_DisplaySmallestDark("POW",  88, 42, false, false);
            GUI_DisplaySmallestDark("MOD",  110,42, false, false);
        } else {
            if (gEeprom.SCRAMBLING_TYPE)
                sprintf(str, "FREQUENCY SCR %d", gEeprom.SCRAMBLING_TYPE);
            else
                sprintf(str, "FREQUENCY");
            UI_PrintStringSmallNormal(str, 0, 0, 0);
            GUI_DisplaySmallestDark("SQL",  6,  42, false, false);
            GUI_DisplaySmallestDark("BND", 28, 42, false, false);
            GUI_DisplaySmallestDark("STP", 58, 42, false, false);
            GUI_DisplaySmallestDark("POW",  88, 42, false, false);
            GUI_DisplaySmallestDark("MOD",  110,42, false, false);
        }

        // ── TX / RX INDICATOR ────────────────────────────────────────
        if (gCurrentFunction == FUNCTION_TRANSMIT)
            GUI_DisplaySmallestDark("TX", 2, 25, false, false);
        else if (FUNCTION_IsRx())
            GUI_DisplaySmallestDark("RX", 2, 25, false, false);

        ST7565_BlitFullScreen();
        return;
}
