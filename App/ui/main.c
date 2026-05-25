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
#include "app/dtmf.h"
#ifdef ENABLE_AM_FIX
    #include "am_fix.h"
#endif
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
#include "audio.h"

#ifdef ENABLE_FEAT_F4HWN
    #include "driver/system.h"
#endif

center_line_t center_line = CENTER_LINE_NONE;

// Глобальный S-уровень для статус-бара (0..13, обновляется в DisplayRSSIBar)
int8_t gSmeterLevel = 0;

#ifdef ENABLE_FEAT_F4HWN
    // static int8_t RxBlink;
    static int8_t RxBlinkLed = 0;
    static int8_t RxBlinkLedCounter;
    static int8_t RxLine = -1;
    static uint32_t RxOnVfofrequency;

    static bool isMainOnly()
    {
        return (gEeprom.DUAL_WATCH == DUAL_WATCH_OFF) && (gEeprom.CROSS_BAND_RX_TX == CROSS_BAND_OFF);
    }
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

// ─── КОМПАКТНЫЙ БАР между линиями y=10 и y=14 ───────────────────────────────
// xpos, y_pos=9 → рисует на пикселях 11,12,13 (dy=2,3,4)
// level_percent: 0–100
#ifdef ENABLE_FEAT_F4HWN
static void DrawCompactBar(uint8_t xpos, uint8_t y_pos, uint8_t level_percent)
{
    const uint8_t bar_width     = 60;
    const uint8_t section_width = 5;
    const uint8_t gap           = 1;
    const uint8_t step_px       = section_width + gap;
    const uint8_t max_sections  = 10;

    uint8_t sections = (max_sections * level_percent) / 100;
    if (sections > max_sections) sections = max_sections;

    // Очистка области бара (3 пикселя в высоту: dy=2,3,4)
    for (uint8_t dy = 2; dy < 5; dy++) {
        uint8_t y = y_pos + dy;
        if (y >= LCD_HEIGHT) continue;
        uint8_t *p; uint8_t bit;
        if (y < 8) { p = gStatusLine; bit = y; }
        else        { p = gFrameBuffer[(y - 8) >> 3]; bit = (y - 8) & 7; }
        for (uint8_t x = xpos; x < xpos + bar_width && x < LCD_WIDTH; x++)
            p[x] &= ~(1u << bit);
    }

    // Заполненные секции
    for (uint8_t s = 0; s < sections; s++) {
        uint8_t sx = xpos + s * step_px;
        for (uint8_t dy = 2; dy < 5; dy++) {
            uint8_t y = y_pos + dy;
            if (y >= LCD_HEIGHT) continue;
            uint8_t *p; uint8_t bit;
            if (y < 8) { p = gStatusLine; bit = y; }
            else        { p = gFrameBuffer[(y - 8) >> 3]; bit = (y - 8) & 7; }
            for (uint8_t dx = 0; dx < section_width; dx++) {
                if (sx + dx >= LCD_WIDTH || sx + dx >= xpos + bar_width) break;
                p[sx + dx] |= (1u << bit);
            }
        }
    }
}
#endif  // ENABLE_FEAT_F4HWN

#ifdef ENABLE_AUDIO_BAR
static void DrawCompactLevelBar(uint8_t xpos, uint8_t y_pos, uint16_t voice_amp)
{
    // Масштабируем: 200(тишина)→0%, 3000→100%
    uint8_t level_percent;
    if (voice_amp <= 200u)
        level_percent = 0;
    else if (voice_amp >= 3000u)
        level_percent = 100;
    else
        level_percent = (uint8_t)((uint32_t)(voice_amp - 200u) * 100u / (3000u - 200u));

    DrawCompactBar(xpos, y_pos, level_percent);
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

#ifdef ENABLE_FEAT_F4HWN
    if (isMainOnly()) {
        // Компактный бар между линиями y=10 и y=14 (как в референсном FW)
        DrawCompactLevelBar(42, 9, BK4819_GetVoiceAmplitudeOut());
    } else {
        UI_DisplayAudioScope();
    }
#else
    UI_DisplayAudioScope();
#endif
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

#ifdef ENABLE_FEAT_F4HWN
    RxBlinkLed = 0;
    RxBlinkLedCounter = 0;
    BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false);
    const unsigned int line = isMainOnly() ? 5 : 3;
#else
    const unsigned int line = 3;
#endif

    uint8_t *p_line = gFrameBuffer[line];
    memset(p_line, 0, LCD_WIDTH);

#ifdef ENABLE_FEAT_F4HWN
    // В main-only скоп на line=5; очищаем ТОЛЬКО 1 нижний пиксель (bit 7)
    // строки выше, чтобы не было артефакта — полная очистка строки не нужна
    if (isMainOnly() && line > 0) {
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
#if defined(ENABLE_RSSI_BAR)

    const unsigned int txt_width    = 7 * 8;                 // 8 text chars
    const unsigned int bar_x        = 2 + txt_width + 4;     // X coord of bar graph

#ifdef ENABLE_FEAT_F4HWN
    /*
    const char empty[] = {
        0b00000000,
        0b00000000,
        0b00000000,
        0b00000000,
        0b00000000,
        0b00000000,
        0b00000000,
    };
    */

    unsigned int line;
    if (isMainOnly())
    {
        line = 5;
    }
    else
    {
        line = 3;
    }

    //char rx[4];
    //sprintf(String, "%d", RxBlink);
    //UI_PrintStringSmallBold(String, 80, 0, RxLine);

    if(RxLine >= 0 && center_line != CENTER_LINE_IN_USE && !isMainOnly())
    {
        static bool clean = false;
        uint8_t *p_line0 = gFrameBuffer[RxLine + 0];

        clean = !clean;

        if(clean) {
            for(uint8_t i = 0; i < sizeof(BITMAP_VFO_Default); i++)
                p_line0[i] = (p_line0[i] & 0x80) | BITMAP_VFO_Default[i];
        } else {
            for(uint8_t i = 0; i < sizeof(BITMAP_VFO_Empty); i++)
                p_line0[i] = (p_line0[i] & 0x80) | BITMAP_VFO_Empty[i];
        }

        ST7565_BlitLine(RxLine);
    }
#else
    const unsigned int line = 3;
#endif
    uint8_t           *p_line        = gFrameBuffer[line];
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

    if ((gEeprom.KEY_LOCK && gKeypadLocked > 0) || center_line != CENTER_LINE_RSSI)
    {
#ifdef ENABLE_FEAT_F4HWN
        // Даже при раннем выходе обновляем gSmeterLevel для статус-бара
        if (isMainOnly() && FUNCTION_IsRx() && gScreenToDisplay == DISPLAY_MAIN
            && gCurrentFunction != FUNCTION_TRANSMIT)
        {
            int16_t rssi_tmp = BK4819_GetRSSI_dBm() + dBmCorrTable[gRxVfo->Band];
            rssi_tmp = -rssi_tmp;
            if (rssi_tmp > 141) rssi_tmp = 141;
            if (rssi_tmp < 53)  rssi_tmp = 53;
            uint8_t sl = 0, ob = 0;
            if (rssi_tmp >= 93) {
                sl = map(rssi_tmp, 141, 93, 1, 9);
            } else {
                sl = 9;
                uint8_t od = map(rssi_tmp, 93, 53, 0, 40);
                ob = map(od, 0, 40, 0, 4);
            }
            gSmeterLevel = (int8_t)(sl + ob);
        }
#endif
        return;     // display is in use
    }

    if (gCurrentFunction == FUNCTION_TRANSMIT ||
        gScreenToDisplay != DISPLAY_MAIN
        )
        return;     // display is in use

    if (now)
        memset(p_line, 0, LCD_WIDTH);

#ifdef ENABLE_FEAT_F4HWN
    int16_t rssi_dBm =
        BK4819_GetRSSI_dBm()

        + dBmCorrTable[gRxVfo->Band];

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
#else
    const int16_t s0_dBm   = -gEeprom.S0_LEVEL;                  // S0 .. base level
    const int16_t rssi_dBm =
        BK4819_GetRSSI_dBm()

        + dBmCorrTable[gRxVfo->Band];

    int s0_9 = gEeprom.S0_LEVEL - gEeprom.S9_LEVEL;
    const uint8_t s_level = MIN(MAX((int32_t)(rssi_dBm - s0_dBm)*100 / (s0_9*100/9), 0), 9); // S0 - S9
    uint8_t overS9dBm = MIN(MAX(rssi_dBm + gEeprom.S9_LEVEL, 0), 99);
    uint8_t overS9Bars = MIN(overS9dBm/10, 4);
#endif

#ifdef ENABLE_FEAT_F4HWN
    // Всегда обновляем gSmeterLevel для статус-бара, независимо от режима
    gSmeterLevel = (int8_t)(s_level + overS9Bars);

    if (isMainOnly()) {
        // В main-only линия 5 занята аудископом — ничего не рисуем на ней
    } else if (gSetting_set_gui) {
        sprintf(str, "%3d", -rssi_dBm);
        UI_PrintStringSmallNormal(str, LCD_WIDTH + 8, 0, line - 1);

        DrawLevelBar(bar_x, line, s_level + overS9Bars, 13);
        if (now) ST7565_BlitLine(line);
    } else {
        sprintf(str, "% 4d %s", -rssi_dBm, "dBm");
        GUI_DisplaySmallest(str, 2, 25, false, true);

        DrawLevelBar(bar_x, line, s_level + overS9Bars, 13);
        if (now) ST7565_BlitLine(line);
    }
#else
    if(overS9Bars == 0) {
        sprintf(str, "% 4d S%d", -rssi_dBm, s_level);
    }
    else {
        sprintf(str, "%4d S9+%d", -rssi_dBm, overS9dBm);
    }

    UI_PrintStringSmallNormal(str, 2, 0, line);
#endif
#else
    int16_t rssi = BK4819_GetRSSI();
    uint8_t Level;

    if (rssi >= gEEPROM_RSSI_CALIB[gRxVfo->Band][3]) {
        Level = 6;
    } else if (rssi >= gEEPROM_RSSI_CALIB[gRxVfo->Band][2]) {
        Level = 4;
    } else if (rssi >= gEEPROM_RSSI_CALIB[gRxVfo->Band][1]) {
        Level = 2;
    } else if (rssi >= gEEPROM_RSSI_CALIB[gRxVfo->Band][0]) {
        Level = 1;
    } else {
        Level = 0;
    }

    uint8_t *pLine = (gEeprom.RX_VFO == 0)? gFrameBuffer[2] : gFrameBuffer[6];
    if (now) {
        memset(pLine, 0, 23);
        ST7565_BlitFullScreen();
    }
#endif

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

        // Update RSSI bar during reception (в main-only обновляет только gSmeterLevel, не рисует на линии 5)
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

                    if(gSetting_set_eot == 1 || gSetting_set_eot == 3)
                    {
                        switch(RxBlinkLedCounter)
                        {
                            case 1:
                            AUDIO_PlayBeep(BEEP_400HZ_30MS);
                            break;

                            case 3:
                            AUDIO_PlayBeep(BEEP_400HZ_30MS);
                            break;

                            case 5:
                            AUDIO_PlayBeep(BEEP_500HZ_30MS);
                            break;

                            case 7:
                            AUDIO_PlayBeep(BEEP_600HZ_30MS);
                            break;
                        }
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
 *  Line 2-3: Center line (RSSI, DTMF, audio, etc.)
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
    
#ifndef ENABLE_FEAT_F4HWN
    if (gEeprom.KEY_LOCK && gKeypadLocked > 0)
    {   // Display keypad lock message
        UI_PrintString("Long press #", 0, LCD_WIDTH, 1, 8);
        UI_PrintString("to unlock",    0, LCD_WIDTH, 3, 8);
        ST7565_BlitFullScreen();
        return;
    }
#else
    if (gEeprom.KEY_LOCK && gKeypadLocked > 0)
    {   // tell user how to unlock the keyboard
        uint8_t shift = 3;

        /*
        BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, true);
        SYSTEM_DelayMs(50);
        BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, false);
        SYSTEM_DelayMs(50);
        */

        if(isMainOnly())
        {
            shift = 5;
        }
        //memcpy(gFrameBuffer[shift] + 2, gFontKeyLock, sizeof(gFontKeyLock));
        UI_PrintStringSmallBold("UNLOCK KEYBOARD", 12, 0, shift);
        //memcpy(gFrameBuffer[shift] + 120, gFontKeyLock, sizeof(gFontKeyLock));

        /*
        for (uint8_t i = 12; i < 116; i++)
        {
            gFrameBuffer[shift][i] ^= 0xFF;
        }
        */
    }
#endif

    // ================================================================
    // RENDER VFO DISPLAYS
    // ================================================================
    
    // Determine which VFO is active for TX
    unsigned int activeTxVFO = gRxVfoIsActive ? gEeprom.RX_VFO : gEeprom.TX_VFO;

    // GUI style: CLASSIC in main-only, MODERN in dual-screen
    gSetting_set_gui = isMainOnly() ? 1 : 0;

    // ================================================================
    // MAIN ONLY DISPLAY — полностью по дизайну SU-75
    // Трогай ТОЛЬКО координаты x_mr/x_vfo y_mr/y_vfo
    // Двойной VFO loop ниже не изменяется
    // ================================================================
    if (isMainOnly())
    {
        const unsigned int vfo_num = activeTxVFO;
        const VFO_Info_t  *vfoInfo = &gEeprom.VfoInfo[vfo_num];
        uint32_t frequency = vfoInfo->pRX->Frequency;
        if (gCurrentFunction == FUNCTION_TRANSMIT)
            frequency = vfoInfo->pTX->Frequency;

        const bool isMR = IS_MR_CHANNEL(gEeprom.ScreenChannel[vfo_num]);

        // ── Очистка ──────────────────────────────────────────────────
        memset(gFrameBuffer, 0, sizeof(gFrameBuffer));

        // ── ГОРИЗОНТАЛЬНЫЕ ЛИНИИ (рисуем ПЕРВЫМИ, чтобы надписи были поверх) ──
        // Структура: {y, x_start, x_end, step(1=сплошная 2=пунктир)}
        typedef struct { uint8_t y, x0, x1, step; } hline_t;
        static const hline_t vfo_hlines[] = {
           // { 10, 0, 127, 2 },
            { 12, 0, 127, 2 },
            { 21, 0, 127, 2 },
            { 50, 0, 127, 2 },   // нижняя: опущена на 5px (было 47)
        };
        static const hline_t mr_hlines[] = {
           // { 10, 0, 127, 2 },
            { 12, 0, 127, 2 },
            { 30, 0, 127, 2 },
            { 50, 0, 127, 2 },   // нижняя: опущена на 5px (было 47)
        };
        const hline_t *hl     = isMR ? mr_hlines  : vfo_hlines;
        uint8_t        hl_cnt = isMR ? 3 : 3;
        for (uint8_t i = 0; i < hl_cnt; i++) {
            for (uint8_t x = hl[i].x0; x <= hl[i].x1; x += hl[i].step) {
                uint8_t y = hl[i].y;
                if (y < 8) gStatusLine[x] |= (1u << y);
                else       gFrameBuffer[(y - 8) >> 3][x] |= (1u << ((y - 8) & 7));
            }
        }

        // ── ВЕРТИКАЛЬНЫЕ ЛИНИИ (рисуем ПЕРВЫМИ, чтобы надписи были поверх) ──
        typedef struct { uint8_t x, y0, y1, step; } vline_t;
        static const vline_t vfo_vlines[] = {
            { 14, 23, 46, 2 },
        };
        static const vline_t mr_vlines[] = {
            { 34, 18, 28, 2 },
            { 14, 32, 47, 2 },
        };
        const vline_t *vl     = isMR ? mr_vlines  : vfo_vlines;
        uint8_t        vl_cnt = isMR ? 2 : 1;
        for (uint8_t i = 0; i < vl_cnt; i++) {
            for (uint8_t y = vl[i].y0; y <= vl[i].y1; y += vl[i].step) {
                uint8_t x = vl[i].x;
                if (y < 8) gStatusLine[x] |= (1u << y);
                else       gFrameBuffer[(y - 8) >> 3][x] |= (1u << ((y - 8) & 7));
            }
        }

        if (isMR)
        {
            const bool inputting = (gInputBoxIndex != 0 && gEeprom.TX_VFO == vfo_num);
            if (!inputting)
                sprintf(String, "M%u", gEeprom.ScreenChannel[vfo_num] + 1);
            else
                sprintf(String, "M%.3s", INPUTBOX_GetAscii());
            // Номер канала слева — по центру в левой зоне
            UI_PrintString(String, 18 - (strlen(String) * 4), 0, 1, 8);

            // Список сканирования
            const ChannelAttributes_t* att = MR_GetChannelAttributes(gEeprom.ScreenChannel[vfo_num]);
            if (att && att->scanlist > 0 && att->scanlist <= MR_CHANNELS_LIST) {
                sprintf(String, "%02d", att->scanlist);
                GUI_DisplaySmallestDark(String, 18, 25, false, false);
            }

            // Имя канала (или частота если нет имени)
            if (!inputting) {
                char dispName[22];
                SETTINGS_FetchChannelName(dispName, gEeprom.ScreenChannel[vfo_num]);
                if (dispName[0] == 0)
                    sprintf(dispName, "%u.%05u", frequency / 100000, frequency % 100000);
                UI_PrintString(dispName, 85 - (strlen(dispName) * 4), 0, 1, 8);
            }
        }

        // ── ЧАСТОТА ───────────────────────────────────────────────────
        if (gInputBoxIndex > 0 && IS_FREQ_CHANNEL(gEeprom.ScreenChannel[vfo_num]) && gEeprom.TX_VFO == vfo_num)
        {
            // Ввод частоты
            const char *ascii = INPUTBOX_GetAscii();
            bool isGigaF = frequency >= _1GHz_in_KHz;
            sprintf(String, "%.*s.%.3s", 3 + isGigaF, ascii, ascii + 3 + isGigaF);
            UI_PrintStringSmallNormal(String + 7, 85, 0, 2);
            String[7] = 0;
            UI_DisplayFrequency(String, 25, 2, false);
        }
        else
        {
            sprintf(String, "%3u.%05u", frequency / 100000, frequency % 100000);

            // Мелкие нули частоты
            uint8_t small_y    = isMR ? 3 : 2;
            uint8_t small_x    = isMR ? 116 : 114;
            UI_PrintString(String + 7, small_x - (strlen(String + 7) * 6 / 2), 0, small_y, 8);
            String[7] = 0;

            // Большая частота
            uint8_t big_y = isMR ? 3 : 2;
            uint8_t big_x = isMR ? 68 : 50;
            if (isMR)
                UI_PrintString(String, big_x - (strlen(String) * 6 / 2), 0, big_y, 8);
            else
                UI_DisplayFrequency(String, big_x - (strlen(String) * 8 / 2), big_y, false);
        }

        // ── МОДУЛЯЦИЯ ─────────────────────────────────────────────────
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
            uint8_t x_mr = 4, y_mr = 2;
            uint8_t x_vfo = 4, y_vfo = 1;
            uint8_t x = isMR ? x_mr : x_vfo;
            uint8_t y = isMR ? y_mr : y_vfo;
            UI_PrintStringSmallBold("S", LCD_WIDTH + x, 0, y);
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

        // ── ШАГ ───────────────────────────────────────────────────────
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

        // ── ШУМОДАВ ───────────────────────────────────────────────────
        {
            uint8_t x_mr = 8, y_mr = 5;
            uint8_t x_vfo = 8, y_vfo = 5;
            char sqlStr[4];
            sprintf(sqlStr, "%u", gEeprom.SQUELCH_LEVEL);
            uint8_t x = isMR ? x_mr : x_vfo;
            uint8_t y = isMR ? y_mr : y_vfo;
            UI_PrintStringSmallBold(sqlStr, LCD_WIDTH + x, 0, y);
        }

        // ── ПОЛОСА ────────────────────────────────────────────────────
        {
            uint8_t x_mr = 34, y_mr = 5;
            uint8_t x_vfo = 34, y_vfo = 5;
            const char *bwNames[] = {"W", "N"};
            const char *bw = bwNames[vfoInfo->CHANNEL_BANDWIDTH & 1];
            uint8_t x = isMR ? x_mr : x_vfo;
            uint8_t y = isMR ? y_mr : y_vfo;
            UI_PrintStringSmallBold(bw, LCD_WIDTH + x - (uint8_t)(strlen(bw) * 3), 0, y);
        }

        // ── СКРЕМБЛЕР ─────────────────────────────────────────────────
#ifdef ENABLE_SCRAMBLER
        if (vfoInfo->SCRAMBLING_TYPE > 0 && gSetting_ScrambleEnable) {
            uint8_t x_mr = 35, y_mr = 25;
            uint8_t x_vfo = 20, y_vfo = 32;
            uint8_t x = isMR ? x_mr : x_vfo;
            uint8_t y = isMR ? y_mr : y_vfo;
            GUI_DisplaySmallest("SC", x, y, false, true);
        }
#endif

        // ── СОСТОЯНИЕ VFO (TIMEOUT/ALARM/etc) ────────────────────────
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

        // ── МЕТКИ "VFO MODE" / "MR MODE" ─────────────────────────────
        if (isMR) {
            GUI_DisplaySmallestDark("MODE", 16, 2, false, true);
            GUI_DisplaySmallestDark("MR",   1,  2, false, true);
            GUI_DisplaySmallestDark("CHAN", 104,2, false, true);
            GUI_DisplaySmallestDark("SQL",  6,  40, false, false);
            GUI_DisplaySmallestDark("BAND", 28, 40, false, false);
            GUI_DisplaySmallestDark("STEP", 58, 40, false, false);
            GUI_DisplaySmallestDark("POW",  88, 40, false, false);
            GUI_DisplaySmallestDark("MOD",  110,40, false, false);
        } else {
            GUI_DisplaySmallestDark("MODE", 16, 2, false, true);
            GUI_DisplaySmallestDark("VFO",  1,  2, false, false);
            GUI_DisplaySmallestDark("FREQ", 104,2, false, true);
            GUI_DisplaySmallestDark("SQL",  6,  40, false, false);
            GUI_DisplaySmallestDark("BAND", 28, 40, false, false);
            GUI_DisplaySmallestDark("STEP", 58, 40, false, false);
            GUI_DisplaySmallestDark("POW",  88, 40, false, false);
            GUI_DisplaySmallestDark("MOD",  110,40, false, false);
        }

        // ── TX / RX ИНДИКАТОР ────────────────────────────────────────
        if (gCurrentFunction == FUNCTION_TRANSMIT)
            GUI_DisplaySmallestDark("TX", 2, 25, false, false);
        else if (FUNCTION_IsRx())
            GUI_DisplaySmallestDark("RX", 2, 25, false, false);

        // ── Перерисовка MR-линии y=30 поверх имени канала ────────────
        // UI_PrintString с 8px шрифтом занимает y=16..31, затирая линию y=30
        if (isMR) {
            for (uint8_t x = 0; x <= 127; x += 2)
                gFrameBuffer[(30 - 8) >> 3][x] |= (1u << ((30 - 8) & 7));
        }

        ST7565_BlitFullScreen();
        return;
    }

    // Render both VFOs (or single VFO in main-only mode)
    for (unsigned int vfo_num = 0; vfo_num < 2; vfo_num++)
    {
#ifdef ENABLE_FEAT_F4HWN
        const unsigned int line0 = 0;  // text screen line
        const unsigned int line1 = 4;
        unsigned int line;
        if (isMainOnly())
        {
            line       = 0;
        }
        else
        {
            line       = (vfo_num == 0) ? line0 : line1;
        }
        const bool         isMainVFO  = (vfo_num == gEeprom.TX_VFO);
        uint8_t           *p_line0    = gFrameBuffer[line + 0];
        enum Vfo_txtr_mode mode       = VFO_MODE_NONE;      
#else
        const unsigned int line0 = 0;  // text screen line
        const unsigned int line1 = 4;
        const unsigned int line       = (vfo_num == 0) ? line0 : line1;
        const bool         isMainVFO  = (vfo_num == gEeprom.TX_VFO);
        uint8_t           *p_line0    = gFrameBuffer[line + 0];
        uint8_t           *p_line1    = gFrameBuffer[line + 1];
        enum Vfo_txtr_mode mode       = VFO_MODE_NONE;
#endif

#ifdef ENABLE_FEAT_F4HWN
    if (isMainOnly())
    {
        if (activeTxVFO != vfo_num)
        {
            continue;
        }
    }
#endif

#ifdef ENABLE_FEAT_F4HWN
        if (activeTxVFO != vfo_num || isMainOnly())
#else
        if (activeTxVFO != vfo_num) // this is not active TX VFO
#endif
        {
            // highlight the selected/used VFO with a marker
            if (isMainVFO)
                memcpy(p_line0 + 0, BITMAP_VFO_Default, sizeof(BITMAP_VFO_Default));
        }
        else // active TX VFO
        {   // highlight the selected/used VFO with a marker
            if (isMainVFO)
                memcpy(p_line0 + 0, BITMAP_VFO_Default, sizeof(BITMAP_VFO_Default));
            else
                memcpy(p_line0 + 0, BITMAP_VFO_NotDefault, sizeof(BITMAP_VFO_NotDefault));
        }

        uint32_t frequency = gEeprom.VfoInfo[vfo_num].pRX->Frequency;

        if(TX_freq_check(frequency) != 0 && gEeprom.VfoInfo[vfo_num].BUSY_CHANNEL_LOCK == true)
        {
            if(isMainOnly())
                memcpy(p_line0 + 14, BITMAP_VFO_Lock, sizeof(BITMAP_VFO_Lock));
            else
                memcpy(p_line0 + 24, BITMAP_VFO_Lock, sizeof(BITMAP_VFO_Lock));
        }

        if (gCurrentFunction == FUNCTION_TRANSMIT)
        {   // transmitting

            {
                if (activeTxVFO == vfo_num)
{   // show the TX symbol
    mode = VFO_MODE_TX;
    // UI_PrintStringSmallBold("TX", 8, 0, line);  <-- ЭТУ СТРОКУ УДАЛИТЬ
}
            }
        }
        else
        {   // receiving .. show the RX symbol
            mode = VFO_MODE_RX;
            //if (FUNCTION_IsRx() && gEeprom.RX_VFO == vfo_num) {
            if (FUNCTION_IsRx()) {
                if (gEeprom.RX_VFO == vfo_num && VfoState[vfo_num] == VFO_STATE_NORMAL) {
#ifdef ENABLE_FEAT_F4HWN
                    RxBlinkLed = 1;
                    RxBlinkLedCounter = 0;
                    RxLine = line;
                    RxOnVfofrequency = frequency;
#else
                    UI_PrintStringSmallBold("RX", 8, 0, line);
#endif
                }
#ifdef ENABLE_FEAT_F4HWN
                else {
                    if(RxBlinkLed == 1)
                        RxBlinkLed = 2;
                }
            }
            else {
                if(RxOnVfofrequency == frequency && !isMainOnly()) {
                    GUI_DisplaySmallest(">>", 8, RxLine == 0 ? 1 : 33, false, true);
                }

                if(RxBlinkLed == 1)
                    RxBlinkLed = 2;

                RxLine = -1;
            }
#endif
        }

        if (IS_MR_CHANNEL(gEeprom.ScreenChannel[vfo_num]))
        {   // channel mode
            const unsigned int x = 2;
            const bool inputting = gInputBoxIndex != 0 && gEeprom.TX_VFO == vfo_num;
            if (!inputting)
                sprintf(String, "M%u", gEeprom.ScreenChannel[vfo_num] + 1);
            else
                sprintf(String, "M%.3s", INPUTBOX_GetAscii());  // show the input text
            UI_PrintStringSmallNormal(String, x, 0, line + 1);
        }
        else if (IS_FREQ_CHANNEL(gEeprom.ScreenChannel[vfo_num]))
        {   // frequency mode
            // show the frequency band number
            const unsigned int x = 2;
            char * buf = gEeprom.VfoInfo[vfo_num].pRX->Frequency < _1GHz_in_KHz ? "" : "+";
            sprintf(String, "F%u%s", 1 + gEeprom.ScreenChannel[vfo_num] - FREQ_CHANNEL_FIRST, buf);
            UI_PrintStringSmallNormal(String, x, 0, line + 1);
        }

        // ************

        enum VfoState_t state = VfoState[vfo_num];

        if (state != VFO_STATE_NORMAL)
        {
            if (state < ARRAY_SIZE(VfoStateStr))
                UI_PrintString(VfoStateStr[state], 31, 0, line, 8);
        }
        else if (gInputBoxIndex > 0 && IS_FREQ_CHANNEL(gEeprom.ScreenChannel[vfo_num]) && gEeprom.TX_VFO == vfo_num)
        {   // user entering a frequency
            const char * ascii = INPUTBOX_GetAscii();
            bool isGigaF = frequency>=_1GHz_in_KHz;
            sprintf(String, "%.*s.%.3s", 3 + isGigaF, ascii, ascii + 3 + isGigaF);
#ifdef ENABLE_BIG_FREQ
            if(!isGigaF) {
                // show the remaining 2 small frequency digits
                UI_PrintStringSmallNormal(String + 7, 113, 0, line + 1);
                String[7] = 0;
                // show the main large frequency digits
                UI_DisplayFrequency(String, 32, line, false);
            }
            else
#endif
            {
                // show the frequency in the main font
                UI_PrintString(String, 32, 0, line, 8);
            }

            continue;
        }
        else
        {
            if (gCurrentFunction == FUNCTION_TRANSMIT)
            {   // transmitting
                if (activeTxVFO == vfo_num)
                    frequency = gEeprom.VfoInfo[vfo_num].pTX->Frequency;
            }

if (IS_MR_CHANNEL(gEeprom.ScreenChannel[vfo_num]))
            {
                const ChannelAttributes_t* att = MR_GetChannelAttributes(gEeprom.ScreenChannel[vfo_num]);

                if(att->exclude == false)
                {
                    uint8_t countList = att->scanlist;
                    if(countList > MR_CHANNELS_LIST + 1) countList = 0;

                    char sl_str[5] = {0};
                    uint8_t sl_x;

                    if (countList == MR_CHANNELS_LIST + 1) {
                        strcpy(sl_str, "MON"); sl_x = 113;
                    } else if (countList == 0) {
                        strcpy(sl_str, "OFF"); sl_x = 113;
                    } else {
                        const char *name = gListName[countList - 1];
                        if (!IsEmptyName(name, sizeof(gListName[0]))) {
                            snprintf(sl_str, sizeof(sl_str), "%.3s", name);
                            sl_x = 113;
                        } else {
                            snprintf(sl_str, sizeof(sl_str), "%02d", countList);
                            sl_x = 117;
                        }
                    }
                    uint8_t sl_y = (uint8_t)((p_line0 - &gFrameBuffer[0][0]) / 128) * 8 + 8;
                    GUI_DisplaySmallestDark(sl_str, sl_x, sl_y, false, false);
                }
                else
                {
                    uint8_t sl_y = (uint8_t)((p_line0 - &gFrameBuffer[0][0]) / 128) * 8 + 8;
                    GUI_DisplaySmallestDark("E", 122, sl_y, false, false);
                }

#ifndef ENABLE_BIG_FREQ
                if (att.compander)
                    memcpy(p_line0 + 120 + LCD_WIDTH, BITMAP_compand, sizeof(BITMAP_compand));
#else
                  
#endif

                switch (gEeprom.CHANNEL_DISPLAY_MODE)
                {
                    case MDF_FREQUENCY:   
                        sprintf(String, "%3u.%05u", frequency / 100000, frequency % 100000);
#ifdef ENABLE_BIG_FREQ
                        if(frequency < _1GHz_in_KHz) {
                              
                            UI_PrintStringSmallNormal(String + 7, 113, 0, line + 1);
                            String[7] = 0;
                              
                            UI_DisplayFrequency(String, 32, line, false);
                        }
                        else
#endif
                        {
                              
                            UI_PrintString(String, 32, 0, line, 8);
                        }

                        break;

                    case MDF_CHANNEL:     
                        sprintf(String, "CH-%03u", gEeprom.ScreenChannel[vfo_num] + 1);
                        UI_PrintString(String, 32, 0, line, 8);
                        break;

                    case MDF_NAME:        
                    case MDF_NAME_FREQ:   
                        // --- 1. ИМЯ КАНАЛА (Верхняя строка VFO) ---
                        SETTINGS_FetchChannelName(String, gEeprom.ScreenChannel[vfo_num]);
                        if (String[0] == 0) {
                            sprintf(String, "CH-%03u", gEeprom.ScreenChannel[vfo_num] + 1);
                        }

                        if (gEeprom.CHANNEL_DISPLAY_MODE == MDF_NAME) {
                            UI_PrintString(String, 32, 0, line, 8);
                        }
                        else {
#ifdef ENABLE_FEAT_F4HWN
                            if (isMainOnly()) {
                                UI_PrintString(String, 32, 0, line, 8); 
                            }
                            else {
#endif
                                // Имя: Жирное для активного, обычное для неактивного
                                if(activeTxVFO == vfo_num) 
                                    UI_PrintStringSmallBold(String, 32 + 4, 0, line);
                                else
                                    UI_PrintStringSmallNormal(String, 32 + 4, 0, line);
#ifdef ENABLE_FEAT_F4HWN
                            }
#endif

                            // --- 2. НОМЕР КАНАЛА (Нижняя строка, Y = line + 1, X = 2) ---
                            sprintf(String, "M%u", gEeprom.ScreenChannel[vfo_num] + 1);
                            if (activeTxVFO == vfo_num) {
                                UI_PrintStringSmallBold(String, 2, 0, line + 1);   // ЖИРНЫЙ номер
                            } else {
                                UI_PrintStringSmallNormal(String, 2, 0, line + 1); // ТОНКИЙ номер
                            }

                            // --- 3. ЧАСТОТА (Нижняя строка, Y = line + 1, X = 36) ---
                            sprintf(String, "%03u.%05u", frequency / 100000, frequency % 100000);
#ifdef ENABLE_FEAT_F4HWN
                            if (isMainOnly()) {
                                if(frequency < _1GHz_in_KHz) {
                                    UI_PrintStringSmallNormal(String + 7, 113, 0, line + 4);
                                    String[7] = 0;
                                    UI_DisplayFrequency(String, 32, line + 3, false);
                                } else {
                                    UI_PrintString(String, 32, 0, line + 3, 8);
                                }
                            }
                            else {
#endif
                                // Частота: Жирная для активного, обычная для неактивного
                                if(activeTxVFO == vfo_num)
                                    UI_PrintStringSmallBold(String, 32 + 4, 0, line + 1);
                                else
                                    UI_PrintStringSmallNormal(String, 32 + 4, 0, line + 1);
#ifdef ENABLE_FEAT_F4HWN
                            }
#endif
                        }
                        break;
                }
            }
            else
            {     
                sprintf(String, "%3u.%05u", frequency / 100000, frequency % 100000);

#ifdef ENABLE_BIG_FREQ
                if(frequency < _1GHz_in_KHz) {
                    // show the remaining 2 small frequency digits
                    UI_PrintStringSmallNormal(String + 7, 113, 0, line + 1);
                    String[7] = 0;
                    // show the main large frequency digits
                    UI_DisplayFrequency(String, 32, line, false);
                }
                else
#endif
                {
                    // show the frequency in the main font
                    UI_PrintString(String, 32, 0, line, 8);
                }

                // show the channel symbols
                const ChannelAttributes_t* att_p = MR_GetChannelAttributes(gEeprom.ScreenChannel[vfo_num]);
                if (att_p && att_p->compander)
#ifdef ENABLE_BIG_FREQ
                    memcpy(p_line0 + 120, BITMAP_compand, sizeof(BITMAP_compand));
#else
                    memcpy(p_line0 + 120 + LCD_WIDTH, BITMAP_compand, sizeof(BITMAP_compand));
#endif
            }
        }
        String[0] = '\0';
        const VFO_Info_t *vfoInfo = &gEeprom.VfoInfo[vfo_num];

        // show the modulation symbol
        const char * s = "";
#ifdef ENABLE_FEAT_F4HWN
        const char * t = "";
#endif
        const ModulationMode_t mod = vfoInfo->Modulation;
        switch (mod){
            case MODULATION_FM: {
                const FREQ_Config_t *pConfig = (mode == VFO_MODE_TX) ? vfoInfo->pTX : vfoInfo->pRX;
                const unsigned int code_type = pConfig->CodeType;
#ifdef ENABLE_FEAT_F4HWN
                const char *code_list[] = {"", "CT", "DC", "DC"};
#else
                const char *code_list[] = {"", "CT", "DCS", "DCR"};
#endif
                if (code_type < ARRAY_SIZE(code_list))
                    s = code_list[code_type];
#ifdef ENABLE_FEAT_F4HWN
                t = gModulationStr[mod]; // модуляция всегда видна, в т.ч. при TX
#endif
                break;
            }
            default:
                t = gModulationStr[mod];
            break;
        }

#if ENABLE_FEAT_F4HWN
        const FREQ_Config_t *pConfig = (mode == VFO_MODE_TX) ? vfoInfo->pTX : vfoInfo->pRX;
        int8_t shift = 0;

        switch((int)pConfig->CodeType)
        {
            case 1:
            sprintf(String, "%u.%u", CTCSS_Options[pConfig->Code] / 10, CTCSS_Options[pConfig->Code] % 10);
            break;

            case 2:
            sprintf(String, "%03oN", DCS_Options[pConfig->Code]);
            break;

            case 3:
            sprintf(String, "%03oI", DCS_Options[pConfig->Code]);
            break;

            default:
            sprintf(String, "%d.%02uK", vfoInfo->StepFrequency / 100, vfoInfo->StepFrequency % 100);
            shift = -10;
        }

        if (gSetting_set_gui)
        {
            UI_PrintStringSmallNormal(s, LCD_WIDTH + 28, 0, line + 1);
            UI_PrintStringSmallBold(t, LCD_WIDTH + 100, 0, line + 1); 

            if (isMainOnly() )
            {
                if(shift == 0)
                {
                    UI_PrintStringSmallBold(String, 2, 0, 6);
                }

                if((vfoInfo->StepFrequency / 100) < 100)
                {
                    sprintf(String, "%d.%02uK", vfoInfo->StepFrequency / 100, vfoInfo->StepFrequency % 100);
                }
                else
                {
                    sprintf(String, "%dK", vfoInfo->StepFrequency / 100);               
                }
                UI_PrintStringSmallBold(String, 46, 0, 6);
            }
        }
        else
        {
            // MODERN dual-screen: фиксированные координаты X для строки параметров
            // Порядок: [SQL x=23] [ПОЛОСА x=45] [ШАГ/КОД x=68] [МОЩНОСТЬ x=97] [s x=110] [МОДУЛЯЦИЯ/+- x=119]
            const uint8_t y_line = line == 0 ? 17 : 49; // Y: VFO0=17, VFO1=49

            // [ШАГ/CTCSS/DCS] — фиксированный x=68 (убрали динамический shift)
            GUI_DisplaySmallest(String, 68, y_line, false, true); // [ШАГ/КОД] x=68

            // [КОД s: CT/DC] — фиксированный x=110
            // [CT/DC] — удалён, значение субтона на x=68 и так понятно

            // [МОДУЛЯЦИЯ t: FM/AM/USB] — фиксированный x=119
            if ((t != NULL) && (t[0] != '\0')) {
                GUI_DisplaySmallest(t, 117, y_line, false, true); // [МОДУЛЯЦИЯ t] x=119
            }

            //sprintf(String, "%d.%02u", vfoInfo->StepFrequency / 100, vfoInfo->StepFrequency % 100);
            //GUI_DisplaySmallest(String, 91, line == 0 ? 2 : 34, false, true);
        }
#else
        UI_PrintStringSmallNormal(s, LCD_WIDTH + 24, 0, line + 1);
#endif

        if (state == VFO_STATE_NORMAL || state == VFO_STATE_ALARM)
        {   // show the TX power
            uint8_t currentPower = (vfoInfo->OUTPUT_POWER < OUTPUT_POWER_LEN) ? vfoInfo->OUTPUT_POWER : OUTPUT_POWER_LOW;

            // Build display string: power letter
            char pwr_gui[4] = {0};
            char pwr_text[6] = {0};
            
            const char *short_base[] = {"L","M","H"};
            const char *long_base[]  = {"LOW","MID","HIGH"};
            sprintf(pwr_gui,  "%s", short_base[currentPower]);
            sprintf(pwr_text, "%s", long_base[currentPower]);
            

            if (gSetting_set_gui)
            {
                // При TX-запрете (X) сдвигаем влево: меняй цифру 5 для подгонки
                uint8_t pwr_x_bold = LCD_WIDTH + 80;
                UI_PrintStringSmallBold(pwr_gui, pwr_x_bold, 0, line + 1);
            }
            else
            {
                // При TX-запрете (X) сдвигаем влево: меняй цифру 3 для подгонки
                uint8_t pwr_x_small = 97;
                GUI_DisplaySmallest(pwr_text, pwr_x_small, line == 0 ? 17 : 49, false, true); // [МОЩНОСТЬ] x=97
            }

        }

        if (vfoInfo->freq_config_RX.Frequency != vfoInfo->freq_config_TX.Frequency)
        {   // show the TX offset symbol
            int i = vfoInfo->TX_OFFSET_FREQUENCY_DIRECTION % 3;
            const char dir_list[][2] = {"", "+", "-"};
           
#if ENABLE_FEAT_F4HWN
        if (gSetting_set_gui)
        {
            UI_PrintStringSmallNormal(dir_list[i], LCD_WIDTH + 60, 0, line + 1);
        }
        else
        {
            GUI_DisplaySmallest(dir_list[i], 40, line == 0 ? 17 : 49, false, true); // [СМЕЩЕНИЕ +/-] x=128 (независимый, меняй только здесь)
        }
#else
            UI_PrintStringSmallNormal(dir_list[i], LCD_WIDTH + 54, 0, line + 1);
#endif
        }

        // show the TX/RX reverse symbol
        // [R реверс] — удалён, кроссбенд не используется
        // if (vfoInfo->FrequencyReverse) ...

#if ENABLE_FEAT_F4HWN
        #ifdef ENABLE_FEAT_F4HWN_NARROWER
            bool narrower = 0;

            if(vfoInfo->CHANNEL_BANDWIDTH == BANDWIDTH_NARROW && gSetting_set_nfm == 1)
            {
                narrower = 1;
            }

            if (gSetting_set_gui)
            {
                const char *bandWidthNames[] = {"W", "N", "N+"};
                UI_PrintStringSmallBold(bandWidthNames[vfoInfo->CHANNEL_BANDWIDTH + narrower], LCD_WIDTH + 60, 0, line + 1);
            }
            else
            {
                const char *bandWidthNames[] = {"WIDE", "NAR", "NAR+"};
                GUI_DisplaySmallest(bandWidthNames[vfoInfo->CHANNEL_BANDWIDTH + narrower], 45, line == 0 ? 17 : 49, false, true); // [ПОЛОСА] x=45
            }
        #else
            if (gSetting_set_gui)
            {
                const char *bandWidthNames[] = {"W", "N"};
                UI_PrintStringSmallBold
            else
            {
                const char *bandWidthNames[] = {"WIDE", "NAR"};
                GUI_DisplaySmallest(bandWidthNames[vfoInfo->CHANNEL_BANDWIDTH], 45, line == 0 ? 17 : 49, false, true); // [ПОЛОСА] x=45
            }
        #endif
#else
        if (vfoInfo->CHANNEL_BANDWIDTH == BANDWIDTH_NARROW)
            UI_PrintStringSmallBold("N", LCD_WIDTH + 70, 0, line + 1);
#endif

        // [DTMF] — удалён, не используется

#ifdef ENABLE_SCRAMBLER
        // [SCR скремблер] — фиксированные координаты в строке параметров
        if (vfoInfo->SCRAMBLING_TYPE > 0 && gSetting_ScrambleEnable)
            GUI_DisplaySmallest("SCR", 56, line == 0 ? 17 : 49, false, true); // [SCR] x=56 (меняй только здесь)
#endif

#ifdef ENABLE_FEAT_F4HWN
        /*
        if(isMainVFO)   
        {
            if(gMonitor)
            {
                sprintf(String, "%s", "MONI");
            }
            
            if (gSetting_set_gui)
            {
                if(!gMonitor)
                {
                    sprintf(String, "SQL%d", gEeprom.SQUELCH_LEVEL);
                }
                UI_PrintStringSmallNormal(String, LCD_WIDTH + 98, 0, line + 1);
            }
            else
            {
                if(!gMonitor)
                {
                    sprintf(String, "SQL%d", gEeprom.SQUELCH_LEVEL);
                }
                GUI_DisplaySmallest(String, 110, line == 0 ? 17 : 49, false, true);
            }
        }
        */
        if (isMainVFO) {
           if (gMonitor) {
                strcpy(String, "MONI");
           } else {
                sprintf(String, "SQ%d", gEeprom.SQUELCH_LEVEL);
           }

           if (gSetting_set_gui) {
                UI_PrintStringSmallBold(String, LCD_WIDTH + 27, 0, line + 1);
           } else {
                GUI_DisplaySmallest(String, 26, line == 0 ? 17 : 49, false, true);  // [SQL/MONI] x=23
           }
        }
#endif
    }

#ifdef ENABLE_AGC_SHOW_DATA
    center_line = CENTER_LINE_IN_USE;
    UI_MAIN_PrintAGC(false);
#endif

    if (center_line == CENTER_LINE_NONE)
    {   // we're free to use the middle line

        const bool rx = FUNCTION_IsRx();

#ifdef ENABLE_AUDIO_BAR
        if (gSetting_mic_bar && gCurrentFunction == FUNCTION_TRANSMIT) {
            center_line = CENTER_LINE_AUDIO_BAR;
            UI_DisplayAudioBar();
        }
        else
#endif

#ifdef ENABLE_RSSI_BAR
        if (rx) {
            center_line = CENTER_LINE_RSSI;
            DisplayRSSIBar(false);
        }
        else
#endif
        if (rx || gCurrentFunction == FUNCTION_FOREGROUND || gCurrentFunction == FUNCTION_POWER_SAVE)
        {

        }
    }

#ifdef ENABLE_FEAT_F4HWN
    // TX / RX индикатор поверх всего, чтобы аудиобар его не затирал
    if (gCurrentFunction == FUNCTION_TRANSMIT)
    {
        uint8_t y_tx = (isMainOnly() || activeTxVFO == 0) ? 1 : 33;
        GUI_DisplaySmallest("TX", 10, y_tx, false, true);
    }
    else if (FUNCTION_IsRx())
    {
        uint8_t y_rx = (isMainOnly() || gEeprom.RX_VFO == 0) ? 1 : 33;
        GUI_DisplaySmallest("RX", 10, y_rx, false, true);
    }
#endif

    ST7565_BlitFullScreen();
}

// ***************************************************************************