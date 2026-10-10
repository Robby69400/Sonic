/* Copyright 2023 Dual Tachyon
 * https://github.com/DualTachyon
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#ifdef ENABLE_FMRADIO
    #include "app/fm.h"
#endif
#include "bitmaps.h"
#include "driver/keyboard.h"
#include "driver/st7565.h"
#include "external/printf/printf.h"
#include "functions.h"
#include "helper/battery.h"
#include "misc.h"
#include "settings.h"
#include "ui/battery.h"
#include "ui/helper.h"
#include "ui/ui.h"
#include "ui/status.h"

#ifdef ENABLE_FEAT_F4HWN
// S-level, updated in ui/main.c during reception
extern int8_t gSmeterLevel;
#endif

#ifdef ENABLE_FEAT_F4HWN_RX_TX_TIMER
#ifndef ENABLE_FEAT_F4HWN_DEBUG
static void convertTime(uint8_t *line, uint8_t type) 
{
    uint16_t t = (type == 0) ? (gTxTimerCountdown_500ms / 2) : (3600 - gRxTimerCountdown_500ms / 2);
    uint8_t m = t / 60;
    uint8_t s = t - (m * 60);

    char str[6];
    sprintf(str, "%02u:%02u", m, s);
    UI_PrintStringSmallBufferBold(str, line);

    gUpdateStatus = true;
}
#endif
#endif

void UI_DisplayStatus()
{
    char str[12] = "";
    gUpdateStatus = false;
    memset(gStatusLine, 0, sizeof(gStatusLine));
    uint8_t POS_MOD  = 1;   // DW, XB, MO (glyphs)
    const uint8_t POS_LOCK = 83;   // Lock (glyph)
    const uint8_t POS_F    = 83;   // Letter F (glyph)

#ifdef ENABLE_FEAT_F4HWN_RX_TX_TIMER
    if (gSetting_set_tmr) {      
        if (gCurrentFunction == FUNCTION_TRANSMIT){
            POS_MOD += 2;
            convertTime(gStatusLine + POS_MOD, 0);
            POS_MOD += 35;
        }
        else if (FUNCTION_IsRx()) {
            POS_MOD += 2;
            convertTime(gStatusLine + POS_MOD, 1);
            POS_MOD += 35;
        }
    }
#endif

#ifdef ENABLE_FEAT_F4HWN
    if (FUNCTION_IsRx()) {
        char smeter_str[4];
        int8_t sl = gSmeterLevel;
        if (sl <= 9)
            sprintf(smeter_str, "S%d", (int)sl);
        else
            sprintf(smeter_str, "S+");
        POS_MOD += 3;
        UI_PrintStringSmallBufferBold(smeter_str, gStatusLine + POS_MOD);
        POS_MOD += 14;
    }
#endif

    // 6. FLASHLIGHT (FlashlightOnRX) - blinking enabled during RX
    if (gEeprom.FlashlightOnRX) {
        POS_MOD += 3;
        gStatusLine[POS_MOD++] |= 0x70;
        gStatusLine[POS_MOD++] |= 0x7E;
        gStatusLine[POS_MOD++] |= 0x61;
        gStatusLine[POS_MOD++] |= 0x61;
        gStatusLine[POS_MOD++] |= 0x61;
        gStatusLine[POS_MOD++] |= 0x7E;
        gStatusLine[POS_MOD++] |= 0x70;
    }

    // 7. BACKLIGHT (B) - VERTICAL, shifted right when the flashlight is also enabled
    if (gBackLight) {
        POS_MOD += 3;
        gStatusLine[POS_MOD++] |= 0x0C;
        gStatusLine[POS_MOD++] |= 0x12;
        gStatusLine[POS_MOD++] |= 0x65;
        gStatusLine[POS_MOD++] |= 0x79;
        gStatusLine[POS_MOD++] |= 0x65;
        gStatusLine[POS_MOD++] |= 0x12;
        gStatusLine[POS_MOD++] |= 0x0C;
    }


    // 7. F-KEY AND LOCK - SEPARATE
    if (gWasFKeyPressed) {
        gStatusLine[POS_F + 0] = 0x7F;
        gStatusLine[POS_F + 1] = 0x41;
        gStatusLine[POS_F + 2] = 0x75;
        gStatusLine[POS_F + 3] = 0x75;
        gStatusLine[POS_F + 4] = 0x75;
        gStatusLine[POS_F + 5] = 0x7D;
        gStatusLine[POS_F + 6] = 0x7F;
    }
    if (gEeprom.KEY_LOCK) {
        gStatusLine[POS_LOCK + 0] = 0x7C;
        gStatusLine[POS_LOCK + 1] = 0x7A;
        gStatusLine[POS_LOCK + 2] = 0x79;
        gStatusLine[POS_LOCK + 3] = 0x49;
        gStatusLine[POS_LOCK + 4] = 0x79;
        gStatusLine[POS_LOCK + 5] = 0x7A;
        gStatusLine[POS_LOCK + 6] = 0x7C;
    }
    if (gFmRadioMode) {
        sprintf(str, "FM");
        UI_PrintStringSmallBufferBold(str, gStatusLine);
    }

    if (gSetting_battery_text == 0);
    else if (gSetting_battery_text == 1) {
        sprintf(str, "%u.%02uV", gBatteryVoltageAverage / 100, gBatteryVoltageAverage % 100);
    } else if (gSetting_battery_text == 2) {
        sprintf(str, "%u%%", BATTERY_VoltsToPercent(gBatteryVoltageAverage));
    }
    
    uint8_t battPos = 125 - (strlen(str) * 7);
    UI_PrintStringSmallBufferBold(str, gStatusLine + battPos);

    ST7565_BlitStatusLine();
}