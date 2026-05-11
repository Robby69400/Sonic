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
#include "driver/py25q16.h"
#include "driver/st7565.h"
#include "external/printf/printf.h"
#include "helper/battery.h"
#include "settings.h"
#include "misc.h"
#include "ui/helper.h"
#include "ui/welcome.h"
#include "ui/status.h"
#include "version.h"
#include "bitmaps.h"

// Boot logo storage in PY25Q16 external flash, aligned on a 4 KB sector,
// placed past the calibration zone (0x010000-0x010200).
//
// Layout inside the sector starting at LOGO_FLASH_ADDR:
//   [0x00..0x07] : 8-byte header (reserved for future magic/version/flags)
//   [0x08..0x407]: 128x64 monochrome bitmap (1024 B)
//                  ST7565-native: 8 pages * 128 columns, column-major LSB-top
#define LOGO_FLASH_ADDR     0x011000
#define LOGO_HEADER_SIZE    8
#define LOGO_BITMAP_ADDR    (LOGO_FLASH_ADDR + LOGO_HEADER_SIZE)

void UI_DisplayReleaseKeys(void)
{
    memset(gStatusLine,  0, sizeof(gStatusLine));
#if defined(ENABLE_FEAT_F4HWN_CTR) || defined(ENABLE_FEAT_F4HWN_INV)
        ST7565_ContrastAndInv();
#endif
    UI_DisplayClear();

    UI_PrintString("RELEASE", 0, 127, 1, 10);
    UI_PrintString("ALL KEYS", 0, 127, 3, 10);

    ST7565_BlitStatusLine();  // blank status line
    ST7565_BlitFullScreen();
}

void UI_DisplayWelcome(void)
{
    char WelcomeString[16];
#if defined(ENABLE_FEAT_F4HWN_CTR) || defined(ENABLE_FEAT_F4HWN_INV)
    ST7565_ContrastAndInv();
#endif
    UI_DisplayClear();

    if (gEeprom.POWER_ON_DISPLAY_MODE == POWER_ON_DISPLAY_MODE_LOGO) {
        // Skip 8-byte header, then read 128x64 bitmap (1024 B):
        // page 0 -> gStatusLine, pages 1..7 -> gFrameBuffer.
        PY25Q16_ReadBuffer(LOGO_BITMAP_ADDR, gStatusLine, sizeof(gStatusLine));
        PY25Q16_ReadBuffer(LOGO_BITMAP_ADDR + sizeof(gStatusLine), gFrameBuffer, sizeof(gFrameBuffer));
        ST7565_BlitStatusLine();
        ST7565_BlitFullScreen();
        return;
    }

    // NONE / SOUND → пустой экран
    if (gEeprom.POWER_ON_DISPLAY_MODE == POWER_ON_DISPLAY_MODE_NONE)
    {
        ST7565_FillScreen(0x00);
        return;
    }

    UI_PrintString("SONIC", 0, 127, 0,10);
    UI_PrintStringSmallbackground("t.me/SonicFw", 0, 127, 4,1);
    sprintf(WelcomeString, "%u.%02uV %u%%",
                gBatteryVoltageAverage / 100,
                gBatteryVoltageAverage % 100,
                BATTERY_VoltsToPercent(gBatteryVoltageAverage));
    UI_PrintString(WelcomeString, 0, 127, 2,10);

    ST7565_BlitStatusLine();
    UI_PrintString(Edition, 0, 64, 5, 10);
    UI_PrintString(VERSION_STRING_2, 64, 127, 5, 10);

    for (uint8_t i = 0; i <= 127; i += 2) {
        UI_DrawLineBuffer(gFrameBuffer, i, 40, i, 40, 1);
    }
    for (uint8_t y = 40; y <= 57; y++) {
        UI_DrawLineBuffer(gFrameBuffer, 64, y, 64, y, 1);
    }
    
    ST7565_BlitFullScreen();
}
