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
    char WelcomeString0[16];
    char WelcomeString1[16];

#if defined(ENABLE_FEAT_F4HWN_CTR) || defined(ENABLE_FEAT_F4HWN_INV)
    ST7565_ContrastAndInv();
#endif
    UI_DisplayClear();

    // NONE / SOUND → пустой экран
    if (gEeprom.POWER_ON_DISPLAY_MODE == POWER_ON_DISPLAY_MODE_NONE ||
        gEeprom.POWER_ON_DISPLAY_MODE == POWER_ON_DISPLAY_MODE_SOUND)
    {
        ST7565_FillScreen(0x00);
        return;
    }

    // ALL — аллигатор + вольтаж
    memset(WelcomeString0, 0, sizeof(WelcomeString0));

    sprintf(WelcomeString1, "%u.%02uV %u%%",
            gBatteryVoltageAverage / 100,
            gBatteryVoltageAverage % 100,
            BATTERY_VoltsToPercent(gBatteryVoltageAverage));

    PY25Q16_ReadBuffer(0x00A0C8, WelcomeString0, 15);
    WelcomeString0[15] = '\0';
    if ((uint8_t)WelcomeString0[0] >= 0x80 || WelcomeString0[0] == (char)0xFF ||
        WelcomeString0[0] == '\0')
        strcpy(WelcomeString0, "SONIC");

    UI_PrintString(WelcomeString0, 0, 127, 0, 10);
    UI_PrintString(WelcomeString1, 0, 127, 2, 10);

    ST7565_BlitStatusLine();
    UI_PrintString(Edition, 0, 127, 4, 10);

    // Горизонтальные декоративные линии
    for (uint8_t i = 0; i <= 127; i += 2) {
        UI_DrawLineBuffer(gFrameBuffer, i, 15, i, 15, 1);
    }
    for (uint8_t i = 0; i <= 127; i += 2) {
        UI_DrawLineBuffer(gFrameBuffer, i, 30, i, 30, 1);
    }
    // Вертикальные линии по бокам ниже КА-52
    for (uint8_t y = 47; y <= 57; y += 2) {
        UI_DrawLineBuffer(gFrameBuffer, 30, y, 30, y, 1);
    }
    for (uint8_t y = 47; y <= 57; y += 2) {
        UI_DrawLineBuffer(gFrameBuffer, 94, y, 94, y, 1);
    }

    GUI_DisplaySmallest("OURO",            5,  49, false, true);
    GUI_DisplaySmallest("MODE",          108,  49, false, true);
    GUI_DisplaySmallestDark(VERSION_STRING_2, 45, 49, false, false);

    ST7565_BlitFullScreen();
}
