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

    //UI_PrintStringSmallNormal("K1 DEV TEAM", 0, 127, 0);
    UI_PrintString("SONIC", 0, 127, 0,12);
    UI_PrintString("t.me/SonicFw", 0, 127, 2,9);

    ST7565_BlitStatusLine();
    UI_PrintString(Edition, 0, 64, 5, 10);
    UI_PrintString(VERSION_STRING_2, 64, 127, 5, 10);

    for (uint8_t i = 0; i <= 127; i += 2) {
        UI_DrawLineBuffer(gFrameBuffer, i, 37, i, 37, 1);
    }
    for (uint8_t y = 37; y <= 57; y++) {
        UI_DrawLineBuffer(gFrameBuffer, 64, y, 64, y, 1);
    }
    
    ST7565_BlitFullScreen();
}
