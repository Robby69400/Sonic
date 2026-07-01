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
#include "driver/bk4819.h"
#include "driver/keyboard.h"
#include "driver/gpio.h"
#include "driver/py25q16.h"
#include "driver/st7565.h"
#include "driver/system.h"
#include "external/printf/printf.h"
#include "helper/boot.h"
#include "misc.h"
#include "radio.h"
#include "settings.h"
#include "ui/helper.h"
#include "ui/menu.h"
#include "ui/ui.h"

BOOT_Mode_t BOOT_GetMode(void)
{
    KEY_Code_t key;
    SYSTEM_DelayMs(20);
    key = KEYBOARD_Poll();
    SYSTEM_DelayMs(20);
    if (key != KEYBOARD_Poll())
        return BOOT_MODE_NORMAL;

    bool ptt = GPIO_IsPttPressed();

    if (ptt && key == KEY_SIDE2)
        return BOOT_MODE_ERASE_NO_CALIB;

    return BOOT_MODE_NORMAL;
}

static void DoErase()
{
    KEY_Code_t hold_key = KEY_SIDE2;
    BACKLIGHT_TurnOn();
    UI_DisplayClear();
    UI_PrintString("ERASE FLASH", 0, 127, 0, 10);
    UI_PrintString("KEEP CALIB",  0, 127, 2, 10);
    UI_PrintString("HOLD SIDE2",  0, 127, 4, 10);
    ST7565_BlitFullScreen();
    for (int t = 0; t < 300; t++) {
        SYSTEM_DelayMs(10);
        if (KEYBOARD_Poll() != hold_key || !GPIO_IsPttPressed()) {
            UI_DisplayClear();
            UI_PrintString("CANCELLED", 0, 127, 2, 10);
            ST7565_BlitFullScreen();
            SYSTEM_DelayMs(1500);
            return;
        }
    }

    UI_DisplayClear();
    UI_PrintString("ERASING...", 0, 127, 0, 10);
    ST7565_BlitFullScreen();
    PY25Q16_ClearBlockProtect();
    SYSTEM_DelayMs(10);
    SETTINGS_FactoryReset(1);
    UI_DisplayClear();
    UI_PrintString("DONE!", 0, 127, 1, 10);
    UI_PrintString("POWER OFF", 0, 127, 3, 10);
    ST7565_BlitFullScreen();

    while (GPIO_IsPttPressed() || KEYBOARD_Poll() != KEY_INVALID)
        SYSTEM_DelayMs(10);
    for (;;) SYSTEM_DelayMs(100);
}

void BOOT_ProcessMode(BOOT_Mode_t Mode)
{
    if (Mode == BOOT_MODE_ERASE_NO_CALIB) {
        // PTT+FN2: ALL reset — same as menu ALL, keeps calibration
        UI_DisplayClear();
        BACKLIGHT_TurnOn();
        UI_PrintString("RESET ALL",   0, 127, 0, 10);
        UI_PrintString("KEEP CALIB",  0, 127, 2, 10);
        UI_PrintString("HOLD FN2",    0, 127, 4, 10);
        ST7565_BlitFullScreen();
        // Hold 3 seconds to confirm
        for (int t = 0; t < 300; t++) {
            SYSTEM_DelayMs(10);
            if (KEYBOARD_Poll() != KEY_SIDE2 || !GPIO_IsPttPressed()) {
                UI_DisplayClear();
                UI_PrintString("CANCELLED", 0, 127, 2, 10);
                ST7565_BlitFullScreen();
                SYSTEM_DelayMs(1500);
                GUI_SelectNextDisplay(DISPLAY_MAIN);
                return;
            }
        }
        UI_DisplayClear();
        UI_PrintString("ERASING...", 0, 127, 1, 10);
        ST7565_BlitFullScreen();
        SETTINGS_FactoryReset(1); // ALL: erase everything except calibration
        UI_DisplayClear();
        UI_PrintString("DONE!",     0, 127, 1, 10);
        UI_PrintString("POWER OFF", 0, 127, 3, 10);
        ST7565_BlitFullScreen();
        while (GPIO_IsPttPressed() || KEYBOARD_Poll() != KEY_INVALID)
            SYSTEM_DelayMs(10);
        for (;;) SYSTEM_DelayMs(100);
    }

    if (Mode == BOOT_MODE_ERASE) {
        DoErase();
        GUI_SelectNextDisplay(DISPLAY_MAIN);
        return;
    }
    GUI_SelectNextDisplay(DISPLAY_MAIN);
}
