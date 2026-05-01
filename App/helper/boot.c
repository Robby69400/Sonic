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

#ifdef ENABLE_AIRCOPY
    #include "app/aircopy.h"
#endif
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

    // Читаем кнопку дважды для стабильности
    SYSTEM_DelayMs(20);
    key = KEYBOARD_Poll();
    SYSTEM_DelayMs(20);
    if (key != KEYBOARD_Poll())
        return BOOT_MODE_NORMAL;

    bool ptt = GPIO_IsPttPressed();

    // PTT + FN2 (SIDE2) = ALL reset: всё кроме калибровок (как в меню ALL)
    if (ptt && key == KEY_SIDE2)
        return BOOT_MODE_ERASE_NO_CALIB;

    // PTT + FN1 (SIDE1) = не используется

    #ifdef ENABLE_FEAT_F4HWN_RESCUE_OPS
    if (ptt && key == (KEY_Code_t)(10 + gEeprom.SET_KEY))
        return BOOT_MODE_RESCUE_OPS;
    #endif

    #ifdef ENABLE_AIRCOPY
    if (!ptt && key == KEY_SIDE2) {
        gAirCopyBootMode = true;
        return BOOT_MODE_AIRCOPY;
    }
    #endif

    return BOOT_MODE_NORMAL;
}

// Стирание с прогресс-баром
// with_calib=false → PTT+SIDE1: всё кроме калибровок
// with_calib=true  → PTT+SIDE2: всё включая калибровки
static void DoErase(bool with_calib)
{
    KEY_Code_t hold_key = with_calib ? KEY_SIDE2 : KEY_SIDE1;

    // Экран предупреждения
    UI_DisplayClear();
    if (with_calib) {
        UI_PrintString("FULL ERASE",  0, 127, 0, 10);
        UI_PrintString("+CALIB DATA", 0, 127, 2, 10);
        UI_PrintString("HOLD SIDE2",  0, 127, 4, 10);
    } else {
        UI_PrintString("ERASE FLASH", 0, 127, 0, 10);
        UI_PrintString("KEEP CALIB",  0, 127, 2, 10);
        UI_PrintString("HOLD SIDE1",  0, 127, 4, 10);
    }
    ST7565_BlitFullScreen();

    // Удержание 3 секунды — защита от случайного нажатия
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

    // Подтверждено — стираем
    UI_DisplayClear();
    UI_PrintString("ERASING...", 0, 127, 0, 10);
    ST7565_BlitFullScreen();

    // Сначала снимаем защиту Block Protect
    // (закрытые прошивки могут её ставить)
    PY25Q16_ClearBlockProtect();
    // Небольшая пауза после снятия защиты
    SYSTEM_DelayMs(10);

    // Базовые зоны (всегда)
    static const uint32_t zones_base[][2] = {
        {0x000000, 0x009000},  // каналы: данные + имена + атрибуты
        {0x00A000, 0x00A000},  // настройки F4HWN + welcome string
        {0x00B000, 0x00B000},  // дополнительные настройки
        {0x00C000, 0x00C000},  // spectrum EEPROM
    };
    const uint8_t base_count = 4;

    // Зоны калибровок (только с PTT+SIDE2)
    static const uint32_t zones_calib[][2] = {
        {0x010000, 0x011000},  // squelch + RSSI + TX мощность + батарея + VOX
    };
    const uint8_t calib_count = 1;

    // Считаем общее число секторов для прогресс-бара
    uint16_t total = 0;
    for (uint8_t z = 0; z < base_count; z++)
        total += (uint16_t)((zones_base[z][1] - zones_base[z][0]) / 0x1000 + 1);
    if (with_calib)
        for (uint8_t z = 0; z < calib_count; z++)
            total += (uint16_t)((zones_calib[z][1] - zones_calib[z][0]) / 0x1000 + 1);

    uint16_t done = 0;

    // Стираем базовые зоны
    for (uint8_t z = 0; z < base_count; z++) {
        for (uint32_t addr = zones_base[z][0]; addr <= zones_base[z][1]; addr += 0x1000) {
            PY25Q16_SectorErase(addr);
            done++;
            uint8_t bar_w = (uint8_t)((uint32_t)done * 120 / total);
            memset(gFrameBuffer[6], 0, 128);
            for (uint8_t x = 4; x < 4 + bar_w; x++)
                gFrameBuffer[6][x] = 0x3C;
            ST7565_BlitLine(6);
        }
    }

    // Стираем калибровки если нужно
    if (with_calib) {
        for (uint8_t z = 0; z < calib_count; z++) {
            for (uint32_t addr = zones_calib[z][0]; addr <= zones_calib[z][1]; addr += 0x1000) {
                PY25Q16_SectorErase(addr);
                done++;
                uint8_t bar_w = (uint8_t)((uint32_t)done * 120 / total);
                memset(gFrameBuffer[6], 0, 128);
                for (uint8_t x = 4; x < 4 + bar_w; x++)
                    gFrameBuffer[6][x] = 0x3C;
                ST7565_BlitLine(6);
            }
        }
    }

    // Готово
    UI_DisplayClear();
    UI_PrintString("DONE!", 0, 127, 1, 10);
    UI_PrintString("POWER OFF", 0, 127, 3, 10);
    ST7565_BlitFullScreen();

    // Ждём отпускания кнопок, потом виснем — нужен power cycle
    while (GPIO_IsPttPressed() || KEYBOARD_Poll() != KEY_INVALID)
        SYSTEM_DelayMs(10);
    for (;;) SYSTEM_DelayMs(100);
}

void BOOT_ProcessMode(BOOT_Mode_t Mode)
{
    if (Mode == BOOT_MODE_ERASE_NO_CALIB) {
        // PTT+FN2: ALL reset — same as menu ALL, keeps calibration
        UI_DisplayClear();
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

    if (Mode == BOOT_MODE_ERASE_WITH_CALIB) {
        DoErase(true);
        GUI_SelectNextDisplay(DISPLAY_MAIN);
        return;
    }

    #ifdef ENABLE_FEAT_F4HWN_RESCUE_OPS
    if (Mode == BOOT_MODE_RESCUE_OPS) {
        gEeprom.MENU_LOCK = !gEeprom.MENU_LOCK;
        SETTINGS_SaveSettings();
    }
    #endif

    #ifdef ENABLE_AIRCOPY
    if (Mode == BOOT_MODE_AIRCOPY)
    {
        gEeprom.DUAL_WATCH               = DUAL_WATCH_OFF;
        gEeprom.BATTERY_SAVE             = 0;
        gEeprom.CROSS_BAND_RX_TX         = CROSS_BAND_OFF;
        gEeprom.AUTO_KEYPAD_LOCK         = false;
        gEeprom.KEY_1_SHORT_PRESS_ACTION = ACTION_OPT_NONE;
        gEeprom.KEY_1_LONG_PRESS_ACTION  = ACTION_OPT_NONE;
        gEeprom.KEY_2_SHORT_PRESS_ACTION = ACTION_OPT_NONE;
        gEeprom.KEY_2_LONG_PRESS_ACTION  = ACTION_OPT_NONE;
        gEeprom.KEY_M_LONG_PRESS_ACTION  = ACTION_OPT_NONE;

        RADIO_InitInfo(gRxVfo, FREQ_CHANNEL_LAST - 1, DEFAULT_FREQ);

        gRxVfo->CHANNEL_BANDWIDTH = BANDWIDTH_NARROW;
        gRxVfo->OUTPUT_POWER      = OUTPUT_POWER_LOW;

        RADIO_ConfigureSquelchAndOutputPower(gRxVfo);

        gCurrentVfo = gRxVfo;

        RADIO_SetupRegisters(true);
        BK4819_SetupAircopy();
        BK4819_ResetFSK();

        gAircopyState = AIRCOPY_READY;

        gEeprom.BACKLIGHT_TIME = 61;
        gEeprom.KEY_LOCK       = 0;

        #ifdef ENABLE_FEAT_F4HWN_RESUME_STATE
            gEeprom.CURRENT_STATE = 0;
        #endif

        GUI_SelectNextDisplay(DISPLAY_AIRCOPY);
        return;
    }
    #endif

    GUI_SelectNextDisplay(DISPLAY_MAIN);
}
