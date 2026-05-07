/* Copyright 2023 Dual Tachyon
 * https://github.com/DualTachyon
 *
 * OURO_KA52 FM UI
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#ifdef ENABLE_FMRADIO

#include <string.h>

#include "app/fm.h"
#include "driver/st7565.h"
#include "external/printf/printf.h"
#include "settings.h"
#include "ui/helper.h"
#include "ui/ui.h"


static void DrawLine(uint8_t x0, uint8_t x1, uint8_t xs, uint8_t y0, uint8_t y1, uint8_t ys)
{
    for (uint8_t x = x0; x <= x1; x += xs)
        for (uint8_t y = y0; y <= y1; y += ys)
            gFrameBuffer[(y - 8) >> 3][x] |= (1u << ((y - 8) & 7));
}

void UI_DisplayFM(void)
{
    char String[16];

    memset(gFrameBuffer, 0, sizeof(gFrameBuffer));

    // Частота большим диджитал шрифтом
    memset(String, 0, sizeof(String));
    sprintf(String, "%3d.%d",
            gEeprom.FM_FrequencyPlaying / 10,
            gEeprom.FM_FrequencyPlaying % 10);
    UI_DisplayFrequency(String, 66, 3, true);

    // Линии
    DrawLine(  3, 49, 1, 15, 15, 1); // гор 1 левая
    DrawLine(116,124, 2, 15, 15, 1); // гор 1 правая пунктир
    DrawLine(  3, 49, 1, 23, 23, 1); // гор 2 левая
    DrawLine(116,124, 2, 23, 23, 1); // гор 2 правая пунктир
    DrawLine(  3, 49, 1, 31, 31, 1); // гор 3
    DrawLine( 50, 64, 2, 39, 39, 1); // гор 3.5 пунктир
    DrawLine(  3, 49, 1, 39, 39, 1); // гор 4
    DrawLine(  3, 49, 1, 47, 47, 1); // гор 5
    DrawLine(  3, 49, 1, 55, 55, 1); // гор 6 левая
    DrawLine(116,124, 2, 55, 55, 1); // гор 6 правая пунктир
    DrawLine( 64, 66, 1, 48, 48, 1); // гор малая низ
    DrawLine( 64, 66, 1, 31, 31, 1); // гор малая верх
    DrawLine( 50, 50, 1, 15, 55, 1); // верт левая
    DrawLine( 64, 64, 1, 31, 48, 1); // верт средняя
    DrawLine(124,124, 1, 15, 55, 1); // верт правая

 //GUI_DisplaySmallestDark("FIND:", 1, 6, false, true);
    // Режим поиска: AUTO или MANU — сразу после BROADCAST
    if (gFM_ManualMode)
        GUI_DisplaySmallestDark("STEP", 92, 5, false, true);
    else
        GUI_DisplaySmallestDark("AUTO", 92, 5, false, true);

//MUTE / ON AIR
    if (gFM_Mute) {
        // Если звук выключен
        GUI_DisplaySmallestDark("SILENT", 80, 13, false, true);
    } else {
        // Если звук включен (активный эфир)
        GUI_DisplaySmallestDark("ON AIR", 80, 13, false, true);
    }

    GUI_DisplaySmallestDark("SONIC FM", 86, 45, false, true);

    // ── 6 ячеек памяти частот ───────────────────────────────────────────
    char mb[8];
    uint16_t f;

    f = gFM_Memory[0]; sprintf(mb, f ? "%03d.%d" : "NO-CH", f / 10, f % 10); GUI_DisplaySmallestDark(mb,  12, 5, false, true);
    GUI_DisplaySmallestDark("1",  2, 5, false, false);
    f = gFM_Memory[1]; sprintf(mb, f ? "%03d.%d" : "NO-CH", f / 10, f % 10); GUI_DisplaySmallestDark(mb,  12, 13, false, true);
    GUI_DisplaySmallestDark("2",  2, 13, false, false);
    f = gFM_Memory[2]; sprintf(mb, f ? "%03d.%d" : "NO-CH", f / 10, f % 10); GUI_DisplaySmallestDark(mb,  12, 21, false, true);
    GUI_DisplaySmallestDark("3",  2, 21, false, false);
    f = gFM_Memory[3]; sprintf(mb, f ? "%03d.%d" : "NO-CH", f / 10, f % 10); GUI_DisplaySmallestDark(mb, 12, 29, false, true);
    GUI_DisplaySmallestDark("4", 2, 29, false, false);
    f = gFM_Memory[4]; sprintf(mb, f ? "%03d.%d" : "NO-CH", f / 10, f % 10); GUI_DisplaySmallestDark(mb, 12, 37, false, true);
    GUI_DisplaySmallestDark("5", 2, 37, false, false);
    f = gFM_Memory[5]; sprintf(mb, f ? "%03d.%d" : "NO-CH", f / 10, f % 10); GUI_DisplaySmallestDark(mb, 12, 45, false, true);
    GUI_DisplaySmallestDark("6", 2, 45, false, false);

    ST7565_BlitFullScreen();
}

#endif
