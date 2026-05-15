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

void UI_DisplayFM(void)
{
    char String[16];

    memset(gFrameBuffer, 0, sizeof(gFrameBuffer));

    // Частота большим диджитал шрифтом
    memset(String, 0, sizeof(String));
    sprintf(String, "%3d.%d",
            gEeprom.FM_FrequencyPlaying / 10,
            gEeprom.FM_FrequencyPlaying % 10);
    UI_DisplayFrequency(String, 66, 2, true);

    if (gFM_ManualMode)
        GUI_DisplaySmallestDark("* MANUAL", 72, 5, false, true);
    else
        GUI_DisplaySmallestDark("* AUTO", 80, 5, false, true);
    GUI_DisplaySmallestDark("SONIC", 86, 45, false, true);

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
