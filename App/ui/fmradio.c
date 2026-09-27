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
    char memoryString[8];
    memset(gFrameBuffer, 0, sizeof(gFrameBuffer));
    
    // Frequency in a large digital font, centered in the top area.
    memset(String, 0, sizeof(String));
    sprintf(String, "%d.%d MHz",
            gEeprom.FM_FrequencyPlaying / 10,
            gEeprom.FM_FrequencyPlaying % 10);
    #define LINE 0
    UI_PrintString(String,0,127,LINE,8);
    
    // Keep the current scan and receiver states visible without borders.
    
    //#define LINE 24
    if (gFM_No_Rx)
        UI_PrintStringSmallBold("NRX",0,0,LINE);
    else
        UI_PrintStringSmallBold("RX",0,0,LINE);
    if (gFM_ManualMode)
        UI_PrintStringSmallBoldRight("MA",126,LINE);
    else
        UI_PrintStringSmallBoldRight("AU",126,LINE);
    const char *stationName = FM_FindRadioName(gEeprom.FM_FrequencyPlaying);

    if (gFmNameDisplay) {
        if (stationName != NULL) UI_PrintString(stationName,0,127,4,8);
    } else {
        // Nine frequency memory slots in a 3 x 3 grid.  The normal font polarity
        // leaves the LCD background clear and avoids separator lines.
        static const uint8_t memoryX[9] = {2, 44, 88, 2, 44, 88, 2, 44, 88};
        static const uint8_t memoryPage[9] = {2, 2, 2, 4, 4, 4, 6, 6, 6};

        for (uint8_t i = 0; i < 9; i++) {
            uint16_t frequency = gFM_Memory[i];
            if (frequency != 0)
                sprintf(memoryString, "%3d.%d", frequency / 10, frequency % 10);
            else
                sprintf(memoryString, " M%d",i+1);
            UI_PrintStringSmallBold(memoryString, memoryX[i], 0, memoryPage[i]);
        }
        // Lignes verticales
        
        gFrameBuffer[2][40] = 0xAA;
        gFrameBuffer[2][84] = 0xAA;
        gFrameBuffer[3][40] = 0xAA;
        gFrameBuffer[3][84] = 0xAA;
        gFrameBuffer[4][40] = 0xAA;
        gFrameBuffer[4][84] = 0xAA;
        gFrameBuffer[5][40] = 0xAA;
        gFrameBuffer[5][84] = 0xAA;
        gFrameBuffer[6][40] = 0xAA;
        gFrameBuffer[6][84] = 0xAA;
        

        // Lignes horizontales
        for (int i = 25; i <= 55; i+=3) {
            gFrameBuffer[3][i] |= 0x08;
            gFrameBuffer[5][i] |= 0x08;
        }
        for (int i = 69; i <= 99; i+=3) {
            gFrameBuffer[3][i] |= 0x08;
            gFrameBuffer[5][i] |= 0x08;
        }
    }
    ST7565_BlitFullScreen();
}

#endif
