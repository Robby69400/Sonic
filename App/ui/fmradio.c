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
    sprintf(String, "%3d.%d MHz",
            gEeprom.FM_FrequencyPlaying / 10,
            gEeprom.FM_FrequencyPlaying % 10);
    
    UI_PrintString(String,40,127,1,8);
    
    // Keep the current scan and receiver states visible without borders.
    #define LINE 1
    //#define LINE 24
    if (gFM_No_Rx)
        //GUI_DisplaySmallestDark("NO RX", 3, LINE, false, true);
        UI_PrintStringSmallBold("NO RX",3,3,LINE);
    else
        //GUI_DisplaySmallestDark("RX ON", 3, LINE, false, true);
        UI_PrintStringSmallBold("RX ON",3,3,LINE);
    if (gFM_ManualMode)
        //GUI_DisplaySmallestDark("MAN", 102, LINE, false, true);
        UI_PrintStringSmallBold("MAN",3,3,LINE+1);
    else
        //GUI_DisplaySmallestDark("AUTO", 102, LINE, false, true);
        UI_PrintStringSmallBold("AUTO",3,3,LINE+1);
    // Station name for the selected frequency, e.g. "FRANCE INTER".
    const char *stationName = FM_FindRadioName(gEeprom.FM_FrequencyPlaying);

    if (stationName != NULL && gFmNameDisplay)
        UI_PrintString(stationName,0,127,4,8);
    else {
        // Nine frequency memory slots in a 3 x 3 grid.  The normal font polarity
        // leaves the LCD background clear and avoids separator lines.
        static const uint8_t memoryX[9] = {2, 44, 88, 2, 44, 88, 2, 44, 88};
        static const uint8_t memoryPage[9] = {4, 4, 4, 5, 5, 5, 6, 6, 6};

        for (uint8_t i = 0; i < 9; i++) {
            uint16_t frequency = gFM_Memory[i];
            if (frequency != 0)
                sprintf(memoryString, "%03d.%d", frequency / 10, frequency % 10);
            else
                sprintf(memoryString, " M%d",i+1);
            UI_PrintStringSmallBold(memoryString, memoryX[i], 0, memoryPage[i]);
        }

        for (uint8_t i = 4; i < FRAME_LINES; i++) //Vertical lines
        {
            gFrameBuffer[i][40] = 0xAA; 
            gFrameBuffer[i][41] = 0xAA; 
            gFrameBuffer[i][84] = 0xAA; 
            gFrameBuffer[i][85] = 0xAA; 
        }

        for (uint8_t x = 38; x < 43; x++) //Horizontal lines
        {
            gFrameBuffer[4][x] |= 0x80;
            gFrameBuffer[5][x] |= 0x80;
        }
        for (uint8_t x = 82; x < 87; x++) //Horizontal lines
        {
            gFrameBuffer[4][x] |= 0x80;
            gFrameBuffer[5][x] |= 0x80;
        }
    }
    ST7565_BlitFullScreen();
}

#endif
