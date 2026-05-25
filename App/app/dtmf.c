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
#include <stdio.h>   // NULL
#ifdef ENABLE_FMRADIO
    #include "app/fm.h"
#endif
#include "audio.h"
#include "driver/bk4819.h"
#include "driver/eeprom.h"
#include "driver/gpio.h"
#include "driver/system.h"
#include "dtmf.h"
#include "external/printf/printf.h"
#include "misc.h"
#include "settings.h"
#include "ui/ui.h"

char              gDTMF_String[15];

char              gDTMF_InputBox[15];
uint8_t           gDTMF_InputBox_Index = 0;
bool              gDTMF_InputMode      = false;
uint8_t           gDTMF_PreviousIndex  = 0;

char              gDTMF_RX_live[20];
uint8_t           gDTMF_RX_live_timeout = 0;

DTMF_ReplyState_t gDTMF_ReplyState;


void DTMF_SendEndOfTransmission(void)
{
    if (gCurrentVfo->DTMF_PTT_ID_TX_MODE == PTT_ID_APOLLO) {
        BK4819_PlaySingleTone(2475, 250, 28, gEeprom.DTMF_SIDE_TONE);
    }

    if ((gCurrentVfo->DTMF_PTT_ID_TX_MODE == PTT_ID_TX_DOWN || gCurrentVfo->DTMF_PTT_ID_TX_MODE == PTT_ID_BOTH)
    ) { // end-of-tx
        if (gEeprom.DTMF_SIDE_TONE) {
            AUDIO_AudioPathOn();
            gEnableSpeaker = true;
            SYSTEM_DelayMs(60);
        }

        BK4819_EnterDTMF_TX(gEeprom.DTMF_SIDE_TONE);

        BK4819_PlayDTMFString(
                gEeprom.DTMF_DOWN_CODE,
                0,
                gEeprom.DTMF_FIRST_CODE_PERSIST_TIME,
                gEeprom.DTMF_HASH_CODE_PERSIST_TIME,
                gEeprom.DTMF_CODE_PERSIST_TIME,
                gEeprom.DTMF_CODE_INTERVAL_TIME);

        AUDIO_AudioPathOff();
        gEnableSpeaker = false;
    }

    BK4819_ExitDTMF_TX(true);
}

bool DTMF_ValidateCodes(char *pCode, const unsigned int size)
{
    unsigned int i;

    if (pCode[0] == 0xFF || pCode[0] == 0)
        return false;

    for (i = 0; i < size; i++)
    {
        if (pCode[i] == 0xFF || pCode[i] == 0)
        {
            pCode[i] = 0;
            break;
        }

        if ((pCode[i] < '0' || pCode[i] > '9') && (pCode[i] < 'A' || pCode[i] > 'D') && pCode[i] != '*' && pCode[i] != '#')
            return false;
    }

    return true;
}


char DTMF_GetCharacter(const unsigned int code)
{
    if (code <= KEY_9)
        return '0' + code;

    switch (code)
    {
        case KEY_MENU: return 'A';
        case KEY_UP:   return 'B';
        case KEY_DOWN: return 'C';
        case KEY_EXIT: return 'D';
        case KEY_STAR: return '*';
        case KEY_F:    return '#';
        default:       return 0xff;
    }
}

void DTMF_clear_input_box(void)
{
    memset(gDTMF_InputBox, 0, sizeof(gDTMF_InputBox));
    gDTMF_InputBox_Index = 0;
    gDTMF_InputMode      = false;
}

void DTMF_Append(const char code)
{
    if (gDTMF_InputBox_Index == 0)
    {
        memset(gDTMF_InputBox, '-', sizeof(gDTMF_InputBox) - 1);
        gDTMF_InputBox[sizeof(gDTMF_InputBox) - 1] = 0;
    }

    if (gDTMF_InputBox_Index < (sizeof(gDTMF_InputBox) - 1))
        gDTMF_InputBox[gDTMF_InputBox_Index++] = code;
}


void DTMF_Reply(void)
{
    uint16_t    Delay;
    const char *pString = NULL;

    switch (gDTMF_ReplyState)
    {
        case DTMF_REPLY_ANI:
            {
                pString = gDTMF_String;
            }

            break;
        default:
        case DTMF_REPLY_NONE:
            if (
                gCurrentVfo->DTMF_PTT_ID_TX_MODE == PTT_ID_APOLLO ||
                gCurrentVfo->DTMF_PTT_ID_TX_MODE == PTT_ID_OFF    ||
                gCurrentVfo->DTMF_PTT_ID_TX_MODE == PTT_ID_TX_DOWN)
            {
                gDTMF_ReplyState = DTMF_REPLY_NONE;
                return;
            }

            // send TX-UP DTMF
            pString = gEeprom.DTMF_UP_CODE;
            break;
    }

    gDTMF_ReplyState = DTMF_REPLY_NONE;

    if (pString == NULL)
        return;

    Delay = (gEeprom.DTMF_PRELOAD_TIME < 200) ? 200 : gEeprom.DTMF_PRELOAD_TIME;

    if (gEeprom.DTMF_SIDE_TONE)
    {   // the user will also hear the transmitted tones
        AUDIO_AudioPathOn();
        gEnableSpeaker = true;
    }

    SYSTEM_DelayMs(Delay);

    BK4819_EnterDTMF_TX(gEeprom.DTMF_SIDE_TONE);

    BK4819_PlayDTMFString(
        pString,
        1,
        gEeprom.DTMF_FIRST_CODE_PERSIST_TIME,
        gEeprom.DTMF_HASH_CODE_PERSIST_TIME,
        gEeprom.DTMF_CODE_PERSIST_TIME,
        gEeprom.DTMF_CODE_INTERVAL_TIME);

    AUDIO_AudioPathOff();

    gEnableSpeaker = false;

    BK4819_ExitDTMF_TX(false);
}
