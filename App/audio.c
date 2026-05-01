/* Copyright 2025 muzkr https://github.com/muzkr
 * Copyright 2023 Dual Tachyon
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

#ifdef ENABLE_FMRADIO
    #include "app/fm.h"
#endif
#include "audio.h"
#ifdef ENABLE_FMRADIO
    #include "driver/bk1080.h"
#endif
#include "driver/bk4819.h"
#include "driver/gpio.h"
#include "driver/system.h"
#include "driver/systick.h"
#include "driver/voice.h"
#include "driver/py25q16.h"
#include "functions.h"
#include "misc.h"
#include "settings.h"
#include "ui/ui.h"


BEEP_Type_t gBeepToPlay = BEEP_NONE;

static void AUDIO_PlayBeepPulse(void) {
    BK4819_ExitTxMute();
    SYSTEM_DelayMs(60);
    BK4819_EnterTxMute();
    SYSTEM_DelayMs(20);
}

void AUDIO_PlayBeep(BEEP_Type_t Beep)
{

    if (Beep != BEEP_880HZ_60MS_DOUBLE_BEEP &&
        Beep != BEEP_500HZ_60MS_DOUBLE_BEEP &&
        Beep != BEEP_440HZ_500MS &&
#ifdef ENABLE_FEAT_F4HWN
        Beep != BEEP_400HZ_30MS &&
        Beep != BEEP_500HZ_30MS &&
        Beep != BEEP_600HZ_30MS &&
#endif
       !gEeprom.BEEP_CONTROL)
        return;
        
    if (gCurrentFunction == FUNCTION_RECEIVE)
        return;

    if (gCurrentFunction == FUNCTION_MONITOR)
        return;

#ifdef ENABLE_FMRADIO
    if (gFmRadioMode)
        BK1080_Mute(true);
#endif

    AUDIO_AudioPathOff();

    if (gCurrentFunction == FUNCTION_POWER_SAVE && gRxIdleMode)
        BK4819_RX_TurnOn();

    SYSTEM_DelayMs(20);

    uint16_t ToneConfig = BK4819_ReadRegister(BK4819_REG_71);

    uint16_t ToneFrequency;
    switch (Beep)
    {
        default:
        case BEEP_NONE:
            ToneFrequency = 220;
            break;
        case BEEP_1KHZ_60MS_OPTIONAL:
            ToneFrequency = 1000;
            break;
        case BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL:
        case BEEP_500HZ_60MS_DOUBLE_BEEP:
            ToneFrequency = 500;
            break;
        case BEEP_440HZ_500MS:
            ToneFrequency = 440;
            break;
        case BEEP_880HZ_60MS_DOUBLE_BEEP:
#ifndef ENABLE_FEAT_F4HWN
        case BEEP_880HZ_200MS:
        case BEEP_880HZ_500MS:
#endif
            ToneFrequency = 880;
            break;
#ifdef ENABLE_FEAT_F4HWN
        case BEEP_400HZ_30MS:
            ToneFrequency = 400;
            break;
        case BEEP_500HZ_30MS:
            ToneFrequency = 500;
            break;
        case BEEP_600HZ_30MS:
            ToneFrequency = 600;
            break;
#endif
    }

    if(Beep == BEEP_400HZ_30MS || Beep == BEEP_500HZ_30MS || Beep == BEEP_600HZ_30MS)
    {
        BK4819_WriteRegister(BK4819_REG_70, BK4819_REG_70_ENABLE_TONE1 | ((1 & 0x7f) << BK4819_REG_70_SHIFT_TONE1_TUNING_GAIN));
    }

    BK4819_PlayTone(ToneFrequency, true);

    SYSTEM_DelayMs(2);

    AUDIO_AudioPathOn();

    SYSTEM_DelayMs(60);

    uint16_t Duration;
    switch (Beep)
    {
        case BEEP_880HZ_60MS_DOUBLE_BEEP:
            AUDIO_PlayBeepPulse();
            [[fallthrough]];
        case BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL:
        case BEEP_500HZ_60MS_DOUBLE_BEEP:
            AUDIO_PlayBeepPulse();
            [[fallthrough]];
        case BEEP_1KHZ_60MS_OPTIONAL:
            BK4819_ExitTxMute();
            Duration = 60;
            break;
#ifdef ENABLE_FEAT_F4HWN
        case BEEP_400HZ_30MS:
        case BEEP_500HZ_30MS:
        case BEEP_600HZ_30MS:
            BK4819_ExitTxMute();
            Duration = 30;
            break;
#endif
        case BEEP_440HZ_500MS:
#ifndef ENABLE_FEAT_F4HWN
        case BEEP_880HZ_200MS:
            BK4819_ExitTxMute();
            Duration = 200;
            break;
        case BEEP_880HZ_500MS:
#endif
        default:
            BK4819_ExitTxMute();
            Duration = 500;
            break;
    }

    SYSTEM_DelayMs(Duration);
    BK4819_EnterTxMute();
    SYSTEM_DelayMs(20);

    AUDIO_AudioPathOff();

    SYSTEM_DelayMs(5);
    BK4819_TurnsOffTones_TurnsOnRX();
    SYSTEM_DelayMs(5);
    BK4819_WriteRegister(BK4819_REG_71, ToneConfig);

#ifdef ENABLE_FMRADIO
    const bool isFmRadio = gFmRadioMode;
    
    if (isFmRadio)
        SYSTEM_DelayMs(10);
#endif


    if (gEnableSpeaker)
        AUDIO_AudioPathOn();

#ifdef ENABLE_FMRADIO
    if (isFmRadio)
        BK1080_Mute(false);
#endif

    if (gCurrentFunction == FUNCTION_POWER_SAVE && gRxIdleMode)
        BK4819_Sleep();


}

