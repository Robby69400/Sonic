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

#include <assert.h>
#include <string.h>

#include "app/action.h"
#include "app/app.h"
#include "app/common.h"

#ifdef ENABLE_FLASHLIGHT
    #include "app/flashlight.h"
#endif
#ifdef ENABLE_FMRADIO
    #include "app/fm.h"
#endif

#ifdef ENABLE_FMRADIO
    #include "driver/bk1080.h"
#endif
#include "driver/bk4819.h"
#include "driver/gpio.h"
#include "driver/backlight.h"
#include "functions.h"
#include "misc.h"
#include "settings.h"
#include "ui/inputbox.h"
#include "ui/ui.h"
void (*action_opt_table[])(void) = {
    [ACTION_OPT_NONE] = &FUNCTION_NOP,
    [ACTION_OPT_POWER] = &ACTION_Power,
    [ACTION_OPT_MONITOR] = &ACTION_Monitor,
    [ACTION_OPT_KEYLOCK] = &COMMON_KeypadLockToggle,
    [ACTION_OPT_A_B] = &COMMON_SwitchVFOs,
    [ACTION_OPT_VFO_MR] = &COMMON_SwitchVFOMode,
    [ACTION_OPT_SWITCH_DEMODUL] = &ACTION_SwitchDemodul,

#ifdef ENABLE_FLASHLIGHT
    [ACTION_OPT_FLASHLIGHT] = &ACTION_FlashLight,
#else
    [ACTION_OPT_FLASHLIGHT] = &FUNCTION_NOP,
#endif

#ifdef ENABLE_FMRADIO
    [ACTION_OPT_FM] = &ACTION_FM,
#else
    [ACTION_OPT_FM] = &FUNCTION_NOP,
#endif

    [ACTION_OPT_ALARM] = &FUNCTION_NOP,

    [ACTION_OPT_1750] = &FUNCTION_NOP,

    [ACTION_OPT_BLMIN_TMP_OFF] = &FUNCTION_NOP,

#ifdef ENABLE_FEAT_F4HWN
    [ACTION_OPT_RXMODE] = &ACTION_RxMode,
    [ACTION_OPT_MAINONLY] = &ACTION_MainOnly,
    [ACTION_OPT_PTT] = &ACTION_Ptt,
    [ACTION_OPT_WN] = &ACTION_Wn,
    [ACTION_OPT_BACKLIGHT] = &ACTION_BackLight,
    #ifdef ENABLE_FEAT_F4HWN_AUDIO
        [ACTION_OPT_RXA] = &ACTION_RxA,
    #else
        [ACTION_OPT_RXA] = &FUNCTION_NOP,
    #endif
#else
    [ACTION_OPT_RXMODE] = &FUNCTION_NOP,
#endif
};

static_assert(ARRAY_SIZE(action_opt_table) == ACTION_OPT_LEN);

void ACTION_Power(void)
{
    gTxVfo->OUTPUT_POWER++;
    if (gTxVfo->OUTPUT_POWER >= OUTPUT_POWER_LEN)
        gTxVfo->OUTPUT_POWER = OUTPUT_POWER_LOW;

    // Save immediately to flash
    SETTINGS_SaveChannel(gTxVfo->CHANNEL_SAVE, gEeprom.TX_VFO, gTxVfo, 2);

    // Apply new power level to hardware immediately (recalculates TXP_CalculatedSetting)
    RADIO_ConfigureSquelchAndOutputPower(gTxVfo);

    gRequestDisplayScreen = gScreenToDisplay;


}

void ACTION_Monitor(void)
{
    if (gCurrentFunction != FUNCTION_MONITOR) { // enable the monitor
        RADIO_SelectVfos();
        RADIO_SetupRegisters(true);
        APP_StartListening(FUNCTION_MONITOR);
        return;
    }

    gMonitor = false;

    RADIO_SetupRegisters(true);

#ifdef ENABLE_FMRADIO
    if (gFmRadioMode) {
        FM_Start();
        gRequestDisplayScreen = DISPLAY_FM;
    }
    else
#endif
        gRequestDisplayScreen = gScreenToDisplay;
}

void ACTION_SwitchDemodul(void)
{
    gRequestSaveChannel = 1;

    gTxVfo->Modulation++;

    if(gTxVfo->Modulation == MODULATION_UKNOWN)
        gTxVfo->Modulation = MODULATION_FM;
}


void ACTION_Handle(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld)
{
    HideFKeyIcon();
    enum ACTION_OPT_t func = ACTION_OPT_NONE;
    switch(Key) {
        case KEY_SIDE1:
            if (bKeyHeld)
                func = gEeprom.KEY_1_LONG_PRESS_ACTION;
            else
                func = gEeprom.KEY_1_SHORT_PRESS_ACTION;
            break;
        case KEY_SIDE2:
            if (bKeyHeld)
                func = gEeprom.KEY_2_LONG_PRESS_ACTION;
            else
                func = gEeprom.KEY_2_SHORT_PRESS_ACTION;
            break;
        case KEY_MENU:
            if (bKeyHeld)
                func = gEeprom.KEY_M_LONG_PRESS_ACTION;
            break;
        default:
            break;
    }

    if (!bKeyHeld && bKeyPressed) // button pushed
    {
        return;
    }

    // held or released beyond this point

    if (bKeyHeld && !bKeyPressed) // button released after hold
    {
        return;
    }

    // held or released after short press beyond this point
    
#ifdef ENABLE_FMRADIO
    if (gFmRadioMode) { // do not run these actions in FM radio mode
        switch (func) {
            case ACTION_OPT_POWER:
            case ACTION_OPT_MONITOR:
            case ACTION_OPT_A_B:
            case ACTION_OPT_VFO_MR:
            case ACTION_OPT_SWITCH_DEMODUL:
    #ifdef ENABLE_FEAT_F4HWN
            case ACTION_OPT_RXMODE:
            case ACTION_OPT_MAINONLY:
            case ACTION_OPT_WN:
        #ifdef ENABLE_FEAT_F4HWN_AUDIO
            case ACTION_OPT_RXA:
        #endif
    #endif
                return;

            default:
                break;
        }
    }
#endif

    action_opt_table[func]();
}


#ifdef ENABLE_FMRADIO
void ACTION_FM(void)
{
    if (gCurrentFunction != FUNCTION_TRANSMIT && gCurrentFunction != FUNCTION_MONITOR)
    {
        gInputBoxIndex = 0;

        if (gFmRadioMode) {
            FM_TurnOff();
            gFlagReconfigureVfos  = true;
            gRequestDisplayScreen = DISPLAY_MAIN;

            return;
        }

        gMonitor = false;

        RADIO_SelectVfos();
        RADIO_SetupRegisters(true);

        FM_Start();

        gRequestDisplayScreen = DISPLAY_FM;
    }
}

#endif





#ifdef ENABLE_FEAT_F4HWN
void ACTION_Update(void)
{
    gSaveRxMode          = false;
    gFlagReconfigureVfos = true;
    gUpdateStatus        = true;
}

void ACTION_RxMode(void)
{
    // 3 режима как в меню RxMode:
    // 0 = MAIN ONLY    (DUAL_WATCH=OFF,  CROSS_BAND=OFF)
    // 1 = DUAL RESPOND (DUAL_WATCH=on,   CROSS_BAND=OFF)
    // 2 = MAIN TX DUAL (DUAL_WATCH=on,   CROSS_BAND=CHAN_A)
    uint8_t mode;
    if      (gEeprom.DUAL_WATCH == DUAL_WATCH_OFF)        mode = 0;
    else if (gEeprom.CROSS_BAND_RX_TX == CROSS_BAND_OFF)  mode = 1;
    else                                                   mode = 2;

    mode = (mode + 1) % 3;

    switch (mode) {
        case 0:
            gEeprom.DUAL_WATCH       = DUAL_WATCH_OFF;
            gEeprom.CROSS_BAND_RX_TX = CROSS_BAND_OFF;
            break;
        case 1:
            gEeprom.DUAL_WATCH       = gEeprom.TX_VFO + 1;
            gEeprom.CROSS_BAND_RX_TX = CROSS_BAND_OFF;
            break;
        case 2:
            gEeprom.DUAL_WATCH       = gEeprom.TX_VFO + 1;
            gEeprom.CROSS_BAND_RX_TX = CROSS_BAND_CHAN_A;
            break;
    }
    ACTION_Update();
}

void ACTION_MainOnly(void)
{
    static bool cycle = 0;
    static uint8_t dw = 0;
    static uint8_t cb = 0;

    if (cycle) {
        gEeprom.DUAL_WATCH = dw;
        gEeprom.CROSS_BAND_RX_TX = cb;
    } else {
        dw = gEeprom.DUAL_WATCH;
        cb = gEeprom.CROSS_BAND_RX_TX;

        gEeprom.DUAL_WATCH = 0;
        gEeprom.CROSS_BAND_RX_TX = 0;
    }

    cycle = !cycle;
    ACTION_Update();
}

#ifdef ENABLE_FEAT_F4HWN_AUDIO
void ACTION_RxA(void)
{
    if(gRxVfo->Modulation == MODULATION_AM)
        gSetting_set_audio_am = (gSetting_set_audio_am + 1) % 3;
    else if (gRxVfo->Modulation == MODULATION_FM)
        gSetting_set_audio_fm = (gSetting_set_audio_fm + 1) % 5;

    RADIO_SetModulation(gRxVfo->Modulation);
}
#endif

void ACTION_Ptt(void)
{
    gSetting_set_ptt_session = !gSetting_set_ptt_session;

    ACTION_Update();
}

void ACTION_Wn(void)
{
    const bool isRx = FUNCTION_IsRx();
    VFO_Info_t *pVfo = isRx ? gRxVfo : gTxVfo;

    pVfo->CHANNEL_BANDWIDTH = !pVfo->CHANNEL_BANDWIDTH;

    if (pVfo->Modulation == MODULATION_AM)
    {
        BK4819_SetFilterBandwidth(BK4819_FILTER_BW_AM, true);
        return;
    }

    uint8_t bw = pVfo->CHANNEL_BANDWIDTH;

    #ifdef ENABLE_FEAT_F4HWN_NARROWER
        if (isRx && bw == BANDWIDTH_NARROW && gSetting_set_nfm == 1)
        {
            bw++; 
        }
    #endif

    BK4819_SetFilterBandwidth(bw, false);
}

void ACTION_BackLight(void)
{
    if(gBackLight)
    {
        gEeprom.BACKLIGHT_TIME = gBacklightTimeOriginal;
    }
    gBackLight = false;
    BACKLIGHT_TurnOn();
}

void ACTION_BackLightOnDemand(void)
{
    if(gBackLight == false)
    {
        gBacklightTimeOriginal = gEeprom.BACKLIGHT_TIME;
        gEeprom.BACKLIGHT_TIME = 61;
        gBackLight = true;
    }
    else
    {
        if(gBacklightBrightnessOld == gEeprom.BACKLIGHT_MAX)
        {
            gEeprom.BACKLIGHT_TIME = 0;
        }
        else
        {
            gEeprom.BACKLIGHT_TIME = 61;
        }
    }
    
    BACKLIGHT_TurnOn();
}

#endif

