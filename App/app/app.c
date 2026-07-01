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
#include <stdint.h>
#include <string.h>
#include "app/action.h"
#include "app/app.h"
#ifdef ENABLE_FLASHLIGHT
    #include "app/flashlight.h"
#endif
#ifdef ENABLE_FMRADIO
    #include "app/fm.h"
#endif
#include "app/generic.h"
#include "app/main.h"
#include "app/menu.h"
#if defined(ENABLE_UART) || defined(ENABLE_USB)
    #include "app/uart.h"
    #include "scheduler.h"
#endif
#include "py32f0xx.h"
#include "board.h"
#include "driver/backlight.h"
#ifdef ENABLE_FMRADIO
    #include "driver/bk1080.h"
#endif
#include "driver/bk4819.h"
#include "driver/gpio.h"
#include "driver/keyboard.h"
#include "driver/st7565.h"
#include "driver/system.h"
#include "external/printf/printf.h"
#include "frequencies.h"
#include "functions.h"
#include "helper/battery.h"
#include "misc.h"
#include "radio.h"
#include "settings.h"

#if defined(ENABLE_OVERLAY)
    #include "sram-overlay.h"
#endif
#include "ui/battery.h"
#include "ui/inputbox.h"
#include "ui/main.h"
#include "ui/menu.h"
#include "ui/status.h"
#include "ui/ui.h"
#include "app/spectrum.h"
#include "driver/py25q16.h"

static bool flagSaveVfo;
static bool flagSaveSettings;
static bool flagSaveChannel;

static void ProcessKey(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld);

void (*ProcessKeysFunctions[])(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld) = {
    [DISPLAY_MAIN] = &MAIN_ProcessKeys,
    [DISPLAY_MENU] = &MENU_ProcessKeys,

#ifdef ENABLE_FMRADIO
    [DISPLAY_FM] = &FM_ProcessKeys,
#endif
};

static_assert(ARRAY_SIZE(ProcessKeysFunctions) == DISPLAY_N_ELEM);

static void CheckForIncoming(void)
{
#ifdef ENABLE_FMRADIO
    if (gFmRadioMode && gFM_Mute)
        return;          // FM MUTE: приём рации полностью заблокирован
#endif

    if (!g_SquelchLost)
        return;          // squelch is closed

    // squelch is open

  // not RF scanning
        if (gEeprom.DUAL_WATCH == DUAL_WATCH_OFF)
        {   // dual watch is disabled

            if (gCurrentFunction != FUNCTION_INCOMING)
            {
                FUNCTION_Select(FUNCTION_INCOMING);
                //gUpdateDisplay = true;
            }

            return;
        }

        // dual watch is enabled and we're RX'ing a signal

        if (gRxReceptionMode != RX_MODE_NONE)
        {
            if (gCurrentFunction != FUNCTION_INCOMING)
            {
                FUNCTION_Select(FUNCTION_INCOMING);
                //gUpdateDisplay = true;
            }
            return;
        }

        gDualWatchCountdown_10ms = dual_watch_count_after_rx_10ms;
        gScheduleDualWatch       = false;

        // let the user see DW is not active
        gDualWatchActive = false;
        gUpdateStatus    = true;



    gRxReceptionMode = RX_MODE_DETECTED;

    if (gCurrentFunction != FUNCTION_INCOMING)
    {
        FUNCTION_Select(FUNCTION_INCOMING);
        //gUpdateDisplay = true;
    }
}

static void HandleIncoming(void)
{
    if (!g_SquelchLost) {   // squelch is closed
        if (gCurrentFunction != FUNCTION_FOREGROUND) {
            FUNCTION_Select(FUNCTION_FOREGROUND);
            gUpdateDisplay = true;
        }
        return;
    }

    bool bFlag = (gCurrentCodeType == CODE_TYPE_OFF);

    if (g_CTCSS_Lost && gCurrentCodeType == CODE_TYPE_CONTINUOUS_TONE) {
        bFlag       = true;
        gFoundCTCSS = false;
    }

    if (g_CDCSS_Lost && gCDCSSCodeType == CDCSS_POSITIVE_CODE
        && (gCurrentCodeType == CODE_TYPE_DIGITAL || gCurrentCodeType == CODE_TYPE_REVERSE_DIGITAL))
    {
        gFoundCDCSS = false;
    }
    else if (!bFlag)
        return;

    APP_StartListening(gMonitor ? FUNCTION_MONITOR : FUNCTION_RECEIVE);
}

static void HandleReceive(void)
{
    #define END_OF_RX_MODE_SKIP 0
    #define END_OF_RX_MODE_END  1
    #define END_OF_RX_MODE_TTE  2

    uint8_t Mode = END_OF_RX_MODE_SKIP;

    if (gFlagTailNoteEliminationComplete)
    {
        Mode = END_OF_RX_MODE_END;
        goto Skip;
    }

    
    switch (gCurrentCodeType)
    {
        default:
        case CODE_TYPE_OFF:
            break;

        case CODE_TYPE_CONTINUOUS_TONE:
        case CODE_TYPE_DIGITAL:
        case CODE_TYPE_REVERSE_DIGITAL:
            if ((gFoundCTCSS && gFoundCTCSSCountdown_10ms == 0) || (gFoundCDCSS && gFoundCDCSSCountdown_10ms == 0))
            {
                gFoundCTCSS = false;
                gFoundCDCSS = false;
                Mode        = END_OF_RX_MODE_END;
                goto Skip;
            }
            break;
    }

    if (g_SquelchLost)
    {
            if (!gEndOfRxDetectedMaybe)
        {
            switch (gCurrentCodeType)
            {
                case CODE_TYPE_OFF:
                    if (gEeprom.SQUELCH_LEVEL)
                    {
                        if (g_CxCSS_TAIL_Found)
                        {
                            Mode               = END_OF_RX_MODE_TTE;
                            g_CxCSS_TAIL_Found = false;
                        }
                    }
                    break;

                case CODE_TYPE_CONTINUOUS_TONE:
                    if (g_CTCSS_Lost)
                    {
                        gFoundCTCSS = false;
                    }
                    else
                    if (!gFoundCTCSS)
                    {
                        gFoundCTCSS               = true;
                        gFoundCTCSSCountdown_10ms = 100;   // 1 sec
                    }

                    if (g_CxCSS_TAIL_Found)
                    {
                        Mode               = END_OF_RX_MODE_TTE;
                        g_CxCSS_TAIL_Found = false;
                    }
                    break;

                case CODE_TYPE_DIGITAL:
                case CODE_TYPE_REVERSE_DIGITAL:
                    if (g_CDCSS_Lost && gCDCSSCodeType == CDCSS_POSITIVE_CODE)
                    {
                        gFoundCDCSS = false;
                    }
                    else
                    if (!gFoundCDCSS)
                    {
                        gFoundCDCSS               = true;
                        gFoundCDCSSCountdown_10ms = 100;   // 1 sec
                    }

                    if (g_CxCSS_TAIL_Found)
                    {
                        if (BK4819_GetCTCType() == 1)
                            Mode = END_OF_RX_MODE_TTE;

                        g_CxCSS_TAIL_Found = false;
                    }

                    break;
            }
        }
    }
    else
        Mode = END_OF_RX_MODE_END;

    if (!gEndOfRxDetectedMaybe         &&
         Mode == END_OF_RX_MODE_SKIP   &&
         gNextTimeslice40ms            &&
         gEeprom.TAIL_TONE_ELIMINATION &&
         (gCurrentCodeType == CODE_TYPE_DIGITAL || gCurrentCodeType == CODE_TYPE_REVERSE_DIGITAL) &&
         BK4819_GetCTCType() == 1)
        Mode = END_OF_RX_MODE_TTE;
    else
        gNextTimeslice40ms = false;

Skip:
    switch (Mode)
    {
        case END_OF_RX_MODE_SKIP:
            break;

        case END_OF_RX_MODE_END:
            RADIO_SetupRegisters(true);

            gUpdateDisplay = true;

             break;

        case END_OF_RX_MODE_TTE:
            if (gEeprom.TAIL_TONE_ELIMINATION) {
                GPIO_DisableAudioPath();

                gTailNoteEliminationCountdown_10ms = 20;
                gFlagTailNoteEliminationComplete   = false;
                gEndOfRxDetectedMaybe = true;
                gEnableSpeaker        = false;
            }
            break;
    }
}

static void HandlePowerSave()
{
    if (!gRxIdleMode) {
        CheckForIncoming();
    }
}

static void (*HandleFunction_fn_table[])(void) = {
    [FUNCTION_FOREGROUND] = &CheckForIncoming,
    [FUNCTION_TRANSMIT] = &FUNCTION_NOP,
    [FUNCTION_MONITOR] = &FUNCTION_NOP,
    [FUNCTION_INCOMING] = &HandleIncoming,
    [FUNCTION_RECEIVE] = &HandleReceive,
    [FUNCTION_POWER_SAVE] = &HandlePowerSave,
    [FUNCTION_BAND_SCOPE] = &FUNCTION_NOP,
};

static_assert(ARRAY_SIZE(HandleFunction_fn_table) == FUNCTION_N_ELEM);

static void HandleFunction(void)
{
    HandleFunction_fn_table[gCurrentFunction]();
}

void APP_StartListening(FUNCTION_Type_t function)
{
    const unsigned int vfo = gEeprom.RX_VFO;

#ifdef ENABLE_FEAT_F4HWN_RX_TX_TIMER
    gRxTimerCountdown_500ms = 7200;
#endif

#ifdef ENABLE_FMRADIO
    if (gFmRadioMode && !gFM_Mute) {
        // Глушим BK1080 только если не идёт FM_Start (избегаем двойного I2C при 250ms init)
        if (gFM_RestoreCountdown_10ms == 0)
            BK1080_Init0();
    }
#endif

    // clear the other vfo's rssi level (to hide the antenna symbol)
    gVFO_RSSI_bar_level[!vfo] = 0;

    GPIO_EnableAudioPath();
    gEnableSpeaker = true;

    if (gSetting_backlight_on_tx_rx & BACKLIGHT_ON_TR_RX) {
        BACKLIGHT_TurnOn();
    }

#ifdef ENABLE_FLASHLIGHT
    // Мигание фонарика при входящем сигнале
    if (gEeprom.FlashlightOnRX &&
        (function == FUNCTION_RECEIVE || function == FUNCTION_INCOMING)) {
        for (int i = 0; i < 2; i++) {
            GPIO_SetOutputPin(GPIO_PIN_FLASHLIGHT);
            SYSTEM_DelayMs(50);
            GPIO_ResetOutputPin(GPIO_PIN_FLASHLIGHT);
            SYSTEM_DelayMs(20);
        }
    }
#endif



    if (gEeprom.DUAL_WATCH != DUAL_WATCH_OFF)
    {   // not scanning, dual watch is enabled

        //gDualWatchCountdown_10ms = dual_watch_count_after_2_10ms;

        const bool isMainTxDualRx =
        (gEeprom.DUAL_WATCH != DUAL_WATCH_OFF) &&
        (gEeprom.CROSS_BAND_RX_TX != CROSS_BAND_OFF);

        // Use a short hold only for MAIN TX DUAL RX, keep legacy hold otherwise
        gDualWatchCountdown_10ms = isMainTxDualRx
            ? dual_watch_count_after_2_10ms / 4 // Short timer = 420 / 4 ...
            : dual_watch_count_after_2_10ms;

        gScheduleDualWatch       = false;

        // when crossband is active only the main VFO should be used for TX
        if(gEeprom.CROSS_BAND_RX_TX == CROSS_BAND_OFF)
            gRxVfoIsActive = true;

        // let the user see DW is not active
        gDualWatchActive = false;
        gUpdateStatus    = true;
    }

    BK4819_WriteRegister(BK4819_REG_48,
        (11u << 12)                |     // ??? .. 0 to 15, doesn't seem to make any difference
        ( 0u << 10)                |     // AF Rx Gain-1
        (gEeprom.VOLUME_GAIN << 4) |     // AF Rx Gain-2
        (gEeprom.DAC_GAIN    << 0));     // AF DAC Gain (after Gain-1 and Gain-2)

        RADIO_SetModulation(gRxVfo->Modulation);  // no need, set it now

    FUNCTION_Select(function);

    if (function == FUNCTION_MONITOR)
    {   // squelch is disabled
        if (gScreenToDisplay != DISPLAY_MENU)     // 1of11 .. don't close the menu
            GUI_SelectNextDisplay(DISPLAY_MAIN);
    }
    else
        gUpdateDisplay = true;

    gUpdateStatus = true;
}

uint32_t APP_SetFreqByStepAndLimits(VFO_Info_t *pInfo, int8_t direction, uint32_t lower, uint32_t upper)
{
    uint32_t Frequency = FREQUENCY_RoundToStep(pInfo->freq_config_RX.Frequency + (direction * pInfo->StepFrequency), pInfo->StepFrequency);

#ifdef ENABLE_FEAT_F4HWN
    if (Frequency > upper)
#else
    if (Frequency >= upper)
#endif
        Frequency =  lower;

    else if (Frequency < lower)
        Frequency = FREQUENCY_RoundToStep(upper - pInfo->StepFrequency, pInfo->StepFrequency);

    return Frequency;
}

uint32_t APP_SetFrequencyByStep(VFO_Info_t *pInfo, int8_t direction)
{
    return APP_SetFreqByStepAndLimits(pInfo, direction, frequencyBandTable[pInfo->Band].lower, frequencyBandTable[pInfo->Band].upper);
}

static void DualwatchAlternate(void)
{
    {   // toggle between VFO's
        gEeprom.RX_VFO = !gEeprom.RX_VFO;
        gRxVfo         = &gEeprom.VfoInfo[gEeprom.RX_VFO];

        if (!gDualWatchActive)
        {   // let the user see DW is active
            gDualWatchActive = true;
            gUpdateStatus    = true;
        }
    }

    RADIO_SetupRegisters(false);

        gDualWatchCountdown_10ms = dual_watch_count_toggle_10ms;
}

static void CheckRadioInterrupts(void)
{

    while (BK4819_ReadRegister(BK4819_REG_0C) & 1u) { // BK chip interrupt request
        // clear interrupts
        BK4819_WriteRegister(BK4819_REG_02, 0);
        // fetch interrupt status bits

        union {
            struct {
                uint16_t __UNUSED : 1;
                uint16_t fskRxSync : 1;
                uint16_t sqlLost : 1;
                uint16_t sqlFound : 1;
                uint16_t ctcssLost : 1;
                uint16_t ctcssFound : 1;
                uint16_t cdcssLost : 1;
                uint16_t cdcssFound : 1;
                uint16_t cssTailFound : 1;
                uint16_t fskFifoAlmostFull : 1;
                uint16_t fskRxFinied : 1;
                uint16_t fskFifoAlmostEmpty : 1;
                uint16_t fskTxFinied : 1;
            };
            uint16_t __raw;
        } interrupts;

        interrupts.__raw = BK4819_ReadRegister(BK4819_REG_02);

        if (interrupts.cssTailFound)
            g_CxCSS_TAIL_Found = true;

        if (interrupts.cdcssLost) {
            g_CDCSS_Lost = true;
            gCDCSSCodeType = BK4819_GetCDCSSCodeType();
        }

        if (interrupts.cdcssFound)
            g_CDCSS_Lost = false;

        if (interrupts.ctcssLost)
            g_CTCSS_Lost = true;

        if (interrupts.ctcssFound)
            g_CTCSS_Lost = false;

        if (interrupts.sqlLost) {
            g_SquelchLost = true;
            BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, true);
            #ifdef ENABLE_FEAT_F4HWN_RX_TX_TIMER
                gRxTimerCountdown_500ms = 7200;
            #endif
        }

        if (interrupts.sqlFound) {
            g_SquelchLost = false;
            BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false);
        }
    }
}

void APP_EndTransmission(void)
{
    // back to RX mode
    RADIO_SendEndOfTransmission();

    gFlagEndTransmission = true;

    if (gMonitor) {
         //turn the monitor back on
        gFlagReconfigureVfos = true;
    }
}

void APP_Update(void)
{

#ifdef ENABLE_USB
    if (UART_IsCommandAvailable(UART_PORT_VCP)) {
        // SCHEDULER_Disable();
        UART_HandleCommand(UART_PORT_VCP);
        // SCHEDULER_Enable();
    }
#endif

#ifdef ENABLE_FEAT_F4HWN
    if (gCurrentFunction == FUNCTION_TRANSMIT && (gTxTimeoutReachedAlert || SerialConfigInProgress()))
    {
        if(gSetting_set_tot >= 2)
        {
            if (gEeprom.BACKLIGHT_TIME == 0) {
                if (gBlinkCounter == 0 || gBlinkCounter == 250)
                {
                    GPIO_TogglePin(GPIO_PIN_FLASHLIGHT);
                }
            }
            else
            {
                if (gBlinkCounter == 0)
                {
                    //BACKLIGHT_TurnOn();
                    BACKLIGHT_SetBrightness(gEeprom.BACKLIGHT_MAX);
                }
                else if(gBlinkCounter == 15000)
                {
                    //BACKLIGHT_TurnOff();
                    BACKLIGHT_SetBrightness(gEeprom.BACKLIGHT_MIN);
                }
            }
        }

        gBlinkCounter++;

        if(
            (gSetting_set_tot == 3 && gEeprom.BACKLIGHT_TIME != 0 && gBlinkCounter > 74000) || 
            (gSetting_set_tot == 3 && gEeprom.BACKLIGHT_TIME == 0 && gBlinkCounter > 79000) || 
            (gSetting_set_tot != 3 && gBlinkCounter > 76000)
            ) // try to calibrate 10 times
        {
            gBlinkCounter = 0;

            if(gSetting_set_tot == 1 || gSetting_set_tot == 3)
            {
                BK4819_PlaySingleTone(gTxTimeoutToneAlert, 30, 1, true);
                gTxTimeoutToneAlert += 100;
            }
        }
    }
#endif

    if (gCurrentFunction == FUNCTION_TRANSMIT && (gTxTimeoutReached || SerialConfigInProgress()))
    {   // transmitter timed out or must de-key
        gTxTimeoutReached = false;

#ifdef ENABLE_FEAT_F4HWN
        if(gBacklightCountdown_500ms > 0 || gEeprom.BACKLIGHT_TIME == 61)
        {
            //BACKLIGHT_TurnOn();
            BACKLIGHT_SetBrightness(gEeprom.BACKLIGHT_MAX);
        }

        gTxTimeoutReachedAlert = false;
        gTxTimeoutToneAlert = 800;

        if (gSetting_set_ptt_session) // Improve OnePush if TOT
        {
            if(gPttOnePushCounter == 1)
            {
                gPttOnePushCounter = 3;
            }
            else if(gPttOnePushCounter == 2)
            {
                ProcessKey(KEY_PTT, false, false);
                gPttIsPressed = false;
                gPttOnePushCounter = 0;
                gPttWasReleased = true;
                //if (gKeyReading1 != KEY_INVALID)
                //  gPttWasReleased = true;
            }
            #if defined(ENABLE_FEAT_F4HWN_CTR) || defined(ENABLE_FEAT_F4HWN_INV)
            ST7565_ContrastAndInv();
            #endif
        }
#endif

        APP_EndTransmission();
        RADIO_SetVfoState(VFO_STATE_TIMEOUT);

        GUI_DisplayScreen();
    }

    if (gReducedService)
        return;

    if (gCurrentFunction != FUNCTION_TRANSMIT)
        HandleFunction();

#ifdef ENABLE_FMRADIO
//  if (gFmRadioCountdown_500ms > 0)
    if (gFmRadioMode && gFmRadioCountdown_500ms > 0)    // 1of11
        return;
#endif

    // toggle between the VFO's if dual watch is enabled
    if (gEeprom.DUAL_WATCH != DUAL_WATCH_OFF
        && gScheduleDualWatch
        && !gPttIsPressed
        && gCurrentFunction != FUNCTION_POWER_SAVE
#ifdef ENABLE_FMRADIO
        && !gFmRadioMode
#endif
    ) {
        DualwatchAlternate();    // toggle between the two VFO's

        if (gRxVfoIsActive && gScreenToDisplay == DISPLAY_MAIN) {
            GUI_SelectNextDisplay(DISPLAY_MAIN);
        }

        gRxVfoIsActive     = false;
        gRxReceptionMode   = RX_MODE_NONE;
        gScheduleDualWatch = false;
    }

#ifdef ENABLE_FMRADIO
    if (gScheduleFM && gFM_ScanState != FM_SCAN_OFF && !FUNCTION_IsRx()) {
        // switch to FM radio mode
        FM_Play();
        gScheduleFM = false;
    }
#endif

    if (gSchedulePowerSave) {
        if (gPttIsPressed
            || gKeyBeingHeld
            || gEeprom.BATTERY_SAVE == 0
            || gCssBackgroundScan
            || gScreenToDisplay != DISPLAY_MAIN
#ifdef ENABLE_FMRADIO
            || gFmRadioMode
#endif
        ) {
            gBatterySaveCountdown_10ms = battery_save_count_10ms;
        } else {
            FUNCTION_Select(FUNCTION_POWER_SAVE);
        }

        gSchedulePowerSave = false;
    }

    if (gPowerSaveCountdownExpired && gCurrentFunction == FUNCTION_POWER_SAVE
    ) {
        static bool goToSleep;
        // wake up, enable RX then go back to sleep
        if (gRxIdleMode)
        {
            BK4819_Conditional_RX_TurnOn_and_GPIO6_Enable();

            if (gEeprom.DUAL_WATCH != DUAL_WATCH_OFF &&
                !gCssBackgroundScan)
            {   // dual watch mode, toggle between the two VFO's
                DualwatchAlternate();
                goToSleep = false;
            }

            FUNCTION_Init();

            gPowerSave_10ms = power_save1_10ms; // come back here in a bit
            gRxIdleMode     = false;            // RX is awake
        }
        else if (gEeprom.DUAL_WATCH == DUAL_WATCH_OFF || gCssBackgroundScan || goToSleep)
        {   // dual watch mode off or scanning or rssi update request
            // go back to sleep

            gPowerSave_10ms = gEeprom.BATTERY_SAVE * 10;
            gRxIdleMode     = true;
            goToSleep = false;
            BK4819_Sleep();
            BK4819_ToggleGpioOut(BK4819_GPIO0_PIN28_RX_ENABLE, false);

            // Authentic device checked removed

        }
        else {
            // toggle between the two VFO's
            DualwatchAlternate();
            gPowerSave_10ms   = power_save1_10ms;
            goToSleep = true;
        }

        gPowerSaveCountdownExpired = false;
    }
}

void StopTransmitting(void) {
    ProcessKey(KEY_PTT, false, false);
    gPttIsPressed = false;
    if (gKeyReading1 != KEY_INVALID)
        gPttWasReleased = true;

    #ifdef ENABLE_FEAT_F4HWN
        #if defined(ENABLE_FEAT_F4HWN_CTR) || defined(ENABLE_FEAT_F4HWN_INV)
        ST7565_ContrastAndInv();
        #endif
    #endif
}

// called every 10ms
void CheckKeys(void)
{
// -------------------- PTT ------------------------
    const bool serialConfigInProgress = SerialConfigInProgress();

#ifdef ENABLE_FEAT_F4HWN
    const bool isPressed = GPIO_IsPttPressed() && !serialConfigInProgress;
#else
    const bool isPressed = !GPIO_CheckBit(&GPIOC->DATA, GPIOC_PIN_PTT) && !serialConfigInProgress;
#endif

#ifdef ENABLE_FEAT_F4HWN
    if (gSetting_set_ptt_session)
    {
        if ((isPressed && (gPttOnePushCounter == 0 || gPttOnePushCounter == 2)) ||
            (!isPressed && (gPttOnePushCounter == 1 || gPttOnePushCounter == 3)) ||
            serialConfigInProgress) 
        {
            if (++gPttDebounceCounter >= 3 || (serialConfigInProgress && gPttOnePushCounter > 0))
            {
                gPttDebounceCounter = 0;
                
                if (gPttOnePushCounter == 0)
                {   // start transmitting
                    boot_counter_10ms   = 0;
                    gPttIsPressed       = true;
                    gPttOnePushCounter = 1;
                    ProcessKey(KEY_PTT, true, false);
                } 
                else if (gPttOnePushCounter == 3 || serialConfigInProgress)
                {   // stop transmitting
                    StopTransmitting();
                    gPttOnePushCounter = 0;
                } 
                else
                    gPttOnePushCounter++;
            }
        } 
        else {
            gPttDebounceCounter = 0;

        }
    } 
    else 
#endif
    {
        if (gPttIsPressed)
        {
            if (!isPressed)
            {   // PTT released or serial comms config in progress
                if (++gPttDebounceCounter >= 3 || serialConfigInProgress)   // 30ms
                {   // stop transmitting
                    gPttDebounceCounter = 0;
                    StopTransmitting();
                }
            } 
            else { 
                gPttDebounceCounter = 0;
            }
        }
        else if (isPressed)
        {   // PTT pressed
            if (++gPttDebounceCounter >= 3)     // 30ms
            {   // start transmitting
                boot_counter_10ms   = 0;
                gPttDebounceCounter = 0;
                gPttIsPressed       = true;
                ProcessKey(KEY_PTT, true, false);
            }
        }
        else { 
            gPttDebounceCounter = 0;
            if (gComeBack) APP_RunSpectrum(); //Robzyl mod for Ninja
        }
    }

// --------------------- OTHER KEYS ----------------------------

    // scan the hardware keys
    KEY_Code_t Key = KEYBOARD_Poll();

    if (Key != KEY_INVALID) // any key pressed
        boot_counter_10ms = 0;   // cancel boot screen/beeps if any key pressed

    if (gKeyReading0 != Key) // new key pressed
    {

        if (gKeyReading0 != KEY_INVALID && Key != KEY_INVALID)
            ProcessKey(gKeyReading1, false, gKeyBeingHeld);  // key pressed without releasing previous key

        gKeyReading0     = Key;
        gDebounceCounter = 0;
        return;
    }

    gDebounceCounter++;

    if (gDebounceCounter == key_debounce_10ms) // debounced new key pressed
    {
        if (Key == KEY_INVALID) //all non PTT keys released
        {
            if (gKeyReading1 != KEY_INVALID) // some button was pressed before
            {
                ProcessKey(gKeyReading1, false, gKeyBeingHeld); // process last button released event
                gKeyReading1 = KEY_INVALID;
            }
        }
        else // process new key pressed
        {
            gKeyReading1 = Key;
            ProcessKey(Key, true, false);
        }

        gKeyBeingHeld = false;
        return;
    }

    if (gDebounceCounter < key_repeat_delay_10ms || Key == KEY_INVALID) // the button is not held long enough for repeat yet, or not really pressed
        return;

    if (gDebounceCounter == key_repeat_delay_10ms) //initial key repeat with longer delay
    {
        if (Key != KEY_PTT)
        {
            gKeyBeingHeld = true;
            ProcessKey(Key, true, true); // key held event
        }
    }
    else //subsequent fast key repeats
    {
        if (Key == KEY_UP || Key == KEY_DOWN) // fast key repeats for up/down buttons
        {
            gKeyBeingHeld = true;
            if ((gDebounceCounter % key_repeat_10ms) == 0)
                ProcessKey(Key, true, true); // key held event
        }

        if (gDebounceCounter < 0xFFFF)
            return;

        gDebounceCounter = key_repeat_delay_10ms+1;
    }
}

void APP_TimeSlice10ms(void)
{
    gNextTimeslice = false;

    SETTINGS_SaveVfoIndicesFlush();

    BACKLIGHT_Update();

    gFlashLightBlinkCounter++;

#ifdef ENABLE_UART
    if (UART_IsCommandAvailable(UART_PORT_UART)) {
        // SCHEDULER_Disable();
        UART_HandleCommand(UART_PORT_UART);
        // SCHEDULER_Enable();
    }
#endif

    if (gReducedService)
        return;

    if (gCurrentFunction != FUNCTION_POWER_SAVE || !gRxIdleMode)
        CheckRadioInterrupts();

    if (gCurrentFunction == FUNCTION_TRANSMIT)
    {   // transmitting
#if defined(ENABLE_AUDIO_BAR) && !defined(ENABLE_FEAT_F4HWN_AUDIO_SCOPE)
        if (gSetting_mic_bar && (gFlashLightBlinkCounter % (150 / 10)) == 0) // once every 150ms
            UI_DisplayAudioBar();
#endif
    }

#ifdef ENABLE_FEAT_F4HWN_AUDIO_SCOPE
    if (gSetting_mic_bar && (gFlashLightBlinkCounter % (20 / 10)) == 0) // once every 20ms
        // Sample audio amplitude and refresh display during TX only (FM RX has no usable audio register)
        UI_DisplayAudioScope();
#endif

    bool gUpdateDisplayCurrent = gUpdateDisplay;
    bool gUpdateStatusCurrent  = gUpdateStatus;

    if (gUpdateDisplayCurrent) {
        gUpdateDisplay = false;
        GUI_DisplayScreen();
    }

    if (gUpdateStatusCurrent) {
        UI_DisplayStatus();
    }

    // Skipping authentic device checks

#ifdef ENABLE_FMRADIO
    if (gFmRadioMode && gFmRadioCountdown_500ms > 0)   // 1of11
        return;
#endif

#if !defined(ENABLE_FEAT_F4HWN)
    #ifdef ENABLE_FLASHLIGHT
        FlashlightTimeSlice();
    #endif
#endif

    if (gCurrentFunction == FUNCTION_TRANSMIT) {
        // repeater tail tone elimination
        if (gRTTECountdown_10ms > 0) {
            if (--gRTTECountdown_10ms == 0) {
                //if (gCurrentFunction != FUNCTION_FOREGROUND)
                    FUNCTION_Select(FUNCTION_FOREGROUND);

                gUpdateStatus  = true;
                gUpdateDisplay = true;
            }
        }
    }

#ifdef ENABLE_FMRADIO
    if (gFmRadioMode && gFM_RestoreCountdown_10ms > 0) {
        if (--gFM_RestoreCountdown_10ms == 0) { 
            FM_Start(); // switch back to FM radio mode
            GUI_SelectNextDisplay(DISPLAY_FM);
        }
    }
#endif

    CheckKeys();
}

void cancelUserInputModes(void)
{
    if (gWasFKeyPressed || gKeyInputCountdown > 0 || gInputBoxIndex > 0)
    {
        HideFKeyIcon();

        gInputBoxIndex      = 0;
        gKeyInputCountdown  = 0;
        gUpdateDisplay      = true;
    }
}

// this is called once every 500ms
void APP_TimeSlice500ms(void)
{
    gNextTimeslice_500ms = false;
    bool exit_menu = false;

    // Skipped authentic device check

    if (gKeypadLocked > 0)
        if (--gKeypadLocked == 0)
            gUpdateDisplay = true;

    if (gKeyInputCountdown > 0)
    {
        if (--gKeyInputCountdown == 0)
        {

            if (IS_MR_CHANNEL(gTxVfo->CHANNEL_SAVE) && (gInputBoxIndex > 0 && gInputBoxIndex < 4) && (!gFmRadioMode))
            {
                channelMoveSwitch();
            }

            cancelUserInputModes();
            gHasVfoBackup = false;
        }
    }

    if (gMenuCountdown > 0)
        if (--gMenuCountdown == 0)
            exit_menu = (gScreenToDisplay == DISPLAY_MENU); // exit menu mode

    // Skipped authentic device check

#ifdef ENABLE_FMRADIO
    if (gFmRadioCountdown_500ms > 0)
    {
        gFmRadioCountdown_500ms--;
        if (gFmRadioMode)           // 1of11
            return;
    }
#endif

    const int m = UI_MENU_GetCurrentMenuId();

    if (gBacklightCountdown_500ms > 0 && !gAskToSave && !gCssBackgroundScan
        // don't turn off backlight if user is in backlight menu option
        && !(gScreenToDisplay == DISPLAY_MENU && (m == MENU_ABR || m == MENU_ABR_MAX || m == MENU_ABR_MIN))
        && --gBacklightCountdown_500ms == 0
        && gEeprom.BACKLIGHT_TIME < 61
    ) {
        BACKLIGHT_TurnOff();
    }

    if (gReducedService)
    {
        BOARD_ADC_GetBatteryInfo(&gBatteryCurrentVoltage, &gBatteryCurrent);

        if (gBatteryCurrent > 500 || gBatteryCalibration[3] < gBatteryCurrentVoltage)
        {
            #ifdef ENABLE_OVERLAY
                overlay_FLASH_RebootToBootloader();
            #else
                NVIC_SystemReset();
            #endif
        }

        return;
    }

    gBatteryCheckCounter++;

    // Skipped authentic device check

    if (gCurrentFunction != FUNCTION_TRANSMIT)
    {

        if ((gBatteryCheckCounter & 1) == 0)
        {
            BOARD_ADC_GetBatteryInfo(&gBatteryVoltages[gBatteryVoltageIndex++], &gBatteryCurrent);
            if (gBatteryVoltageIndex > 3)
                gBatteryVoltageIndex = 0;
            BATTERY_GetReadings(true);
        }
    }

    // regular display updates (once every 2 sec) - if need be
    if ((gBatteryCheckCounter & 3) == 0)
    {
        if (gChargingWithTypeC || gSetting_battery_text > 0)
            gUpdateStatus = true;
    }

    if (!gCssBackgroundScan
#ifdef ENABLE_FMRADIO
        && (gFM_ScanState == FM_SCAN_OFF || gAskToSave)
#endif
    ) {
        if (gEeprom.AUTO_KEYPAD_LOCK && gKeyLockCountdown > 0
            && gScreenToDisplay != DISPLAY_MENU && --gKeyLockCountdown == 0)
        {
            gEeprom.KEY_LOCK = true;     // lock the keyboard
            gUpdateStatus = true;            // lock symbol needs showing
        }

        if (exit_menu) {
            gMenuCountdown = 0;

            const int m = UI_MENU_GetCurrentMenuId();

            if (gScreenToDisplay == DISPLAY_MENU && (m == MENU_ABR || m == MENU_ABR_MAX || m == MENU_ABR_MIN)) {
                BACKLIGHT_TurnOn();
            }
            HideFKeyIcon();
            gInputBoxIndex   = 0;

            gAskToSave       = false;
            gAskToDelete     = false;

            gUpdateDisplay   = true;

            GUI_DisplayType_t disp = DISPLAY_INVALID;

#ifdef ENABLE_FMRADIO
            if (gFmRadioMode && ! FUNCTION_IsRx()) {
                disp = DISPLAY_FM;
            }
#endif

            if (disp == DISPLAY_INVALID
            ) {
                disp = DISPLAY_MAIN;
            }

            if (disp != DISPLAY_INVALID) {
                GUI_SelectNextDisplay(disp);
            }
        }
    }

    if (!gPttIsPressed && gVFOStateResumeCountdown_500ms > 0 && --gVFOStateResumeCountdown_500ms == 0) {
            RADIO_SetVfoState(VFO_STATE_NORMAL);
#ifdef ENABLE_FMRADIO
        // Только если RestoreCountdown уже не ждёт — иначе двойной BK1080_Init вешает I2C
        if (gFmRadioMode && !FUNCTION_IsRx() && gFM_RestoreCountdown_10ms == 0) {
            FM_Start();
            GUI_SelectNextDisplay(DISPLAY_FM);
        }
#endif
    }

    BATTERY_TimeSlice500ms();
    UI_MAIN_TimeSlice500ms();

}

static void ProcessKey(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld)
{

    if (Key == KEY_EXIT && !BACKLIGHT_IsOn() && gEeprom.BACKLIGHT_TIME > 0)
    {   // just turn the light on for now so the user can see what's what
        BACKLIGHT_TurnOn();
        return;
    }

    if (gCurrentFunction == FUNCTION_POWER_SAVE)
        FUNCTION_Select(FUNCTION_FOREGROUND);

    gBatterySaveCountdown_10ms = battery_save_count_10ms;

    if (gEeprom.AUTO_KEYPAD_LOCK)
        gKeyLockCountdown = gEeprom.AUTO_KEYPAD_LOCK * 30;     // 15 seconds step

    if (!bKeyPressed) { // key released
        if (flagSaveVfo) {
            SETTINGS_SaveVfoIndices();
            flagSaveVfo = false;
        }

        if (flagSaveSettings) {
            SETTINGS_SaveSettings();
            flagSaveSettings = false;
        }

#ifdef ENABLE_FMRADIO
        if (gFlagSaveFM) {
            SETTINGS_SaveFM();
            gFlagSaveFM = false;
        }
#endif

        if (flagSaveChannel) {
            SETTINGS_SaveChannel(gTxVfo->CHANNEL_SAVE, gEeprom.TX_VFO, gTxVfo, flagSaveChannel);
            flagSaveChannel = false;

            if (gVfoConfigureMode == VFO_CONFIGURE_NONE)
                // gVfoConfigureMode is so as we don't wipe out previously setting this variable elsewhere
                gVfoConfigureMode = VFO_CONFIGURE;
        }
    }
    else { // key pressed or held
        const int m = UI_MENU_GetCurrentMenuId();
        if  (   //not when PTT and the backlight shouldn't turn on on TX
                !(Key == KEY_PTT && !(gSetting_backlight_on_tx_rx & BACKLIGHT_ON_TR_TX))
                // not in the backlight menu
                && !(gScreenToDisplay == DISPLAY_MENU && ( m == MENU_ABR || m == MENU_ABR_MAX || m == MENU_ABR_MIN))
            )
        {
            BACKLIGHT_TurnOn();
        }

        if (Key == KEY_EXIT && bKeyHeld) { // exit key held pressed
            // cancel user input
            cancelUserInputModes();

            if (gMonitor)
                ACTION_Monitor(); //turn off the monitor
        }

        if (gScreenToDisplay == DISPLAY_MENU)       // 1of11
            gMenuCountdown = menu_timeout_500ms;

    }

    bool lowBatPopup = gLowBattery && !gLowBatteryConfirmed &&  gScreenToDisplay == DISPLAY_MAIN;

#ifdef ENABLE_FEAT_F4HWN // Disable PTT if KEY_LOCK
    bool lck_condition = false;

    if(gSetting_set_lck)
        lck_condition = (gEeprom.KEY_LOCK || lowBatPopup) && gCurrentFunction != FUNCTION_TRANSMIT;
    else
        lck_condition = (gEeprom.KEY_LOCK || lowBatPopup) && gCurrentFunction != FUNCTION_TRANSMIT && Key != KEY_PTT;

    if (lck_condition)
#else
    if ((gEeprom.KEY_LOCK || lowBatPopup) && gCurrentFunction != FUNCTION_TRANSMIT && Key != KEY_PTT)
#endif
    {   // keyboard is locked or low battery popup

        // close low battery popup
        if(Key == KEY_EXIT && bKeyPressed && lowBatPopup) {
            gLowBatteryConfirmed = true;
            gUpdateDisplay = true;
            return;
        }

        if (Key == KEY_F) { // function/key-lock key
            if (!bKeyPressed)
                return;

            if (!bKeyHeld) { // keypad is locked, tell the user
                gKeypadLocked  = 4;      // 2 seconds
                gUpdateDisplay = true;
                return;
            }
        }
        // KEY_MENU has a special treatment here, because we want to pass hold event to ACTION_Handle
        // but we don't want it to complain when initial press happens
        // we want to react on realese instead
        else if (Key != KEY_SIDE1 && Key != KEY_SIDE2 &&        // pass side buttons
                 !(Key == KEY_MENU && bKeyHeld)) // pass KEY_MENU held
        {
            if ((!bKeyPressed || bKeyHeld || (Key == KEY_MENU && bKeyPressed)) && // prevent released or held, prevent KEY_MENU pressed
                !(Key == KEY_MENU && !bKeyPressed))  // pass KEY_MENU released
                return;

            // keypad is locked, tell the user
            gKeypadLocked  = 4;          // 2 seconds
            gUpdateDisplay = true;
            return;
        }
    }

    bool bFlag = false;
    if (Key == KEY_PTT) {
        if (gPttWasPressed) {
            bFlag = bKeyHeld;
            if (!bKeyPressed) {
                bFlag          = true;
                gPttWasPressed = false;
            }
        }
    }
    else if (gPttWasReleased) {
        if (bKeyHeld)
            bFlag = true;
        if (!bKeyPressed) {
            bFlag           = true;
            gPttWasReleased = false;
        }
    }

#ifdef ENABLE_FEAT_F4HWN // For F + SIDE1 or F + SIDE2
    if (gWasFKeyPressed && (Key == KEY_PTT || Key == KEY_EXIT)) { 
#else
    if (gWasFKeyPressed && (Key == KEY_PTT || Key == KEY_EXIT || Key == KEY_SIDE1 || Key == KEY_SIDE2)) { 
#endif
        // cancel the F-key
        HideFKeyIcon();
    }

    if (bFlag) {
        goto Skip;
    }

    if (gCurrentFunction == FUNCTION_TRANSMIT) {
        {
            // PTT key always handled; other keys during TX do nothing
            if (Key == KEY_PTT) {
                GENERIC_Key_PTT(bKeyPressed);
            }
            goto Skip;
        }
    }
    else if (gScreenToDisplay != DISPLAY_INVALID && (
            (Key != KEY_SIDE1 && Key != KEY_SIDE2)
#ifdef ENABLE_FEAT_F4HWN // For F + SIDE1 or F + SIDE2
            || (gWasFKeyPressed && (Key == KEY_SIDE1 || Key == KEY_SIDE2))
#endif
    )) {
        if (Key == KEY_EXIT && !bKeyHeld && bKeyPressed ) {
            gRequestDisplayScreen = DISPLAY_MAIN;
        } else {
            ProcessKeysFunctions[gScreenToDisplay](Key, bKeyPressed, bKeyHeld);
        }
    }
    else ACTION_Handle(Key, bKeyPressed, bKeyHeld);
   


Skip:
    if (gFlagAcceptSetting) {
        gMenuCountdown = menu_timeout_500ms;

        MENU_AcceptSetting();

        gFlagRefreshSetting = true;
        gFlagAcceptSetting  = false;
    }

    if (gRequestSaveSettings) {
        if (!bKeyHeld)
            SETTINGS_SaveSettings();
        else
            flagSaveSettings = 1;
        gRequestSaveSettings = false;
        gUpdateStatus        = true;
    }

#ifdef ENABLE_FMRADIO
    if (gRequestSaveFM) {
        gRequestSaveFM = false;
        if (!bKeyHeld)
            SETTINGS_SaveFM();
        else
            gFlagSaveFM = true;
    }
#endif

    if (gRequestSaveVFO) {
        gRequestSaveVFO = false;
        if (!bKeyHeld)
            SETTINGS_SaveVfoIndices();
        else
            flagSaveVfo = true;
    }

    if (gRequestSaveChannel > 0) { // TODO: remove the gRequestSaveChannel, why use global variable for that??
        if ((!bKeyHeld && !bKeyPressed) || (UI_MENU_GetCurrentMenuId() != 0 && gScreenToDisplay == DISPLAY_MENU) )
        {
            SETTINGS_SaveChannel(gTxVfo->CHANNEL_SAVE, gEeprom.TX_VFO, gTxVfo, gRequestSaveChannel);

            if (gVfoConfigureMode == VFO_CONFIGURE_NONE)
                // gVfoConfigureMode is so as we don't wipe out previously setting this variable elsewhere
                gVfoConfigureMode = VFO_CONFIGURE;
        }
        else { // this is probably so settings are not saved when up/down button is held and save is postponed to btn release
            flagSaveChannel = gRequestSaveChannel;

            if (gRequestDisplayScreen == DISPLAY_INVALID)
                gRequestDisplayScreen = DISPLAY_MAIN;
        }

        gRequestSaveChannel = 0;
    }

    if (gVfoConfigureMode != VFO_CONFIGURE_NONE) {
        if (gFlagResetVfos) {
            RADIO_ConfigureChannel(0, gVfoConfigureMode);
            RADIO_ConfigureChannel(1, gVfoConfigureMode);
        }
        else
            RADIO_ConfigureChannel(gEeprom.TX_VFO, gVfoConfigureMode);

        if (gRequestDisplayScreen == DISPLAY_INVALID)
            gRequestDisplayScreen = DISPLAY_MAIN;

        gFlagReconfigureVfos = true;
        gVfoConfigureMode    = VFO_CONFIGURE_NONE;
        gFlagResetVfos       = false;
    }

    if (gFlagReconfigureVfos) {
        RADIO_SelectVfos();

        RADIO_SetupRegisters(true);

        gVFO_RSSI_bar_level[0]      = 0;
        gVFO_RSSI_bar_level[1]      = 0;

        gFlagReconfigureVfos        = false;

        if (gMonitor)
            ACTION_Monitor();   // 1of11
    }

    if (gFlagRefreshSetting) {
        gFlagRefreshSetting = false;
        gMenuCountdown      = menu_timeout_500ms;

        MENU_ShowCurrentSetting();
    }

    if (gFlagPrepareTX) {
        RADIO_PrepareTX();
        gFlagPrepareTX = false;
    }

    GUI_SelectNextDisplay(gRequestDisplayScreen);
    gRequestDisplayScreen = DISPLAY_INVALID;

    gUpdateDisplay = true;
}
