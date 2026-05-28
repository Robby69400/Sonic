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

#include "scheduler.h"
#ifdef ENABLE_FMRADIO
    #include "app/fm.h"
#endif

#include "functions.h"
#include "helper/battery.h"
#include "misc.h"
#include "settings.h"

#include "driver/backlight.h"
#include "driver/gpio.h"

#define DECREMENT(cnt) \
    do {               \
        if (cnt > 0)   \
            cnt--;     \
    } while (0)

#define DECREMENT_AND_TRIGGER(cnt, flag) \
    do {                                 \
        if (cnt > 0)                     \
            if (--cnt == 0)              \
                flag = true;             \
    } while (0)

static volatile uint32_t gGlobalSysTickCounter;

// we come here every 10ms
void SysTick_Handler(void)
{
    gGlobalSysTickCounter++;
    
    gNextTimeslice = true;
    gNextTimeslice_10ms = true;
    if ((gGlobalSysTickCounter % 50) == 0)  {gNextTimeslice_ShowNames = true;} 
    if ((gGlobalSysTickCounter % 20) == 0)  {gNextTimeslice_listening = true;} 
    if ((gGlobalSysTickCounter % 15) == 0)  {gNextTimeslice_display = true;}
    if ((gGlobalSysTickCounter % 100) == 0) {gNextTimeslice_HTimeS = true;}
    if ((gGlobalSysTickCounter % 50) == 0)  {gNextTimeslice_history = true;}
    if ((gGlobalSysTickCounter % 47) == 0)  {gNextTimeslice_Monitor = true;}
    if ((gGlobalSysTickCounter % (gAutoPtt_Time*100)) == 0) {gNextTimeslice_AutoPtt = true;}
    if ((gGlobalSysTickCounter % 50) == 0)  {
        gNextTimeslice_500ms = true;
#ifdef ENABLE_FEAT_F4HWN
        DECREMENT_AND_TRIGGER(gVfoSaveCountdown_10ms, gScheduleVfoSave);
        DECREMENT_AND_TRIGGER(gTxTimerCountdownAlert_500ms - ALERT_TOT * 2, gTxTimeoutReachedAlert);
        #ifdef ENABLE_FEAT_F4HWN_RX_TX_TIMER
            DECREMENT(gRxTimerCountdown_500ms);
        #endif
#endif
        
        DECREMENT_AND_TRIGGER(gTxTimerCountdown_500ms, gTxTimeoutReached);
        DECREMENT(gSerialConfigCountDown_500ms);
    }

    if ((gGlobalSysTickCounter & 3) == 0)
        gNextTimeslice40ms = true;


    DECREMENT(gFoundCDCSSCountdown_10ms);

    DECREMENT(gFoundCTCSSCountdown_10ms);

    if (gCurrentFunction == FUNCTION_FOREGROUND)
        DECREMENT_AND_TRIGGER(gBatterySaveCountdown_10ms, gSchedulePowerSave);

    if (gCurrentFunction == FUNCTION_POWER_SAVE)
        DECREMENT_AND_TRIGGER(gPowerSave_10ms, gPowerSaveCountdownExpired);

    if (!gCssBackgroundScan && gEeprom.DUAL_WATCH != DUAL_WATCH_OFF)
        if (gCurrentFunction != FUNCTION_MONITOR && gCurrentFunction != FUNCTION_TRANSMIT && gCurrentFunction != FUNCTION_RECEIVE)
            DECREMENT_AND_TRIGGER(gDualWatchCountdown_10ms, gScheduleDualWatch);


    
    DECREMENT_AND_TRIGGER(gTailNoteEliminationCountdown_10ms, gFlagTailNoteEliminationComplete);


#ifdef ENABLE_FMRADIO
    if (gFM_ScanState != FM_SCAN_OFF && gCurrentFunction != FUNCTION_MONITOR)
        if (gCurrentFunction != FUNCTION_TRANSMIT && gCurrentFunction != FUNCTION_RECEIVE)
            DECREMENT_AND_TRIGGER(gFmPlayCountdown_10ms, gScheduleFM);
#endif


    DECREMENT(boot_counter_10ms);
}
