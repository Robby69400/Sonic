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


// we come here every 10ms (100Hz tick)
void SysTick_Handler(void)
{
    static uint8_t  cnt_500ms   = 50;
    static uint8_t  cnt_200ms   = 20;
    static uint8_t  cnt_150ms   = 15;
    static uint8_t  cnt_1000ms  = 100;
    static uint16_t cnt_scan_led = 500;
    static uint8_t  cnt_470ms   = 47;
    static uint16_t cnt_autoptt = 0;

    gGlobalSysTickCounter++;
    
    gNextTimeslice = true;
    gNextTimeslice_10ms = true;

    // 200ms task (listening)
    if (--cnt_200ms == 0) {
        cnt_200ms = 20;
        gNextTimeslice_listening = true;
    }

    // 150ms task (display refresh trigger)
    if (--cnt_150ms == 0) {
        cnt_150ms = 15;
        gNextTimeslice_display = true;
    }

    // 470ms task (Monitor check)
    if (--cnt_470ms == 0) {
        cnt_470ms = 47;
        gNextTimeslice_Monitor = true;
    }

    // 1000ms task (HTimes)
    if (--cnt_1000ms == 0) {
        cnt_1000ms = 100;
        gNextTimeslice_HTimeS = true;
    }

    // 5000ms scan led beacon
    if (--cnt_scan_led == 0) {
        cnt_scan_led = 500;
        gNextTimeslice_SCAN_LED = true;
    } else if (cnt_scan_led == 470) {
        gNextTimeslice_SCAN_LED_OFF = true;
    }

    // AutoPTT tick down-counter (avoiding division with variable time)
    if (gAutoPtt_Time > 0) {
        if (++cnt_autoptt >= (uint16_t)(gAutoPtt_Time * 100)) {
            cnt_autoptt = 0;
            gNextTimeslice_AutoPtt = true;
        }
    }
#ifdef ENABLE_FEAT_F4HWN
        DECREMENT_AND_TRIGGER(gVfoSaveCountdown_10ms, gScheduleVfoSave);
#endif
    // 500ms tasks (ShowNames, history, timers)
    if (--cnt_500ms == 0) {
        cnt_500ms = 50;
        gNextTimeslice_ShowNames = true;
        gNextTimeslice_history = true;
        gNextTimeslice_500ms = true;
#ifdef ENABLE_FEAT_F4HWN
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

    DECREMENT_AND_TRIGGER(gTailNoteEliminationCountdown_10ms, gFlagTailNoteEliminationComplete);


#ifdef ENABLE_FMRADIO
    if (gFM_ScanState != FM_SCAN_OFF && gCurrentFunction != FUNCTION_MONITOR)
        if (gCurrentFunction != FUNCTION_TRANSMIT && gCurrentFunction != FUNCTION_RECEIVE)
            DECREMENT_AND_TRIGGER(gFmPlayCountdown_10ms, gScheduleFM);
#endif


    DECREMENT(boot_counter_10ms);
}
