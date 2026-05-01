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

#ifndef AUDIO_H
#define AUDIO_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"

enum BEEP_Type_t
{
    BEEP_NONE = 0,
    BEEP_1KHZ_60MS_OPTIONAL,
    BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL,
    BEEP_440HZ_500MS,
    BEEP_500HZ_60MS_DOUBLE_BEEP,
#ifdef ENABLE_FEAT_F4HWN
    BEEP_400HZ_30MS,
    BEEP_500HZ_30MS,
    BEEP_600HZ_30MS,
#endif
    BEEP_880HZ_60MS_DOUBLE_BEEP
};

typedef enum BEEP_Type_t BEEP_Type_t;

extern BEEP_Type_t       gBeepToPlay;

void AUDIO_PlayBeep(BEEP_Type_t Beep);

#define AUDIO_AudioPathOn() GPIO_EnableAudioPath()

#define AUDIO_AudioPathOff() GPIO_DisableAudioPath()


#endif
