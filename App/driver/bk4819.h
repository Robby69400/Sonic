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

#ifndef DRIVER_BK4819_h
#define DRIVER_BK4819_h

#include <stdbool.h>
#include <stdint.h>
#include <radio.h>

#include "driver/bk4819-regs.h"

enum BK4819_AF_Type_t
{
    BK4819_AF_MUTE      =  0u,  //
    BK4819_AF_FM        =  1u,  // FM
    BK4819_AF_ALAM      =  2u,  //
    BK4819_AF_BEEP      =  3u,  //
    BK4819_AF_BASEBAND1 =  4u,  // RAW
    BK4819_AF_BASEBAND2 =  5u,  // USB
    BK4819_AF_CTCO      =  6u,  // strange LF audio .. maybe the CTCSS LF line ?
    BK4819_AF_AM        =  7u,  // AM
    BK4819_AF_FSKO      =  8u,  // nothing
    BK4819_AF_UNKNOWN3  =  9u,  // BYP
    BK4819_AF_UNKNOWN4  = 10u,  // nothing at all
    BK4819_AF_UNKNOWN5  = 11u,  // distorted
    BK4819_AF_UNKNOWN6  = 12u,  // distorted
    BK4819_AF_UNKNOWN7  = 13u,  // interesting
    BK4819_AF_UNKNOWN8  = 14u,  // interesting
    BK4819_AF_UNKNOWN9  = 15u   // not a lot
};

typedef enum BK4819_AF_Type_t BK4819_AF_Type_t;

enum BK4819_FilterBandwidth_t
{
    BK4819_FILTER_BW_WIDE = 0,
    BK4819_FILTER_BW_NARROW,
    BK4819_FILTER_BW_NARROWER,
    BK4819_FILTER_BW_AM
};

typedef enum BK4819_FilterBandwidth_t BK4819_FilterBandwidth_t;

enum BK4819_CssScanResult_t
{
    BK4819_CSS_RESULT_NOT_FOUND = 0,
    BK4819_CSS_RESULT_CTCSS,
    BK4819_CSS_RESULT_CDCSS
};

typedef enum BK4819_CssScanResult_t BK4819_CssScanResult_t;

// radio is asleep, not listening
extern bool gRxIdleMode;
//extern uint16_t regs_cache[128];
extern uint16_t reg_30_cache;
extern uint16_t reg_47_cache;
extern uint16_t reg_4A_cache;
extern uint16_t reg_7E_cache;

void     BK4819_Init(void);
uint16_t BK4819_ReadRegister(BK4819_REGISTER_t Register);
void     BK4819_WriteRegister(BK4819_REGISTER_t Register, uint16_t Data);
void     BK4819_SetRegValue(RegisterSpec s, uint16_t v);

void     BK4819_SetAGC(bool enable);
void BK4819_InitAGC(ModulationMode_t modulation);

void     BK4819_ToggleGpioOut(BK4819_GPIO_PIN_t Pin, bool bSet);

void     BK4819_SetCDCSSCodeWord(uint32_t CodeWord);
void     BK4819_SetCTCSSFrequency(uint32_t BaudRate);
void     BK4819_SetTailDetection(const uint32_t freq_10Hz);
void     BK4819_SetFilterBandwidth(const BK4819_FilterBandwidth_t Bandwidth, const bool weak_no_different);
void     BK4819_SetupPowerAmplifier(const uint8_t bias, const uint32_t frequency);
void     BK4819_SetFrequency(uint32_t Frequency);
void     BK4819_SetupSquelch(
            uint8_t SquelchOpenRSSIThresh,
            uint8_t SquelchCloseRSSIThresh,
            uint8_t SquelchOpenNoiseThresh,
            uint8_t SquelchCloseNoiseThresh,
            uint8_t SquelchCloseGlitchThresh,
            uint8_t SquelchOpenGlitchThresh);

void     BK4819_SetAF(BK4819_AF_Type_t AF);
void     BK4819_RX_TurnOn(void);
void     BK4819_PickRXFilterPathBasedOnFrequency(uint32_t Frequency);
bool     BK4819_CompanderEnabled(void);
void     BK4819_SetCompander(const unsigned int mode);
void     BK4819_PlayTone(uint16_t Frequency, bool bTuningGainSwitch);
void     BK4819_PlaySingleTone(const unsigned int tone_Hz, const unsigned int delay, const unsigned int level, const bool play_speaker);
void     BK4819_EnterTxMute(void);
void     BK4819_ExitTxMute(void);
void     BK4819_Sleep(void);
void     BK4819_TurnsOffTones_TurnsOnRX(void);
void     BK4819_ResetFSK(void);
void     BK4819_Idle(void);
void     BK4819_ExitBypass(void);
void     BK4819_PrepareTransmit(void);
void     BK4819_TxOn_Beep(void);
void     BK4819_ExitSubAu(void);
void     BK4819_Conditional_RX_TurnOn_and_GPIO6_Enable(void);
void     BK4819_EnableTXLink(void);
void     BK4819_GenTail(uint8_t Tail);
void     BK4819_PlayCDCSSTail(void);
void     BK4819_PlayCTCSSTail(void);
uint16_t BK4819_GetRSSI(void);
int8_t   BK4819_GetRxGain_dB(void);
int16_t  BK4819_GetRSSI_dBm(void);
uint8_t  BK4819_GetGlitchIndicator(void);
uint8_t  BK4819_GetExNoiceIndicator(void);
uint16_t BK4819_GetVoiceAmplitudeOut(void);
uint8_t  BK4819_GetAfTxRx(void);
bool     BK4819_GetFrequencyScanResult(uint32_t *pFrequency);
BK4819_CssScanResult_t BK4819_GetCxCSSScanResult(uint32_t *pCdcssFreq, uint16_t *pCtcssFreq);
void     BK4819_DisableFrequencyScan(void);
void     BK4819_EnableFrequencyScan(void);
void     BK4819_SetScanFrequency(uint32_t Frequency);

void     BK4819_Disable(void);

uint8_t  BK4819_GetCDCSSCodeType(void);
uint8_t  BK4819_GetCTCShift(void);
uint8_t  BK4819_GetCTCType(void);



void     BK4819_PlayRoger(uint8_t song);

void     BK4819_Enable_AfDac_DiscMode_TxDsp(void);

#endif
