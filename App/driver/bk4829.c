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

#include <stdint.h>
#include <stdio.h>

#include "settings.h"
#include "misc.h"

#include "audio.h"

#include "driver/bk4819.h"
#include "driver/gpio.h"
#include "driver/system.h"
#include "driver/systick.h"

#define PIN_CSN GPIO_MAKE_PIN(GPIOF, LL_GPIO_PIN_9)
#define PIN_SCL GPIO_MAKE_PIN(GPIOB, LL_GPIO_PIN_8)
#define PIN_SDA GPIO_MAKE_PIN(GPIOB, LL_GPIO_PIN_9)

static uint16_t gBK4819_GpioOutState;

bool gRxIdleMode;

static inline void CS_Assert()
{
    GPIO_ResetOutputPin(PIN_CSN);
}

static inline void CS_Release()
{
    GPIO_SetOutputPin(PIN_CSN);
}

static inline void SCL_Reset()
{
    GPIO_ResetOutputPin(PIN_SCL);
}

static inline void SCL_Set()
{
    GPIO_SetOutputPin(PIN_SCL);
}

static inline void SDA_Reset()
{
    GPIO_ResetOutputPin(PIN_SDA);
}

static inline void SDA_Set()
{
    GPIO_SetOutputPin(PIN_SDA);
}

static inline void SDA_SetDir(bool Output)
{
    LL_GPIO_SetPinMode(GPIO_PORT(PIN_SDA), GPIO_PIN_MASK(PIN_SDA), Output ? LL_GPIO_MODE_OUTPUT : LL_GPIO_MODE_INPUT);
}

static inline uint32_t SDA_ReadInput()
{
    return GPIO_IsInputPinSet(PIN_SDA) ? 1 : 0;
}

static inline uint16_t scale_freq(const uint16_t freq)
{
//  return (((uint32_t)freq * 1032444u) + 50000u) / 100000u;   // with rounding
    return (((uint32_t)freq * 1353245u) + (1u << 16)) >> 17;   // with rounding
}

void BK4819_Init(void)
{
    CS_Release();
    SCL_Set();
    SDA_Set();

    BK4819_WriteRegister(BK4819_REG_00, 0x8000);
    BK4819_WriteRegister(BK4819_REG_00, 0x0000);

    BK4819_WriteRegister(BK4819_REG_37, 0x9D1F);
    BK4819_WriteRegister(BK4819_REG_36, 0x0022);
    BK4819_WriteRegister(BK4819_REG_10, 0x0318);
    BK4819_WriteRegister(BK4819_REG_11, 0x033A);
    BK4819_WriteRegister(BK4819_REG_12, 0x03DB);
    BK4819_WriteRegister(BK4819_REG_13, 0x03DF);
    BK4819_WriteRegister(BK4819_REG_14, 0x0210);
    BK4819_WriteRegister(BK4819_REG_49, 0x2AB2);
    BK4819_WriteRegister(BK4819_REG_7B, 0x8420);

    // BK4819_WriteRegister(BK4819_REG_19, 0b0001000001000001);   // <15> MIC AGC  1 = disable  0 = enable

    BK4819_WriteRegister(BK4819_REG_7D, 0xE920);

    // REG_48 .. RX AF level
    //
    // <15:12> 11  ???  0 to 15
    //
    // <11:10> 0 AF Rx Gain-1
    //         0 =   0dB
    //         1 =  -6dB
    //         2 = -12dB
    //         3 = -18dB
    //
    // <9:4>   60 AF Rx Gain-2  -26dB ~ 5.5dB   0.5dB/step
    //         63 = max
    //          0 = mute
    //
    // <3:0>   15 AF DAC Gain (after Gain-1 and Gain-2) approx 2dB/step
    //         15 = max
    //          0 = min
    //
    BK4819_WriteRegister(BK4819_REG_48, //  0xB3A8);     // 1011 00 111010 1000
        // (11u << 12) |     // ??? 0..15
        // ( 0u << 10) |     // AF Rx Gain-1
        // (58u <<  4) |     // AF Rx Gain-2
        // ( 8u <<  0));     // AF DAC Gain (after Gain-1 and Gain-2)
        0x33A8);

    BK4819_WriteRegister(0x40, 0x3516);
    BK4819_WriteRegister(0x1C, 0x07C0);
    BK4819_WriteRegister(0x1D, 0xE555);
    BK4819_WriteRegister(0x1E, 0x4C58);

    BK4819_WriteRegister(BK4819_REG_1F, 0xC65A);
    BK4819_WriteRegister(BK4819_REG_3E, 0x94C6);

    BK4819_WriteRegister(0x73, 0x4691);
    BK4819_WriteRegister(0x77, 0x88EF);
    BK4819_WriteRegister(BK4819_REG_19, 0x1041);
    BK4819_WriteRegister(BK4819_REG_28, 0x0B40);
    BK4819_WriteRegister(BK4819_REG_29, 0xAA00);
    BK4819_WriteRegister(0x2A, 0x6600);
    BK4819_WriteRegister(0x2C, 0x1822);
    BK4819_WriteRegister(0x2F, 0x9890);
    BK4819_WriteRegister(0x53, 0x2028);
    BK4819_WriteRegister(BK4819_REG_7E, 0x303E);
    BK4819_WriteRegister(BK4819_REG_46, 0x600A);
    BK4819_WriteRegister(0x4A, 0x5430);
    BK4819_WriteRegister(BK4819_REG_07, 0x61CE);

    gBK4819_GpioOutState = 0x9000;

    BK4819_WriteRegister(BK4819_REG_33, 0x9000);
    BK4819_WriteRegister(BK4819_REG_3F, 0);
}

static uint16_t BK4819_ReadU16(void)
{
    unsigned int i;
    uint16_t     Value;

    SDA_SetDir(false);
    SYSTICK_DelayUs(1);
    Value = 0;
    for (i = 0; i < 16; i++)
    {
        Value <<= 1;
        Value |= SDA_ReadInput();
        SCL_Set();
        SYSTICK_DelayUs(1);
        SCL_Reset();
        SYSTICK_DelayUs(1);
    }
    SDA_SetDir(true);

    return Value;
}

uint16_t regs_cache[128] = {[0 ... 127] = 0xFFFF};

uint16_t BK4819_ReadRegister(BK4819_REGISTER_t Register)
{
    uint16_t Value;

    CS_Release();
    SCL_Reset();

    SYSTICK_DelayUs(1);

    CS_Assert();
    BK4819_WriteU8(Register | 0x80);
    Value = BK4819_ReadU16();
    CS_Release();

    SYSTICK_DelayUs(1);

    SCL_Set();
    SDA_Set();
	regs_cache[Register] = Value;
    return Value;
}

void BK4819_WriteRegister(BK4819_REGISTER_t Register, uint16_t Data)
{
    if(Data == regs_cache[Register])return;
	regs_cache[Register] = Data;
    CS_Release();
    SCL_Reset();

    SYSTICK_DelayUs(1);

    CS_Assert();
    BK4819_WriteU8(Register);

    SYSTICK_DelayUs(1);

    BK4819_WriteU16(Data);

    SYSTICK_DelayUs(1);

    CS_Release();

    SYSTICK_DelayUs(1);

    SCL_Set();
    SDA_Set();
}

void BK4819_WriteU8(uint8_t Data)
{
    unsigned int i;

    SCL_Reset();
    for (i = 0; i < 8; i++)
    {
        if ((Data & 0x80) == 0)
            SDA_Reset();
        else
            SDA_Set();

        SYSTICK_DelayUs(1);
        SCL_Set();
        SYSTICK_DelayUs(1);

        Data <<= 1;

        SCL_Reset();
        SYSTICK_DelayUs(1);
    }
}

void BK4819_WriteU16(uint16_t Data)
{
    unsigned int i;

    SCL_Reset();
    for (i = 0; i < 16; i++)
    {
        if ((Data & 0x8000) == 0)
            SDA_Reset();
        else
            SDA_Set();

        SYSTICK_DelayUs(1);
        SCL_Set();

        Data <<= 1;

        SYSTICK_DelayUs(1);
        SCL_Reset();
        SYSTICK_DelayUs(1);
    }
}

void BK4819_SetAGC(bool enable)
{
    uint16_t regVal = BK4819_ReadRegister(BK4819_REG_7E);
    if(!(regVal & (1 << 15)) == enable)
        return;

    BK4819_WriteRegister(BK4819_REG_7E, (regVal & ~(1 << 15) & ~(0b111 << 12))
        | (!enable << 15)   // 0  AGC fix mode
        | (3u << 12)       // 3  AGC fix index
    );

    // if(enable) {
    //  BK4819_WriteRegister(BK4819_REG_7B, 0x8420);
    // }
    // else {
    //  BK4819_WriteRegister(BK4819_REG_7B, 0x318C);

    //  BK4819_WriteRegister(BK4819_REG_7C, 0x595E);
    //  BK4819_WriteRegister(BK4819_REG_20, 0x8DEF);

    //  for (uint8_t i = 0; i < 8; i++) {
    //      //BK4819_WriteRegister(BK4819_REG_06, ((i << 13) | 0x2500u) + 0x036u);
    //      BK4819_WriteRegister(BK4819_REG_06, (i & 7) << 13 | 0x4A << 7 | 0x36);
    //  }
    // }
}

void BK4819_InitAGC(ModulationMode_t modulation)
{
    BK4819_WriteRegister(BK4819_REG_10, 0x0318);
    BK4819_WriteRegister(BK4819_REG_11, 0x033A);
    BK4819_WriteRegister(BK4819_REG_12, 0x03DB);
    BK4819_WriteRegister(BK4819_REG_13, 0x03DF);
    BK4819_WriteRegister(BK4819_REG_14, 0x0210);

    if (modulation == MODULATION_AM) {
        // AM: узкое окно AGC (high=50, low=20) - меньше hunting, нет эффекта вертолёта
        // Особенно критично для авиадиапазона (AM DSB)
        BK4819_WriteRegister(BK4819_REG_49, (0u << 14) | (50u << 7) | (20u << 0));
        BK4819_WriteRegister(BK4819_REG_7B, 0x8420); // стабильный AGC для AM
    } else {
        // FM/USB: узкое окно как SU75 (high=84,low=66 = 18dB) — AGC дольше держит максимальное усиление
        BK4819_WriteRegister(BK4819_REG_49, (0u << 14) | (84u << 7) | (66u << 0));
        BK4819_WriteRegister(BK4819_REG_7B, 0x8420); // единое значение как в SU75
    }
}

int8_t BK4819_GetRxGain_dB(void)
{
    union {
        struct {
            uint16_t pga:3;
            uint16_t mixer:2;
            uint16_t lna:3;
            uint16_t lnaS:2;
        };
        uint16_t __raw;
    } agcGainReg;

    union {
        struct {
            uint16_t _ : 5;
            uint16_t agcSigStrength : 7;
            int16_t gainIdx : 3;
            uint16_t agcEnab : 1;
        };
        uint16_t __raw;
    } reg7e;

    reg7e.__raw = BK4819_ReadRegister(BK4819_REG_7E);
    uint8_t gainAddr = reg7e.gainIdx < 0 ? BK4819_REG_14 : BK4819_REG_10 + reg7e.gainIdx;
    agcGainReg.__raw = BK4819_ReadRegister(gainAddr);
    int8_t lnaShortTab[] = {-28, -24, -19, 0};
    int8_t lnaTab[] = {-24, -19, -14, -9, -6, -4, -2, 0};
    int8_t mixerTab[] = {-8, -6, -3, 0};
    int8_t pgaTab[] = {-33, -27, -21, -15, -9, -6, -3, 0};
    return lnaShortTab[agcGainReg.lnaS] + lnaTab[agcGainReg.lna] + mixerTab[agcGainReg.mixer] + pgaTab[agcGainReg.pga];
}

int16_t BK4819_GetRSSI_dBm(void)
{
    uint16_t rssi = BK4819_GetRSSI();
    return (rssi / 2) - 160;// - BK4819_GetRxGain_dB();
}

void BK4819_ToggleGpioOut(BK4819_GPIO_PIN_t Pin, bool bSet)
{
    if (bSet)
        gBK4819_GpioOutState |=  (0x40u >> Pin);
    else
        gBK4819_GpioOutState &= ~(0x40u >> Pin);

    BK4819_WriteRegister(BK4819_REG_33, gBK4819_GpioOutState);
}

void BK4819_SetCDCSSCodeWord(uint32_t CodeWord)
{
    // REG_51
    //
    // <15>  0
    //       1 = Enable TxCTCSS/CDCSS
    //       0 = Disable
    //
    // <14>  0
    //       1 = GPIO0Input for CDCSS
    //       0 = Normal Mode (for BK4819 v3)
    //
    // <13>  0
    //       1 = Transmit negative CDCSS code
    //       0 = Transmit positive CDCSS code
    //
    // <12>  0 CTCSS/CDCSS mode selection
    //       1 = CTCSS
    //       0 = CDCSS
    //
    // <11>  0 CDCSS 24/23bit selection
    //       1 = 24bit
    //       0 = 23bit
    //
    // <10>  0 1050HzDetectionMode
    //       1 = 1050/4 Detect Enable, CTC1 should be set to 1050/4 Hz
    //
    // <9>   0 Auto CDCSS Bw Mode
    //       1 = Disable
    //       0 = Enable
    //
    // <8>   0 Auto CTCSS Bw Mode
    //       0 = Enable
    //       1 = Disable
    //
    // <6:0> 0 CTCSS/CDCSS Tx Gain1 Tuning
    //       0   = min
    //       127 = max

    // Enable CDCSS
    // Transmit positive CDCSS code
    // CDCSS Mode
    // CDCSS 23bit
    // Enable Auto CDCSS Bw Mode
    // Enable Auto CTCSS Bw Mode
    // CTCSS/CDCSS Tx Gain1 Tuning = 51
    //
    BK4819_WriteRegister(BK4819_REG_51, 0xA033);
        // BK4819_REG_51_ENABLE_CxCSS         |
        // BK4819_REG_51_GPIO6_PIN2_NORMAL    |
        // BK4819_REG_51_TX_CDCSS_POSITIVE    |
        // BK4819_REG_51_MODE_CDCSS           |
        // BK4819_REG_51_CDCSS_23_BIT         |
        // BK4819_REG_51_1050HZ_NO_DETECTION  |
        // BK4819_REG_51_AUTO_CDCSS_BW_ENABLE |
        // BK4819_REG_51_AUTO_CTCSS_BW_ENABLE |
        // (51u << BK4819_REG_51_SHIFT_CxCSS_TX_GAIN1));

    // REG_07 <15:0>
    //
    // When <13> = 0 for CTC1
    // <12:0> = CTC1 frequency control word =
    //                          freq(Hz) * 20.64888 for XTAL 13M/26M or
    //                          freq(Hz) * 20.97152 for XTAL 12.8M/19.2M/25.6M/38.4M
    //
    // When <13> = 1 for CTC2 (Tail 55Hz Rx detection)
    // <12:0> = CTC2 (should below 100Hz) frequency control word =
    //                          25391 / freq(Hz) for XTAL 13M/26M or
    //                          25000 / freq(Hz) for XTAL 12.8M/19.2M/25.6M/38.4M
    //
    // When <13> = 2 for CDCSS 134.4Hz
    // <12:0> = CDCSS baud rate frequency (134.4Hz) control word =
    //                          freq(Hz) * 20.64888 for XTAL 13M/26M or
    //                          freq(Hz) * 20.97152 for XTAL 12.8M/19.2M/25.6M/38.4M
    //
    BK4819_WriteRegister(BK4819_REG_07, BK4819_REG_07_MODE_CTC1 | 2775u);

    // REG_08 <15:0> <15> = 1 for CDCSS high 12bit
    //               <15> = 0 for CDCSS low  12bit
    // <11:0> = CDCSShigh/low 12bit code
    //
    BK4819_WriteRegister(BK4819_REG_08, (0u << 15) | ((CodeWord >>  0) & 0x0FFF)); // LS 12-bits
    BK4819_WriteRegister(BK4819_REG_08, (1u << 15) | ((CodeWord >> 12) & 0x0FFF)); // MS 12-bits
}

void BK4819_SetCTCSSFrequency(uint32_t FreqControlWord)
{
    // REG_51 <15>  0                                 1 = Enable TxCTCSS/CDCSS           0 = Disable
    // REG_51 <14>  0                                 1 = GPIO0Input for CDCSS           0 = Normal Mode.(for BK4819v3)
    // REG_51 <13>  0                                 1 = Transmit negative CDCSS code   0 = Transmit positive CDCSScode
    // REG_51 <12>  0 CTCSS/CDCSS mode selection      1 = CTCSS                          0 = CDCSS
    // REG_51 <11>  0 CDCSS 24/23bit selection        1 = 24bit                          0 = 23bit
    // REG_51 <10>  0 1050HzDetectionMode             1 = 1050/4 Detect Enable, CTC1 should be set to 1050/4 Hz
    // REG_51 <9>   0 Auto CDCSS Bw Mode              1 = Disable                        0 = Enable.
    // REG_51 <8>   0 Auto CTCSS Bw Mode              0 = Enable                         1 = Disable
    // REG_51 <6:0> 0 CTCSS/CDCSS Tx Gain1 Tuning     0 = min                            127 = max

    uint16_t Config;
    if (FreqControlWord == 2625)
    {   // Enables 1050Hz detection mode
        // Enable TxCTCSS
        // CTCSS Mode
        // 1050/4 Detect Enable
        // Enable Auto CDCSS Bw Mode
        // Enable Auto CTCSS Bw Mode
        // CTCSS/CDCSS Tx Gain1 Tuning = 74
        //
        Config = 0x9440;   // 1 0 0 1 0 1 0 0 0 1001010
    }
    else
    {   // Enable TxCTCSS
        // CTCSS Mode
        // Enable Auto CDCSS Bw Mode
        // Enable Auto CTCSS Bw Mode
        // CTCSS/CDCSS Tx Gain1 Tuning = 74
        //
        Config = 0x9040;   // 1 0 0 1 0 0 0 0 0 1001010
    }
    BK4819_WriteRegister(BK4819_REG_51, Config);

    // REG_07 <15:0>
    //
    // When <13> = 0 for CTC1
    // <12:0> = CTC1 frequency control word =
    //                          freq(Hz) * 20.64888 for XTAL 13M/26M or
    //                          freq(Hz) * 20.97152 for XTAL 12.8M/19.2M/25.6M/38.4M
    //
    // When <13> = 1 for CTC2 (Tail RX detection)
    // <12:0> = CTC2 (should below 100Hz) frequency control word =
    //                          25391 / freq(Hz) for XTAL 13M/26M or
    //                          25000 / freq(Hz) for XTAL 12.8M/19.2M/25.6M/38.4M
    //
    // When <13> = 2 for CDCSS 134.4Hz
    // <12:0> = CDCSS baud rate frequency (134.4Hz) control word =
    //                          freq(Hz) * 20.64888 for XTAL 13M/26M or
    //                          freq(Hz) * 20.97152 for XTAL 12.8M/19.2M/25.6M/38.4M
    //
    BK4819_WriteRegister(BK4819_REG_07, BK4819_REG_07_MODE_CTC1 | (((FreqControlWord * 206488u) + 50000u) / 100000u));   // with rounding
}

// freq_10Hz is CTCSS Hz * 10
void BK4819_SetTailDetection(const uint32_t freq_10Hz)
{
    // REG_07 <15:0>
    //
    // When <13> = 0 for CTC1
    // <12:0> = CTC1 frequency control word =
    //                          freq(Hz) * 20.64888 for XTAL 13M/26M or
    //                          freq(Hz) * 20.97152 for XTAL 12.8M/19.2M/25.6M/38.4M
    //
    // When <13> = 1 for CTC2 (Tail RX detection)
    // <12:0> = CTC2 (should below 100Hz) frequency control word =
    //                          25391 / freq(Hz) for XTAL 13M/26M or
    //                          25000 / freq(Hz) for XTAL 12.8M/19.2M/25.6M/38.4M
    //
    // When <13> = 2 for CDCSS 134.4Hz
    // <12:0> = CDCSS baud rate frequency (134.4Hz) control word =
    //                          freq(Hz) * 20.64888 for XTAL 13M/26M or
    //                          freq(Hz) * 20.97152 for XTAL 12.8M/19.2M/25.6M/38.4M
    //
    BK4819_WriteRegister(BK4819_REG_07, BK4819_REG_07_MODE_CTC2 | ((253910 + (freq_10Hz / 2)) / freq_10Hz));  // with rounding
}

void BK4819_SetFilterBandwidth(const BK4819_FilterBandwidth_t Bandwidth, const bool weak_no_different)
{
    (void)weak_no_different;

    // REG_43 on BK4829, per BK4829 Registers Table.
    //
    // <14:12> RF filter bandwidth (Apass = 0.1dB)
    //         000 = 2.0 kHz
    //         001 = 2.5 kHz
    //         010 = 3.0 kHz
    //         011 = 3.5 kHz
    //         100 = 4.0 kHz
    //         101 = 4.5 kHz
    //         110 = 5.0 kHz
    //         111 = 5.5 kHz
    // if REG_43<5> == 1, RF filter bandwidth *= 2
    //
    // <11:9> RF filter bandwidth when signal is weak (Apass = 0.1dB)
    //         000 = 2.0 kHz
    //         001 = 2.5 kHz
    //         010 = 3.0 kHz
    //         011 = 3.5 kHz
    //         100 = 4.0 kHz
    //         101 = 4.5 kHz
    //         110 = 5.0 kHz
    //         111 = 5.5 kHz
    // if REG_43<5> == 1, RF filter bandwidth *= 2
    //
    // <8:6> AF Tx LPF2 filter bandwidth (Apass = 1dB)
    //         100 = 5.5 kHz
    //         101 = 5.0 kHz
    //         110 = 4.5 kHz
    //         111 = 4.0 kHz
    //         000 = 3.0 kHz
    //         001 = 2.5 kHz
    //         010 = 2.75 kHz
    //         011 = 3.5 kHz
    //
    // <5:4> BW mode selection
    //         00 = 12.5k
    //         01 =  6.25k
    //         10 = 25k/20k
    //
    // <2> gain after FM demodulation
    //         0 = 0 dB
    //         1 = 6 dB
    //
    // <3> and <1:0> remain undocumented here.

    uint16_t val = 0;
    switch (Bandwidth)
    {
        case BK4819_FILTER_BW_WIDE: // 25kHz
            // 0x3028 = (0b011 << 12) | (0b000 << 9) | (0b000 << 6) |
            //          (0b10 << 4)  | (1 << 3)
            //        = RF 3.5 kHz and weak-RF 2.0 kHz, both doubled because
            //          bit<5> is set in 25k/20k mode => RF 7.0 kHz, weak-RF
            //          4.0 kHz, AF Tx LPF2 3.0 kHz, FM gain 0 dB.
            val = 0x3028;
            break;

        case BK4819_FILTER_BW_NARROW:   // 12.5kHz
            // 0x4048 = (0b100 << 12) | (0b000 << 9) | (0b001 << 6) |
            //          (0b00 << 4)  | (1 << 3)
            //        = RF 4.0 kHz, weak-RF 2.0 kHz, AF Tx LPF2 2.5 kHz,
            //          12.5k mode, FM gain 0 dB.
            val = 0x4048;
            break;

        case BK4819_FILTER_BW_NARROWER: // 6.25kHz
            // 0x205C = (0b010 << 12) | (0b000 << 9) | (0b001 << 6) |
            //          (0b01 << 4)  | (1 << 3)     | (1 << 2)
            //        = RF 3.0 kHz, weak-RF 2.0 kHz, AF Tx LPF2 2.5 kHz,
            //          6.25k mode, FM gain +6 dB.
            val = 0x205C;
            break;

        case BK4819_FILTER_BW_AM:   // Stock AM preset
            // 0x345C = (0b011 << 12) | (0b010 << 9) | (0b001 << 6) |
            //          (0b01 << 4)  | (1 << 3)     | (1 << 2)
            //        = RF 3.5 kHz, weak-RF 3.0 kHz, AF Tx LPF2 2.5 kHz,
            //          6.25k mode, bit<2> set. The "8.33kHz AM" label used in
            //          the codebase is functional/empirical; it does not come
            //          directly from the BW mode bits of REG_43.
            val = 0x345C;
            break;

        default:
            val = 0x5C;
            break;
    }

    BK4819_WriteRegister(BK4819_REG_43, val);
}

void BK4819_SetupPowerAmplifier(const uint8_t bias, const uint32_t frequency)
{
    // REG_36 <15:8> 0 PA Bias output 0 ~ 3.2V
    //               255 = 3.2V
    //                 0 = 0V
    //
    // REG_36 <7>    0
    //               1 = Enable PA-CTL output
    //               0 = Disable (Output 0 V)
    //
    // REG_36 <5:3>  7 PA gain 1 tuning
    //               7 = max
    //               0 = min
    //
    // REG_36 <2:0>  7 PA gain 2 tuning
    //               7 = max
    //               0 = min
    //
    //                                  280MHz       g1=1  g2=0 (-14.9dBm),  g1=4  g2=2 (0.13dBm)
    const uint8_t gain   = (frequency < 28000000) ? // (1u << 3) | (0u << 0) : (4u << 3) | (2u << 0);
                                                    0x8 : 0x22;
    const uint8_t enable = 1;
    BK4819_WriteRegister(BK4819_REG_36, (bias << 8) | (enable << 7) | (gain << 0));
}

void BK4819_SetFrequency(uint32_t Frequency)
{
    BK4819_WriteRegister(BK4819_REG_38, (Frequency >>  0) & 0xFFFF);
    BK4819_WriteRegister(BK4819_REG_39, (Frequency >> 16) & 0xFFFF);
}

void BK4819_SetupSquelch(
        uint8_t SquelchOpenRSSIThresh,
        uint8_t SquelchCloseRSSIThresh,
        uint8_t SquelchOpenNoiseThresh,
        uint8_t SquelchCloseNoiseThresh,
        uint8_t SquelchCloseGlitchThresh,
        uint8_t SquelchOpenGlitchThresh)
{
    // REG_70
    //
    // <15>   0 Enable TONE1
    //        1 = Enable
    //        0 = Disable
    //
    // <14:8> 0 TONE1 tuning gain
    //        0 ~ 127
    //
    // <7>    0 Enable TONE2
    //        1 = Enable
    //        0 = Disable
    //
    // <6:0>  0 TONE2/FSK tuning gain
    //        0 ~ 127
    //
    BK4819_WriteRegister(BK4819_REG_70, 0);

    // Glitch threshold for Squelch = close
    //
    // 0 ~ 255
    //
    BK4819_WriteRegister(BK4819_REG_4D, 0xA000 | SquelchCloseGlitchThresh);

    // REG_4E
    //
    // <15:14> 1 ???
    //
    // <13:11> 5 Squelch = open  Delay Setting
    //         0 ~ 7
    //
    // <10:9>  3 Squelch = close Delay Setting
    //         0 ~ 3
    //
    // <8>     1 ???
    //
    // <7:0>   8 Glitch threshold for Squelch = open
    //         0 ~ 255
    //
    BK4819_WriteRegister(BK4819_REG_4E,
    (1u << 14) |                  // 1 ???
    (5u << 11) |                  // 5 squelch = open  delay .. 0 ~ 7
    (3u <<  9) |                  // 3 squelch = close delay .. 0 ~ 3
    (1u <<  8) |                  // 1 ??? (matches stock BK4829)
    SquelchOpenGlitchThresh);     // 0 ~ 255


    // REG_4F
    //
    // <14:8> 47 Ex-noise threshold for Squelch = close
    //        0 ~ 127
    //
    // <7>    ???
    //
    // <6:0>  46 Ex-noise threshold for Squelch = open
    //        0 ~ 127
    //
    BK4819_WriteRegister(BK4819_REG_4F, ((uint16_t)SquelchCloseNoiseThresh << 8) | SquelchOpenNoiseThresh);

    // REG_78
    //
    // <15:8> 72 RSSI threshold for Squelch = open    0.5dB/step
    //
    // <7:0>  70 RSSI threshold for Squelch = close   0.5dB/step
    //
    BK4819_WriteRegister(BK4819_REG_78, ((uint16_t)SquelchOpenRSSIThresh   << 8) | SquelchCloseRSSIThresh);

    BK4819_SetAF(BK4819_AF_MUTE);

    BK4819_RX_TurnOn();
}

void BK4819_SetAF(BK4819_AF_Type_t AF)
{
    // AF Output Inverse Mode = Inverse
    // Undocumented bits 0x2040
    //
    BK4819_WriteRegister(BK4819_REG_47, 0x6042 | (AF << 8));
    // BK4819_WriteRegister(BK4819_REG_47, (6u << 12) | (AF << 8) | (1u << 6));
}

void BK4819_SetRegValue(RegisterSpec s, uint16_t v) {
  uint16_t reg = BK4819_ReadRegister(s.num);
  reg &= ~(s.mask << s.offset);
  BK4819_WriteRegister(s.num, reg | (v << s.offset));
}

void BK4819_RX_TurnOn(void)
{
    // DSP Voltage Setting = 1
    // ANA LDO = 2.7v
    // VCO LDO = 2.7v
    // RF LDO  = 2.7v
    // PLL LDO = 2.7v
    // ANA LDO bypass
    // VCO LDO bypass
    // RF LDO  bypass
    // PLL LDO bypass
    // Reserved bit is 1 instead of 0
    // Enable  DSP
    // Enable  XTAL
    // Enable  Band Gap
    //
    BK4819_WriteRegister(BK4819_REG_37, 0x9F1F);  // 0001111100001111

    // Turn off everything
    BK4819_WriteRegister(BK4819_REG_30, 0);


    BK4819_WriteRegister(BK4819_REG_30, 0xBFF1);
        // BK4819_REG_30_ENABLE_VCO_CALIB |
        // BK4819_REG_30_DISABLE_UNKNOWN |
        // BK4819_REG_30_ENABLE_RX_LINK |
        // BK4819_REG_30_ENABLE_AF_DAC |
        // BK4819_REG_30_ENABLE_DISC_MODE |
        // BK4819_REG_30_ENABLE_PLL_VCO |
        // BK4819_REG_30_DISABLE_PA_GAIN |
        // BK4819_REG_30_DISABLE_MIC_ADC |
        // BK4819_REG_30_DISABLE_TX_DSP |
        // BK4819_REG_30_ENABLE_RX_DSP );
}

void BK4819_PickRXFilterPathBasedOnFrequency(uint32_t Frequency)
{
    if (Frequency < 28000000)
    {   // VHF
        BK4819_ToggleGpioOut(BK4819_GPIO4_PIN32_VHF_LNA, true);
        BK4819_ToggleGpioOut(BK4819_GPIO3_PIN31_UHF_LNA, false);
    }
    else
    if (Frequency == 0xFFFFFFFF)
    {   // OFF
        BK4819_ToggleGpioOut(BK4819_GPIO4_PIN32_VHF_LNA, false);
        BK4819_ToggleGpioOut(BK4819_GPIO3_PIN31_UHF_LNA, false);
    }
    else
    {   // UHF
        BK4819_ToggleGpioOut(BK4819_GPIO4_PIN32_VHF_LNA, false);
        BK4819_ToggleGpioOut(BK4819_GPIO3_PIN31_UHF_LNA, true);
    }
}

bool BK4819_CompanderEnabled(void)
{
    return (BK4819_ReadRegister(BK4819_REG_31) & (1u << 3)) ? true : false;
}

void BK4819_SetCompander(const unsigned int mode)
{
    // mode 0 .. OFF
    // mode 1 .. TX
    // mode 2 .. RX
    // mode 3 .. TX and RX

    const uint16_t r31 = BK4819_ReadRegister(BK4819_REG_31);

    if (mode == 0)
    {   // disable
        BK4819_WriteRegister(BK4819_REG_31, r31 & ~(1u << 3));
        return;
    }

    // REG_29
    //
    // <15:14> 10 Compress (AF Tx) Ratio
    //         00 = Disable
    //         01 = 1.333:1
    //         10 = 2:1
    //         11 = 4:1
    //
    // <13:7>  86 Compress (AF Tx) 0 dB point (dB)
    //
    // <6:0>   64 Compress (AF Tx) noise point (dB)
    //
    const uint16_t compress_ratio    = (mode == 1 || mode >= 3) ? 2 : 0;  // 2:1
    const uint16_t compress_0dB      = 86;
    const uint16_t compress_noise_dB = 64;
//  AB40  10 1010110 1000000
    BK4819_WriteRegister(BK4819_REG_29, // (BK4819_ReadRegister(BK4819_REG_29) & ~(3u << 14)) | (compress_ratio << 14));
        (compress_ratio    << 14) |
        (compress_0dB      <<  7) |
        (compress_noise_dB <<  0));

    // REG_28
    //
    // <15:14> 01 Expander (AF Rx) Ratio
    //         00 = Disable
    //         01 = 1:2
    //         10 = 1:3
    //         11 = 1:4
    //
    // <13:7>  86 Expander (AF Rx) 0 dB point (dB)
    //
    // <6:0>   56 Expander (AF Rx) noise point (dB)
    //
    const uint16_t expand_ratio    = (mode >= 2) ? 1 : 0;   // 1:2
    const uint16_t expand_0dB      = 86;
    const uint16_t expand_noise_dB = 56;
//  6B38  01 1010110 0111000
    BK4819_WriteRegister(BK4819_REG_28, // (BK4819_ReadRegister(BK4819_REG_28) & ~(3u << 14)) | (expand_ratio << 14));
        (expand_ratio    << 14) |
        (expand_0dB      <<  7) |
        (expand_noise_dB <<  0));

    // enable
    BK4819_WriteRegister(BK4819_REG_31, r31 | (1u << 3));
}

void BK4819_PlayTone(uint16_t Frequency, bool bTuningGainSwitch)
{
    uint16_t ToneConfig = BK4819_REG_70_ENABLE_TONE1;

    BK4819_EnterTxMute();
    BK4819_SetAF(BK4819_AF_BEEP);

    if (bTuningGainSwitch == 0)
        ToneConfig |=  96u << BK4819_REG_70_SHIFT_TONE1_TUNING_GAIN;
    else
        ToneConfig |= 28u << BK4819_REG_70_SHIFT_TONE1_TUNING_GAIN;
    BK4819_WriteRegister(BK4819_REG_70, ToneConfig);

    BK4819_WriteRegister(BK4819_REG_30, 0);
    BK4819_WriteRegister(BK4819_REG_30, BK4819_REG_30_ENABLE_AF_DAC | BK4819_REG_30_ENABLE_DISC_MODE | BK4819_REG_30_ENABLE_TX_DSP);

    BK4819_WriteRegister(BK4819_REG_71, scale_freq(Frequency));
}

// level 0 ~ 127
void BK4819_PlaySingleTone(const unsigned int tone_Hz, const unsigned int delay, const unsigned int level, const bool play_speaker)
{
    BK4819_EnterTxMute();

    if (play_speaker)
    {
        AUDIO_AudioPathOn();
        BK4819_SetAF(BK4819_AF_BEEP);
    }
    else
        BK4819_SetAF(BK4819_AF_MUTE);


    BK4819_WriteRegister(BK4819_REG_70, BK4819_REG_70_ENABLE_TONE1 | ((level & 0x7f) << BK4819_REG_70_SHIFT_TONE1_TUNING_GAIN));

    BK4819_EnableTXLink();
    SYSTEM_DelayMs(50);

    BK4819_WriteRegister(BK4819_REG_71, scale_freq(tone_Hz));

    BK4819_ExitTxMute();
    SYSTEM_DelayMs(delay);
    BK4819_EnterTxMute();

    if (play_speaker)
    {
        AUDIO_AudioPathOff();
        BK4819_SetAF(BK4819_AF_MUTE);
    }

    BK4819_WriteRegister(BK4819_REG_70, 0x0000);
    BK4819_WriteRegister(BK4819_REG_30, 0xC1FE);
    BK4819_ExitTxMute();
}

void BK4819_EnterTxMute(void)
{
    BK4819_WriteRegister(BK4819_REG_50, 0xBB18);
}

void BK4819_ExitTxMute(void)
{
    BK4819_WriteRegister(BK4819_REG_50, 0x3B18);
}

void BK4819_Sleep(void)
{
    BK4819_WriteRegister(BK4819_REG_30, 0);
    BK4819_WriteRegister(BK4819_REG_37, 0x1D00);
}

void BK4819_TurnsOffTones_TurnsOnRX(void)
{
    BK4819_WriteRegister(BK4819_REG_70, 0);
    BK4819_SetAF(BK4819_AF_MUTE);

    BK4819_ExitTxMute();

    BK4819_WriteRegister(BK4819_REG_30, 0);
    BK4819_WriteRegister(BK4819_REG_30,
        BK4819_REG_30_ENABLE_VCO_CALIB |
        BK4819_REG_30_ENABLE_RX_LINK   |
        BK4819_REG_30_ENABLE_AF_DAC    |
        BK4819_REG_30_ENABLE_DISC_MODE |
        BK4819_REG_30_ENABLE_PLL_VCO   |
        BK4819_REG_30_ENABLE_RX_DSP);
}

#ifdef ENABLE_AIRCOPY
    void BK4819_SetupAircopy(void)
    {
        BK4819_WriteRegister(BK4819_REG_70, 0x00C3);    // Enable Tone2, tuning gain 48
        BK4819_WriteRegister(BK4819_REG_72, 0x3065);    // Tone2 baudrate 1200
        BK4819_WriteRegister(BK4819_REG_58, 0x00C1);    // FSK Enable, FSK 1.2K RX Bandwidth, Preamble 0xAA or 0x55, RX Gain 0, RX Mode
                                                        // (FSK1.2K, FSK2.4K Rx and NOAA SAME Rx), TX Mode FSK 1.2K and FSK 2.4K Tx
        BK4819_WriteRegister(BK4819_REG_5C, 0x5665);    // Enable CRC among other things we don't know yet
        BK4819_WriteRegister(BK4819_REG_5D, 0x4700);    // FSK Data Length 72 Bytes (0xabcd + 2 byte length + 64 byte payload + 2 byte CRC + 0xdcba)
        BK4819_WriteRegister(0x5E, 0x3204);
    }
#endif

void BK4819_ResetFSK(void)
{
    BK4819_WriteRegister(BK4819_REG_3F, 0x0000);        // Disable interrupts
    BK4819_WriteRegister(BK4819_REG_59, 0x0068);        // Sync length 4 bytes, 7 byte preamble

    SYSTEM_DelayMs(30);

    BK4819_Idle();
}

void BK4819_Idle(void)
{
    BK4819_WriteRegister(BK4819_REG_30, 0x0000);
}


void BK4819_ExitBypass(void)
{
    BK4819_SetAF(BK4819_AF_MUTE);


    // Keep existing REG_7E logic from your current implementation.
    uint16_t regVal = BK4819_ReadRegister(BK4819_REG_7E);
    BK4819_WriteRegister(BK4819_REG_7E, (regVal & ~(0b111 << 3)) | (5u << 3));
}

void BK4819_PrepareTransmit(void)
{
    BK4819_ExitBypass();
    BK4819_ExitTxMute();
    BK4819_TxOn_Beep();
}

void BK4819_TxOn_Beep(void)
{
    BK4819_WriteRegister(BK4819_REG_36, 0);
    BK4819_WriteRegister(BK4819_REG_37, 0x9D1F);
    BK4819_WriteRegister(BK4819_REG_52, 0x028F);
    BK4819_WriteRegister(BK4819_REG_30, 0x0000);
    BK4819_WriteRegister(BK4819_REG_30, 0xC1FE);
}

void BK4819_ExitSubAu(void)
{
    // REG_51
    //
    // <15>  0
    //       1 = Enable TxCTCSS/CDCSS
    //       0 = Disable
    //
    // <14>  0
    //       1 = GPIO0Input for CDCSS
    //       0 = Normal Mode (for BK4819 v3)
    //
    // <13>  0
    //       1 = Transmit negative CDCSS code
    //       0 = Transmit positive CDCSS code
    //
    // <12>  0 CTCSS/CDCSS mode selection
    //       1 = CTCSS
    //       0 = CDCSS
    //
    // <11>  0 CDCSS 24/23bit selection
    //       1 = 24bit
    //       0 = 23bit
    //
    // <10>  0 1050HzDetectionMode
    //       1 = 1050/4 Detect Enable, CTC1 should be set to 1050/4 Hz
    //
    // <9>   0 Auto CDCSS Bw Mode
    //       1 = Disable
    //       0 = Enable
    //
    // <8>   0 Auto CTCSS Bw Mode
    //       0 = Enable
    //       1 = Disable
    //
    // <6:0> 0 CTCSS/CDCSS Tx Gain1 Tuning
    //       0   = min
    //       127 = max
    //
    BK4819_WriteRegister(BK4819_REG_51, 0x0000);
}

void BK4819_Conditional_RX_TurnOn_and_GPIO6_Enable(void)
{
    if (gRxIdleMode)
    {
        BK4819_ToggleGpioOut(BK4819_GPIO0_PIN28_RX_ENABLE, true);
        BK4819_RX_TurnOn();
    }
}

void BK4819_EnableTXLink(void)
{
    BK4819_WriteRegister(BK4819_REG_30,
        BK4819_REG_30_ENABLE_VCO_CALIB |
        BK4819_REG_30_ENABLE_UNKNOWN   |
        BK4819_REG_30_DISABLE_RX_LINK  |
        BK4819_REG_30_ENABLE_AF_DAC    |
        BK4819_REG_30_ENABLE_DISC_MODE |
        BK4819_REG_30_ENABLE_PLL_VCO   |
        BK4819_REG_30_ENABLE_PA_GAIN   |
        BK4819_REG_30_DISABLE_MIC_ADC  |
        BK4819_REG_30_ENABLE_TX_DSP    |
        BK4819_REG_30_DISABLE_RX_DSP);
}

void BK4819_GenTail(uint8_t Tail)
{
    // REG_52
    //
    // <15>    0 Enable 120/180/240 degree shift CTCSS or 134.4Hz Tail when CDCSS mode
    //         0 = Normal
    //         1 = Enable
    //
    // <14:13> 0 CTCSS tail mode selection (only valid when REG_52 <15> = 1)
    //         00 = for 134.4Hz CTCSS Tail when CDCSS mode
    //         01 = CTCSS0 120° phase shift
    //         10 = CTCSS0 180° phase shift
    //         11 = CTCSS0 240° phase shift
    //
    // <12>    0 CTCSSDetectionThreshold Mode
    //         1 = ~0.1%
    //         0 =  0.1 Hz
    //
    // <11:6>  0x0A CTCSS found detect threshold
    //
    // <5:0>   0x0F CTCSS lost  detect threshold

    // REG_07 <15:0>
    //
    // When <13> = 0 for CTC1
    // <12:0> = CTC1 frequency control word =
    //                          freq(Hz) * 20.64888 for XTAL 13M/26M or
    //                          freq(Hz) * 20.97152 for XTAL 12.8M/19.2M/25.6M/38.4M
    //
    // When <13> = 1 for CTC2 (Tail 55Hz Rx detection)
    // <12:0> = CTC2 (should below 100Hz) frequency control word =
    //                          25391 / freq(Hz) for XTAL 13M/26M or
    //                          25000 / freq(Hz) for XTAL 12.8M/19.2M/25.6M/38.4M
    //
    // When <13> = 2 for CDCSS 134.4Hz
    // <12:0> = CDCSS baud rate frequency (134.4Hz) control word =
    //                          freq(Hz) * 20.64888 for XTAL 13M/26M or
    //                          freq(Hz)*20.97152 for XTAL 12.8M/19.2M/25.6M/38.4M

    switch (Tail)
    {
        case 0: // 134.4Hz CTCSS Tail
            BK4819_WriteRegister(BK4819_REG_52, 0x828F);   // 1 00 0 001010 001111
            break;
        case 1: // 120° phase shift
            BK4819_WriteRegister(BK4819_REG_52, 0xA28F);   // 1 01 0 001010 001111
            break;
        case 2: // 180° phase shift
            BK4819_WriteRegister(BK4819_REG_52, 0xC28F);   // 1 10 0 001010 001111
            break;
        case 3: // 240° phase shift
            BK4819_WriteRegister(BK4819_REG_52, 0xE28F);   // 1 11 0 001010 001111
            break;
        case 4: // 55Hz tone freq
            BK4819_WriteRegister(BK4819_REG_07, 0x046f);   // 0 00 0 010001 101111
            break;
    }
}

void BK4819_PlayCDCSSTail(void)
{
    BK4819_GenTail(0);     // CTC134
    BK4819_WriteRegister(BK4819_REG_51, 0x8040); // 1 0 0 0 0 0 0 0  0  1001010
}

void BK4819_PlayCTCSSTail(void)
{
        BK4819_GenTail(4);       // 55Hz tone freq

    // REG_51
    //
    // <15>  0
    //       1 = Enable TxCTCSS/CDCSS
    //       0 = Disable
    //
    // <14>  0
    //       1 = GPIO0Input for CDCSS
    //       0 = Normal Mode (for BK4819 v3)
    //
    // <13>  0
    //       1 = Transmit negative CDCSS code
    //       0 = Transmit positive CDCSS code
    //
    // <12>  0 CTCSS/CDCSS mode selection
    //       1 = CTCSS
    //       0 = CDCSS
    //
    // <11>  0 CDCSS 24/23bit selection
    //       1 = 24bit
    //       0 = 23bit
    //
    // <10>  0 1050HzDetectionMode
    //       1 = 1050/4 Detect Enable, CTC1 should be set to 1050/4 Hz
    //
    // <9>   0 Auto CDCSS Bw Mode
    //       1 = Disable
    //       0 = Enable
    //
    // <8>   0 Auto CTCSS Bw Mode
    //       0 = Enable
    //       1 = Disable
    //
    // <6:0> 0 CTCSS/CDCSS Tx Gain1 Tuning
    //       0   = min
    //       127 = max

    BK4819_WriteRegister(BK4819_REG_51, 0x9040); // 1 0 0 1 0 0 0 0  0  1001010
}

uint16_t BK4819_GetRSSI(void)
{
    return BK4819_ReadRegister(BK4819_REG_67) & 0x01FF;
}

uint8_t  BK4819_GetGlitchIndicator(void)
{
    return BK4819_ReadRegister(BK4819_REG_63) & 0x00FF;
}

uint8_t  BK4819_GetExNoiceIndicator(void)
{
    return BK4819_ReadRegister(BK4819_REG_65) & 0x007F;
}

uint16_t BK4819_GetVoiceAmplitudeOut(void)
{
    return BK4819_ReadRegister(BK4819_REG_64);
}

uint8_t BK4819_GetAfTxRx(void)
{
    return BK4819_ReadRegister(BK4819_REG_6F) & 0x003F;
}

bool BK4819_GetFrequencyScanResult(uint32_t *pFrequency)
{
    const uint16_t High     = BK4819_ReadRegister(BK4819_REG_0D);
    const bool     Finished = (High & 0x8000) == 0;
    if (Finished)
    {
        const uint16_t Low = BK4819_ReadRegister(BK4819_REG_0E);
        *pFrequency = (uint32_t)((High & 0x7FF) << 16) | Low;
    }
    return Finished;
}

BK4819_CssScanResult_t BK4819_GetCxCSSScanResult(uint32_t *pCdcssFreq, uint16_t *pCtcssFreq)
{
    uint16_t Low;
    uint16_t High = BK4819_ReadRegister(BK4819_REG_69);

    if ((High & 0x8000) == 0)
    {
        Low         = BK4819_ReadRegister(BK4819_REG_6A);
        *pCdcssFreq = ((High & 0xFFF) << 12) | (Low & 0xFFF);
        return BK4819_CSS_RESULT_CDCSS;
    }

    Low = BK4819_ReadRegister(BK4819_REG_68);

    if ((Low & 0x8000) == 0)
    {
        *pCtcssFreq = ((Low & 0x1FFF) * 4843) / 10000;
        return BK4819_CSS_RESULT_CTCSS;
    }

    return BK4819_CSS_RESULT_NOT_FOUND;
}

void BK4819_DisableFrequencyScan(void)
{
    // REG_32
    //
    // <15:14> 0 frequency scan time
    //         0 = 0.2 sec
    //         1 = 0.4 sec
    //         2 = 0.8 sec
    //         3 = 1.6 sec
    //
    // <13:1>  ???
    //
    // <0>     0 frequency scan enable
    //         1 = enable
    //         0 = disable
    //
    BK4819_WriteRegister(BK4819_REG_32, // 0x0244);    // 00 0000100100010 0
        (  0u << 14) |          // 0 frequency scan Time
        (290u <<  1) |          // ???
        (  0u <<  0));          // 0 frequency scan enable
}

void BK4819_EnableFrequencyScan(void)
{
    // REG_32
    //
    // <15:14> 0 frequency scan time
    //         0 = 0.2 sec
    //         1 = 0.4 sec
    //         2 = 0.8 sec
    //         3 = 1.6 sec
    //
    // <13:1>  ???
    //
    // <0>     0 frequency scan enable
    //         1 = enable
    //         0 = disable
    //
    BK4819_WriteRegister(BK4819_REG_32, // 0x0245);   // 00 0000100100010 1
        (  0u << 14) |          // 0 frequency scan time
        (290u <<  1) |          // ???
        (  1u <<  0));          // 1 frequency scan enable
}

void BK4819_SetScanFrequency(uint32_t Frequency)
{
    BK4819_SetFrequency(Frequency);

    // REG_51
    //
    // <15>  0
    //       1 = Enable TxCTCSS/CDCSS
    //       0 = Disable
    //
    // <14>  0
    //       1 = GPIO0Input for CDCSS
    //       0 = Normal Mode (for BK4819 v3)
    //
    // <13>  0
    //       1 = Transmit negative CDCSS code
    //       0 = Transmit positive CDCSS code
    //
    // <12>  0 CTCSS/CDCSS mode selection
    //       1 = CTCSS
    //       0 = CDCSS
    //
    // <11>  0 CDCSS 24/23bit selection
    //       1 = 24bit
    //       0 = 23bit
    //
    // <10>  0 1050HzDetectionMode
    //       1 = 1050/4 Detect Enable, CTC1 should be set to 1050/4 Hz
    //
    // <9>   0 Auto CDCSS Bw Mode
    //       1 = Disable
    //       0 = Enable
    //
    // <8>   0 Auto CTCSS Bw Mode
    //       0 = Enable
    //       1 = Disable
    //
    // <6:0> 0 CTCSS/CDCSS Tx Gain1 Tuning
    //       0   = min
    //       127 = max
    //
    BK4819_WriteRegister(BK4819_REG_51,
        BK4819_REG_51_DISABLE_CxCSS         |
        BK4819_REG_51_GPIO6_PIN2_NORMAL     |
        BK4819_REG_51_TX_CDCSS_POSITIVE     |
        BK4819_REG_51_MODE_CDCSS            |
        BK4819_REG_51_CDCSS_23_BIT          |
        BK4819_REG_51_1050HZ_NO_DETECTION   |
        BK4819_REG_51_AUTO_CDCSS_BW_DISABLE |
        BK4819_REG_51_AUTO_CTCSS_BW_DISABLE);

    BK4819_RX_TurnOn();
}

void BK4819_Disable(void)
{
    BK4819_WriteRegister(BK4819_REG_30, 0);
}

uint8_t BK4819_GetCDCSSCodeType(void)
{
    return (BK4819_ReadRegister(BK4819_REG_0C) >> 14) & 3u;
}

uint8_t BK4819_GetCTCShift(void)
{
    return (BK4819_ReadRegister(BK4819_REG_0C) >> 12) & 3u;
}

uint8_t BK4819_GetCTCType(void)
{
    return (BK4819_ReadRegister(BK4819_REG_0C) >> 10) & 3u;
}

//###########################################################################################

static void play_note(uint32_t freq, uint32_t duration) {
    BK4819_WriteRegister(BK4819_REG_71, scale_freq(freq));
    BK4819_ExitTxMute();
    SYSTEM_DelayMs(duration);
    BK4819_EnterTxMute();
}

//###########################################################################################

static void play_mario() {
    play_note(660, 100);
    play_note(660, 100);
    play_note(0, 100);
    play_note(660, 100);
    play_note(0, 100);
    play_note(523, 100);
    play_note(660, 100);
    play_note(0, 100);
    play_note(784, 100);
    play_note(0, 300);
    play_note(392, 100);
}

//###########################################################################################

static void play_ambulance() {
    play_note(960, 150); // Note haute (La)
    play_note(635, 150); // Note basse (Ré)
    play_note(960, 150); // Note haute (La)

}

//###########################################################################################

static void roger_r2d2(void){
    // R2-D2 Style Acknowledgment Beep
    play_note(1046, 50);  // C6
    play_note(1318, 50);  // E6
    play_note(1568, 70);  // G6
    play_note(0, 30);     // Micro-pause for chirp effect
    play_note(1760, 40);  // A6
    play_note(1568, 40);  // G6
    play_note(1318, 100); // E6 (ending the phrase)
} 

//###########################################################################################

static void Roger1(void)
{
    // motorola type
    play_note(1540, 50); 
    play_note(1540, 80); 
    play_note(1310, 80); 

}

//###########################################################################################

static void Roger2(void) {
    for (uint16_t i=2000;i>500;i-=50) play_note(i, 5); 
	for (uint16_t i=2000;i>500;i-=50) play_note(i, 5); 
	for (uint16_t i=500;i<2550;i+=50) play_note(i, 7);
}

//###########################################################################################

/* static void sonicBeep() {
    play_note(1320, 50);   // 880 Hz for 50 ms
    play_note(1569, 50);  // 1046 Hz
    play_note(1977, 50);  // 1318 Hz
    play_note(2352, 300);  // 1568 Hz, slightly longer
} */
//###########################################################################################

// OURO — глубокий протяжный гудок ~340 Гц (из OURO.mp3)
static void roger_beep_OURO(void) {
play_note(1100, 40);
    play_note(1300, 40);
    play_note(1500, 40);
    play_note(1800, 60);
}

// KLAC — механический клик-бёрст (из KLAC.mp3)
static void roger_beep_KLAC(void) {
    play_note(1800, 10);
    play_note(700,  10);
    play_note(1100, 10);
    play_note(400,  10);
    play_note(1300, 10);
    play_note(1200, 10);
    play_note(1600, 10);
    play_note(700,  10);
    play_note(0,    10);
    play_note(700,  10);
    play_note(1200, 10);
    play_note(400,  20);
    play_note(500,  10);
    play_note(0,   100);
    play_note(2000, 10);
    play_note(1100, 10);
    play_note(2900, 10);
}

//###########################################################################################

// PIU — лазер: восходящий свип 400→800→1600 Гц (из PIU.mp3)
void roger_beep_PIU(void) {
    play_note(400,  160);
    play_note(800,  100);
    play_note(1600,  90);
}

//###########################################################################################

// ICQ — классический аська "uh-oh" (из ICQ.mp3)
void roger_beep_ICQ(void) {
    play_note(1400,  40);
    play_note(0,     70);
    play_note(1300,  10);
    play_note(1100,  20);
    play_note(1200, 110);
    play_note(1100, 130);
}
//###########################################################################################

void BK4819_PlayRoger(uint8_t song)
{
	BK4819_EnterTxMute();
	BK4819_SetAF(BK4819_AF_MUTE);
    BK4819_WriteRegister(BK4819_REG_70, BK4819_REG_70_ENABLE_TONE1 | (66u << BK4819_REG_70_SHIFT_TONE1_TUNING_GAIN));
	BK4819_EnableTXLink();
	SYSTEM_DelayMs(50);
    switch (song)
    {
        case 1:	play_mario();	        break;
    	case 2: Roger2();		        break;
    	case 3: roger_r2d2();           break;
    	case 4:	Roger1();               break;
        case 5: play_ambulance();       break;
    	case 6: roger_beep_OURO();      break;  // OURO
    	case 7: roger_beep_KLAC();      break;  // KLAC
    	case 8: roger_beep_PIU();       break;  // PIU
    	case 9: roger_beep_ICQ();       break;  // ICQ
        default:                    	break;
    }
	BK4819_WriteRegister(BK4819_REG_70, 0x0000);
	BK4819_WriteRegister(BK4819_REG_30, 0xC1FE);   // 1 1 0000 0 1 1111 1 1 1 0
}

void BK4819_Enable_AfDac_DiscMode_TxDsp(void)
{
    BK4819_WriteRegister(BK4819_REG_30, 0x0000);
    BK4819_WriteRegister(BK4819_REG_30, 0x0302);
}
