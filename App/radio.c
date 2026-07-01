/* Copyright 2025 OURO.SU
 * https://github.com/igimalek
 */

#include "driver/bk4819-regs.h"
#include <string.h>



#ifdef ENABLE_FMRADIO
    #include "app/fm.h"
#endif

#include "dcs.h"
#include "driver/bk4819.h"
#include "driver/py25q16.h"
#include "driver/gpio.h"
#include "driver/system.h"
#include "frequencies.h"
#include "functions.h"
#include "helper/battery.h"
#include "misc.h"
#include "radio.h"
#include "settings.h"
#include "ui/menu.h"

VFO_Info_t    *gTxVfo;
VFO_Info_t    *gRxVfo;
VFO_Info_t    *gCurrentVfo;
DCS_CodeType_t gCurrentCodeType;
VfoState_t     VfoState[2];

const char gModulationStr[MODULATION_UKNOWN][4] = {
    [MODULATION_FM]="FM",
    [MODULATION_AM]="AM",
    [MODULATION_USB]="USB",

};

#ifdef ENABLE_FEAT_F4HWN_AUDIO

    static void AUDIO_ApplyFMProfile(uint8_t profile)
    {
        switch (profile)
        {
            default:
            case 0: // FLAT
                BK4819_WriteRegister(0x54, 0x9009);
                BK4819_WriteRegister(0x55, 0x3200);
                break;

            case 1: // CLEAN
                BK4819_WriteRegister(0x54, 0x9009);
                BK4819_WriteRegister(0x55, 0x33A9);
                break;

            case 2: // MID
                BK4819_WriteRegister(0x54, 0x9009);
                BK4819_WriteRegister(0x55, 0x3600);
                break;

            case 3: // BOOST
                BK4819_WriteRegister(0x54, 0x8546);
                BK4819_WriteRegister(0x55, 0x3AF0);
                break;

            case 4: // MAX
                BK4819_WriteRegister(0x54, 0x8566);
                BK4819_WriteRegister(0x55, 0x3D00);
                break;
        }
    }

    static void AUDIO_ApplyAMProfile(uint8_t profile)
    {
        switch (profile)
        {
            default:
            case 0: // SHARP (ALPHA test profile) - Narrow IF filter (REG54 bits[14:8]=0, bits[7:0]=9), low IF gain (REG55 bits[11:8]=1, ref=169)
                    // Selective and crisp, best adjacent channel rejection, may sound harsh on strong signals
                BK4819_WriteRegister(0x2b, 0x0300);
                BK4819_WriteRegister(0x2f, 0x9990);
                BK4819_WriteRegister(0x54, 0x9009);
                BK4819_WriteRegister(0x55, 0x31A9);
                break;
            case 1: // STOCK - Narrow IF filter (REG54 bits[14:8]=0, bits[7:0]=9), moderate IF gain (REG55 bits[11:8]=4, ref=180)
                    // Selective filter with balanced gain, punchy and detailed, good compromise between rejection and sensitivity
                BK4819_WriteRegister(0x2b, 0x0500);
                BK4819_WriteRegister(0x2f, 0x9990);
                BK4819_WriteRegister(0x54, 0x9009);
                BK4819_WriteRegister(0x55, 0x31A9);
                break;
            case 2: // OPEN (BRAVO test profile) - Medium-wide IF filter (REG54 bits[14:8]=8, bits[7:0]=70), high IF gain (REG55 bits[11:8]=8, ref=192)
                    // Wide and pleasant, better sensitivity on weak signals, may struggle with adjacent channel interference
                BK4819_WriteRegister(0x2b, 0x0300);
                BK4819_WriteRegister(0x2f, 0x9990);
                BK4819_WriteRegister(0x54, 0x8846);
                BK4819_WriteRegister(0x55, 0x38C0);
                break;
        }
    }

    static void AUDIO_ApplyUSBProfile(void)
    {
        BK4819_WriteRegister(0x54, 0x9009);
        BK4819_WriteRegister(0x55, 0x31A9);
    }
#endif

bool RADIO_CheckValidList(uint8_t scanList)
{
    if(scanList == MR_CHANNELS_LIST + 1)
        return true;

    for (uint16_t i = 0; IS_MR_CHANNEL(i); i++) {
        const ChannelAttributes_t* att = MR_GetChannelAttributes(i);
        if(att->scanlist == scanList && att->exclude == false)
        {
            return true;
        }
    }
    return false;
}

void RADIO_NextValidList(int8_t direction)
{
    uint8_t startList = gEeprom.SCAN_LIST_DEFAULT;
    uint8_t attempts = 0;
    const uint8_t MAX_VALUE = MR_CHANNELS_LIST + 1;  // 21 (1-20 lists + Monitor)
    
    do {
        if (direction > 0) {
            // Forward: 1 → 2 → ... → 25 → 1
            gEeprom.SCAN_LIST_DEFAULT = (gEeprom.SCAN_LIST_DEFAULT % MAX_VALUE) + 1;
        } else {
            // Backward: 25 → 24 → ... → 1 → 25
            gEeprom.SCAN_LIST_DEFAULT = ((gEeprom.SCAN_LIST_DEFAULT - 2 + MAX_VALUE) % MAX_VALUE) + 1;
        }
        attempts++;
        
        if (RADIO_CheckValidList(gEeprom.SCAN_LIST_DEFAULT))
            return;
            
    } while (gEeprom.SCAN_LIST_DEFAULT != startList && attempts < MAX_VALUE);
    
    // Safety fallback: switch to ALL mode
    if (!RADIO_CheckValidList(gEeprom.SCAN_LIST_DEFAULT)) {
        gEeprom.SCAN_LIST_DEFAULT = MAX_VALUE;  // ALL (25)
    }
}

bool RADIO_CheckValidChannel(uint16_t channel, bool checkScanList, uint8_t scanList)
{
    const ChannelAttributes_t* att = MR_GetChannelAttributes(channel);

    // return true if the channel appears valid
    if (!IS_MR_CHANNEL(channel))
        return false;
    if (checkScanList && att->exclude == true)
        return false;
    if (att->band > BAND7_470MHz)
        return false;
    if (!checkScanList || (scanList > MR_CHANNELS_LIST && att->scanlist != 0) || (scanList > 0 && att->scanlist == MR_CHANNELS_LIST + 1))
        return true;
    if ((scanList == 0) || (scanList != att->scanlist)) {
        return false;
    }
    
    // Exclude priority channels ONLY if SCAN_LIST_ENABLED is active
    // Otherwise, treat them as normal channels in the list
    if (gEeprom.SCAN_LIST_ENABLED)
    {
        const uint16_t PriorityCh1 = gEeprom.SCANLIST_PRIORITY_CH[0];
        const uint16_t PriorityCh2 = gEeprom.SCANLIST_PRIORITY_CH[1];
        if (PriorityCh1 == channel || PriorityCh2 == channel)
            return false;  // Excluded because it's a priority channel and they are enabled
    }
    
    return true;
}

uint16_t RADIO_FindNextChannel(uint16_t Channel, int8_t Direction, bool bCheckScanList, uint8_t VFO)
{
    for (uint16_t i = 0; IS_MR_CHANNEL(i); i++, Channel += Direction) {
        if (Channel == 0xFFFF) {
            Channel = MR_CHANNEL_LAST;
        } else if (!IS_MR_CHANNEL(Channel)) {
            Channel = MR_CHANNEL_FIRST;
        }

        if (RADIO_CheckValidChannel(Channel, bCheckScanList, VFO)) {
            return Channel;
        }
    }

    return 0xFFFF;
}

void RADIO_InitInfo(VFO_Info_t *pInfo, const uint16_t ChannelSave, const uint32_t Frequency)
{
    memset(pInfo, 0, sizeof(*pInfo));

    pInfo->Band                     = FREQUENCY_GetBand(Frequency);
    pInfo->SCANLIST_PARTICIPATION   = 0;
    pInfo->STEP_SETTING             = STEP_12_5kHz;
    pInfo->StepFrequency            = gStepFrequencyTable[pInfo->STEP_SETTING];
    pInfo->CHANNEL_SAVE             = ChannelSave;
    pInfo->FrequencyReverse         = false;
    pInfo->OUTPUT_POWER             = OUTPUT_POWER_LOW;
    pInfo->freq_config_RX.Frequency = Frequency;
    pInfo->freq_config_TX.Frequency = Frequency;
    pInfo->pRX                      = &pInfo->freq_config_RX;
    pInfo->pTX                      = &pInfo->freq_config_TX;
    pInfo->Compander                = 0;  // off

    if (ChannelSave == (FREQ_CHANNEL_FIRST + BAND2_108MHz))
        pInfo->Modulation = MODULATION_AM;
    else
        pInfo->Modulation = MODULATION_FM;

    RADIO_ConfigureSquelchAndOutputPower(pInfo);
}

void RADIO_ConfigureChannel(const unsigned int VFO, const unsigned int configure)
{
    VFO_Info_t *pVfo = &gEeprom.VfoInfo[VFO];
    uint16_t channel = gEeprom.ScreenChannel[VFO];

    if (IS_VALID_CHANNEL(channel)) {

        if (IS_MR_CHANNEL(channel)) {
            channel = RADIO_FindNextChannel(channel, RADIO_CHANNEL_UP, false, VFO);
            if (channel == 0xFFFF) {
                channel                    = gEeprom.FreqChannel[VFO];
                gEeprom.ScreenChannel[VFO] = gEeprom.FreqChannel[VFO];
            }
            else {
                gEeprom.ScreenChannel[VFO] = channel;
                gEeprom.MrChannel[VFO]     = channel;
            }
        }
    }
    else
        channel = FREQ_CHANNEL_LAST - 1;

    ChannelAttributes_t* att = MR_GetChannelAttributes(channel);
    if (att->__val == 0xFFFF) { // invalid/unused channel
        if (IS_MR_CHANNEL(channel)) {
            channel                    = gEeprom.FreqChannel[VFO];
            gEeprom.ScreenChannel[VFO] = channel;
        }

        uint16_t bandIdx = channel - FREQ_CHANNEL_FIRST;
        RADIO_InitInfo(pVfo, channel, frequencyBandTable[bandIdx].lower);
        return;
    }

    uint8_t band = att->band;
    if (band > BAND7_470MHz) {
        band = BAND1_50MHz;
    }

    uint8_t bParticipation;

    if (IS_MR_CHANNEL(channel)) {
        bParticipation = att->scanlist;
    }
    else {
        band = channel - FREQ_CHANNEL_FIRST;
        bParticipation = MR_CHANNELS_LIST + 1;
    }

    pVfo->Band                    = band;
    pVfo->SCANLIST_PARTICIPATION = bParticipation;
    pVfo->CHANNEL_SAVE            = channel;

    uint32_t base;
    if (IS_MR_CHANNEL(channel))
        base = channel * 16;
    else
        base = 0x009000 + ((channel - FREQ_CHANNEL_FIRST) * 32) + (VFO * 16);

    if (configure == VFO_CONFIGURE_RELOAD || IS_FREQ_CHANNEL(channel))
    {
        uint8_t tmp;
        uint8_t data[8];
        
        // ***************

        PY25Q16_ReadBuffer(base + 8, data, sizeof(data));

        tmp = data[3] & 0x0F;
        if (tmp > TX_OFFSET_FREQUENCY_DIRECTION_SUB)
            tmp = 0;
        pVfo->TX_OFFSET_FREQUENCY_DIRECTION = tmp;
        tmp = data[3] >> 4;
        if (tmp >= MODULATION_UKNOWN)
            tmp = MODULATION_FM;
        pVfo->Modulation = tmp;

        tmp = data[6];
        if (tmp >= STEP_N_ELEM)
            tmp = STEP_12_5kHz;
        pVfo->STEP_SETTING  = tmp;
        pVfo->StepFrequency = gStepFrequencyTable[tmp];
        pVfo->freq_config_RX.CodeType = (data[2] >> 0) & 0x0F;
        pVfo->freq_config_TX.CodeType = (data[2] >> 4) & 0x0F;

        tmp = data[0];
        switch (pVfo->freq_config_RX.CodeType)
        {
            default:
            case CODE_TYPE_OFF:
                pVfo->freq_config_RX.CodeType = CODE_TYPE_OFF;
                tmp = 0;
                break;

            case CODE_TYPE_CONTINUOUS_TONE:
                if (tmp > (ARRAY_SIZE(CTCSS_Options) - 1))
                    tmp = 0;
                break;

            case CODE_TYPE_DIGITAL:
            case CODE_TYPE_REVERSE_DIGITAL:
                if (tmp > (ARRAY_SIZE(DCS_Options) - 1))
                    tmp = 0;
                break;
        }
        pVfo->freq_config_RX.Code = tmp;

        tmp = data[1];
        switch (pVfo->freq_config_TX.CodeType)
        {
            default:
            case CODE_TYPE_OFF:
                pVfo->freq_config_TX.CodeType = CODE_TYPE_OFF;
                tmp = 0;
                break;

            case CODE_TYPE_CONTINUOUS_TONE:
                if (tmp > (ARRAY_SIZE(CTCSS_Options) - 1))
                    tmp = 0;
                break;

            case CODE_TYPE_DIGITAL:
            case CODE_TYPE_REVERSE_DIGITAL:
                if (tmp > (ARRAY_SIZE(DCS_Options) - 1))
                    tmp = 0;
                break;
        }
        pVfo->freq_config_TX.Code = tmp;

        if (data[4] == 0xFF)
        {
            pVfo->FrequencyReverse  = false;
            pVfo->CHANNEL_BANDWIDTH = BK4819_FILTER_BW_WIDE;
            pVfo->OUTPUT_POWER      = OUTPUT_POWER_LOW;
            pVfo->BUSY_CHANNEL_LOCK = false;
        }
        else
        {
            const uint8_t d4 = data[4];
            pVfo->FrequencyReverse  = !!((d4 >> 0) & 1u);
            pVfo->CHANNEL_BANDWIDTH = !!((d4 >> 1) & 1u);
            {
                uint8_t pwr = ((d4 >> 2) & 7u);
                // Guard: X=0 is only valid when stored by this firmware.
                // Old firmware had LOW=0 (no X slot), so EEPROM value 0 -> treat as LOW.
                // X (0) is now a valid stored value — do NOT strip it.
                // Only reject truly invalid values (>= LEN but not X).
                if (pwr >= OUTPUT_POWER_LEN)
                    pwr = OUTPUT_POWER_LOW;
                pVfo->OUTPUT_POWER = pwr;
            }
            pVfo->BUSY_CHANNEL_LOCK = !!((d4 >> 5) & 1u);
        }

        // ***************

        struct {
            uint32_t Frequency;
            uint32_t Offset;
        } __attribute__((packed)) info;
        PY25Q16_ReadBuffer(base, &info, sizeof(info));
        if(info.Frequency==0xFFFFFFFF)
            pVfo->freq_config_RX.Frequency = frequencyBandTable[band].lower;
        else
            pVfo->freq_config_RX.Frequency = info.Frequency;

        if (info.Offset >= _1GHz_in_KHz)
            info.Offset = _1GHz_in_KHz / 100;

        pVfo->TX_OFFSET_FREQUENCY = info.Offset;

        // ***************
    }

    uint32_t frequency = pVfo->freq_config_RX.Frequency;

    // fix previously set incorrect band
    band = FREQUENCY_GetBand(frequency);

    if (frequency < frequencyBandTable[band].lower)
        frequency = frequencyBandTable[band].lower;
    else if (frequency > frequencyBandTable[band].upper)
        frequency = frequencyBandTable[band].upper;
    else if (channel >= FREQ_CHANNEL_FIRST)
        frequency = FREQUENCY_RoundToStep(frequency, pVfo->StepFrequency);

    pVfo->freq_config_RX.Frequency = frequency;

    if (!IS_MR_CHANNEL(channel))
        pVfo->TX_OFFSET_FREQUENCY = FREQUENCY_RoundToStep(pVfo->TX_OFFSET_FREQUENCY, pVfo->StepFrequency);

    RADIO_ApplyOffset(pVfo);

    if (IS_MR_CHANNEL(channel))
    {   // 16 bytes allocated to the channel name but only 10 used, the rest are 0's
        SETTINGS_FetchChannelName(pVfo->Name, channel);
    }

    if (!pVfo->FrequencyReverse)
    {
        pVfo->pRX = &pVfo->freq_config_RX;
        pVfo->pTX = &pVfo->freq_config_TX;
    }
    else
    {
        pVfo->pRX = &pVfo->freq_config_TX;
        pVfo->pTX = &pVfo->freq_config_RX;
    }

    pVfo->Compander = att->compander;

    RADIO_ConfigureSquelchAndOutputPower(pVfo);
}

void RADIO_ConfigureSquelchAndOutputPower(VFO_Info_t *pInfo)
{

    // *******************************
    // squelch

    FREQUENCY_Band_t Band = FREQUENCY_GetBand(pInfo->pRX->Frequency);
    // 0x1E60 : 0x1E00
    uint32_t Base = (Band < BAND4_174MHz) ? 0x010060 : 0x010000;

    if (gEeprom.SQUELCH_LEVEL == 0)
    {   // squelch == 0 (off)
        pInfo->SquelchOpenRSSIThresh    = 0;     // 0 ~ 255
        pInfo->SquelchOpenNoiseThresh   = 127;   // 127 ~ 0
        pInfo->SquelchCloseGlitchThresh = 255;   // 255 ~ 0

        pInfo->SquelchCloseRSSIThresh   = 0;     // 0 ~ 255
        pInfo->SquelchCloseNoiseThresh  = 127;   // 127 ~ 0
        pInfo->SquelchOpenGlitchThresh  = 255;   // 255 ~ 0
    }
    else
    {   // squelch >= 1
        Base += gEeprom.SQUELCH_LEVEL;                                        // my eeprom squelch-1
                                                                              // VHF   UHF
        PY25Q16_ReadBuffer(Base + 0x00, &pInfo->SquelchOpenRSSIThresh,    1);  //  50    10
        PY25Q16_ReadBuffer(Base + 0x10, &pInfo->SquelchCloseRSSIThresh,   1);  //  40     5

        PY25Q16_ReadBuffer(Base + 0x20, &pInfo->SquelchOpenNoiseThresh,   1);  //  65    90
        PY25Q16_ReadBuffer(Base + 0x30, &pInfo->SquelchCloseNoiseThresh,  1);  //  70   100

        PY25Q16_ReadBuffer(Base + 0x40, &pInfo->SquelchCloseGlitchThresh, 1);  //  90    90
        PY25Q16_ReadBuffer(Base + 0x50, &pInfo->SquelchOpenGlitchThresh,  1);  // 100   100

        uint16_t noise_open   = pInfo->SquelchOpenNoiseThresh;
        uint16_t noise_close  = pInfo->SquelchCloseNoiseThresh;

#if ENABLE_SQUELCH_MORE_SENSITIVE
        uint16_t rssi_open    = pInfo->SquelchOpenRSSIThresh;
        uint16_t rssi_close   = pInfo->SquelchCloseRSSIThresh;
        uint16_t glitch_open  = pInfo->SquelchOpenGlitchThresh;
        uint16_t glitch_close = pInfo->SquelchCloseGlitchThresh;
        // make squelch more sensitive
        // note that 'noise' and 'glitch' values are inverted compared to 'rssi' values
        rssi_open   = (rssi_open   * 1) / 2;
        noise_open  = (noise_open  * 2) / 1;
        glitch_open = (glitch_open * 2) / 1;

        // ensure the 'close' threshold is lower than the 'open' threshold
        if (rssi_close == rssi_open && rssi_close >= 2)
            rssi_close -= 2;
        if (noise_close == noise_open && noise_close  <= 125)
            noise_close += 2;
        if (glitch_close == glitch_open && glitch_close <= 253)
            glitch_close += 2;

        pInfo->SquelchOpenRSSIThresh    = (rssi_open    > 255) ? 255 : rssi_open;
        pInfo->SquelchCloseRSSIThresh   = (rssi_close   > 255) ? 255 : rssi_close;
        pInfo->SquelchOpenGlitchThresh  = (glitch_open  > 255) ? 255 : glitch_open;
        pInfo->SquelchCloseGlitchThresh = (glitch_close > 255) ? 255 : glitch_close;
#endif

        pInfo->SquelchOpenNoiseThresh   = (noise_open   > 127) ? 127 : noise_open;
        pInfo->SquelchCloseNoiseThresh  = (noise_close  > 127) ? 127 : noise_close;
    }

    // *******************************
    // output power

    Band = FREQUENCY_GetBand(pInfo->pTX->Frequency);

    uint8_t Txp[3];
    
    PY25Q16_ReadBuffer(0x100D0 + (Band * 16) + (pInfo->OUTPUT_POWER * 3), Txp, 3);
    const uint8_t p1 = 4;
	const uint8_t p2 = 1;
	const uint8_t p3 = 70;	
    
    for (uint8_t p = 0; p < 3; p++){
        if (pInfo->OUTPUT_POWER == OUTPUT_POWER_LOW)    Txp[p] /= p1;
		if (pInfo->OUTPUT_POWER == OUTPUT_POWER_MID)    Txp[p] /= p2;
		if (pInfo->OUTPUT_POWER == OUTPUT_POWER_HIGH)   Txp[p] += p3;
    }


    pInfo->TXP_CalculatedSetting = FREQUENCY_CalculateOutputPower(
        Txp[0],
        Txp[1],
        Txp[2],
         frequencyBandTable[Band].lower,
        (frequencyBandTable[Band].lower + frequencyBandTable[Band].upper) / 2,
         frequencyBandTable[Band].upper,
        pInfo->pTX->Frequency);

    // *******************************
}

void RADIO_ApplyOffset(VFO_Info_t *pInfo)
{
    uint32_t Frequency = pInfo->freq_config_RX.Frequency;

    switch (pInfo->TX_OFFSET_FREQUENCY_DIRECTION)
    {
        case TX_OFFSET_FREQUENCY_DIRECTION_OFF:
            break;
        case TX_OFFSET_FREQUENCY_DIRECTION_ADD:
            Frequency += pInfo->TX_OFFSET_FREQUENCY;
            break;
        case TX_OFFSET_FREQUENCY_DIRECTION_SUB:
            Frequency -= pInfo->TX_OFFSET_FREQUENCY;
            break;
    }

    pInfo->freq_config_TX.Frequency = Frequency;
}

static void RADIO_SelectCurrentVfo(void)
{
    // if crossband is active and DW not the gCurrentVfo is gTxVfo (gTxVfo/TX_VFO is only ever changed by the user)
    // otherwise it is set to gRxVfo which is set to gTxVfo in RADIO_SelectVfos
    // so in the end gCurrentVfo is equal to gTxVfo unless dual watch changes it on incomming transmition (again, this can only happen when XB off)
    // note: it is called only in certain situations so could be not up-to-date
    gCurrentVfo = (gEeprom.CROSS_BAND_RX_TX == CROSS_BAND_OFF || gEeprom.DUAL_WATCH != DUAL_WATCH_OFF) ? gRxVfo : gTxVfo;
}

void RADIO_SelectVfos(void)
{
    // if crossband without DW is used then RX_VFO is the opposite to the TX_VFO
    gEeprom.RX_VFO = (gEeprom.CROSS_BAND_RX_TX == CROSS_BAND_OFF || gEeprom.DUAL_WATCH != DUAL_WATCH_OFF) ? gEeprom.TX_VFO : !gEeprom.TX_VFO;

    gTxVfo = &gEeprom.VfoInfo[gEeprom.TX_VFO];
    gRxVfo = &gEeprom.VfoInfo[gEeprom.RX_VFO];

    RADIO_SelectCurrentVfo();
}

void RADIO_SetupRegisters(bool switchToForeground)
{
    BK4819_FilterBandwidth_t Bandwidth = gRxVfo->CHANNEL_BANDWIDTH;

    #ifdef ENABLE_FEAT_F4HWN_NARROWER
        if(Bandwidth == BK4819_FILTER_BW_NARROW && gSetting_set_nfm == 1)
        {
            Bandwidth = BK4819_FILTER_BW_NARROWER;
        }
    #endif

    GPIO_DisableAudioPath();

    gEnableSpeaker = false;

    BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false);

    if (gRxVfo->Modulation == MODULATION_AM)
        BK4819_SetFilterBandwidth(BK4819_FILTER_BW_AM, true);
    else
    {
        switch (Bandwidth)
        {
            default:
                Bandwidth = BK4819_FILTER_BW_WIDE;
                [[fallthrough]];
            case BK4819_FILTER_BW_WIDE:
            case BK4819_FILTER_BW_NARROW:
            case BK4819_FILTER_BW_NARROWER:
                BK4819_SetFilterBandwidth(Bandwidth, false);
                break;
        }
    }

    BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, false);

    BK4819_SetupPowerAmplifier(0, 0);

    BK4819_ToggleGpioOut(BK4819_GPIO1_PIN29_PA_ENABLE, false);

    while (1)
    {
        const uint16_t Status = BK4819_ReadRegister(BK4819_REG_0C);
        if ((Status & 1u) == 0) // INTERRUPT REQUEST
            break;

        BK4819_WriteRegister(BK4819_REG_02, 0);
        SYSTEM_DelayMs(1);
    }
    BK4819_WriteRegister(BK4819_REG_3F, 0);

    // mic gain 0.5dB/step 0 to 63
    BK4819_WriteRegister(BK4819_REG_7D, 0xE940 | (gEeprom.MIC_SENSITIVITY_TUNING & 0x3f));

    uint32_t Frequency;
        Frequency = gRxVfo->pRX->Frequency;
    BK4819_SetFrequency(Frequency);

    BK4819_SetupSquelch(
        gRxVfo->SquelchOpenRSSIThresh,    gRxVfo->SquelchCloseRSSIThresh,
        gRxVfo->SquelchOpenNoiseThresh,   gRxVfo->SquelchCloseNoiseThresh,
        gRxVfo->SquelchCloseGlitchThresh, gRxVfo->SquelchOpenGlitchThresh);

    BK4819_PickRXFilterPathBasedOnFrequency(Frequency);

    // what does this in do ?
    BK4819_ToggleGpioOut(BK4819_GPIO0_PIN28_RX_ENABLE, true);

    // AF RX Gain and DAC
    //BK4819_WriteRegister(BK4819_REG_48, 0xB3A8);  // 1011 00 111010 1000
    BK4819_WriteRegister(BK4819_REG_48,
        (11u << 12)                 |     // ??? .. 0 ~ 15, doesn't seem to make any difference
        ( 0u << 10)                 |     // AF Rx Gain-1
        (gEeprom.VOLUME_GAIN << 4) |     // AF Rx Gain-2
        (gEeprom.DAC_GAIN    << 0));     // AF DAC Gain (after Gain-1 and Gain-2)

    uint16_t InterruptMask = BK4819_REG_3F_SQUELCH_FOUND | BK4819_REG_3F_SQUELCH_LOST;

    {
        if (gRxVfo->Modulation == MODULATION_FM)
        {   // FM
            uint8_t CodeType = gRxVfo->pRX->CodeType;
            uint8_t Code     = gRxVfo->pRX->Code;

            switch (CodeType)
            {
                default:
                case CODE_TYPE_OFF:
                    BK4819_SetCTCSSFrequency(SQL_TONE);
                    BK4819_SetTailDetection(SQL_TONE); // Default 550 = QS's 55Hz tone method

                    InterruptMask = BK4819_REG_3F_CxCSS_TAIL | BK4819_REG_3F_SQUELCH_FOUND | BK4819_REG_3F_SQUELCH_LOST;
                    break;

                case CODE_TYPE_CONTINUOUS_TONE:
                    BK4819_SetCTCSSFrequency(CTCSS_Options[Code]);

                    //    BK4819_SetTailDetection(550);       // QS's 55Hz tone method
                    //#else
                    //  BK4819_SetTailDetection(CTCSS_Options[Code]);
                    //#endif

                    InterruptMask = 0
                        | BK4819_REG_3F_CxCSS_TAIL
                        | BK4819_REG_3F_CTCSS_FOUND
                        | BK4819_REG_3F_CTCSS_LOST
                        | BK4819_REG_3F_SQUELCH_FOUND
                        | BK4819_REG_3F_SQUELCH_LOST;

                    break;

                case CODE_TYPE_DIGITAL:
                case CODE_TYPE_REVERSE_DIGITAL:
                    BK4819_SetCDCSSCodeWord(DCS_GetGolayCodeWord(CodeType, Code));
                    InterruptMask = 0
                        | BK4819_REG_3F_CxCSS_TAIL
                        | BK4819_REG_3F_CDCSS_FOUND
                        | BK4819_REG_3F_CDCSS_LOST
                        | BK4819_REG_3F_SQUELCH_FOUND
                        | BK4819_REG_3F_SQUELCH_LOST;
                    break;
            }

        }
    }

    // RX expander
    BK4819_SetCompander((gRxVfo->Modulation == MODULATION_FM && gRxVfo->Compander >= 2) ? gRxVfo->Compander : 0);
    //RADIO_SetupAGC(gRxVfo->Modulation == MODULATION_AM, false);
    RADIO_SetupAGC(false, false);

    // enable/disable BK4819 selected interrupts
    BK4819_WriteRegister(BK4819_REG_3F, InterruptMask);

    FUNCTION_Init();

    if (switchToForeground)
        FUNCTION_Select(FUNCTION_FOREGROUND);
}

void RADIO_SetTxParameters(void)
{
    BK4819_FilterBandwidth_t Bandwidth = gCurrentVfo->CHANNEL_BANDWIDTH;

    #ifdef ENABLE_FEAT_F4HWN_NARROWER
        if(Bandwidth == BK4819_FILTER_BW_NARROW && gSetting_set_nfm == 1)
        {
            Bandwidth = BK4819_FILTER_BW_NARROWER;
        }
    #endif

    GPIO_DisableAudioPath();

    gEnableSpeaker = false;

    BK4819_ToggleGpioOut(BK4819_GPIO0_PIN28_RX_ENABLE, false);

    switch (Bandwidth)
    {
        default:
            Bandwidth = BK4819_FILTER_BW_WIDE;
            [[fallthrough]];
        case BK4819_FILTER_BW_WIDE:
        case BK4819_FILTER_BW_NARROW:
        case BK4819_FILTER_BW_NARROWER:
            BK4819_SetFilterBandwidth(Bandwidth, false);
            break;
    }

    BK4819_SetFrequency(gCurrentVfo->pTX->Frequency);

    // TX compressor
    BK4819_SetCompander((gRxVfo->Modulation == MODULATION_FM && (gRxVfo->Compander == 1 || gRxVfo->Compander >= 3)) ? gRxVfo->Compander : 0);

    BK4819_PrepareTransmit();

    SYSTEM_DelayMs(10);

    BK4819_PickRXFilterPathBasedOnFrequency(gCurrentVfo->pTX->Frequency);

    BK4819_ToggleGpioOut(BK4819_GPIO1_PIN29_PA_ENABLE, true);

    SYSTEM_DelayMs(5);

    BK4819_SetupPowerAmplifier(gCurrentVfo->TXP_CalculatedSetting, gCurrentVfo->pTX->Frequency);

    SYSTEM_DelayMs(10);

    switch (gCurrentVfo->pTX->CodeType)
    {
        default:
        case CODE_TYPE_OFF:
            BK4819_ExitSubAu();
            break;

        case CODE_TYPE_CONTINUOUS_TONE:
            BK4819_SetCTCSSFrequency(CTCSS_Options[gCurrentVfo->pTX->Code]);
            break;

        case CODE_TYPE_DIGITAL:
        case CODE_TYPE_REVERSE_DIGITAL:
            BK4819_SetCDCSSCodeWord(DCS_GetGolayCodeWord(gCurrentVfo->pTX->CodeType, gCurrentVfo->pTX->Code));
            break;
    }
}

void RADIO_SetModulation(ModulationMode_t modulation)
{

    BK4819_AF_Type_t mod;
    switch(modulation) {
        default:
        case MODULATION_FM:
            mod = BK4819_AF_FM;
            break;
        case MODULATION_AM:
            mod = BK4819_AF_FM; // AM no longer needs special AF setting
            break;
        case MODULATION_USB:
            mod = BK4819_AF_BASEBAND2;
            break;

    }

    BK4819_SetAF(mod);

    // 
    // What follows is a direct copy of the AM enable/disable code from
    // the original UV-K1 firmware. It is not clear why these specific register
    // values are used for AM all of a sudden instead of the AF setting like on
    // the BK4819, nor what exactly they do.
    // So for now we just keep it as is to maintain compatibility.
    //

    switch (modulation)
    {
        case MODULATION_AM:
        {
            uint16_t uVar1 = BK4819_ReadRegister(0x31);
            BK4819_WriteRegister(0x31, uVar1 | 1); // AM Demodulation Enable
            BK4819_WriteRegister(0x42, 0x6f5c);
            BK4819_WriteRegister(0x2a, 0x7434);
            BK4819_WriteRegister(0x2B, 0x0400); // FAGCI: HP filter off, LP on, de-emph on
            BK4819_WriteRegister(0x2F, 0x9990); // FAGCI: AM filter
            BK4819_WriteRegister(0x28, 0x0B40); // FAGCI: noise gate AM
            BK4819_WriteRegister(0x2C, 0x1822); // FAGCI: emph AM

            #ifdef ENABLE_FEAT_F4HWN_AUDIO
                AUDIO_ApplyAMProfile(gSetting_set_audio_am);
            #else
                BK4819_WriteRegister(0x54, 0x9009);
                BK4819_WriteRegister(0x55, 0x31a9);
            #endif

            BK4819_SetFilterBandwidth(BK4819_FILTER_BW_AM, true);
            break;
        }

        case MODULATION_USB:
        {
            uint16_t uVar1 = BK4819_ReadRegister(0x31);
            BK4819_WriteRegister(0x31, uVar1 & 0xfffe); // AM Demodulation Disable
            BK4819_WriteRegister(0x42, 0x6b5a);
            BK4819_WriteRegister(0x2a, 0x7400);
            BK4819_WriteRegister(0x2b, 0x0000);
            BK4819_WriteRegister(0x2f, 0x9890);
            BK4819_WriteRegister(0x28, 0x0B40); // FAGCI: noise gate USB
            BK4819_WriteRegister(0x2C, 0x1822); // FAGCI: emph USB

            #ifdef ENABLE_FEAT_F4HWN_AUDIO
                AUDIO_ApplyUSBProfile();
            #else
                BK4819_WriteRegister(0x54, 0x9009);
                BK4819_WriteRegister(0x55, 0x31a9);
            #endif
            break;
        }

        case MODULATION_FM:
        default:
        {
            uint16_t uVar1 = BK4819_ReadRegister(0x31);
            BK4819_WriteRegister(0x31, uVar1 & 0xfffe); // AM Demodulation Disable
            BK4819_WriteRegister(0x28, 1536);   // 0x0600 — noise gate FM
            BK4819_WriteRegister(0x2C, 26210);  // 0x6662 — de-emph / tx gain FM
            BK4819_WriteRegister(0x4A, (reg_4A_cache & ~127U) | 40); // AF volume
            BK4819_WriteRegister(0x42, 0x6b5a);
            BK4819_WriteRegister(0x2a, 0x7400);
            BK4819_WriteRegister(0x2b, 0x0000);
            BK4819_WriteRegister(0x2f, 0x9890);

            #ifdef ENABLE_FEAT_F4HWN_AUDIO
                AUDIO_ApplyFMProfile(gSetting_set_audio_fm);
            #else
                BK4819_WriteRegister(0x54, 0x9009);
                BK4819_WriteRegister(0x55, 0x31a9);
            #endif
            break;
        }
    }
    
    BK4819_SetRegValue(afDacGainRegSpec, 0xF);
    BK4819_WriteRegister(BK4819_REG_3D, modulation == MODULATION_USB ? 0 : 0x2AAB);
    BK4819_SetRegValue(afcDisableRegSpec, modulation != MODULATION_FM);

    RADIO_SetupAGC(modulation == MODULATION_AM, false);
}

void RADIO_SetupAGC(bool listeningAM, bool disable)
{
    static uint8_t lastSettings = 0xFF;
    uint8_t newSettings = (listeningAM << 1) | disable;
    if (lastSettings == newSettings)
        return;
    lastSettings = newSettings;
    BK4819_SetAGC(!disable);
    BK4819_InitAGC(listeningAM);
}

void RADIO_SetVfoState(VfoState_t State)
{
    if (State == VFO_STATE_NORMAL) {
        VfoState[0] = VFO_STATE_NORMAL;
        VfoState[1] = VFO_STATE_NORMAL;
    } else if (State == VFO_STATE_VOLTAGE_HIGH) {
        VfoState[0] = VFO_STATE_VOLTAGE_HIGH;
        VfoState[1] = VFO_STATE_TX_DISABLE;
    } else {
        // 1of11
        const unsigned int vfo = (gEeprom.CROSS_BAND_RX_TX == CROSS_BAND_OFF) ? gEeprom.RX_VFO : gEeprom.TX_VFO;
        VfoState[vfo] = State;
    }

    gVFOStateResumeCountdown_500ms = (State == VFO_STATE_NORMAL) ? 0 : vfo_state_resume_countdown_500ms;
    gUpdateDisplay = true;
}

void RADIO_PrepareTX(void)
{
    VfoState_t State = VFO_STATE_NORMAL;  // default to OK to TX

    if (gEeprom.DUAL_WATCH != DUAL_WATCH_OFF)
    {   // dual-RX is enabled

        gDualWatchCountdown_10ms = dual_watch_count_after_tx_10ms;
        gScheduleDualWatch       = false;

        if (!gRxVfoIsActive)
        {   // use the current RX vfo
            gEeprom.RX_VFO = gEeprom.TX_VFO;
            gRxVfo         = gTxVfo;
            gRxVfoIsActive = true;
        }

        // let the user see that DW is not active
        gDualWatchActive = false;
        gUpdateStatus    = true;
    }

    RADIO_SelectCurrentVfo();

    if(TX_freq_check(gCurrentVfo->pTX->Frequency) != 0
    ){
        // TX frequency not allowed
        State = VFO_STATE_TX_DISABLE;
        gVfoConfigureMode = VFO_CONFIGURE;
    } else if (SerialConfigInProgress()) {
        // TX is disabled or config upload/download in progress
        State = VFO_STATE_TX_DISABLE;
    } else if (gCurrentVfo->BUSY_CHANNEL_LOCK && gCurrentFunction == FUNCTION_RECEIVE) {
        // busy RX'ing a station
        State = VFO_STATE_BUSY;
    } else if (gBatteryDisplayLevel == 0) {
        // charge your battery !git co
        State = VFO_STATE_BAT_LOW;
    } else if (gBatteryDisplayLevel > 6) {
        // over voltage .. this is being a pain
        State = VFO_STATE_VOLTAGE_HIGH;
    }
    else if (gCurrentVfo->Modulation != MODULATION_FM) {
        // not allowed to TX if in AM mode
        State = VFO_STATE_TX_DISABLE;
    }

    if (State != VFO_STATE_NORMAL) {
        // TX not allowed
        RADIO_SetVfoState(State);
        return;
    }

    // TX is allowed

    FUNCTION_Select(FUNCTION_TRANSMIT);

    gTxTimerCountdown_500ms = 0;            // no timeout

    {

        gTxTimerCountdown_500ms = ((gEeprom.TX_TIMEOUT_TIMER + 1) * 5) * 2;

        /*
        if (gEeprom.TX_TIMEOUT_TIMER == 0)
            gTxTimerCountdown_500ms = 60;   // 30 sec
        else if (gEeprom.TX_TIMEOUT_TIMER < (ARRAY_SIZE(gSubMenu_TOT) - 1))
            gTxTimerCountdown_500ms = 120 * gEeprom.TX_TIMEOUT_TIMER;  // minutes
        else
            gTxTimerCountdown_500ms = 120 * 15;  // 15 minutes
        */

#ifdef ENABLE_FEAT_F4HWN 
        gTxTimerCountdownAlert_500ms = gTxTimerCountdown_500ms;
#endif
    }

    gTxTimeoutReached    = false;

#ifdef ENABLE_FEAT_F4HWN 
    gTxTimeoutReachedAlert = false;
#endif
    
    gFlagEndTransmission = false;
    gRTTECountdown_10ms  = 0;

}

void RADIO_SendCssTail(void)
{
    switch (gCurrentVfo->pTX->CodeType) {
    case CODE_TYPE_DIGITAL:
    case CODE_TYPE_REVERSE_DIGITAL:
        BK4819_PlayCDCSSTail();
        break;
    default:
        BK4819_PlayCTCSSTail();
        break;
    }

    SYSTEM_DelayMs(200);
}

void RADIO_SendEndOfTransmission(void)
{
    BK4819_PlayRoger(gEeprom.ROGER);
    // send the CTCSS/DCS tail tone - allows the receivers to mute the usual FM squelch tail/crash
    if(gEeprom.TAIL_TONE_ELIMINATION)
        RADIO_SendCssTail();
    RADIO_SetupRegisters(false);
}

void RADIO_PrepareCssTX(void)
{
    RADIO_PrepareTX();

    SYSTEM_DelayMs(200);

    if(gEeprom.TAIL_TONE_ELIMINATION)
        RADIO_SendCssTail();
    RADIO_SetupRegisters(true);
}
