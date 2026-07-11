//K1 Spectrum Sonic
/* Copyright 2026 Robby
 * https://github.com/Robby69400
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
// ============================================================
// SECTION: Includes
// ============================================================
#include "app/spectrum.h"
#include "nav_invert.h"
#include "driver/backlight.h"
#include "driver/eeprom.h"
#include "ui/helper.h"
#include "common.h"
#include "action.h"
#include "ui/main.h"
#include "scheduler.h"

#include "misc.h"
#include "driver/py25q16.h"
#include "version.h"
#ifdef ENABLE_DEV
#include "debugging.h"
#endif
#include <stdlib.h>
#include "settings.h"
// ============================================================
// SECTION: Compile-time configuration
// ============================================================
#define MAX_VISIBLE_LINES 6
#define NoisLvl 40
#define NoiseHysteresis 15
          /////////////////////////DEBUG//////////////////////////
//char str[64] = "";sprintf(str, "%d\r\n", Spectrum_state );LogUart(str);

// ============================================================
// SECTION: State variables
// ============================================================
static volatile bool gSpectrumChangeRequested = false;
static volatile uint8_t gRequestedSpectrumState = 0;

// ============================================================
// SECTION: HISTORY
// ============================================================
uint8_t code = 0;
typedef struct {
    uint32_t    HFreqs;
    uint8_t     HBlacklisted;
    uint16_t    HTimeS;
} HistoryStruct;

#if defined(ENABLE_USB)
    #define HISTORY_SIZE 100
#elif defined(ENABLE_UART)
    #define HISTORY_SIZE 200
#endif

static uint16_t historyListIndex = 0;
static int historyScrollOffset = 0;
static bool gHistoryScan = false;
static uint32_t CodeFreq = 0;
static uint16_t cachedChannels[6];      // Stocke les numéros de canaux trouvés
static uint16_t cachedAbsoluteIdx[6];   // Mémorise à quel index d'historique ils appartiennent
static uint8_t  cacheWriteHead = 0;     // Tête d'écriture circulaire
static int lastHistoryScrollOffset = -1;
// ============================================================

static uint16_t indexFs = 0;
static uint8_t MonitorIndex = 0;
static uint32_t SpectrumRangeStart = 1400000;
static uint32_t SpectrumRangeStop = 110000000;
#define MONITOR_SIZE 20

/////////////////////////////Parameters://///////////////////////////
static bool     Light_Mode = true;   
static uint16_t DelayRssi = 2000;     
static uint16_t SpectrumDelay = 0;    
static uint16_t MaxListenTime = 0;    
static uint32_t RangeStart = 1400000; 
static uint32_t RangeStop = 11000000; 
static uint16_t SpectrumSleepMs = 0;  
static uint8_t  Noislvl_OFF = NoisLvl;
static uint8_t  Noislvl_ON = NoisLvl - NoiseHysteresis;
static uint16_t osdPopupSetting = 500;      
static uint16_t UOO_trigger = 15;
static uint8_t  AUTO_KEYLOCK = AUTOLOCK_OFF;
static uint8_t  GlitchMax = 20;             
static bool     SoundBoost = 0;             
static uint8_t  PttEmission = 0;            
static bool     gMonitorScan = true;       

// Configuration des index du menu des paramètres
#define PARAM_LIGHT_MODE       0
#define PARAM_RSSI_DELAY       1
#define PARAM_SPECTRUM_DELAY   2
#define PARAM_MAX_LISTEN_TIME  3
#define PARAM_RANGE_START      4
#define PARAM_RANGE_STOP       5
#define PARAM_SCAN_STEP        6
#define PARAM_LISTEN_BW        7
#define PARAM_MODULATION       8
#define PARAM_POWER_SAVE       9
#define PARAM_AUTO_KEYLOCK     10
#define PARAM_NOISE_LEVEL_OFF  11
#define PARAM_GLITCH_MAX       12
#define PARAM_OSD_POPUP        13
#define PARAM_RECORD_TRIGGER   14
#define PARAM_SOUND_BOOST      15
#define PARAM_PTT_EMISSION     16
#define PARAM_MONITOR_SCAN     17
#define PARAM_RESET_DEFAULT    18

static const uint8_t lightModeMenuMapping[] = {
    PARAM_LIGHT_MODE,
    PARAM_SPECTRUM_DELAY,
    PARAM_MAX_LISTEN_TIME,
    PARAM_RANGE_START,
    PARAM_RANGE_STOP,
    PARAM_SCAN_STEP
};

uint16_t GetMaxVisualRows(void) {
    return (Light_Mode) ? 6 : 19; 
}

////////////////////////////////////////////////////////////////////

static uint8_t IndexDelayRssi = 3;
static const char *DelayRssiText[] =       {".75",".8",".9","1","2","4"};
static const uint16_t DelayRssiValues[] =   {750,800,900,1000,2000,4000}; //in ms

static bool     Backlight_On = 1;
uint8_t osdPopupIndex = 3;

#ifdef ENABLE_BENCH
    static uint32_t benchTickMs = 0;      
    static uint16_t benchStepsThisSec = 0;
    static uint16_t benchRatePerSec = 0;  
    static uint32_t benchLapMs = 0;       
    static uint32_t benchLastLapMs = 0;   
    static bool benchLapDone = false;
#endif

bool Cleared = 0;
static bool SettingsLoaded = false;
uint8_t  gKeylockCountdown = 0;
bool     gIsKeylocked = false;
static uint16_t osdPopupTimer = 0;
static uint32_t Fmax = 0;
static uint32_t spectrumElapsedCount = 0;
static uint32_t SpectrumPauseCount = 0;
static bool SPECTRUM_PAUSED;
static uint8_t IndexMaxLT = 0;
static const char *labels[] = {"OFF","3s","6s","10s","20s", "1m", "5m", "10m", "20m", "30m"};
static const uint16_t listenSteps[] = {0, 3, 6, 10, 20, 60, 300, 600, 1200, 1800}; //in s
#define LISTEN_STEP_COUNT 9
static uint8_t IndexPS = 0;
static const char *labelsPS[] = {"OFF","200ms","500ms", "1s", "2s", "5s"};
static const uint16_t PS_Steps[] = {0, 20, 50, 100, 200, 500}; //in 10 ms
#define PS_STEP_COUNT 5
static uint32_t lastReceivingFreq = 0;
static bool gIsPeak = false;
static bool historyListActive = false;
static bool gForceModulation = 0;
static uint8_t SpectrumMonitor = 0;
static uint8_t prevSpectrumMonitor = 0;
static bool Key_1_pressed = 0;
static uint16_t WaitSpectrum = 0; 
static uint8_t ArrowLine = 1;
static void ToggleRX(bool on);
static void NextScanStep();
static void BuildValidScanListIndices();
static void RenderHistoryList();
static void RenderScanListSelect();
static void RenderParametersSelect();
static void UpdateScan();
static uint8_t bandListSelectedIndex = 0;
static int bandListScrollOffset = 0;
static void RenderBandSelect();
static void ClearHistory(uint8_t mode);
static void DrawMeter(int);
static uint8_t scanListSelectedIndex = 0;
static uint8_t scanListScrollOffset = 0;
static uint8_t parametersSelectedIndex = 0;
static uint8_t parametersScrollOffset = 0;
static uint8_t validScanListCount = 0;
static KeyboardState kbd = {KEY_INVALID, KEY_INVALID, 0,0};
struct FrequencyBandInfo {
    uint32_t lower;
    uint32_t upper;
    uint32_t middle;
};
static uint32_t cdcssFreq;
static uint16_t ctcssFreq;
//static uint8_t refresh = 0; // СУБТОНО ЗАПРОС ВСЕГДА
#define F_MAX frequencyBandTable[ARRAY_SIZE(frequencyBandTable) - 1].upper
#define Bottom_print 51 //Robby69
static Mode appMode;
//#define UHF_NOISE_FLOOR 0

static uint16_t scanChannelsCount;
static uint8_t monitorChannelsCount;
static void ToggleScanList();
static void SaveSettings();
static const uint16_t RSSI_MAX_VALUE = 255;
static uint16_t R30, R37, R3D, R43, R47, R48, R7E, R02, R3F, R7B, R12, R11, R14, R54, R55, R75;
static char String[48];
static char StringC[10];
static bool isKnownChannel = false;
static uint16_t  gChannel;
static char channelName[12];
ModulationMode_t  channelModulation;
static BK4819_FilterBandwidth_t channelBandwidth;
static bool isInitialized = false;
static bool isListening = true;
static bool newScanStart = true;
static bool audioState = true;
static uint8_t bl;
static State currentState = SPECTRUM, previousState = SPECTRUM;
static uint8_t Spectrum_state = 0; 
static PeakInfo peak;
static ScanInfo scanInfo;
static char latestScanListName[12];
static bool refreshScanListName = true;
static bool IsBlacklisted(uint32_t f);
static void SetState(State state);

typedef struct {
    char left[20];
    char right[20];
} ListRow;

typedef void (*GetListRowFn)(uint16_t index, ListRow *row);

/***************************BIG RAM******************************************/
static uint32_t         *ScanFrequencies = NULL;
static bandparameters   *BParams = NULL;
static uint32_t         HFreqs[HISTORY_SIZE];
static bool             HBlacklisted[HISTORY_SIZE];
static uint16_t         HTimeS[HISTORY_SIZE];  
static uint32_t         MonitorFreqs[MONITOR_SIZE];
/****************************************************************************/

SpectrumSettings settings = {stepsCount: STEPS_128,
                             scanStepIndex: STEP_500kHz,
                             frequencyChangeStep: 80000,
                             rssiTriggerLevelUp: 20,
                             bw: BK4819_FILTER_BW_WIDE,
                             listenBw: BK4819_FILTER_BW_WIDE,
                             modulationType: false,
                             dbMin: -120,
                             dbMax: -60,
                             scanList: S_SCAN_LIST_ALL,
                             scanListEnabled: {0},
                             bandEnabled: {0}
                            };

static uint32_t currentFreq, tempFreq;
static uint8_t rssiHistory[128];
#ifdef ENABLE_PERSIST
static uint8_t  peakHoldY[128];       // Peak Y value per display column (0=top)
static uint8_t  peakHoldAge[64];      // Shared decay timer (1 per 2 columns)
#define PEAK_HOLD_DELAY  15           // Sweeps before decay starts
#define PEAK_HOLD_INIT   0xFF         // "no peak" sentinel (same as SPECTRUM_TOPY_SKIP)
#endif
static int ShowLines = 1;
static uint8_t freqInputIndex = 0;
static uint8_t freqInputDotIndex = 0;
static KEY_Code_t freqInputArr[10];
char freqInputString[11];
static uint8_t nextBandToScanIndex = 0;
static void LookupChannelModulation();

static uint8_t validScanListIndices[MR_CHANNELS_LIST];
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                              K1 SPECIFIC
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
bool gComeBack = 0;
static void LoadActiveBands(void);
uint16_t BOARD_gMR_fetchChannel(const uint32_t freq);
static void LoadActiveScanFrequencies(void);
static uint8_t bandCount;
STEP_Setting_t channelStep;
int Rssi2DBm(const uint16_t rssi) {return (rssi >> 1) - 160;}

static int clamp(int v, int min, int max) {
  return v <= min ? min : (v >= max ? max : v);
}

static void UpdateDBMaxAuto() { //Zoom
    scanInfo.rssiMax = 0;
    scanInfo.rssiMin = 65535;
    for (uint8_t i = 0; i < 127;i++) {
        if (rssiHistory[i] > scanInfo.rssiMax) {scanInfo.rssiMax = rssiHistory[i];}
        else if (rssiHistory[i] < scanInfo.rssiMin) {scanInfo.rssiMin = rssiHistory[i];}
    }

  static uint8_t z = 5;
  int newDbMax;
    if (scanInfo.rssiMax > 0) {
        newDbMax = Rssi2DBm(scanInfo.rssiMax);

        if (newDbMax > settings.dbMax + z) {
            settings.dbMax = settings.dbMax + z;
        } else if (newDbMax < settings.dbMax - z) {
            settings.dbMax = settings.dbMax - z;
        } else {
            settings.dbMax = newDbMax;
        }
    }

    if (scanInfo.rssiMin > 0) {
        settings.dbMin = Rssi2DBm(scanInfo.rssiMin);
    }
}


BK4819_FilterBandwidth_t ACTION_NextBandwidth(BK4819_FilterBandwidth_t currentBandwidth, const bool dynamic, bool increase)
{
    BK4819_FilterBandwidth_t nextBandwidth =
        (increase && currentBandwidth == BK4819_FILTER_BW_NARROWER) ? BK4819_FILTER_BW_WIDE :
        (!increase && currentBandwidth == BK4819_FILTER_BW_WIDE)     ? BK4819_FILTER_BW_NARROWER :
        (increase ? currentBandwidth + 1 : currentBandwidth - 1);

    BK4819_SetFilterBandwidth(nextBandwidth, dynamic);
    gRequestSaveChannel = 1;
    return nextBandwidth;
}

const char *bwNames[5] = {"25k", "12.5k", "8.33k", "6.25k", "5k"};

int16_t BK4819_GetAFCValue() { //from Hawk5
  int16_t signedAfc = (int16_t)BK4819_ReadRegister(0x6D);
  return (signedAfc * 10) / 3;
}

typedef struct
{
	uint8_t      sLevel;      // S-level value
	uint8_t      over;        // over S9 value
	int          dBmRssi;     // RSSI in dBm
	bool         overSquelch; // determines whether signal is over squelch open threshold
}  __attribute__((packed))  sLevelAttributes;

#define HF_FREQUENCY 3000000

sLevelAttributes GetSLevelAttributes(const int16_t rssi, const uint32_t frequency)
{
	sLevelAttributes att;
	// S0 .. base level
	int16_t      s0_dBm       = -130;
	// all S1 on max gain, no antenna
	const int8_t dBmCorrTable[7] = {
		-5,  // band 1
		-38, // band 2
		-37, // band 3
		-20, // band 4
		-23, // band 5
		-23, // band 6
		-16  // band 7
	};

	if(frequency > HF_FREQUENCY)
	s0_dBm-=20;
	att.dBmRssi = Rssi2DBm(rssi)+dBmCorrTable[FREQUENCY_GetBand(frequency)];
	att.sLevel  = MIN(MAX((att.dBmRssi - s0_dBm) / 6, 0), 9);
	att.over    = MIN(MAX(att.dBmRssi - (s0_dBm + 9*6), 0), 99);
	att.overSquelch = att.sLevel > 5;

	return att;
}

ChannelInfo_t FetchChannelFrequency(const uint16_t Channel) {
    ChannelInfo_t info;
    PY25Q16_ReadBuffer(0x0000 + (uint32_t)Channel * 16, &info, sizeof(info));
    if (info.frequency == 0xFFFFFFFF) {
        ChannelInfo_t empty = {0, 0};
        return empty;
    }
    return info;
}

#define BLOCK_SIZE 16

typedef struct {
    uint32_t freq;       // 0x00
    uint32_t offset;     // 0x04
    uint8_t  rxcode;     // 0x08
    uint8_t  txcode;     // 0x09
    uint8_t  codeflags;  // 0x0A (regroupement des bitfields pour simplifier la lecture)
    uint8_t  mod_dir;    // 0x0B
    uint8_t  settings;   // 0x0C
    uint8_t  dtmf;       // 0x0D
    uint8_t  step;       // 0x0E
    uint8_t  unused;     // 0x0F
} __attribute__((packed)) FlashChannel_t;

uint16_t BOARD_gMR_fetchChannel(const uint32_t freq) {
    static FlashChannel_t block[BLOCK_SIZE]; 
    
    for (uint16_t start_ch = MR_CHANNEL_FIRST; start_ch <= MR_CHANNEL_LAST; start_ch += BLOCK_SIZE) {
        uint16_t remaining = MR_CHANNEL_LAST - start_ch + 1;
        uint16_t chunk_size = (remaining > BLOCK_SIZE) ? BLOCK_SIZE : remaining;
        
        uint32_t physical_index = start_ch - MR_CHANNEL_FIRST;
        uint32_t block_addr = (physical_index * sizeof(FlashChannel_t));
        
        PY25Q16_ReadBuffer(block_addr, (uint8_t*)block, chunk_size * sizeof(FlashChannel_t));
        
        for (uint16_t k = 0; k < chunk_size; k++) {
            if (block[k].freq == freq) {
                return start_ch + k;
            }
        }
    }
    return 0xFFFF;
}

uint16_t RADIO_ValidMemoryChannelsCount(bool bCheckScanList, uint8_t CurrentScanList)
{
	uint16_t count=0;
	for (uint16_t i = MR_CHANNEL_FIRST; i<=MR_CHANNEL_LAST; ++i) {
			if(RADIO_CheckValidChannel(i, bCheckScanList, CurrentScanList)) count++;
		}
	return count;
}

static void LoadActiveBands(void) {
    memset(BParams, 0, (MAX_BANDS) * sizeof(bandparameters));
    bandCount = 0;

    for (uint16_t bd = 0; bd < MAX_BANDS; bd++) 
    {
        uint16_t targetChannel = bd + MR_CHANNELS_MAX - MAX_BANDS;
        ChannelInfo_t freqs = FetchChannelFrequency(targetChannel);

        if (freqs.frequency >= 1400000 && freqs.frequency <= 130000000)
        {
            gChannel = targetChannel;
            LookupChannelModulation(); 

            BParams[bandCount].modulationType = channelModulation;
            BParams[bandCount].scanStep       = channelStep;
            BParams[bandCount].Startfrequency = freqs.frequency;
            BParams[bandCount].Stopfrequency  = freqs.offset;

            PY25Q16_ReadBuffer(0x004000 + (targetChannel * 16), BParams[bandCount].BandName, 10);

            bandCount++;
        }
    }
}

static char osdPopupText[32] = "";

static void ShowOSDPopup(const char *str)
{   osdPopupTimer = osdPopupSetting;
    strncpy(osdPopupText, str, sizeof(osdPopupText)-1);
    osdPopupText[sizeof(osdPopupText)-1] = '\0';
    spectrumElapsedCount = 0;
}

//#define MAX_CHANNELS 500

static uint16_t CountValidFrequencies(void) {
    uint16_t count = 0;
    ChannelAttributes_t cache;
    //for (uint16_t ch = MR_CHANNEL_FIRST; ch <= MR_CHANNEL_LAST && count < MAX_CHANNELS; ch++) {
    for (uint16_t ch = MR_CHANNEL_FIRST; ch <= MR_CHANNEL_LAST; ch++) {
        MR_LoadChannelAttributesFromFlash(ch, &cache);
        if (cache.scanlist > 0 && cache.scanlist <= MR_CHANNELS_LIST) {
            if (FetchChannelFrequency(ch).frequency && settings.scanListEnabled[cache.scanlist-1]) count++;
        }
    }
    if (count > 0) return count; 
    //for (uint16_t ch = MR_CHANNEL_FIRST; ch <= MR_CHANNEL_LAST && count < MAX_CHANNELS; ch++) {
    for (uint16_t ch = MR_CHANNEL_FIRST; ch <= MR_CHANNEL_LAST; ch++) {
        if (FetchChannelFrequency(ch).frequency) count++;
    }
    return count;
}

uint8_t CountActiveBands(void) {
    uint8_t activeCount = 0;
    for (uint8_t i = 0; i < MAX_BANDS; i++) {
        if (settings.bandEnabled[i]) {
            activeCount++;
        }
    }
    return activeCount;
}

static void LoadActiveScanFrequencies(void)
{   if (ScanFrequencies != NULL) { free(ScanFrequencies); ScanFrequencies = NULL; }
    uint16_t needed = CountValidFrequencies();
    if (needed > 0) {
        ScanFrequencies = (uint32_t *)malloc(needed * sizeof(uint32_t));
        if (!ScanFrequencies) return;
    }
    char str[32];
    if(appMode == CHANNEL_MODE) {sprintf(str, "CHANNELS:%d", needed);}
    if(appMode == SCAN_BAND_MODE) {sprintf(str, "BANDS:%d",CountActiveBands());}
    if(appMode == FREQUENCY_MODE) {sprintf(str, "FREQUENCY");}
    if(appMode == SCAN_RANGE_MODE) {sprintf(str, "RANGE");}
    if (!gComeBack) ShowOSDPopup(str);
    scanChannelsCount = 0;
    ChannelAttributes_t cache;
    for (uint16_t ch = MR_CHANNEL_FIRST; ch <= MR_CHANNEL_LAST; ch++)
    {
        MR_LoadChannelAttributesFromFlash(ch, &cache);
        if (cache.scanlist <= MR_CHANNELS_LIST) {
            ChannelInfo_t freqs  = FetchChannelFrequency(ch);
            if (freqs.frequency) {
                if (settings.scanListEnabled[cache.scanlist-1])
                    {   ScanFrequencies[scanChannelsCount] = freqs.frequency;
                        scanChannelsCount++;
                    }
                }
        }
    }
    if (!scanChannelsCount) { //No active scanlist
    for (uint16_t ch = MR_CHANNEL_FIRST; ch <= MR_CHANNEL_LAST; ch++)
    {
        ChannelInfo_t freqs  = FetchChannelFrequency(ch);
        if (freqs.frequency) {
                {   ScanFrequencies[scanChannelsCount] = freqs.frequency;
                    scanChannelsCount++;
                }
            }
    }
    }
}

static void LoadMonitorFrequencies(void)
{   
    memset(MonitorFreqs, 0, (MONITOR_SIZE) * sizeof(uint32_t));
    monitorChannelsCount = 0;
    ChannelAttributes_t cache;
    for (uint16_t ch = MR_CHANNEL_FIRST; ch <= MR_CHANNEL_LAST; ch++)
    {   MR_LoadChannelAttributesFromFlash(ch, &cache);
        if (cache.scanlist == 21) {
            ChannelInfo_t freqs  = FetchChannelFrequency(ch);
            if (freqs.frequency) {
                MonitorFreqs[monitorChannelsCount] = freqs.frequency;
                if (++monitorChannelsCount > MONITOR_SIZE) return; //Limit monitor freqs
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef ENABLE_SPECTRUM_LINES
static void MyDrawShortHLine(uint8_t y, uint8_t x_start, uint8_t x_end, uint8_t step, bool white); //ПРОСТОЙ РЕЖИМ ЛИНИИ
static void MyDrawVLine(uint8_t x, uint8_t y_start, uint8_t y_end, uint8_t step); //ПРОСТОЙ РЕЖИМ ЛИНИИ
#endif

const RegisterSpec allRegisterSpecs[] = {
    {"13_LNAs",  0x13, 8, 0b11,  1},
    {"13_LNA",   0x13, 5, 0b111, 1},
    {"13_PGA",   0x13, 0, 0b111, 1},
    {"13_MIX",   0x13, 3, 0b11,  1},
    {"XTAL F Mode Select", 0x3C, 6, 0b11, 1},
// {"-- MIC--",},
    {"RF Tx Deviation", 0x40, 0, 0xFFF, 10},
    {"Compress AF Tx Ratio", 0x29, 14, 0b11, 1},
    {"Compress AF Tx 0 dB", 0x29, 7, 0x7F, 1},
    {"Compress AF Tx noise", 0x29, 0, 0x7F, 1},
    {"MIC AGC Disable", 0x19, 15, 1, 1},
// {"----AFC----",},
    {"AFC Range Select", 0x73, 11, 0b111, 1},
    {"AFC Disable", 0x73, 4, 1, 1},
    {"AFC Speed", 0x73, 5, 0b111111, 1},
    {"3kHz AF Resp K Tx", 0x74, 0, 0xFFFF, 100},
    {"300Hz AF Resp K Tx", 0x44, 0, 0xFFFF, 100},
    {"300Hz AF Resp K Tx", 0x45, 0, 0xFFFF, 100},
//  {"--RX FILT--",},
    {"300Hz AF Resp K Rx", 0x54, 0, 0xFFFF, 100},
    {"300Hz AF Resp K Rx", 0x55, 0, 0xFFFF, 100},
    {"3kHz AF Resp K Rx", 0x75, 0, 0xFFFF, 100},
};

#define STILL_REGS_MAX_LINES 3
static uint8_t stillRegSelected = 0;
static uint8_t stillRegScroll = 0;
static bool stillEditRegs = false; // false = edycja czestotliwosci, true = edycja rejestrow

uint16_t statuslineUpdateTimer = 0;

static void RelaunchScan();
static void ResetInterrupts();
static char StringCode[10] = "";

static bool parametersStateInitialized = false;

static uint32_t stillFreq = 0;
static uint32_t GetInitialStillFreq(void) {
    uint32_t f = 0;

    if (historyListActive) {
        f = HFreqs[historyListIndex];
    } else if (SpectrumMonitor) {
        f = lastReceivingFreq;
    } else if (gIsPeak) {
        f = peak.f;
    } else {
        f = scanInfo.f;
    }

    if (f < 1400000 || f > 130000000) {
        if (scanInfo.f >= 1400000 && scanInfo.f <= 130000000) return scanInfo.f;
        if (currentFreq >= 1400000 && currentFreq <= 130000000) return currentFreq;
        return SpectrumRangeStart; // ostateczny fallback
    }

    return f;
}

static uint16_t GetRegMenuValue(uint8_t st) {
  RegisterSpec s = allRegisterSpecs[st];
  return (BK4819_ReadRegister(s.num) >> s.offset) & s.mask;
}

static void SetRegMenuValue(uint8_t st, bool add) {
  uint16_t v = GetRegMenuValue(st);
  RegisterSpec s = allRegisterSpecs[st];

  uint16_t reg = BK4819_ReadRegister(s.num);
  if (add && v <= s.mask - s.inc) {
    v += s.inc;
  } else if (!add && v >= 0 + s.inc) {
    v -= s.inc;
  }
  reg &= ~(s.mask << s.offset);
  BK4819_WriteRegister(s.num, reg | (v << s.offset));
  
}

KEY_Code_t GetKey() {
  KEY_Code_t btn = KEYBOARD_Poll();
  if (GPIO_IsPttPressed()) {btn = KEY_PTT;}
  if (gSetting_nav_invert) {
    if (btn == KEY_UP)   btn = KEY_DOWN;
    else if (btn == KEY_DOWN) btn = KEY_UP;
  }
  return btn;
}

static void SetState(State state) {
  previousState = currentState;
  currentState = state;
  spectrumElapsedCount = 0;
}

// ============================================================
// SECTION: Radio / hardware functions
// ============================================================

static void BackupRegisters() {
  R30 = BK4819_ReadRegister(BK4819_REG_30);
  R37 = BK4819_ReadRegister(BK4819_REG_37);
  R3D = BK4819_ReadRegister(BK4819_REG_3D);
  R43 = BK4819_ReadRegister(BK4819_REG_43);
  R47 = BK4819_ReadRegister(BK4819_REG_47);
  R48 = BK4819_ReadRegister(BK4819_REG_48);
  R7E = BK4819_ReadRegister(BK4819_REG_7E);
  R02 = BK4819_ReadRegister(BK4819_REG_02);
  R3F = BK4819_ReadRegister(BK4819_REG_3F);
  R7B = BK4819_ReadRegister(BK4819_REG_7B);
  R12 = BK4819_ReadRegister(BK4819_REG_12);
  R11 = BK4819_ReadRegister(BK4819_REG_11);
  R14 = BK4819_ReadRegister(BK4819_REG_14);
  R54 = BK4819_ReadRegister(BK4819_REG_54);
  R55 = BK4819_ReadRegister(BK4819_REG_55);
  R75 = BK4819_ReadRegister(BK4819_REG_75);
}

static void RestoreRegisters() {
  BK4819_WriteRegister(BK4819_REG_30, R30);
  BK4819_WriteRegister(BK4819_REG_37, R37);
  BK4819_WriteRegister(BK4819_REG_3D, R3D);
  BK4819_WriteRegister(BK4819_REG_43, R43);
  BK4819_WriteRegister(BK4819_REG_47, R47);
  BK4819_WriteRegister(BK4819_REG_48, R48);
  BK4819_WriteRegister(BK4819_REG_7E, R7E);
  BK4819_WriteRegister(BK4819_REG_02, R02);
  BK4819_WriteRegister(BK4819_REG_3F, R3F);
  BK4819_WriteRegister(BK4819_REG_7B, R7B);
  BK4819_WriteRegister(BK4819_REG_12, R12);
  BK4819_WriteRegister(BK4819_REG_11, R11);
  BK4819_WriteRegister(BK4819_REG_14, R14);
  BK4819_WriteRegister(BK4819_REG_54, R54);
  BK4819_WriteRegister(BK4819_REG_55, R55);
  BK4819_WriteRegister(BK4819_REG_75, R75);
}

static void ToggleAFBit(bool on) {
  uint32_t reg = reg_47_cache;
    reg &= ~(1 << 8);
    if (on)
        reg |= on << 8;
    BK4819_WriteRegister(BK4819_REG_47, reg);
}

static void ToggleAFDAC(bool on) {
  uint32_t Reg = reg_30_cache;
    Reg &= ~(1 << 9);
    if (on)
        Reg |= (1 << 9);
    BK4819_WriteRegister(BK4819_REG_30, Reg);
}

static void SetF(uint32_t sf) {
    uint32_t f = sf;
    if (f < 1400000 || f > 130000000) return;
    if (SPECTRUM_PAUSED) return;
    BK4819_SetFrequency(f);
    static uint8_t lastFilterPath = 0xFF; 
    uint8_t currentFilterPath = (f < 28000000) ? 1 : 0;
    if (currentFilterPath != lastFilterPath) BK4819_PickRXFilterPathBasedOnFrequency(f);
    uint16_t reg = reg_30_cache;
    BK4819_WriteRegister(BK4819_REG_30, 0x200); //AF DAC Enable.
    BK4819_WriteRegister(BK4819_REG_30, reg);
    lastFilterPath = currentFilterPath;
}

static void ResetInterrupts()
{
  // disable interupts
  BK4819_WriteRegister(BK4819_REG_3F, 0);
  // reset the interrupt
  BK4819_WriteRegister(BK4819_REG_02, 0);
}

// scan step in 0.01khz
static uint32_t GetScanStep() { return scanStepValues[settings.scanStepIndex]; }

static uint16_t GetStepsCount() 
{ 
  if (appMode==CHANNEL_MODE)    { return scanChannelsCount; }
  if (appMode==SCAN_RANGE_MODE) { return (SpectrumRangeStop - SpectrumRangeStart) / scanInfo.scanStep;}
  if (appMode==SCAN_BAND_MODE)  { return (SpectrumRangeStop - SpectrumRangeStart) / scanInfo.scanStep;}
  
  return 128 >> settings.stepsCount;
}

static uint32_t GetBW() { return GetStepsCount() * GetScanStep(); }

static uint16_t GetRandomChannelFromRSSI(uint16_t maxChannels) {
  uint32_t rssi = rssiHistory[1]*rssiHistory[maxChannels/2];
  if (maxChannels == 0 || rssi == 0) { return 1; }
    // Scale RSSI to [1, maxChannels]
    return 1 + (rssi % maxChannels);
}

static void DeInitSpectrum() {
  RestoreRegisters();
  gVfoConfigureMode = VFO_CONFIGURE;
  isInitialized = false;
  SetState(SPECTRUM);
  #ifdef ENABLE_FEAT_F4HWN_RESUME_STATE
        gEeprom.CURRENT_STATE = 0;
        SETTINGS_WriteCurrentState();
  #endif
  ToggleRX(0);
  SYSTEM_DelayMs(50);
}

static void DeleteHistoryItem(void) {
    if (!historyListActive || indexFs == 0) return;
    if (historyListIndex >= indexFs) {
        historyListIndex = (indexFs > 0) ? indexFs - 1 : 0;
        if (indexFs == 0) return;
    }
    uint16_t indexToDelete = historyListIndex;
    for (uint16_t i = indexToDelete; i < indexFs - 1; i++) {
        HFreqs[i]       = HFreqs[i + 1];
        HBlacklisted[i] = HBlacklisted[i + 1];
        HTimeS[i]       = HTimeS[i + 1];
    }
    indexFs--;
    
    HFreqs[indexFs]         = 0;
    HBlacklisted[indexFs]   = 0xFF;
    HTimeS[indexFs]         = 0;
    if (historyListIndex >= indexFs && indexFs > 0) {
        historyListIndex = indexFs - 1;
    } else if (indexFs == 0) {
        historyListIndex = 0;
    }
    ShowOSDPopup("Deleted");
    
}

static void SaveHistoryToFreeChannel(void) {
    if (!historyListActive) return;

    uint32_t f = HFreqs[historyListIndex];
    if (f < 1000000) return;
    char str[32];
    for (int i = 0; i < MR_CHANNEL_LAST; i++) {
        uint32_t freqInMem;
        PY25Q16_ReadBuffer(0x0000 + (i * 16), (uint8_t *)&freqInMem, 4);
        if (freqInMem != 0xFFFFFFFF && freqInMem == f) {
            sprintf(str, "Exist CH %d", i + 1);
            ShowOSDPopup(str);
            return;
        }
    }
    int freeCh = -1;
    for (int i = 0; i < MR_CHANNEL_LAST; i++) {
        uint8_t checkByte;
        PY25Q16_ReadBuffer(0x0000 + (i * 16), &checkByte, 1);
        if (checkByte == 0xFF) { 
            freeCh = i;
            break;
        }
    }

    if (freeCh != -1) {
        VFO_Info_t tempVFO;
        memset(&tempVFO, 0, sizeof(tempVFO)); 
        tempVFO.freq_config_RX.Frequency = f;
        tempVFO.freq_config_TX.Frequency = f; 
        tempVFO.TX_OFFSET_FREQUENCY = 0;
        tempVFO.Modulation = settings.modulationType;
        tempVFO.CHANNEL_BANDWIDTH = settings.listenBw; 
        tempVFO.OUTPUT_POWER = OUTPUT_POWER_LOW;
        tempVFO.STEP_SETTING = STEP_12_5kHz; 
        SETTINGS_SaveChannel(freeCh, 0, &tempVFO, 2);
        LoadActiveScanFrequencies();
        sprintf(str, "SAVED TO CH %d", freeCh + 1);
        ShowOSDPopup(str);
    } else {
        ShowOSDPopup("MEMORY FULL");
    }
}

static bool historyLoaded = false; // flaga stanu wczytania histotii spectrum

void LoadHistory(void) {
    HistoryStruct History = {0};
    memset(HFreqs, 0, sizeof(HFreqs));
    memset(HBlacklisted, 0, sizeof(HBlacklisted));
    memset(HTimeS, 0, sizeof(HTimeS));
    indexFs = 0;

    for (uint16_t position = 0; position < HISTORY_SIZE; position++) {
        PY25Q16_ReadBuffer(ADRESS_HISTORY + position * sizeof(HistoryStruct),
                          (uint8_t *)&History, sizeof(HistoryStruct));
        if (History.HBlacklisted == 0xFF) {
            indexFs = position;
            break;
        }
      if (History.HFreqs){
        HFreqs[position]        = History.HFreqs;
        HBlacklisted[position]  = History.HBlacklisted;
        HTimeS[position]        = History.HTimeS;
        indexFs                 = position + 1;
      }
    }
}

static void CompactHistory(void) {
    uint16_t w = 0;
    uint16_t limit = (indexFs > HISTORY_SIZE) ? HISTORY_SIZE : indexFs;

    for (uint16_t r = 0; r < limit; r++) {
        if (HFreqs[r] == 0) continue;
        if (w != r) {
            HFreqs[w]       = HFreqs[r];
            HBlacklisted[w] = HBlacklisted[r];
            HTimeS[w]       = HTimeS[r];
        }
        w++;
    }

    for (uint16_t i = w; i < limit; i++) {
        HFreqs[i]       = 0;
        HBlacklisted[i] = 0;
        HTimeS[i]       = 0;
    }

    indexFs = w;
    if (indexFs == 0) {
        historyListIndex = 0;
        historyScrollOffset = 0;
    } else {
        if (historyListIndex >= indexFs) historyListIndex = indexFs - 1;
        if (historyScrollOffset >= indexFs) {
            historyScrollOffset = (indexFs > MAX_VISIBLE_LINES) ? (indexFs - MAX_VISIBLE_LINES) : 0;
        }
    }
}

void SaveHistory(void) {
    HistoryStruct History = {0};
    CompactHistory();
    for (uint16_t position = 0; position < indexFs; position++) {
        History.HFreqs          = HFreqs[position];
        History.HBlacklisted    = HBlacklisted[position];
        History.HTimeS          = HTimeS[position];
        PY25Q16_WriteBuffer(ADRESS_HISTORY + position * sizeof(HistoryStruct),
                           (uint8_t *)&History, sizeof(HistoryStruct), 0);
    }

    History.HFreqs = 0;
    History.HBlacklisted = 0xFF;
    History.HTimeS    = 0;
    
    PY25Q16_WriteBuffer(ADRESS_HISTORY + indexFs * sizeof(HistoryStruct),
                       (uint8_t *)&History, sizeof(HistoryStruct), 0);
    
    ShowOSDPopup("HISTORY SAVED");
}

static void ExitAndCopyToVfo() {
    RestoreRegisters();

    if (historyListActive) {
        SetF(HFreqs[historyListIndex]);
        gCurrentVfo->Modulation = MODULATION_FM;
        gRequestSaveChannel = 1;
        DeInitSpectrum();
    }

    switch (currentState) {
        case SPECTRUM:
            // PTT Mode 1: NINJA MODE (Random channel with low RSSI)
            if (PttEmission == 1 && scanChannelsCount > 0) {
                uint16_t randomChannel = GetRandomChannelFromRSSI(scanChannelsCount);
                uint32_t rndfreq = 0;
                uint16_t attempts = 0;
                SpectrumDelay = 0; //not compatible with ninja
                while (attempts < scanChannelsCount) {
                    rndfreq = ScanFrequencies[randomChannel];
                    if (rssiHistory[randomChannel] <= 120 && rndfreq) {break;}
                    attempts++;
                    randomChannel = (randomChannel + 1) % scanChannelsCount;
                }
                if (rndfreq) {
                    gCurrentVfo->freq_config_TX.Frequency = rndfreq;
                    gCurrentVfo->freq_config_RX.Frequency = rndfreq;
                    gEeprom.MrChannel[0]     = randomChannel;
                    gEeprom.ScreenChannel[0] = randomChannel;
                    gCurrentVfo->Modulation   = MODULATION_FM;
                    gCurrentVfo->STEP_SETTING = STEP_0_01kHz;
                    gRequestSaveChannel       = 1;
                }
            }
            // PTT Mode 2: Last RX
            if (PttEmission == 2) {
                SpectrumDelay = 0;
                gCurrentVfo->freq_config_TX.Frequency = lastReceivingFreq;
                gCurrentVfo->freq_config_RX.Frequency = lastReceivingFreq;
                gEeprom.MrChannel[0]     = 0;
                gEeprom.ScreenChannel[0] = 0;
                gCurrentVfo->STEP_SETTING = STEP_0_01kHz;
                gCurrentVfo->Modulation   = MODULATION_FM;
                gCurrentVfo->OUTPUT_POWER  = OUTPUT_POWER_HIGH;
                gRequestSaveChannel        = 1;
            }
            // PTT Mode 0: VFO Freq
            gComeBack = 1;
            DeInitSpectrum();
            break;

        default:
            DeInitSpectrum();
            break;
    }

    SYSTEM_DelayMs(200);
    isInitialized = false;
}

static uint16_t GetRssi(void) {
    uint16_t rssi;
    if (isListening) SYSTICK_DelayUs(12000);
    else             SYSTICK_DelayUs(DelayRssi);
    rssi = BK4819_GetRSSI();
    //if (scanInfo.f > 30000000) {rssi += UHF_NOISE_FLOOR;}
    return rssi;
}

static void ToggleAudio(bool on) {
    if (on == audioState) { return; }
    audioState = on;
    if (on) {GPIO_EnableAudioPath();}
    else {GPIO_DisableAudioPath();}
}

static uint16_t CountValidHistoryItems() {
    return (indexFs > HISTORY_SIZE) ? HISTORY_SIZE : indexFs;
}

static void FillfreqHistory()
{   
    lastHistoryScrollOffset = -1;
    uint32_t f = peak.f;
    if (f == 0 || f < 1400000 || f > 130000000) return;

    uint16_t foundIndex = 0xFFFF;
    uint16_t foundTime = 0;
    bool foundBlacklisted = false;
    
    for (uint16_t i = 0; i < indexFs; i++) {
        if (HFreqs[i] == f) {
            foundIndex = i;
            foundTime = HTimeS[i];
            foundBlacklisted = HBlacklisted[i];
            break;
        }
    }
    bool freezeOrder = historyListActive && (SpectrumMonitor || gHistoryScan);
    if (freezeOrder) {
            if (foundIndex != 0xFFFF) { HTimeS[foundIndex] = foundTime; }
            lastReceivingFreq = f;
            return;
        }

    if (foundIndex != 0xFFFF) {
        for (uint16_t i = foundIndex; i + 1 < indexFs; i++) {
            HFreqs[i]       = HFreqs[i + 1];
            HBlacklisted[i] = HBlacklisted[i + 1];
            HTimeS[i] = HTimeS[i + 1];
        }
        if (indexFs > 0) indexFs--;
    }

    uint16_t limit = (indexFs < HISTORY_SIZE) ? indexFs : (HISTORY_SIZE - 1);
    for (int i = limit; i > 0; i--) {
        HFreqs[i]       = HFreqs[i - 1];
        HBlacklisted[i] = HBlacklisted[i - 1];
        HTimeS[i]       = HTimeS[i - 1];
    }

    HFreqs[0] = f;
    HBlacklisted[0] = foundBlacklisted;
    HTimeS[0] = foundTime;
    if (indexFs < HISTORY_SIZE) indexFs++;
    historyListIndex = 0;
    lastReceivingFreq = f;
} 

static void ToggleRX(bool on) {
    if (SPECTRUM_PAUSED || settings.rssiTriggerLevelUp == 50) return;
    if(!on && SpectrumMonitor == 2) {isListening = 1;return;}
    isListening = on;
    gChannel = BOARD_gMR_fetchChannel(scanInfo.f);
    isKnownChannel = (gChannel != 0xFFFF);
    if (on && isKnownChannel) {
        LookupChannelModulation();
        settings.modulationType = channelModulation;
        SETTINGS_FetchChannelName(channelName,gChannel );
        if(!gForceModulation) settings.modulationType = channelModulation;
        RADIO_SetupAGC(settings.modulationType == MODULATION_AM, false);
    }
    if(on && appMode == SCAN_BAND_MODE) {
            if (!gForceModulation) settings.modulationType = BParams[bl].modulationType;
            RADIO_SetupAGC(settings.modulationType == MODULATION_AM, false);
          }
    
    if (on) { 
        Fmax = peak.f;
        BK4819_RX_TurnOn();
        SYSTEM_DelayMs(20);
        RADIO_SetModulation(settings.modulationType);
        BK4819_SetFilterBandwidth(settings.listenBw, false);
        BK4819_WriteRegister(BK4819_REG_3F, BK4819_REG_02_CxCSS_TAIL);

    } else { 
        RADIO_SetModulation(MODULATION_FM);
        BK4819_SetFilterBandwidth(BK4819_FILTER_BW_WIDE, false); //Scan in 25K bandwidth
        BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, 0);
        BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, 0);
        channelName[0] = '\0';
    }
    if (on != audioState) {
        ToggleAudio(on);
        ToggleAFDAC(on);
        ToggleAFBit(on);
    }
}

#ifdef ENABLE_BENCH
static void ResetBenchStats(void) {
    benchTickMs = 0;
    benchStepsThisSec = 0;
    benchRatePerSec = 0;
    benchLapMs = 0;
    benchLastLapMs = 0;
}
#endif

static void ResetScanStats() {
  scanInfo.rssiMax = scanInfo.rssiMin + 20 ; 
}

static bool InitScan() {
    ResetScanStats();
    scanInfo.i = 0;
    peak.i = 0;
    peak.f = 0;
    
    bool scanInitializedSuccessfully = false;
    switch (appMode) {
        case SCAN_BAND_MODE:
            uint8_t checkedBandCount = 0;
            while (checkedBandCount < bandCount) { 
                if (settings.bandEnabled[nextBandToScanIndex]) {
                    bl = nextBandToScanIndex; 
                    scanInfo.f = BParams[bl].Startfrequency;
                    scanInfo.scanStep = scanStepValues[BParams[bl].scanStep];
                    settings.scanStepIndex = BParams[bl].scanStep; 
                    if(BParams[bl].Startfrequency>0) SpectrumRangeStart = BParams[bl].Startfrequency;
                    if(BParams[bl].Stopfrequency>0)  SpectrumRangeStop = BParams[bl].Stopfrequency;
                    if (!gForceModulation) settings.modulationType = BParams[bl].modulationType;
                    nextBandToScanIndex = (nextBandToScanIndex + 1) % bandCount;
                    scanInitializedSuccessfully = true;
                    break;
                }
                nextBandToScanIndex = (nextBandToScanIndex + 1) % bandCount;
                checkedBandCount++;
            }
            break;

        case SCAN_RANGE_MODE:
            SpectrumRangeStart = RangeStart;
            SpectrumRangeStop  = RangeStop;
            if(SpectrumRangeStart > SpectrumRangeStop)
	    	    SWAP(SpectrumRangeStart, SpectrumRangeStop);
            scanInfo.f = SpectrumRangeStart;
            scanInfo.scanStep = GetScanStep();
            scanInitializedSuccessfully = true;
            break;

        case FREQUENCY_MODE:
            currentFreq = gTxVfo->pRX->Frequency;
            SpectrumRangeStart = currentFreq - (GetBW() >> 1);
            SpectrumRangeStop  = currentFreq + (GetBW() >> 1);
            break;

        case CHANNEL_MODE:
            if (scanChannelsCount == 0) {return false;}
            scanInfo.f = ScanFrequencies[0];
            peak.f = scanInfo.f;
            peak.i = 0;
            break;
    }
    return scanInitializedSuccessfully;
}

static void ResetModifiers() {
  memset(StringC, 0, sizeof(StringC)); 
  for (int i = 0; i < 128; ++i) {
    if (rssiHistory[i] == RSSI_MAX_VALUE) rssiHistory[i] = 0;
  }
  LoadActiveScanFrequencies();
  RelaunchScan();
}

static void RelaunchScan() {
    InitScan();
    ToggleRX(false);
    scanInfo.rssiMin = RSSI_MAX_VALUE;
    gIsPeak = false;
#ifdef ENABLE_BENCH
    	ResetBenchStats();
#endif
}

uint8_t  BK4819_GetExNoiseIndicator(void)
{
	return BK4819_ReadRegister(BK4819_REG_65) & 0x007F;
}

static void UpdateNoiseOff(){
    if (!Noislvl_OFF) return;
    if( BK4819_GetExNoiseIndicator() > Noislvl_OFF) {gIsPeak = false;ToggleRX(0);}		
}

static void UpdateNoiseOn(){
	if( BK4819_GetExNoiseIndicator() < Noislvl_ON) {gIsPeak = true;ToggleRX(1);}
}

static void UpdateScanInfo() {
  if (scanInfo.rssi > scanInfo.rssiMax) {
    scanInfo.rssiMax = scanInfo.rssi;
  }
  if (scanInfo.rssi < scanInfo.rssiMin && scanInfo.rssi > 0) {
    scanInfo.rssiMin = scanInfo.rssi;
  }
}
static void UpdateGlitch() {
    if (!GlitchMax) return;
    uint8_t glitch = BK4819_GetGlitchIndicator();
    if (glitch > GlitchMax) {gIsPeak = false;} 
    else {gIsPeak = true;}// if glitch is too high, receiving stopped
}

static void Measure() {
    static int16_t previousRssi = 0;
    static bool isFirst = true;
    uint16_t rssi = scanInfo.rssi = GetRssi();
    UpdateScanInfo();
    if (scanInfo.f % 1300000 == 0 || IsBlacklisted(scanInfo.f)) rssi = scanInfo.rssi = 0;
    if (isFirst) {
        previousRssi = rssi;
        gIsPeak      = false;
        isFirst      = false;
    }
    if (settings.rssiTriggerLevelUp == 50) {
        if  (rssi > previousRssi + UOO_trigger)  {
            peak.f = scanInfo.f;
            peak.i = scanInfo.i;
            gIsPeak = false;
            isListening = false;
            FillfreqHistory();
        }
    } else {
            if (!gIsPeak && rssi > previousRssi + settings.rssiTriggerLevelUp) {
                SYSTEM_DelayMs(10);
                uint16_t rssi2 = scanInfo.rssi = GetRssi();
                if (rssi2 > rssi + 10) {
                    peak.f = scanInfo.f;
                    peak.i = scanInfo.i-1;
                    if (settings.rssiTriggerLevelUp < 50) {
                        gIsPeak = true;
                        UpdateNoiseOff();
                        UpdateGlitch();
                    }
                }
            
            scanInfo.rssi = GetRssi();
            }
    } 
    if (!gIsPeak || !isListening) previousRssi = rssi;
    else if (rssi < previousRssi) previousRssi = rssi;
    if (ShowLines == 2) return;
    
    uint16_t count = GetStepsCount();
    uint16_t i = scanInfo.i;
    static uint8_t pixel;
    if (count > 128) {
        uint32_t diff = (scanInfo.f - SpectrumRangeStart) / 100;
        uint32_t span = (SpectrumRangeStop - SpectrumRangeStart) / 100;
        if (span > 0) {
            pixel = (diff * 127) / span;
            if (pixel < 128) {
                rssiHistory[pixel] = rssi;
            }
        }
    }

    if (count <= 128) {
        uint16_t base = 128 / count;
        uint16_t rem  = 128 % count;
        uint16_t start = i * base + (i < rem ? i : rem);
        uint16_t end   = (i + 1) * base + ((i + 1) < rem ? (i + 1) : rem);
        if (end > 128) end = 128;
        for (uint16_t j = start; j < end; ++j) {rssiHistory[j] = rssi;}
    }

}

static void AutoAdjustFreqChangeStep() {
  settings.frequencyChangeStep = SpectrumRangeStop - SpectrumRangeStart;
}

static void UpdateScanStep(bool inc) {
if (inc) {
    settings.scanStepIndex = (settings.scanStepIndex >= STEP_500kHz) 
                          ? STEP_0_01kHz 
                          : settings.scanStepIndex + 1;
} else {
    settings.scanStepIndex = (settings.scanStepIndex <= STEP_0_01kHz) 
                          ? STEP_500kHz 
                          : settings.scanStepIndex - 1;
}
  AutoAdjustFreqChangeStep();
  scanInfo.scanStep = settings.scanStepIndex;
}

static void UpdateCurrentFreq(bool inc) {
    AutoAdjustFreqChangeStep();
    if (inc) {
        gTxVfo->pRX->Frequency += settings.frequencyChangeStep;
    } else {
        gTxVfo->pRX->Frequency -= settings.frequencyChangeStep;
    }
  ResetModifiers();
}

static void ToggleModulation() {
  if (settings.modulationType < MODULATION_UKNOWN - 1) {
    settings.modulationType++;
  } else {
    settings.modulationType = MODULATION_FM;
  }
  RADIO_SetModulation(settings.modulationType);
  BK4819_InitAGC(settings.modulationType);
  gForceModulation = 1;
}

static void ToggleListeningBW(bool inc) {
  settings.listenBw = ACTION_NextBandwidth(settings.listenBw, false, inc);
  BK4819_SetFilterBandwidth(settings.listenBw, false);
  
}

static void ToggleStepsCount() {
  if (settings.stepsCount == STEPS_128) {
    settings.stepsCount = STEPS_16;
  } else {
    settings.stepsCount--;
  }
  AutoAdjustFreqChangeStep();
  ResetModifiers();
  
}

static void ResetFreqInput() {
  tempFreq = 0;
  for (int i = 0; i < 10; ++i) {
    freqInputString[i] = '-';
  }
}

static void FreqInput() {
  freqInputIndex = 0;
  freqInputDotIndex = 0;
  ResetFreqInput();
  SetState(FREQ_INPUT);
  Key_1_pressed = 1;
}

static void UpdateFreqInput(KEY_Code_t key) {
  if (key != KEY_EXIT && freqInputIndex >= 10) {
    return;
  }
  if (key == KEY_STAR) {
    if (freqInputIndex == 0 || freqInputDotIndex) {
      return;
    }
    freqInputDotIndex = freqInputIndex;
  }
  if (key == KEY_EXIT) {
    freqInputIndex--;
    if(freqInputDotIndex==freqInputIndex)
      freqInputDotIndex = 0;    
  } else {
    freqInputArr[freqInputIndex++] = key;
  }

  ResetFreqInput();

  uint8_t dotIndex =
      freqInputDotIndex == 0 ? freqInputIndex : freqInputDotIndex;

  KEY_Code_t digitKey;
  for (int i = 0; i < 10; ++i) {
    if (i < freqInputIndex) {
      digitKey = freqInputArr[i];
      freqInputString[i] = digitKey <= KEY_9 ? '0' + digitKey-KEY_0 : '.';
    } else {
      freqInputString[i] = '-';
    }
  }

  uint32_t base = 100000; // 1MHz in BK units
  for (int i = dotIndex - 1; i >= 0; --i) {
    tempFreq += (freqInputArr[i]-KEY_0) * base;
    base *= 10;
  }

  base = 10000; // 0.1MHz in BK units
  if (dotIndex < freqInputIndex) {
    for (int i = dotIndex + 1; i < freqInputIndex; ++i) {
      tempFreq += (freqInputArr[i]-KEY_0) * base;
      base /= 10;
    }
  }
  
}

static void Skip() {
    isListening = 0;
    WaitSpectrum = 0;
    spectrumElapsedCount = 0;
    NextScanStep();
    gIsPeak = false;
    ToggleRX(false);
    peak.f = scanInfo.f;
    peak.i = scanInfo.i;
    ToggleAudio(0);
    ToggleAFDAC(0);
    ToggleAFBit(0);
}

static bool IsBlacklisted(uint32_t f) {
    for (uint16_t i = 0; i < HISTORY_SIZE; i++) {
        if (HFreqs[i] == f && HBlacklisted[i]) {
            return true;
        }
    }
    return false;
}

static void Blacklist() {
    if (lastReceivingFreq == 0) return;
    Skip();
    for (uint16_t i = 0; i < HISTORY_SIZE; i++) {
        if (HFreqs[i] == lastReceivingFreq) {
            HBlacklisted[i] = true;
            historyListIndex = i;
            return;
        }
    }
    HFreqs[indexFs]   = lastReceivingFreq;
    HBlacklisted[indexFs] = true;
    historyListIndex = indexFs;
    if (++indexFs >= HISTORY_SIZE) {
      historyScrollOffset = 0;
      indexFs=0;
    }  
}

// ============================================================
// SECTION: Display / rendering helpers
// ============================================================

#ifdef ENABLE_PERSIST
static uint8_t iSqrt(uint16_t n)
{
    if (n == 0) return 0;
    uint16_t x = n;
    uint16_t y = (x + 1) >> 1;
    while (y < x) { x = y; y = (x + n / x) >> 1; }
    return (uint8_t)x;
}

uint8_t Rssi2PX(uint16_t rssi, uint8_t pxMin, uint8_t pxMax)
{
    const int DB_MIN = settings.dbMin << 1;
    const int DB_MAX = settings.dbMax << 1;
    const int DB_RANGE = DB_MAX - DB_MIN;
    const uint8_t PX_RANGE = pxMax - pxMin;
    int dbm = clamp(Rssi2DBm(rssi) << 1, DB_MIN, DB_MAX);
    uint8_t linear = (uint8_t)(((dbm - DB_MIN) * PX_RANGE + DB_RANGE / 2) / DB_RANGE);
    uint8_t compressed = iSqrt((uint16_t)linear * PX_RANGE);
    return ((uint16_t)linear + compressed) / 2 + pxMin;
}

uint8_t Rssi2Y(uint16_t rssi)
{
  int delta = ArrowLine*8;
  return DrawingEndY + delta -Rssi2PX(rssi, delta, DrawingEndY);
}
#else

static uint16_t Rssi2PX(uint16_t rssi, uint16_t pxMin, uint16_t pxMax) {
  const int16_t DB_MIN = settings.dbMin << 1;
  const int16_t DB_MAX = settings.dbMax << 1;
  const int16_t DB_RANGE = DB_MAX - DB_MIN;
  const int16_t PX_RANGE = pxMax - pxMin;
  int dbm = clamp(rssi - (160 << 1), DB_MIN, DB_MAX);
  return ((dbm - DB_MIN) * PX_RANGE + DB_RANGE / 2) / DB_RANGE + pxMin;
}

static int16_t Rssi2Y(uint16_t rssi) {
  int delta = ArrowLine*8;
  return DrawingEndY + delta -Rssi2PX(rssi, delta, DrawingEndY);
}

    #ifdef ENABLE_ADVANCED_SPECTRUM
static void DrawSpectrum(void) {
    // Build topY[] with rssi → Y-coordinate conversion
    uint8_t topY[128];
    for (uint8_t x = 0; x < 128; x++) {
        if (!rssiHistory[x]) { topY[x] = 0xFF; continue; }
        int16_t y = Rssi2Y(rssiHistory[x]);
        topY[x] = (y >= 0 && y <= DrawingEndY) ? (uint8_t)y : 0xFF;
    }

    // Peak smoothing: each pixel = average with neighbors
    uint8_t prev = topY[0];
    for (uint8_t x = 1; x < 127; x++) {
        uint8_t cur = topY[x], nxt = topY[x + 1];
        if (cur == 0xFF) { prev = cur; continue; }
        uint16_t s = cur; uint8_t n = 1;
        if (prev != 0xFF) { s += prev; n++; }
        if (nxt  != 0xFF) { s += nxt;  n++; }
        prev = cur;
        topY[x] = (uint8_t)((s + n / 2) / n);
    }

    // Draw the fill
    for (uint8_t x = 0; x < 128; x++) {
        uint8_t y0 = topY[x];
        if (y0 == 0xFF || y0 > DrawingEndY) continue;

        // Bridge to neighbors — solid ridge
        uint8_t ct = y0, cb = y0;
        if (x > 0   && topY[x-1] != 0xFF) { uint8_t m = (y0 + topY[x-1] + 1) >> 1; if (m < ct) ct = m; if (m > cb) cb = m; }
        if (x < 127 && topY[x+1] != 0xFF) { uint8_t m = (y0 + topY[x+1] + 1) >> 1; if (m < ct) ct = m; if (m > cb) cb = m; }
        for (uint8_t y = ct; y <= cb; y++)
            gFrameBuffer[y >> 3][x] |= 1 << (y & 7);

        // Make the black line above thinner: only 25% of remaining height instead of 50%
        uint8_t mid = cb + ((DrawingEndY - cb) >> 3);  // >> 2 ticker
        
        // Solid top area (thinner)
        for (uint8_t y = cb + 1; y < mid; y++)
            gFrameBuffer[y >> 3][x] |= 1 << (y & 7);
            
        // Checkerboard bottom area
        for (uint8_t y = mid; y <= DrawingEndY; y++)
            if (!((x + y) & 1))
                gFrameBuffer[y >> 3][x] |= 1 << (y & 7);
    }
}
    #else
    static void DrawSpectrum(void) {
        int16_t y_baseline = Rssi2Y(0); 
        for (uint8_t i = 0; i < 128; i++) {
            int16_t y_curr = Rssi2Y(rssiHistory[i]);
            for (int16_t y = y_curr; y <= y_baseline; y++) {
                    gFrameBuffer[y >> 3][i] |= (1 << (y & 7));
                }
            }
    }
    #endif
#endif



static void RemoveTrailZeros(char *s) {
    char *p;
    if (strchr(s, '.')) {
        p = s + strlen(s) - 1;
        while (p > s && *p == '0') {
            *p-- = '\0';
        }
        if (*p == '.') {
            *p = '\0';
        }
    }
}

static void DrawStatus() {

  int len=0;
  int pos=0;
   
switch(SpectrumMonitor) {
    case 0:
      len = sprintf(&String[pos],"");
      pos += len;
      if (settings.rssiTriggerLevelUp == 50) len = sprintf(&String[pos],"");
      else len = sprintf(&String[pos],"DS%d ", settings.rssiTriggerLevelUp);
      pos += len;
    break;

    case 1:
      len = sprintf(&String[pos],"FL ");
      pos += len;
    break;

    case 2:
      len = sprintf(&String[pos],"M ");
      pos += len;
    break;
  } 
  
  
  len = sprintf(&String[pos],"%sms %s BW%s ", DelayRssiText[IndexDelayRssi], gModulationStr[settings.modulationType],bwNames[settings.listenBw]);
  pos += len;
  int16_t afcVal = BK4819_GetAFCValue();
  if (afcVal) {
      len = sprintf(&String[pos],"A%+d ", afcVal);
      pos += len;
  } else {
      len = sprintf(&String[pos], "ST%s", scanStepNames[settings.scanStepIndex]);
      pos += len;
    }
   
  GUI_DisplaySmallest(String, 0, 1, true,true);
  BOARD_ADC_GetBatteryInfo(&gBatteryVoltages[gBatteryCheckCounter++ % 4],&gBatteryCurrent);

  uint16_t voltage = (gBatteryVoltages[0] + gBatteryVoltages[1] + gBatteryVoltages[2] +
             gBatteryVoltages[3]) /
            4 * 760 / gBatteryCalibration[3];

  unsigned perc = BATTERY_VoltsToPercent(voltage);
  sprintf(String,"%d%%", perc);
  GUI_DisplaySmallest(String, 112, 1, true,true);
}

// ------------------ Frequency string ------------------
static void FormatFrequency(uint32_t f, char *buf, size_t buflen) {
    snprintf(buf, buflen, "%u.%05u", f / 100000, f % 100000);

}

// ------------------ CSS detection ------------------


static void UpdateCssDetection(void) {
    static uint8_t LCode = 0;
    if (CodeFreq == peak.f) return; //we already have a code for this freq
    BK4819_WriteRegister(BK4819_REG_51,
        BK4819_REG_51_ENABLE_CxCSS |
        BK4819_REG_51_AUTO_CDCSS_BW_ENABLE |
        BK4819_REG_51_AUTO_CTCSS_BW_ENABLE |
        (51u << BK4819_REG_51_SHIFT_CxCSS_TX_GAIN1));

    BK4819_CssScanResult_t scanResult = BK4819_GetCxCSSScanResult(&cdcssFreq, &ctcssFreq);

    if (scanResult == BK4819_CSS_RESULT_CDCSS) {
        LCode = DCS_GetCdcssCode(cdcssFreq);
        if (LCode != 0xFF) {
            CodeFreq = peak.f;
            snprintf(StringCode, sizeof(StringCode), " D%03oN ", DCS_Options[LCode]);
            code = LCode += 100;
            return;
        }
    } else if (scanResult == BK4819_CSS_RESULT_CTCSS) {
        LCode = DCS_GetCtcssCode(ctcssFreq);
        if (LCode != 0xFF) {
            CodeFreq = peak.f;
            snprintf(StringCode, sizeof(StringCode), " %u.%uHz ", CTCSS_Options[LCode] / 10, CTCSS_Options[LCode] % 10);
            code = LCode;
            return;
        }
    }

    StringCode[0] = '\0';
}

static void ScanProgress_DrawGaugeLine(uint8_t line)
{
    if (line >= 8) return; 
    
    static uint32_t total = 0;
    static uint16_t current_index = 0;
    static uint32_t globalStepOffset = 0;

    if (appMode == SCAN_BAND_MODE) {
        globalStepOffset = 0;
        
        for (uint8_t i = 0; i < bl; i++) {
            if (scanStepValues[BParams[i].scanStep] && settings.bandEnabled[i]) {
                globalStepOffset += (BParams[i].Stopfrequency - BParams[i].Startfrequency) / scanStepValues[BParams[i].scanStep];
            }
        }
        current_index = globalStepOffset + scanInfo.i;
        total = 0;
        for (uint8_t i = 0; i < bandCount; i++) {
            if (scanStepValues[BParams[i].scanStep] && settings.bandEnabled[i]) {
                total += (BParams[i].Stopfrequency - BParams[i].Startfrequency) / scanStepValues[BParams[i].scanStep];
            }
        }
    } else {
        total = GetStepsCount();
        current_index = scanInfo.i;
    }
    const uint8_t fill_start = 4;
    const uint8_t fill_cols  = 121;

    if (total == 0) total = 1;
    if (current_index > total) current_index = total;

    uint8_t filled_until = (uint8_t)((current_index * (uint32_t)fill_cols) / total);
    for (uint8_t col = 0; col < fill_cols; col++) {
        gFrameBuffer[line][fill_start + col] = (col < filled_until) ? 0x3C : 0x00;
    }
}

static void DrawF(uint32_t f) {
    static uint32_t fprev;
    if ((f == 0) || f < 1400000 || f > 130000000) f=fprev;
    else fprev = f;
    char freqStr[18];
    snprintf(freqStr, sizeof(freqStr), "%u.%05u", f / 100000, f % 100000);
    char line1[19] = "";
    char line2[19] = "";
    char line3[19] = "";
    sprintf(line1, "%s", freqStr);
    char prefix[9] = "";
    UpdateCssDetection();
    if (appMode == SCAN_BAND_MODE) {
        snprintf(prefix, sizeof(prefix), "B%u ", bl + 1);
        if (isListening && isKnownChannel) {
            snprintf(line2, sizeof(line2), " %-3s%s %s ", prefix, channelName, StringCode);
        } else {
            snprintf(line2, sizeof(line2), " %s%s %s ", prefix, BParams[bl].BandName, StringCode);
        }
    } else if (appMode == CHANNEL_MODE) {

        if (channelName[0] != '\0') {
            snprintf(line2, sizeof(line2), "%s %s ", channelName, StringCode);
        } else {
            snprintf(line2, sizeof(line2), "%s ", StringCode);
        }
    } else {
        line2[0] = '\0';
    }

    line3[0] = '\0';
    int pos = 0;

    if(isListening){
        if (MaxListenTime > 0) {
            pos += sprintf(&line3[pos], " RX %d/%s ", spectrumElapsedCount/1000, labels[IndexMaxLT]);

            if (WaitSpectrum > 0) {
                if (WaitSpectrum < 61000) {
                    pos += sprintf(&line3[pos], "| Wait %ds ", WaitSpectrum / 1000);
                } else {
                    pos += sprintf(&line3[pos], "End OO ");
                }
            }
        }
        else {
            pos += sprintf(&line3[pos], " RX %ds  ", spectrumElapsedCount/1000);

            if (WaitSpectrum > 0) {
                if (WaitSpectrum < 61000) {
                    pos += sprintf(&line3[pos], "| Wait %ds ", WaitSpectrum / 1000);
                } else {
                    pos += sprintf(&line3[pos], "End OO ");
                }
            }
        }
    } else ArrowLine = 2;
    static char Text[20]="";
    
    switch(ShowLines) {
            case 1: {       // BIG FREQUENCY
                if(isListening) { sprintf(Text, "%d dBm", Rssi2DBm(scanInfo.rssi)); }
                else { 
                    if (lastReceivingFreq >= 1400000 && lastReceivingFreq <= 130000000) {
                        snprintf(Text, sizeof(Text), "%u.%05u", lastReceivingFreq / 100000, lastReceivingFreq % 100000);
                    }
                }
                UI_DisplayFrequency(line1, 3, 0, 1);  
                UI_PrintStringSmallbackground(line2, 0, 127, 2, 0);  
                GUI_DisplaySmallest(Text, 42, Bottom_print, false, true);
                ArrowLine = 3;
                break;
            }
            case 2: {       //SCAN
                if(isListening) { sprintf(Text, "Signal %d dBm", Rssi2DBm(scanInfo.rssi)); }
                else {switch(PttEmission) {
                    case 0:
                        snprintf(Text, sizeof(Text), "TX %s %u.%05u", gCurrentVfo->Name, gCurrentVfo->freq_config_TX.Frequency  / 100000, gCurrentVfo->freq_config_TX.Frequency  % 100000);
                        break;
                    case 1:
                        if (lastReceivingFreq >= 1400000 && lastReceivingFreq <= 130000000) 
                            snprintf(Text, sizeof(Text), "Ninja RX %u.%05u", lastReceivingFreq / 100000, lastReceivingFreq % 100000);
                            else snprintf(Text, sizeof(Text), "Ninja");
                        break;
                    case 2:
                        if (lastReceivingFreq >= 1400000 && lastReceivingFreq <= 130000000) 
                            snprintf(Text, sizeof(Text), "LastRX %u.%05u", lastReceivingFreq / 100000, lastReceivingFreq % 100000);
                            else snprintf(Text, sizeof(Text), "LastRX");
                        break;
                    case 3:
                    case 4:
                    case 5:
                    case 6:
                    case 7:
                    case 8:
                        snprintf(Text, sizeof(Text), "RO %s %u.%05u", gCurrentVfo->Name, gCurrentVfo->freq_config_TX.Frequency  / 100000, gCurrentVfo->freq_config_TX.Frequency  % 100000);
                        break;
                    
                    }
                
                    
#ifdef ENABLE_BENCH
                snprintf(line3, sizeof(line3), "Rate: %u/s", benchRatePerSec);
#endif
                }
                UI_DisplayFrequency(line1, 3, 0, 1);
                UI_PrintStringSmallbackground(line2, 0, 127, 2, 0);  
                ScanProgress_DrawGaugeLine(3);
                if(isListening) DrawMeter(4);
                UI_PrintStringSmallbackground(Text, 0, 127, 5, 0);
                UI_PrintStringSmallbackground(line3, 0, 127, 6, 0);
                ST7565_BlitLine(4); 
                ST7565_BlitLine(5); 
                ST7565_BlitLine(6);
                break;
            }
    }
}

static void LookupChannelModulation() {
	uint8_t tmp;
	uint8_t data[8];
	PY25Q16_ReadBuffer(gChannel * 16 + 8, data, sizeof(data));
	tmp = data[3] >> 4;
	if (tmp >= MODULATION_UKNOWN)
		tmp = MODULATION_FM;
	channelModulation = tmp;
	if (data[4] == 0xFF) {channelBandwidth = BK4819_FILTER_BW_WIDE;}
	else {
		const uint8_t d4 = data[4];
		channelBandwidth = !!((d4 >> 1) & 1u);
		if(channelBandwidth != BK4819_FILTER_BW_WIDE)
			channelBandwidth = ((d4 >> 5) & 3u) + 1;
	}	
    tmp = data[6];
    if (tmp >= STEP_N_ELEM)
        tmp = STEP_12_5kHz;
    channelStep = tmp;
}




static void DrawNums() {
if (appMode==CHANNEL_MODE) 
{
  uint8_t selectedCount = 0;
  for (uint8_t i = 0; i < validScanListCount; i++) {
      if (settings.scanListEnabled[validScanListIndices[i]]) selectedCount++;
  }
  sprintf(String, "SL:%u/%u", selectedCount, validScanListCount);
  GUI_DisplaySmallest(String, 2, Bottom_print, false, true);
  
  sprintf(String, "CH:%u", scanChannelsCount);
  GUI_DisplaySmallest(String, 101, Bottom_print, false, true);

  return;
}

if(appMode!=CHANNEL_MODE){
    sprintf(String, "%u.%05u", SpectrumRangeStart / 100000, SpectrumRangeStart % 100000);
    GUI_DisplaySmallest(String, 2, Bottom_print, false, true);
 
    sprintf(String, "%u.%05u", SpectrumRangeStop / 100000, SpectrumRangeStop % 100000);
    GUI_DisplaySmallest(String, 90, Bottom_print, false, true);
    }
}

static void NextScanStep() {
    spectrumElapsedCount = 0;
#ifdef ENABLE_BENCH
    benchLapDone = false;
#endif
    static uint32_t StartF;
    if (appMode == CHANNEL_MODE) {
        if (scanChannelsCount == 0) return;
#ifdef ENABLE_BENCH
        uint16_t prevI = scanInfo.i;
#endif
        if (++scanInfo.i >= scanChannelsCount) scanInfo.i = 0;
#ifdef ENABLE_BENCH
        if (scanInfo.i < prevI) benchLapDone = true;
#endif
        scanInfo.f = ScanFrequencies[scanInfo.i];
        return;
    }
    // FREQUENCY / SCAN_RANGE / SCAN_BAND
#ifdef ENABLE_BENCH
    uint16_t prevI = scanInfo.i;
    uint16_t steps = GetStepsCount();
#endif
    if (scanInfo.i == 0) {
        StartF = SpectrumRangeStart;
        scanInfo.f = StartF;
    } else {
            scanInfo.f += jumpSizes[settings.scanStepIndex];
            if (scanInfo.f >= SpectrumRangeStop) {
                StartF += scanInfo.scanStep;
                scanInfo.f = StartF;
            }
    }
    scanInfo.i++;
#ifdef ENABLE_BENCH
    if (scanInfo.i > steps) {
        scanInfo.i = 0;
        newScanStart = true;
        benchLapDone = true;          // pełna pętla zakresu/pasma/freq
    } else if (scanInfo.i < prevI) {
        benchLapDone = true;
    }
#else
    if (scanInfo.i > GetStepsCount()) {
        scanInfo.i = 0;
        newScanStart = true;
    }
#endif
}

void NextAppMode(void) {
    if (Spectrum_state == 1 || !scanChannelsCount) {
        Spectrum_state = 3;
        appMode = SCAN_BAND_MODE;
    } else {
        Spectrum_state = 1;
        appMode = CHANNEL_MODE;
    }
   
    gRequestedSpectrumState  = Spectrum_state;
    gSpectrumChangeRequested = true;
    isInitialized            = false;
    spectrumElapsedCount     = 0;
    WaitSpectrum             = 0;
    gIsPeak                  = false;
    SPECTRUM_PAUSED          = false;
    SpectrumPauseCount       = 0;
    newScanStart             = true;
    ToggleRX(false);
}


static void SetTrigger50(){
    char triggerText[32];
    if (settings.rssiTriggerLevelUp == 50) {
        sprintf(triggerText, "DYN SQL: OFF");
        gMonitorScan = 0;
    }
    else {
        sprintf(triggerText, "DYN SQL: %d", settings.rssiTriggerLevelUp);
    }
    ShowOSDPopup(triggerText);
    Skip();
}
static const uint8_t durations[] = {0, 20, 40, 60};

// ============================================================
// SECTION: Per-state keyboard handlers
// ============================================================

/* --- BAND_LIST_SELECT: navigate and toggle band enable flags --- */
static void HandleKeyBandList(uint8_t key) {
        switch (key) {
            case KEY_UP:
                if (bandListSelectedIndex > 0) {
                    bandListSelectedIndex--;
                if (bandListSelectedIndex < bandListScrollOffset)
                        bandListScrollOffset = bandListSelectedIndex;
                } else {
                    bandListSelectedIndex = bandCount - 1;
                }
                break;
            case KEY_DOWN:
                if (bandListSelectedIndex < bandCount - 1) {
                    bandListSelectedIndex++;
                    if (bandListSelectedIndex >= bandListScrollOffset + MAX_VISIBLE_LINES)
                        bandListScrollOffset = bandListSelectedIndex - MAX_VISIBLE_LINES + 1;
                } else {
                    bandListSelectedIndex = 0;
                }
                break;
            case KEY_4: /* toggle selected band */
                if (bandListSelectedIndex < bandCount) {
                    settings.bandEnabled[bandListSelectedIndex] = !settings.bandEnabled[bandListSelectedIndex]; 
                    nextBandToScanIndex = bandListSelectedIndex; 
                    bandListSelectedIndex++;
                }
                break;
            case KEY_5: /* select only this band */
                if (bandListSelectedIndex < bandCount) {
                    memset(settings.bandEnabled, 0, sizeof(settings.bandEnabled));
                    settings.bandEnabled[bandListSelectedIndex] = true;
                    nextBandToScanIndex = bandListSelectedIndex; 
                }
                break;
            case KEY_MENU:
                if (bandListSelectedIndex < bandCount) {
                    memset(settings.bandEnabled, 0, sizeof(settings.bandEnabled));
                    settings.bandEnabled[bandListSelectedIndex] = true;
                    nextBandToScanIndex = bandListSelectedIndex;
                    gForceModulation = 0; // KOLYAN ADD
                    SetState(SPECTRUM);
                    ResetModifiers();
                    RelaunchScan();
                }
                break;
            case KEY_EXIT:
                    SpectrumMonitor = 0;
                    SetState(SPECTRUM);
                    ResetModifiers();
                    RelaunchScan(); 
                    gForceModulation = 0; // KOLYAN ADD
                    break;
            default:
                break;
        }
    }

/* --- SCANLIST_SELECT: navigate and toggle scan-list enable flags --- */
static void HandleKeyScanList(uint8_t key) {
        switch (key) {
        case KEY_UP:
                if (scanListSelectedIndex > 0) {
                    scanListSelectedIndex--;
                if (scanListSelectedIndex < scanListScrollOffset)
                        scanListScrollOffset = scanListSelectedIndex;
                } else {
                scanListSelectedIndex = validScanListCount - 1;
                    }
                break;
            case KEY_DOWN:
                if (scanListSelectedIndex < validScanListCount - 1) {
                    scanListSelectedIndex++;
                if (scanListSelectedIndex >= scanListScrollOffset + MAX_VISIBLE_LINES)
                        scanListScrollOffset = scanListSelectedIndex - MAX_VISIBLE_LINES + 1;
                } else {
                scanListSelectedIndex = 0;
                }
                break;
        case KEY_4: /* toggle selected list, advance cursor */
            ToggleScanList(validScanListIndices[scanListSelectedIndex], 0);
            if (scanListSelectedIndex < validScanListCount - 1)
                scanListSelectedIndex++;
            break;
        case KEY_5: /* activate only selected list */
            ToggleScanList(validScanListIndices[scanListSelectedIndex], 1);
            break;
        case KEY_MENU: /* activate selected list and start scanning */
            if (scanListSelectedIndex < MR_CHANNELS_LIST) {
                ToggleScanList(validScanListIndices[scanListSelectedIndex], 1);
                SetState(SPECTRUM);
                ResetModifiers();
                gForceModulation = 0; //1 kolyan
            }
            break;
        case KEY_EXIT:
            SpectrumMonitor = 0;
            SetState(SPECTRUM);
            ResetModifiers();
            gForceModulation = 0; //1 kolyan
            break;
        default:
            break;
    }
}

/* --- PARAMETERS_SELECT: navigate settings, edit values --- */
static void HandleKeyParameters(uint8_t key) {
    uint16_t maxRows = GetMaxVisualRows();
    switch (key) {
        case KEY_UP:
            parametersSelectedIndex = (parametersSelectedIndex - 1 + maxRows) % maxRows;
            break;
        case KEY_DOWN:
            parametersSelectedIndex = (parametersSelectedIndex + 1) % maxRows;
            break;
        case KEY_1:
        case KEY_3: {
            bool isKey3 = (key == KEY_3);
            uint16_t realIndex = parametersSelectedIndex;
            if (Light_Mode) {
                realIndex = lightModeMenuMapping[parametersSelectedIndex];
            }
            switch (realIndex) {
                case PARAM_RSSI_DELAY:
                    IndexDelayRssi = isKey3 ?
                                 (IndexDelayRssi >= 5 ? 0 : IndexDelayRssi + 1) :
                                 (IndexDelayRssi == 0 ? 5 : IndexDelayRssi - 1);
                    DelayRssi = DelayRssiValues[IndexDelayRssi];
                    break;
                case PARAM_SPECTRUM_DELAY:
                    if (isKey3) {
                          if (SpectrumDelay < 61000)
                              SpectrumDelay += (SpectrumDelay < 10000) ? 1000 : 5000;
                      } else if (SpectrumDelay >= 1000) {
                          SpectrumDelay -= (SpectrumDelay < 10000) ? 1000 : 5000;
                      }
                      break;
                case PARAM_MAX_LISTEN_TIME:
                    if (isKey3) {
                          if (++IndexMaxLT > LISTEN_STEP_COUNT) IndexMaxLT = 0;
                      } else {
                          if (IndexMaxLT == 0) IndexMaxLT = LISTEN_STEP_COUNT;
                          else IndexMaxLT--;
                      }
                      MaxListenTime = listenSteps[IndexMaxLT];
                      break;
                case PARAM_RANGE_START:
                case PARAM_RANGE_STOP:
                          appMode = SCAN_RANGE_MODE;
                          FreqInput();
                      break;
                case PARAM_SCAN_STEP:
                    UpdateScanStep(isKey3);
                    break;
                case PARAM_LISTEN_BW:
                case PARAM_MODULATION:
                    if (isKey3 || key == KEY_1) {
                        if (parametersSelectedIndex == PARAM_LISTEN_BW)
                              ToggleListeningBW(isKey3 ? 0 : 1);
                        else
                              ToggleModulation();
                      }
                      break;
                case PARAM_POWER_SAVE:
                        if (isKey3) {
                        if (++IndexPS > PS_STEP_COUNT) IndexPS = 0;
                        } else {
                          if (IndexPS == 0) IndexPS = PS_STEP_COUNT;
                          else IndexPS--;
                        }
                        SpectrumSleepMs = PS_Steps[IndexPS];
                      break;
                case PARAM_NOISE_LEVEL_OFF:
                      Noislvl_OFF = isKey3 ? 
                                  (Noislvl_OFF >= 80 ? 0  : Noislvl_OFF + 1) :
                                  (Noislvl_OFF <= 0  ? 80 : Noislvl_OFF - 1);
                      Noislvl_ON = Noislvl_OFF - NoiseHysteresis;                      
                      break;
                case PARAM_OSD_POPUP:
                      static const int osdPopupTimes[] = {0, 200, 300, 500, 1000, 2000, 3000};
                      osdPopupIndex = isKey3 ? 
                                      (osdPopupIndex >= 6 ? 0 : osdPopupIndex + 1):
                                      (osdPopupIndex <= 0 ? 6 : osdPopupIndex - 1);
                      osdPopupSetting = osdPopupTimes[osdPopupIndex];
                      break;
                case PARAM_RECORD_TRIGGER:
                      UOO_trigger = isKey3 ? 
                                  (UOO_trigger >= 50 ? 0  : UOO_trigger + 1) :
                                  (UOO_trigger <= 0  ? 50 : UOO_trigger - 1);
                      break;
                case PARAM_AUTO_KEYLOCK:
                      AUTO_KEYLOCK = isKey3 ? 
                                   (AUTO_KEYLOCK > 2  ? 0 : AUTO_KEYLOCK + 1) :
                                 (AUTO_KEYLOCK <= 0 ? 3 : AUTO_KEYLOCK - 1);
                      gKeylockCountdown = durations[AUTO_KEYLOCK];
                      break;
                case PARAM_GLITCH_MAX:
                    if (isKey3) { if (GlitchMax < 75) GlitchMax += 5; }
                    else        { if (GlitchMax >= 5) GlitchMax -= 5; }
                      break;
                case PARAM_SOUND_BOOST:
                      SoundBoost = !SoundBoost;
                      break;
                case PARAM_PTT_EMISSION:
                      PttEmission = isKey3 ?
                            (PttEmission >= 8 ? 0 : PttEmission + 1) :
                            (PttEmission <= 0 ? 8 : PttEmission - 1);
                      break;  
                case PARAM_MONITOR_SCAN:
                    gMonitorScan = !gMonitorScan; 
                    break;
                case PARAM_RESET_DEFAULT:
                      if (isKey3) ClearSettings();
                      break;
                case PARAM_LIGHT_MODE:
                    Light_Mode = !Light_Mode;
                    parametersSelectedIndex = 0;
                    parametersScrollOffset = 0;
                    break;
            }
            break;
            }
        case KEY_7:
          SaveSettings(); 
        break;
        case KEY_EXIT:
            SetState(SPECTRUM);
            RelaunchScan();
            ResetModifiers();
            if(Key_1_pressed) {Spectrum_state = 2;APP_RunSpectrum();}
            break;
        default:
            break;
      }
}

/* --- SPECTRUM state: main spectrum view keys, including list entry shortcuts --- */
static void HandleKeySpectrum(uint8_t key) {

    switch (key) {
        case KEY_5: {
        if (historyListActive) {
            gHistoryScan = !gHistoryScan;
            ShowOSDPopup(gHistoryScan ? "SCAN HISTORY ON" : "SCAN HISTORY OFF");
            if (gHistoryScan) { gIsPeak = false; SpectrumMonitor = 0; }
        } else {
            SetState(PARAMETERS_SELECT);
            if (!parametersStateInitialized) {
                parametersSelectedIndex = 0;
                parametersScrollOffset = 0;
                parametersStateInitialized = true;
            }
            }
            break;
        }
        case KEY_STAR: {
                int step = (settings.rssiTriggerLevelUp >= 20) ? 5 : 1;
                settings.rssiTriggerLevelUp =
                    (settings.rssiTriggerLevelUp >= 50 ? 0 : settings.rssiTriggerLevelUp + step);
            SPECTRUM_PAUSED = true;
            SetTrigger50();
            break;
        }
        case KEY_F: {
            int step = (settings.rssiTriggerLevelUp <= 20) ? 1 : 5;
            settings.rssiTriggerLevelUp =
                (settings.rssiTriggerLevelUp <= 0 ? 50 : settings.rssiTriggerLevelUp - step);
            SPECTRUM_PAUSED = true;
            SetTrigger50();
            break;
        }
        case KEY_3:
            if (historyListActive) {
                DeleteHistoryItem();
            } else {
                ToggleListeningBW(1);
                char bwText[32];
                sprintf(bwText, "BW: %s", bwNames[settings.listenBw]);
                ShowOSDPopup(bwText);
            }
            break;
        case KEY_9: {
            ToggleModulation();
            char modText[32];
            sprintf(modText, "MOD: %s", gModulationStr[settings.modulationType]);
            ShowOSDPopup(modText);
            break;
        }
        case KEY_1:
            if (historyListActive) {ClearHistory(0);SpectrumMonitor = 0;}   //clear everything
            else {
                Skip();
                ShowOSDPopup("SKIPPED");
            }
            break;
        case KEY_4:
            if (historyListActive) {ClearHistory(2);return;}                // Clear BL
            if (appMode == SCAN_BAND_MODE) {
                SetState(BAND_LIST_SELECT);
                bandListSelectedIndex = 0;
                bandListScrollOffset  = 0;
                return;
            }
            if (appMode == CHANNEL_MODE) {
                SetState(SCANLIST_SELECT);
                scanListSelectedIndex = 0;
                scanListScrollOffset  = 0;
                return;
            }
            if (appMode != SCAN_RANGE_MODE) ToggleStepsCount();
            break;
        case KEY_7:
            if (historyListActive) {ClearHistory(1);}                       // Clear NBL
            else {SaveSettings();}
            break;
        case KEY_2:
            if (historyListActive) SaveHistoryToFreeChannel();
            else {
                Backlight_On = !Backlight_On;
                if (Backlight_On) {BACKLIGHT_TurnOn();}
                else {BACKLIGHT_TurnOff();}
            }
            break;
        case KEY_8:
            if (historyListActive) {SaveHistory();
            } else {
                ShowLines++;
                if (ShowLines > 2 || ShowLines < 1) ShowLines = 1;
                char viewText[15];
                const char *viewName           = "SPECTRUM";
				if (ShowLines == 2) viewName   = "SCAN";
                sprintf(viewText, "VIEW: %s", viewName);
                ShowOSDPopup(viewText);
                spectrumElapsedCount = 0;
            }
            break;
        case KEY_UP:
            if (historyListActive) {
                lastHistoryScrollOffset = -1;
                uint16_t count = CountValidHistoryItems();
                if (!count) return;
                SpectrumMonitor = 1;
                if (historyListIndex == 0) {
                    historyListIndex    = count - 1;
                    historyScrollOffset = (count > MAX_VISIBLE_LINES) ? count - MAX_VISIBLE_LINES : 0;
                } else {
                    historyListIndex--;
                }
                if (historyListIndex < historyScrollOffset)
                    historyScrollOffset = historyListIndex;
                lastReceivingFreq = HFreqs[historyListIndex];
                SetF(lastReceivingFreq);
            } else {
                switch (appMode) {
                    case SCAN_BAND_MODE:
                        bandListSelectedIndex = (bandListSelectedIndex < 1 ? bandCount - 1 : bandListSelectedIndex - 1);
                        ToggleScanList(bandListSelectedIndex, 1);
                        settings.bandEnabled[bandListSelectedIndex] = true;
                        RelaunchScan();
                        break;
                    case FREQUENCY_MODE:
                        UpdateCurrentFreq(0);
                        break;
                    case CHANNEL_MODE:
                        BuildValidScanListIndices();
                        scanListSelectedIndex = (scanListSelectedIndex < 1 ? validScanListCount - 1 : scanListSelectedIndex - 1);
                        ToggleScanList(validScanListIndices[scanListSelectedIndex], 1);
                        SetState(SPECTRUM);
                        ResetModifiers();
                        break;
                    case SCAN_RANGE_MODE:
                        uint32_t rstep = RangeStop - RangeStart;
                        RangeStop  -= rstep;
                        RangeStart -= rstep;
                        RelaunchScan();
                        break;
                }
            } 
            break;
        case KEY_DOWN:
            if (historyListActive) {
                lastHistoryScrollOffset = -1;
                uint16_t count = CountValidHistoryItems();
                if (!count) return;
                SpectrumMonitor = 1;
                if (++historyListIndex >= count) {
                    historyListIndex    = 0;
                    historyScrollOffset = 0;
                }
                if (historyListIndex >= historyScrollOffset + MAX_VISIBLE_LINES)
                    historyScrollOffset = historyListIndex - MAX_VISIBLE_LINES + 1;
                lastReceivingFreq = HFreqs[historyListIndex];
                SetF(lastReceivingFreq);
            } else {
                switch (appMode) {
                    case SCAN_BAND_MODE:
                        bandListSelectedIndex = (bandListSelectedIndex + 1) % bandCount;
                        ToggleScanList(bandListSelectedIndex, 1);
                        settings.bandEnabled[bandListSelectedIndex]= true; //Inverted for K1
                        RelaunchScan();
                        break;
                    case FREQUENCY_MODE:
                        UpdateCurrentFreq(1);
                        break;
                    case CHANNEL_MODE:
                        BuildValidScanListIndices();
                        if (validScanListCount > 0) {
                            scanListSelectedIndex = (scanListSelectedIndex + 1) % validScanListCount;
                            ToggleScanList(validScanListIndices[scanListSelectedIndex], 1);
                        }
                        SetState(SPECTRUM);
                        ResetModifiers();
                        break;
                    case SCAN_RANGE_MODE:
                        uint32_t rstep = RangeStop - RangeStart;
                        RangeStop  += rstep;
                        RangeStart += rstep;
                        RelaunchScan();
                        break;
                }
            }
            break;
        case KEY_0:
                if (!historyListActive) {
                    CompactHistory();
                    historyListActive   = true;
                    historyListIndex    = 0;
                    historyScrollOffset = 0;
                    lastHistoryScrollOffset = -1; // Force preload on initial history list activation
                    prevSpectrumMonitor = SpectrumMonitor;
                }
                break;
  
    case KEY_6: // next mode
        NextAppMode();
        break;
    case KEY_SIDE1:
        if (SPECTRUM_PAUSED) return;
        if (++SpectrumMonitor > 2) SpectrumMonitor = 0;
        if (SpectrumMonitor == 1) {
            if (lastReceivingFreq < 1400000 || lastReceivingFreq > 130000000) {
                lastReceivingFreq = (scanInfo.f >= 1400000) ? scanInfo.f : SpectrumRangeStart;}
                peak.f     = lastReceivingFreq;
                scanInfo.f = lastReceivingFreq;
                SetF(lastReceivingFreq);
        }
    if (SpectrumMonitor == 2) ToggleRX(1);
    {
		char monitorText[32];
        const char *modes[] = {"NORMAL", "FREQ LOCK", "MONITOR"};
        sprintf(monitorText, "MODE: %s", modes[SpectrumMonitor]);
	    ShowOSDPopup(monitorText);
    }
    break;
    case KEY_SIDE2:
        if (historyListActive) {
            HBlacklisted[historyListIndex] = !HBlacklisted[historyListIndex];
            ShowOSDPopup(HBlacklisted[historyListIndex] ? "BL ADDED" : "BL REMOVED");
            RenderHistoryList();
            gIsPeak = 0;
            ToggleRX(false);
            ResetScanStats();
            NextScanStep();
        } else {
            Blacklist();
            char Text[32];
            sprintf(Text, "BL %u.%05u Hz", lastReceivingFreq / 100000, lastReceivingFreq % 100000);
            ShowOSDPopup(Text);
            }
        break;
    case KEY_PTT:
        ExitAndCopyToVfo();
        break;
    case KEY_MENU:
            if (historyListActive) scanInfo.f = HFreqs[historyListIndex];
            SpectrumMonitor = 1;
            SetState(STILL);      
            stillFreq = GetInitialStillFreq();
            if (stillFreq >= 1400000 && stillFreq <= 130000000) {
                scanInfo.f = stillFreq;
                peak.f     = stillFreq;
                SetF(stillFreq);
            }
            break;
    case KEY_EXIT:
        if (historyListActive) {
            gHistoryScan        = false;
            SetState(SPECTRUM);
            historyListActive   = false;
            SpectrumMonitor     = prevSpectrumMonitor;
            SetF(scanInfo.f);
            break;
        }
        if (WaitSpectrum) WaitSpectrum = 0;
        DeInitSpectrum(0);
    break;
   default:
      break;
  }
}

// ============================================================
// SECTION: Main keyboard dispatcher
// ============================================================

static void OnKeyDown(uint8_t key) {
    /* Key-lock guard: only KEY_F unlocks */
    if (gIsKeylocked) {
        if (key == KEY_F) {
            gIsKeylocked = false;
            ShowOSDPopup("Unlocked");
            gKeylockCountdown = durations[AUTO_KEYLOCK];
        } else {
            ShowOSDPopup("Unlock:F");
        }
        return;
    }
    gKeylockCountdown = durations[AUTO_KEYLOCK];

    /* Dispatch to the handler for the currently active state */
    switch (currentState) {
        case BAND_LIST_SELECT:  HandleKeyBandList(key);         break;
        case SCANLIST_SELECT:   HandleKeyScanList(key);         break;
        case PARAMETERS_SELECT: HandleKeyParameters(key);       break;
        default:                HandleKeySpectrum(key);         break;
    }
}

static void OnKeyDownFreqInput(uint8_t key) {
  switch (key) {
  case KEY_0:
  case KEY_1:
  case KEY_2:
  case KEY_3:
  case KEY_4:
  case KEY_5:
  case KEY_6:
  case KEY_7:
  case KEY_8:
  case KEY_9:
  case KEY_STAR:
    UpdateFreqInput(key);
    break;
  case KEY_EXIT: //EXIT from freq input
    if (freqInputIndex == 0) {
      SetState(previousState);
      WaitSpectrum = 0;
      break;
    }
    UpdateFreqInput(key);
    break;
  case KEY_MENU: //OnKeyDownFreqInput
    if (tempFreq > F_MAX) {
      break;
    }
    SetState(previousState);
    if (currentState == SPECTRUM) {
        currentFreq = tempFreq;
      ResetModifiers();
    }
    if (currentState == PARAMETERS_SELECT && parametersSelectedIndex == PARAM_RANGE_START)
        RangeStart = tempFreq;
    if (currentState == PARAMETERS_SELECT && parametersSelectedIndex == PARAM_RANGE_STOP)
        RangeStop = tempFreq;

    break;
  default:
    break;
  }
}

static int16_t storedScanStepIndex = -1;

static void OnKeyDownStill(KEY_Code_t key) {
  switch (key) {
      case KEY_3:
         ToggleListeningBW(1);
      break;
     
      case KEY_9:
        ToggleModulation();
      break;
      case KEY_DOWN:
          if (stillEditRegs) {
            SetRegMenuValue(stillRegSelected, true);
          } else if (SpectrumMonitor > 0) {
                uint32_t step = GetScanStep();
                stillFreq += step;
                scanInfo.f = stillFreq;
                peak.f = stillFreq;
                SetF(stillFreq);
            }
        break;
      case KEY_UP:
          if (stillEditRegs) {
            SetRegMenuValue(stillRegSelected, false);
          } else if (SpectrumMonitor > 0) {
                uint32_t step = GetScanStep();
                if (stillFreq > step) stillFreq -= step;
                scanInfo.f = stillFreq;
                peak.f = stillFreq;
                SetF(stillFreq);
            }
          break;
      case KEY_2:
          if (stillEditRegs && stillRegSelected > 0) {
            stillRegSelected--;
          }
      break;
      case KEY_8: // przewijanie w dół po liście rejestrów
          if (stillEditRegs && stillRegSelected < ARRAY_SIZE(allRegisterSpecs)-1) {
            stillRegSelected++;
          }
      break;
      case KEY_STAR:
            if (storedScanStepIndex == -1) {
                storedScanStepIndex = settings.scanStepIndex;
            }
            UpdateScanStep(1);
      break;
      case KEY_F:
            if (storedScanStepIndex == -1) {
                storedScanStepIndex = settings.scanStepIndex;
            }
            UpdateScanStep(0);
      break;
      case KEY_5:
      case KEY_0:
      case KEY_6:
        break;
      case KEY_7:
        SaveSettings();
        break;
          
      case KEY_SIDE1: 
          SpectrumMonitor++;
    if (SpectrumMonitor > 2) SpectrumMonitor = 0;

    if (SpectrumMonitor == 1) {
        if (lastReceivingFreq < 1400000 || lastReceivingFreq > 130000000) {
            lastReceivingFreq = (stillFreq >= 1400000) ? stillFreq : scanInfo.f;
        }
        peak.f = lastReceivingFreq;
        scanInfo.f = lastReceivingFreq;
        SetF(lastReceivingFreq);
    }

    if (SpectrumMonitor == 2) ToggleRX(1);
      break;

      case KEY_PTT:
        ExitAndCopyToVfo();
        break;
      case KEY_MENU:
          stillEditRegs = !stillEditRegs;
      break;
      case KEY_EXIT: //EXIT from regs
        if (stillEditRegs) {
          stillEditRegs = false;
        break;
        }
        if (storedScanStepIndex != -1) {
            settings.scanStepIndex = storedScanStepIndex;
            storedScanStepIndex = -1;
            scanInfo.scanStep = storedScanStepIndex; 
        }
        SpectrumMonitor = 0;
        SetState(SPECTRUM);
        WaitSpectrum = 0; //Prevent coming back to still directly
        
    break;
  default:
    break;
  }
}


static void RenderFreqInput() {
  UI_PrintString(freqInputString, 2, 127, 0, 8);
}

static void RenderStatus() {
    memset(gStatusLine, 0, sizeof(gStatusLine));
    DrawStatus();
    ST7565_BlitStatusLine();
}
#ifdef ENABLE_SPECTRUM_LINES

static void MyDrawHLine(uint8_t y, bool white)
{
    if (y >= 64) return;
    uint8_t byte_idx = y / 8;
    uint8_t bit_mask = 1U << (y % 8);
    for (uint8_t x = 0; x < 128; x++) {
        if (white) {
            gFrameBuffer[byte_idx][x] &= ~bit_mask;  // белая
        } else {
            gFrameBuffer[byte_idx][x] |= bit_mask;   // чёрная
        }
    }
}

// Короткая горизонтальная пунктирная линия
static void MyDrawShortHLine(uint8_t y, uint8_t x_start, uint8_t x_end, uint8_t step, bool white)
{
    if (y >= 64 || x_start >= x_end || x_end > 127) return;
    uint8_t byte_idx = y / 8;
    uint8_t bit_mask = 1U << (y % 8);

    for (uint8_t x = x_start; x <= x_end; x++) {
        if (step > 1 && (x % step) != 0) continue;  // пунктир

        if (white) {
            gFrameBuffer[byte_idx][x] &= ~bit_mask;  // белая
        } else {
            gFrameBuffer[byte_idx][x] |= bit_mask;   // чёрная
        }
    }
}

static void MyDrawVLine(uint8_t x, uint8_t y_start, uint8_t y_end, uint8_t step)
{
    if (x >= 128) return;
    for (uint8_t y = y_start; y <= y_end && y < 64; y++) {
        if (step > 1 && (y % step) != 0) continue;  // пунктир
        uint8_t byte_idx = y / 8;
        uint8_t bit_mask = 1U << (y % 8);
        gFrameBuffer[byte_idx][x] |= bit_mask;  // чёрная (для белой сделай отдельно или параметр)
    }
}

static void MyDrawFrameLines(void)
{

    MyDrawVLine(0,   0, 17, 1);   // Left vertical solid line (top section)
    MyDrawVLine(127, 0, 17, 1);   // Right vertical solid line (top section)
    
    MyDrawShortHLine(0, 0, 3, 1, false);      // Top short horizontal line (left edge)
    MyDrawShortHLine(0, 4, 8, 2, false);      // Top short horizontal line (inner left)
    
    MyDrawShortHLine(0, 124, 127, 1, false);  // Top short horizontal line (right edge)
    MyDrawShortHLine(0, 118, 123, 2, false);  // Top short horizontal line (inner right)
    
    MyDrawShortHLine(17, 0, 10, 1, false);    // Mid-top short horizontal line (left)
    MyDrawShortHLine(17, 120, 127, 1, false); // Mid-top short horizontal line (right)
    
    if (ShowLines == 1) {
        MyDrawShortHLine(21, 0, 10, 1, false);    // Mid-bottom short horizontal line (left)
        MyDrawShortHLine(21, 120, 127, 1, false); // Mid-bottom short horizontal line (right)
        MyDrawHLine(50, true);  // Black horizontal line at y=49
        MyDrawHLine(49, false);  // Black horizontal line at y=49
        MyDrawVLine(0,   21, 49, 1);  // Left vertical solid line (bottom section)
        MyDrawVLine(127, 21, 49, 1);  // Right vertical solid line (bottom section)
    }
}
#endif

#ifdef ENABLE_PERSIST
static bool IsRssiHistoryInvalid(uint16_t rssi)
{
    // rssiHistory is cleared to 0 on (re)entry; treat it as "not measured yet"
    // so the renderer does not draw an artificial horizontal baseline.
    return rssi == 0 || rssi == RSSI_MAX_VALUE;
}



// Resolve the RSSI value at fractional sample index (Q8 fixed-point) using
// linear interpolation. Blacklisted samples (RSSI_MAX_VALUE) are skipped by
// falling back to the other neighbour; if both are blacklisted, returns
// RSSI_MAX_VALUE so the caller can skip the column.
static uint16_t InterpolateRssi(uint8_t bars, uint16_t pos256)
{
    uint8_t i = pos256 >> 8;
    uint8_t frac = pos256 & 0xFF;

    if (i >= bars - 1)
    {
        i = bars - 1;
        frac = 0;
    }

    uint16_t rssiA = rssiHistory[i];
    uint16_t rssiB = rssiHistory[(i + 1 < bars) ? (i + 1) : i];

    if (IsRssiHistoryInvalid(rssiA) && IsRssiHistoryInvalid(rssiB))
        return RSSI_MAX_VALUE;
    if (IsRssiHistoryInvalid(rssiA))
        return rssiB;
    if (IsRssiHistoryInvalid(rssiB))
        return rssiA;

    return ((uint32_t)rssiA * (256 - frac) + (uint32_t)rssiB * frac) >> 8;
}

// Sentinel value in topY[] to mark a column that should not be drawn
// (blacklisted RSSI sample on both neighbours).
#define SPECTRUM_TOPY_SKIP 0xFF

// Half-step bridging helper: compute crestTop/crestBot for column x
// from a topY-like array.
static void CalcCrest(const uint8_t *yArr, uint8_t x,
                      uint8_t *crestTop, uint8_t *crestBot)
{
    uint8_t y0 = yArr[x];
    *crestTop = y0;
    *crestBot = y0;

    bool goBack = true;
    uint8_t n = 0;

    if (x > 0) {
        n = yArr[x - 1];
        goto Start;
    }

Back:
    goBack = false;

    if (x + 1 < 128) {
        n = yArr[x + 1];
        goto Start;
    }

    return;

Start:
    if (n != SPECTRUM_TOPY_SKIP && n <= DrawingEndY) {
        uint8_t mid = (y0 + n + 1) >> 1;
        if (mid < *crestTop) *crestTop = mid;
        if (mid > *crestBot) *crestBot = mid;
    }

    if (goBack)
        goto Back;
}

// Draw the spectrum curve (solid crest + checkerboard body) and the peak hold
// dotted trace.  Both use the same half-step bridging so the peak hold crest
// shape mirrors the live crest exactly, just rendered with a dotted pattern.
static void DrawSpectrumCurve(const uint8_t *topY)
{
    // Pass 1: update peakHoldY[] from topY[] before rendering so that the
    // bridging in Pass 2 already sees fully-updated neighbour values.
    for (uint8_t x = 0; x < 128; x++)
    {
        uint8_t y0 = topY[x];
        if (y0 == SPECTRUM_TOPY_SKIP || y0 > DrawingEndY) {
            peakHoldY[x] = PEAK_HOLD_INIT;
            continue;
        }

        uint8_t ph = peakHoldY[x];
        if (ph == PEAK_HOLD_INIT || y0 <= ph)
        {
            peakHoldY[x]        = y0;
            peakHoldAge[x >> 1] = 0;
        }
        else
        {
            if (peakHoldAge[x >> 1] < PEAK_HOLD_DELAY) {
                if (!(x & 1)) peakHoldAge[x >> 1]++;
            } else {
                ph += 2;
                peakHoldY[x] = (ph <= DrawingEndY) ? ph : PEAK_HOLD_INIT;
            }
        }
    }

    // Pass 2: draw live curve (solid) then peak hold (dotted).
    for (uint8_t x = 0; x < 128; x++)
    {
        // --- Live spectrum crest + body ---
        uint8_t y0 = topY[x];
        if (y0 != SPECTRUM_TOPY_SKIP && y0 <= DrawingEndY)
        {
            uint8_t crestTop, crestBot;
            CalcCrest(topY, x, &crestTop, &crestBot);

            // Solid crest contour.
            for (uint8_t y = crestTop; y <= crestBot; y++)
                PutPixel(x, y, true);

            // Checkerboard body below the crest.
            for (uint8_t y = crestBot + 1; y <= DrawingEndY; y++)
                if (((x + y) & 1) == 0)
                    PutPixel(x, y, true);
        }

        // --- Peak hold dotted crest ---
        uint8_t ph = peakHoldY[x];
        if (ph != PEAK_HOLD_INIT && ph <= DrawingEndY)
        {
            uint8_t phTop, phBot;
            CalcCrest(peakHoldY, x, &phTop, &phBot);

            // Dotted crest: checkerboard pattern over the full crest range.
            for (uint8_t y = phTop; y <= phBot; y++)
                if (((x + y) & 1) == 0)
                    PutPixel(x, y, true);
        }
    }
}

// Spatial smoothing: 3-bin moving average on topY for a cleaner curve.
// Only averages valid (non-SKIP) neighbours.
static void SmoothTopY(uint8_t *topY)
{
    uint8_t prev = topY[0];
    for (uint8_t x = 1; x < 127; x++)
    {
        uint8_t cur = topY[x];
        uint8_t next = topY[x + 1];
        if (cur == SPECTRUM_TOPY_SKIP) {
            prev = cur;
            continue;
        }
        uint16_t sum = cur;
        uint8_t n = 1;
        if (prev != SPECTRUM_TOPY_SKIP) { sum += prev; n++; }
        if (next != SPECTRUM_TOPY_SKIP) { sum += next; n++; }
        prev = cur;                       // save unsmoothed value for next iteration
        topY[x] = (sum + n / 2) / n;     // rounded average
    }
}

// Fill topY[0..127] by linear interpolation of `bars` RSSI samples across the
// 128 display columns. Invalid (blacklisted) samples become SPECTRUM_TOPY_SKIP.
static void BuildSpectrumTopY(uint8_t *topY, uint8_t bars)
{
    if (bars == 0)
    {
        for (uint8_t x = 0; x < 128; x++)
            topY[x] = SPECTRUM_TOPY_SKIP;
        return;
    }

    if (bars == 1)
    {
        uint16_t rssi = rssiHistory[0];
        uint8_t y = IsRssiHistoryInvalid(rssi) ? SPECTRUM_TOPY_SKIP : Rssi2Y(rssi);
        for (uint8_t x = 0; x < 128; x++)
            topY[x] = y;
        return;
    }

    // Q8 fixed-point: step256 / 256 advances one sample, multiplied by x.
    uint16_t step256 = ((uint16_t)(bars - 1) << 8) / 127;

    for (uint8_t x = 0; x < 128; x++)
    {
        uint16_t rssi = InterpolateRssi(bars, (uint16_t)x * step256);
        topY[x] = (rssi == RSSI_MAX_VALUE) ? SPECTRUM_TOPY_SKIP : Rssi2Y(rssi);
    }
}

static void BuildCurrentSpectrumTopY(uint8_t *topY)
{
#ifdef ENABLE_FEAT_F4HWN
    uint16_t steps = GetStepsCount();
    // max bars at 128 to correctly draw larger numbers of samples
    uint8_t bars = (steps > 128) ? 128 : steps;
#else
    uint8_t bars = 128 >> settings.stepsCount;
    if (bars == 0)
        bars = 1;
#endif

    BuildSpectrumTopY(topY, bars);
    // Skip cosmetic smoothing in manual mode so the rendered curve matches
    // the raw RSSI used by the squelch detector — narrow peaks must visibly
    // cross the trigger line when the radio opens the squelch.
    SmoothTopY(topY);
}
#endif

static void RenderSpectrum()
{
    if (ShowLines < 2) {
#ifdef ENABLE_PERSIST
        uint8_t topY[128];
        BuildCurrentSpectrumTopY(topY);
        DrawNums();
        UpdateDBMaxAuto();
        DrawSpectrumCurve(topY);
#else
        DrawNums();
        UpdateDBMaxAuto();
        DrawSpectrum();
#endif
    }

}

static void DrawMeter(int line) {
    const uint8_t METER_PAD_LEFT = 4;
    const uint8_t LINE_HEIGHT    = 4;           // Height of the vertical lines
    const uint8_t Y_START_BIT    = 1;
    const uint8_t SPACING        = 2;           // Space between each vertical line
    
    // Total width available for the meter
    uint8_t max_width_px = 120; 
    uint8_t fill_px      = Rssi2PX(scanInfo.rssi, 0, max_width_px);
    
    settings.dbMax = -60; 
    settings.dbMin = -120;

    // Clear the current line in the frame buffer
    for (uint8_t px = 0; px < 128; px++) {
        gFrameBuffer[line][px] = 0;
    }

    // Draw alternating vertical lines instead of squares
    for (uint8_t x = 0; x < fill_px; x += SPACING) {
        uint8_t x_pos = METER_PAD_LEFT + x;

        // Boundary check
        if (x_pos >= 128) break;

        // Draw a single vertical line at the current X position
        for (uint8_t bit = Y_START_BIT; bit < Y_START_BIT + LINE_HEIGHT; bit++) {
            gFrameBuffer[line][x_pos] |= (1 << bit);
        }
    }
}

static void RenderStill() {
  char freqStr[18];
  //if (SpectrumMonitor) FormatFrequency(HFreqs[historyListIndex], freqStr, sizeof(freqStr));
  //else
  FormatFrequency(stillFreq, freqStr, sizeof(freqStr));
  UI_DisplayFrequency(freqStr, 0, 0, 0);
  DrawMeter(2);
  sLevelAttributes sLevelAtt;
  sLevelAtt = GetSLevelAttributes(scanInfo.rssi, stillFreq);

  if(sLevelAtt.over > 0)
    snprintf(String, sizeof(String), "S%2d+%2d", sLevelAtt.sLevel, sLevelAtt.over);
  else
    snprintf(String, sizeof(String), "S%2d", sLevelAtt.sLevel);

  GUI_DisplaySmallest(String, 4, 25, false, true);
  snprintf(String, sizeof(String), "%d dBm", sLevelAtt.dBmRssi);
  GUI_DisplaySmallest(String, 40, 25, false, true);
  uint8_t total = ARRAY_SIZE(allRegisterSpecs);
  uint8_t lines = STILL_REGS_MAX_LINES;
  if (total < lines) lines = total;
  if (stillRegSelected >= total) stillRegSelected = total-1;
  if (stillRegSelected < stillRegScroll) stillRegScroll = stillRegSelected;
  if (stillRegSelected >= stillRegScroll + lines) stillRegScroll = stillRegSelected - lines + 1;

  for (uint8_t i = 0; i < lines; ++i) {
    uint8_t idx = i + stillRegScroll;
    RegisterSpec s = allRegisterSpecs[idx];
    uint16_t v = GetRegMenuValue(idx);
    char buf[32];
    // Przygotuj tekst do wyświetlenia
    if (stillEditRegs && idx == stillRegSelected)
      snprintf(buf, sizeof(buf), ">%-18s %6u", s.name, v);
    else
      snprintf(buf, sizeof(buf), " %-18s %6u", s.name, v);
    uint8_t y = 32 + i * 8;
    if (stillEditRegs && idx == stillRegSelected) {
      // Najpierw czarny prostokąt na wysokość linii
      for (uint8_t px = 0; px < 128; ++px)
        for (uint8_t py = y; py < y + 6; ++py) // 6 = wysokość fontu 3x5
          PutPixel(px, py, true); // 
      // Następnie białe litery (fill = true)
      GUI_DisplaySmallest(buf, 0, y, false, false);
    } else {
      // Zwykły tekst: czarne litery na białym
      GUI_DisplaySmallest(buf, 0, y, false, true);
    }
  }
}

static void Render() {
    memset(gFrameBuffer, 0, sizeof(gFrameBuffer));
    if(historyListActive) {
        if(gNextTimeslice_history) RenderHistoryList();
        gNextTimeslice_history = 0;
        return;
    }
    switch (currentState) {
        case SPECTRUM:
#ifdef ENABLE_SPECTRUM_LINES
            MyDrawFrameLines();
#endif
            if(isListening) { DrawF(peak.f);}
            else {
                    if (SpectrumMonitor) DrawF(lastReceivingFreq);
                    else DrawF(scanInfo.f);
            }

            if (spectrumElapsedCount < 500) {
                RenderSpectrum();
                ST7565_BlitLine(4);
                ST7565_BlitLine(5);
                ST7565_BlitLine(6);
            } else return;
            break;
        case FREQ_INPUT:
            RenderFreqInput();
            break;
        case STILL:
            RenderStill();
            break;
        case BAND_LIST_SELECT:
            RenderBandSelect();
            return;
        case SCANLIST_SELECT:
            RenderScanListSelect();
            return;
        case PARAMETERS_SELECT:
            RenderParametersSelect();
            return;
    }
ST7565_BlitFullScreen();
}

static void HandleUserInput(void) {
    
    kbd.prev = kbd.current;
    kbd.current = GetKey();
    // ---- Anti-rebond + répétition ----
    if (kbd.current != KEY_INVALID && kbd.current == kbd.prev) {
        kbd.counter++;
    } else kbd.counter =0;

    if (kbd.counter == 2 || (kbd.counter > 30 && (kbd.counter % 10 == 0))) {
        if(Backlight_On && !backlightOn && gEeprom.BACKLIGHT_TIME) {
            BACKLIGHT_TurnOn();
            return;
        }
        switch (currentState) {
            case SPECTRUM:
            case BAND_LIST_SELECT:
            case SCANLIST_SELECT:
            case PARAMETERS_SELECT:
                OnKeyDown(kbd.current);
                break;
            case FREQ_INPUT:
                OnKeyDownFreqInput(kbd.current);
                break;
            case STILL:
                OnKeyDownStill(kbd.current);
                break;
        }
    }
}

static void NextHistoryScanStep() {
    uint16_t count = CountValidHistoryItems();
    if (count == 0) return;
    uint16_t start = historyListIndex;
    do {
        historyListIndex++;
        if (historyListIndex >= count) historyListIndex = 0;
        if (historyListIndex == start && HBlacklisted[historyListIndex]) return;
    } while (HBlacklisted[historyListIndex]);

    if (historyListIndex < historyScrollOffset) {
        historyScrollOffset = historyListIndex;
    } else if (historyListIndex >= historyScrollOffset + MAX_VISIBLE_LINES) {
        historyScrollOffset = historyListIndex - MAX_VISIBLE_LINES + 1;
    }
    scanInfo.f = HFreqs[historyListIndex];
    spectrumElapsedCount = 0;
}

static uint32_t savedScanF;

static void UpdateScan() {
    if (SPECTRUM_PAUSED || gIsPeak || SpectrumMonitor || WaitSpectrum) return;
    SetF(scanInfo.f);
    Measure();
    if (gIsPeak || SpectrumMonitor || WaitSpectrum) return;
#ifdef ENABLE_BENCH
    benchStepsThisSec++;
#endif

    if (gMonitorScan && gNextTimeslice_Monitor && monitorChannelsCount) { 
        gNextTimeslice_Monitor = false;
        savedScanF = scanInfo.f; // Sauvegarde avant interruption
        MonitorIndex = monitorChannelsCount + 1;
    }
    if (MonitorIndex) {
        scanInfo.f = MonitorFreqs[--MonitorIndex-1];
        if (!MonitorIndex) {
            scanInfo.f = savedScanF;
            NextScanStep();
        }
        return;
    }
    if (gHistoryScan && historyListActive) NextHistoryScanStep();
    else NextScanStep();
#ifdef ENABLE_BENCH
    if (benchLapDone) { benchLastLapMs = benchLapMs; benchLapMs = 0; }
#endif
    if (SpectrumSleepMs && !scanInfo.i) {
        BK4819_Sleep();
        BK4819_ToggleGpioOut(BK4819_GPIO0_PIN28_RX_ENABLE, false);
        SPECTRUM_PAUSED = true;
        SpectrumPauseCount = SpectrumSleepMs;
    }
}


static void UpdateListening(void) {
    
    static uint32_t stableFreq = 1;
    static uint16_t stableCount = 0;
    static bool SoundBoostsave = false; // Initialisation
    
    scanInfo.rssi = GetRssi();

    if (SoundBoost != SoundBoostsave) {
        if (SoundBoost) {
            BK4819_WriteRegister(0x54, 0x90D1);    // default is 0x9009
            BK4819_WriteRegister(0x55, 0x3271);    // default is 0x31a9
            BK4819_WriteRegister(0x75, 0xFC13);    // default is 0xF50B
        } 
        else {
            BK4819_WriteRegister(0x54, 0x9009);
            BK4819_WriteRegister(0x55, 0x31a9);
            BK4819_WriteRegister(0x75, 0xF50B);
        }
        SoundBoostsave = SoundBoost;
    }
    if (peak.f == stableFreq) {
        if (++stableCount >= 2) {  
            if (!SpectrumMonitor) FillfreqHistory();
            if (gEeprom.BACKLIGHT_MAX > 5){
                BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, 1);
                BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, 0);
            }
            if(Backlight_On) BACKLIGHT_TurnOn();
        }
    } else {
        stableFreq = peak.f;
        stableCount = 0;
    }
    
    UpdateNoiseOff();
    if (!isListening) {
        UpdateNoiseOn();
        UpdateGlitch();
    }
    spectrumElapsedCount += 200; 
    if (peak.f >= 1400000 && peak.f <= 130000000 && gNextTimeslice_HTimeS) {
    gNextTimeslice_HTimeS = 0;
    for (uint16_t i = 0; i < indexFs; i++) {
        if (HFreqs[i] == peak.f) {
            if (HTimeS[i] < 3600) {
                HTimeS[i] += 1;
            } else {HTimeS[i] = 3599;}
            break;
        }
    }
}
    uint32_t maxCount = (uint32_t)MaxListenTime * 1000;

    if (MaxListenTime && spectrumElapsedCount >= maxCount && !SpectrumMonitor) {
        Skip();
        return;
    }

    // --- Gestion du pic ---
    if (gIsPeak) {
        WaitSpectrum = SpectrumDelay;   // reset timer
        return;
    }

    if (WaitSpectrum > 61000)
        return;

    if (WaitSpectrum > 200) {
        WaitSpectrum -= 200;
        return;
    }
    // timer écoulé
    WaitSpectrum = 0;
    ResetScanStats();

}

static void Tick() {
    if (gNextTimeslice_10ms) {
        gNextTimeslice_10ms = 0;
        HandleUserInput();
        BACKLIGHT_Update();
#ifdef ENABLE_BENCH
        if (!isListening && !SPECTRUM_PAUSED && !SpectrumMonitor && !WaitSpectrum) {
            benchTickMs += 10;
            benchLapMs  += 10;
            if (benchTickMs >= 1000) {
                benchTickMs -= 1000;
                benchRatePerSec = benchStepsThisSec;
                benchStepsThisSec = 0;
            }
        }
#endif
        if(SpectrumPauseCount) SpectrumPauseCount--;
        if (osdPopupTimer) {
            osdPopupTimer -= 10; 
            if (osdPopupTimer <= 0) {osdPopupText[0] = '\0';}
            UI_DisplayPopup(osdPopupText);
            ST7565_BlitFullScreen();
            return;
            }
    }
    
    if (gNextTimeslice_500ms) {
            gNextTimeslice_500ms = false;
        if (gBacklightCountdown_500ms > 0) --gBacklightCountdown_500ms;
        if (gEeprom.BACKLIGHT_TIME <61 && gBacklightCountdown_500ms == 0) {BACKLIGHT_TurnOff();}

        if (gKeylockCountdown > 0) {gKeylockCountdown--;}
        if (AUTO_KEYLOCK && !gKeylockCountdown) {
            if (!gIsKeylocked) ShowOSDPopup("Locked"); 
            gIsKeylocked = true;
	    }
    }

    if (SPECTRUM_PAUSED && (SpectrumPauseCount == 0)) {
        // fin de la pause
        SPECTRUM_PAUSED = false;
        BK4819_ToggleGpioOut(BK4819_GPIO0_PIN28_RX_ENABLE, true);
        BK4819_RX_TurnOn(); //Wake up
        SYSTEM_DelayMs(10);
    }

    if(!isListening && gIsPeak && !SpectrumMonitor && !SPECTRUM_PAUSED) {
        SetF(peak.f);
        ToggleRX(true);
        return;
    }

    if (newScanStart) {
        newScanStart = false;
        InitScan();
    }

    if (!isListening && currentState == SPECTRUM) {UpdateScan();}
    if (gNextTimeslice_listening){
        gNextTimeslice_listening = 0;
        if (isListening || SpectrumMonitor || WaitSpectrum) UpdateListening();
    }
    if (gNextTimeslice_display) {
        gNextTimeslice_display = 0;
        latestScanListName[0] = '\0';
        RenderStatus();
        Render();
    } 
    if (gNextTimeslice_SCAN_LED) {
        gNextTimeslice_SCAN_LED = 0;
        if (!isListening) {
            BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, 1);
            BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, 1);
        } 
    }
    if (gNextTimeslice_SCAN_LED_OFF) {
        gNextTimeslice_SCAN_LED_OFF = 0;
        if (!isListening) {
        BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, 0);
        BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, 0);
        }
    }

    if (gNextTimeslice_AutoPtt && PttEmission >= 3) {
        gNextTimeslice_AutoPtt = 0;
        RADIO_PrepareTX();
        SYSTEM_DelayMs(50);
        RADIO_SendEndOfTransmission();
    }
}
void APP_RunSpectrumMode(uint8_t mode) {
    Spectrum_state = mode & 3;
    APP_RunSpectrum();
}

void APP_RunSpectrum(void) {    
    for (;;) {
        LoadMonitorFrequencies ();
        Mode mode;
        if (!Key_1_pressed ) LoadSettings();
        Key_1_pressed = 0;
        
        switch (Spectrum_state) {
            case 0:  mode = FREQUENCY_MODE;  break;
            case 1:  mode = CHANNEL_MODE;    break;
            case 2:  mode = SCAN_RANGE_MODE; break;
            case 3:  mode = SCAN_BAND_MODE;  break;
            default: mode = FREQUENCY_MODE;  break;
        }
        LoadActiveScanFrequencies();
        if(mode == SCAN_BAND_MODE){
            if (BParams == NULL) {
                BParams = (bandparameters *)malloc((MAX_BANDS) * sizeof(bandparameters));}
            if(BParams) LoadActiveBands();
        }
#ifdef ENABLE_FEAT_F4HWN_RESUME_STATE
        gEeprom.CURRENT_STATE = 4;
        SETTINGS_WriteCurrentState();
#endif
        appMode = mode;
        ResetModifiers();
        if (appMode==FREQUENCY_MODE && !Key_1_pressed) {
            currentFreq = gTxVfo->pRX->Frequency;
            SpectrumRangeStart = currentFreq - (GetBW() >> 1);
            SpectrumRangeStop  = currentFreq + (GetBW() >> 1);
        }
        BackupRegisters();
        BK4819_WriteRegister(BK4819_REG_30, 0);
        SYSTEM_DelayMs(10);
        BK4819_RX_TurnOn();
        SYSTEM_DelayMs(50);
        uint8_t CodeType = gTxVfo->pRX->CodeType;
        uint8_t Code     = gTxVfo->pRX->Code;
        BK4819_SetCDCSSCodeWord(DCS_GetGolayCodeWord(CodeType, Code));
        ResetInterrupts();
        BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, 0);
        BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, 0);
        BK4819_WriteRegister(BK4819_REG_47, 0x6042);
        BK4819_WriteRegister(BK4819_REG_48, 0xB3AA);  // AF gain
	    ToggleRX(true), ToggleRX(false); // hack to prevent noise when squelch off
        RADIO_SetModulation(settings.modulationType = gTxVfo->Modulation);
        BK4819_SetFilterBandwidth(settings.listenBw, false);
        isListening = true;
        newScanStart = true;
        AutoAdjustFreqChangeStep();
        RelaunchScan();
        for (int i = 0; i < 128; ++i) { rssiHistory[i] = 0; }
        isInitialized = true;
        historyListActive = false;
#ifdef ENABLE_CPU_TEMP
        temp_dc = CpuTemp_ReadDeciCelsius();
        //cpu_hz = CpuInfo_GetClockHz();
#endif
        gComeBack = 0;
        while (isInitialized) {Tick();}

        if (gSpectrumChangeRequested) {
            Spectrum_state = gRequestedSpectrumState;
            gSpectrumChangeRequested = false;
            RestoreRegisters(); 
            continue;
        } else {
            RestoreRegisters();
            break;
        }
        if (ScanFrequencies) { free(ScanFrequencies); ScanFrequencies = NULL; }
        if (BParams)         { free(BParams);         BParams = NULL; }
        break;
    } 
}



static void ToggleScanList(int scanListNumber, int single )
{
    if (appMode == SCAN_BAND_MODE) {
      if (single) memset(settings.bandEnabled, 0, sizeof(settings.bandEnabled));
        else settings.bandEnabled[scanListNumber-1] = !settings.bandEnabled[scanListNumber-1];
    }
    if (appMode == CHANNEL_MODE) {
        if (single) {memset(settings.scanListEnabled, 0, sizeof(settings.scanListEnabled));}
        if(scanListNumber < MR_CHANNELS_LIST){
            settings.scanListEnabled[scanListNumber] = !settings.scanListEnabled[scanListNumber];
            refreshScanListName = true;
        }
    }
}

// ============================================================
// SECTION: EEPROM / Settings persistence
// ============================================================

typedef struct {
    int ShowLines;
    uint8_t IndexDelayRssi;
    uint8_t PttEmission; 
    uint8_t listenBw;
	uint64_t bandListFlags;            // Bits 0-63: bandEnabled[0..63]
    uint32_t scanListFlags;            // Bits 0-31: scanListEnabled[0..31]
    int16_t Trigger;
    uint32_t RangeStart;
    uint32_t RangeStop;
    STEP_Setting_t scanStepIndex;
    uint16_t R40;                      // RF TX Deviation
    uint16_t R29;                      // AF TX noise compressor, AF TX 0dB compressor, AF TX compression ratio
    uint16_t R19;                      // Disable MIC AGC
    uint16_t R73;                      // AFC range select
    uint16_t R10;
    uint16_t R11;
    uint16_t R12;
    uint16_t R13;
    uint16_t R14;
    uint16_t R3C;
    uint16_t R43;
    uint16_t R2B;
    uint16_t SpectrumDelay;
    uint8_t IndexMaxLT;
    uint8_t IndexPS;
    uint8_t Noislvl_OFF;
    uint16_t UOO_trigger;
    uint16_t osdPopupSetting;
    uint8_t GlitchMax;  
    uint8_t Spectrum_state;  
    bool Backlight_On;
    bool SoundBoost;  
    bool gMonitorScan;
    bool Light_Mode;
} SettingsEEPROM;


void LoadSettings()
{
  if(SettingsLoaded) return;
  SettingsEEPROM  eepromData  = {0};
  PY25Q16_ReadBuffer(ADRESS_PARAMS, &eepromData, sizeof(eepromData));
  
  BK4819_WriteRegister(BK4819_REG_13, eepromData.R13);
  BK4819_WriteRegister(BK4819_REG_40, eepromData.R40);
  BK4819_WriteRegister(BK4819_REG_73, eepromData.R73);
  BK4819_WriteRegister(BK4819_REG_10, eepromData.R10);
  BK4819_WriteRegister(BK4819_REG_11, eepromData.R11);
  BK4819_WriteRegister(BK4819_REG_12, eepromData.R12);
  BK4819_WriteRegister(BK4819_REG_14, eepromData.R14);
  BK4819_WriteRegister(BK4819_REG_19, eepromData.R19);
  BK4819_WriteRegister(BK4819_REG_29, eepromData.R29);
  BK4819_WriteRegister(BK4819_REG_2B, eepromData.R2B);
  BK4819_WriteRegister(BK4819_REG_3C, eepromData.R3C);
  BK4819_WriteRegister(BK4819_REG_43, eepromData.R43);
  for (int i = 0; i < MR_CHANNELS_LIST; i++) {
    settings.scanListEnabled[i] = (eepromData.scanListFlags >> i) & 0x01;
  }
  settings.rssiTriggerLevelUp = eepromData.Trigger;
  settings.listenBw = eepromData.listenBw;
  BK4819_SetFilterBandwidth(settings.listenBw, false);
  if (eepromData.RangeStart >= 1400000) RangeStart = eepromData.RangeStart;
  if (eepromData.RangeStop >= 1400000)  RangeStop = eepromData.RangeStop;
  settings.scanStepIndex = eepromData.scanStepIndex;
  for (int i = 0; i < MAX_BANDS; i++) {
    settings.bandEnabled[i] = (eepromData.bandListFlags & ((uint64_t)1 << i)) != 0;
    }
  IndexDelayRssi = eepromData.IndexDelayRssi;
  DelayRssi = DelayRssiValues[eepromData.IndexDelayRssi];
  PttEmission = eepromData.PttEmission;
  validScanListCount = 0;
  ShowLines = eepromData.ShowLines;
  SpectrumDelay = eepromData.SpectrumDelay;
  IndexMaxLT = eepromData.IndexMaxLT;
  MaxListenTime = listenSteps[IndexMaxLT];
  IndexPS = eepromData.IndexPS;
  SpectrumSleepMs = PS_Steps[IndexPS];
  Noislvl_OFF = eepromData.Noislvl_OFF;
  Noislvl_ON  = Noislvl_OFF - NoiseHysteresis; 
  UOO_trigger = eepromData.UOO_trigger;
  osdPopupSetting = eepromData.osdPopupSetting;
  Backlight_On = eepromData.Backlight_On;
  GlitchMax = eepromData.GlitchMax;    
  Spectrum_state = eepromData.Spectrum_state;    
  SoundBoost = eepromData.SoundBoost;
  gMonitorScan = eepromData.gMonitorScan;    
  Light_Mode = eepromData.Light_Mode;    
  

  
 if (!historyLoaded) {
        LoadHistory();
        historyLoaded = true;
 }
SettingsLoaded = true;
}

static void SaveSettings() 
{
  SettingsEEPROM  eepromData  = {0};
  for (int i = 0; i < MR_CHANNELS_LIST; i++) {
    if (settings.scanListEnabled[i]) eepromData.scanListFlags |= (1 << i);
  }
  eepromData.Trigger = settings.rssiTriggerLevelUp;
  eepromData.listenBw = settings.listenBw;
  eepromData.RangeStart = RangeStart;
  eepromData.RangeStop =  RangeStop;
  eepromData.IndexDelayRssi = IndexDelayRssi;
  eepromData.PttEmission = PttEmission;
  eepromData.scanStepIndex = settings.scanStepIndex;
  eepromData.ShowLines = ShowLines;
  eepromData.SpectrumDelay = SpectrumDelay;
  eepromData.IndexMaxLT = IndexMaxLT;
  eepromData.IndexPS = IndexPS;
  eepromData.Backlight_On = Backlight_On;
  eepromData.Noislvl_OFF = Noislvl_OFF;
  eepromData.UOO_trigger = UOO_trigger;
  eepromData.osdPopupSetting = osdPopupSetting;
  eepromData.GlitchMax = 20;
  eepromData.GlitchMax  = GlitchMax;   
  eepromData.Spectrum_state = Spectrum_state;    
  eepromData.SoundBoost = SoundBoost;
  eepromData.gMonitorScan = gMonitorScan;
  eepromData.Light_Mode = Light_Mode;
  for (int i = 0; i < MAX_BANDS; i++) { 
    if (settings.bandEnabled[i]) {
        eepromData.bandListFlags |= ((uint64_t)1 << i);
    }
    }
  eepromData.R40 = BK4819_ReadRegister(BK4819_REG_40);
  eepromData.R29 = BK4819_ReadRegister(BK4819_REG_29);
  eepromData.R19 = BK4819_ReadRegister(BK4819_REG_19);
  eepromData.R73 = BK4819_ReadRegister(BK4819_REG_73);
  eepromData.R10 = BK4819_ReadRegister(BK4819_REG_10);
  eepromData.R11 = BK4819_ReadRegister(BK4819_REG_11);
  eepromData.R12 = BK4819_ReadRegister(BK4819_REG_12);
  eepromData.R13 = BK4819_ReadRegister(BK4819_REG_13);
  eepromData.R14 = BK4819_ReadRegister(BK4819_REG_14);
  eepromData.R3C = BK4819_ReadRegister(BK4819_REG_3C);
  eepromData.R43 = BK4819_ReadRegister(BK4819_REG_43);
  eepromData.R2B = BK4819_ReadRegister(BK4819_REG_2B);
  PY25Q16_WriteBuffer(ADRESS_PARAMS, ((uint8_t*)&eepromData),sizeof(eepromData),0);
  if (Cleared)   ShowOSDPopup("DEFAULT SETTINGS");
  else ShowOSDPopup("PARAMS SAVED");
  Cleared = 0;
}

static void ClearHistory(uint8_t mode) {
    if (mode == 0) {
        memset(HFreqs, 0, sizeof(HFreqs));
        memset(HBlacklisted, 0, sizeof(HBlacklisted));
        memset(HTimeS, 0, sizeof(HTimeS));
    } 
    if (mode == 1) {
        for (int i = 0; i < HISTORY_SIZE; i++) {
            if (!HBlacklisted[i]) {
                HFreqs[i] = 0;
                HTimeS[i] = 0;
            }
        }
    } 
    if (mode == 2) {
        for (int i = 0; i < HISTORY_SIZE; i++) {
            if (HBlacklisted[i]) {
                HFreqs[i] = 0;
                HBlacklisted[i] = 0;
                HTimeS[i] = 0;
            }
        }
    }
    // Force a reload of the history cache after clearing items
    lastHistoryScrollOffset = -1;
    CompactHistory();
}

void ClearSettings() 
{
  for (int i = 1; i < MR_CHANNELS_LIST; i++) {settings.scanListEnabled[i] = 0;}
  settings.scanListEnabled[0] = 1;
  settings.rssiTriggerLevelUp = 5;
  settings.listenBw = 0;
  RangeStart = 43000000;
  RangeStop  = 44000000;
  DelayRssi = 2000;
  IndexDelayRssi = 3;
  PttEmission = 2;
  settings.scanStepIndex = STEP_10kHz;
  ShowLines = 1;
  SpectrumDelay = 0;
  MaxListenTime = 0;
  IndexMaxLT = 0;
  IndexPS = 0;
  Backlight_On = 1;
  Noislvl_OFF = NoisLvl; 
  Noislvl_ON = NoisLvl - NoiseHysteresis;  
  UOO_trigger = 5;
  osdPopupSetting = 200;
  GlitchMax = 10;  
  Spectrum_state = 1; 
  SoundBoost = 0;
  gMonitorScan = false;
  Light_Mode = false;
  settings.bandEnabled[0] = 1;
  for (int i = 1; i < MAX_BANDS; i++) {settings.bandEnabled[i] = 0;}
  BK4819_WriteRegister(BK4819_REG_10, 0x0145);
  BK4819_WriteRegister(BK4819_REG_11, 0x01B5);
  BK4819_WriteRegister(BK4819_REG_12, 0x0393);
  BK4819_WriteRegister(BK4819_REG_13, 0x03FF);
  BK4819_WriteRegister(BK4819_REG_14, 0x0019);
  BK4819_WriteRegister(BK4819_REG_40, 13738);
  BK4819_WriteRegister(BK4819_REG_29, 43840);
  BK4819_WriteRegister(BK4819_REG_19, 4161);
  BK4819_WriteRegister(BK4819_REG_73, 18066);
  BK4819_WriteRegister(BK4819_REG_3C, 20360);
  BK4819_WriteRegister(BK4819_REG_43, 13896);
  BK4819_WriteRegister(BK4819_REG_2B, 49152);
  Cleared = 1;
  SaveSettings();
}

// ============================================================
// SECTION: List item text helpers
// ============================================================

static bool GetScanListLabel(uint8_t scanListIndex, char* bufferOut) {
    char channel_name[12];
    uint16_t first_channel = 0xFFFF;
    for (uint16_t ch = MR_CHANNEL_FIRST; ch <= MR_CHANNEL_LAST; ch++) {
        ChannelAttributes_t *att = MR_GetChannelAttributes(ch);
        if (att->scanlist == (uint8_t)(scanListIndex + 1)) {
            first_channel = ch;
            break;
        }
    }
    if (first_channel == 0xFFFF) return false; 
    SETTINGS_FetchChannelName(channel_name, first_channel);
    char nameOrFreq[13];
    if (channel_name[0] == '\0') {
        uint32_t freq = 0;
        PY25Q16_ReadBuffer(0x0000 + (first_channel * 16), (uint8_t *)&freq, 4);
        if (freq < 1400000) {
            return false;
        }

        sprintf(nameOrFreq, "%u.%05u", freq / 100000, freq % 100000);
    } else {
        strncpy(nameOrFreq, channel_name, 12);
        nameOrFreq[12] = '\0';
    }

    if (settings.scanListEnabled[scanListIndex]) {
      sprintf(bufferOut, "%d:%-11s*", scanListIndex + 1, nameOrFreq);
    } else {
        sprintf(bufferOut, "%d:%-11s", scanListIndex + 1, nameOrFreq);
    }

    return true;
}

static void BuildValidScanListIndices() {
    uint8_t ScanListCount = 0;
    char tempName[17];
    for (uint8_t i = 0; i < MR_CHANNELS_LIST; i++) {

        if (GetScanListLabel(i, tempName)) {
            validScanListIndices[ScanListCount++] = i;
        }
    }
    validScanListCount = ScanListCount;
}


static void GetFilteredScanListText(uint16_t displayIndex, char* buffer) {
    uint8_t realIndex = validScanListIndices[displayIndex];
    GetScanListLabel(realIndex, buffer);
}


// ============================================================
// SECTION: Unified list renderer
// ============================================================

#define CHAR_WIDTH_PX 7
static uint8_t ListRightX(const char *s) {
    size_t len = strlen(s);
    return (len > 0 && len * CHAR_WIDTH_PX < 128)
           ? (uint8_t)(128 - len * CHAR_WIDTH_PX) : 1;
}

static void ListDrawSelectedBg(uint8_t line) {
    for (uint8_t x = 0; x < LCD_WIDTH; x++)
        for (uint8_t y = (uint8_t)(line * 8); y < (uint8_t)((line + 1) * 8); y++)
            PutPixel(x, y, true);
}

static void ListDrawRow(uint8_t line, const char *left, const char *right, bool inv) {
    if (inv) ListDrawSelectedBg(line);
    uint8_t bg = inv ? 1 : 0;
    if (inv) ListDrawSelectedBg(line);
    UI_PrintStringSmallbackground(left, 1, 0, line, bg);
    if (right[0])
        UI_PrintStringSmallbackground(right, ListRightX(right), 0, line, bg);
}
static bool inv = 0;

static void RenderUnifiedList(
    const char  *title,
    bool         useMeter,
    uint16_t     numItems,
    uint16_t     selectedIndex,
    uint16_t     scrollOffset,
    bool         invertSelected,
    GetListRowFn getRow)
{
    if (selectedIndex < scrollOffset) {
        scrollOffset = selectedIndex;
    } else {
        uint8_t occupiedLines = 0;
        const uint8_t MAX_LINES = 6;
        
        for (uint16_t i = scrollOffset; i <= selectedIndex; i++) {
            ListRow tempRow;
            getRow(i, &tempRow);
            uint8_t h = ((strlen(tempRow.left) + strlen(tempRow.right)) > 19) ? 2 : 1;
            
            if (i < selectedIndex) {
                occupiedLines += h;
            } else {
                if (occupiedLines + h > MAX_LINES) {
                    while (occupiedLines + h > MAX_LINES && scrollOffset < selectedIndex) {
                        ListRow firstRow;
                        getRow(scrollOffset, &firstRow);
                        uint8_t hFirst = ((strlen(firstRow.left) + strlen(firstRow.right)) > 19) ? 2 : 1;
                        occupiedLines -= hFirst;
                        scrollOffset++;
                    }
                }
            }
        }
    }
    

    memset(gFrameBuffer, 0, sizeof(gFrameBuffer));

    /* Header */
    if (useMeter && historyListActive && SpectrumMonitor > 0)
        DrawMeter(0);
    else if (title)
        UI_PrintStringSmallbackground(title, 1, LCD_WIDTH - 1, 0, 0);
    uint8_t currentLine = 1;
    for (uint16_t itemIndex = scrollOffset; itemIndex < numItems; itemIndex++) {
        ListRow row;
        getRow(itemIndex, &row);

        bool needsTwoLines = ((strlen(row.left) + strlen(row.right)) > 19);
        uint8_t itemHeight = needsTwoLines ? 2 : 1;

        if (currentLine + itemHeight > 7) break; // Sécurité limite physique LCD

        bool sel = (itemIndex == selectedIndex);
        bool inv = sel && invertSelected;

        if (!needsTwoLines) {
            ListDrawRow(currentLine, row.left, row.right, inv);
        } else {
            if (inv) {
                ListDrawSelectedBg(currentLine);
                ListDrawSelectedBg(currentLine + 1);
            }
            UI_PrintStringSmallbackground(row.left,  0, 0, currentLine, inv);
            UI_PrintStringSmallbackground(row.right, 0, 0, currentLine + 1, inv);
        }
        currentLine += itemHeight;
    }
    ST7565_BlitFullScreen();
}

// === Effectue un unique balayage pour les 6 lignes visibles au départ ===
void PreloadHistoryChannels(uint16_t startScrollIndex, uint16_t totalCount) {
    // 1. Initialisation propre du cache
    for (uint8_t i = 0; i < 6; i++) {
        cachedChannels[i] = 0xFFFF;
        cachedAbsoluteIdx[i] = 0xFFFF; 
    }
    cacheWriteHead = 0;

    if (!inv || totalCount == 0) return;

    // 2. Un seul parcours de tous les canaux de la radio pour le bloc visible
    for (uint16_t ch = MR_CHANNEL_FIRST; ch <= MR_CHANNEL_LAST; ch++) {
        ChannelInfo_t freqcmp = FetchChannelFrequency(ch);
        if (freqcmp.frequency == 0 || freqcmp.frequency == 0xFFFFFFFF) continue;

        for (uint8_t i = 0; i < 6; i++) {
            uint16_t historyIdx = startScrollIndex + i;
            if (historyIdx >= totalCount) break;

            if (HFreqs[historyIdx] == freqcmp.frequency && cachedAbsoluteIdx[i] == 0xFFFF) {
                cachedChannels[i] = ch;
                cachedAbsoluteIdx[i] = historyIdx;
            }
        }
    }
}

// === Récupère la ligne et gère le cache tournant si la ligne est hors-cache ===
static void GetHistoryRow(uint16_t index, ListRow *row) {
    row->left[0]  = '\0';
    row->right[0] = '\0';
    
    uint32_t f = HFreqs[index];
    if (!f) return;

    char freqStr[10];
    char timeStr[16];
    snprintf(freqStr, sizeof(freqStr), "%u.%05u", f / 100000, f % 100000);
    RemoveTrailZeros(freqStr);
    char Name[12] = "";
    
    
        uint16_t ch = 0xFFFF;
        bool foundInCache = false;

        // 1. Chercher d'abord si l'index est déjà présent dans le mini-cache
        for (uint8_t i = 0; i < 6; i++) {
            if (cachedAbsoluteIdx[i] == index) {
                ch = cachedChannels[i];
                foundInCache = true;
                break;
            }
        }

        // 2. Si non trouvé (hors-cadre suite au scroll rapide), chercher à la volée 
        //    et l'insérer dans le cache circulaire
        if (!foundInCache) {
            ch = BOARD_gMR_fetchChannel(f); // Recherche unique
            
            // Écriture dans le cache circulaire
            cachedAbsoluteIdx[cacheWriteHead] = index;
            cachedChannels[cacheWriteHead] = ch;
            cacheWriteHead = (cacheWriteHead + 1) % 6; // Avance la tête de 0 à 5
        }

        // 3. Récupérer le nom si le canal est valide
        if (ch != 0xFFFF) {
            SETTINGS_FetchChannelName(Name, ch);
            Name[10] = '\0';
        }
    
    
    const char *prefix = HBlacklisted[index] ? "#" : "";
    if (HTimeS[index] > 59)
        snprintf(timeStr, sizeof(timeStr), " %02d:%02d", HTimeS[index] / 60, HTimeS[index] % 60);
    else 
        snprintf(timeStr, sizeof(timeStr), " %02d", HTimeS[index]);
        
    snprintf(row->right, sizeof(row->right), "%s", timeStr);
    snprintf(row->left, sizeof(row->left), "%s%s %s", prefix, freqStr, Name);
}

/* Scanlist multiselect: "N:name" on left, "*" on right when enabled */
static void GetScanListRow(uint16_t displayIndex, ListRow *row) {
    char buf[20];
    GetFilteredScanListText(displayIndex, buf);
    /* Strip trailing '*' marker and padding spaces added by GetScanListLabel */
    size_t len = strlen(buf);
    bool enabled = (len > 0 && buf[len - 1] == '*');
    if (enabled) buf[--len] = '\0';
    while (len > 0 && buf[len - 1] == ' ') buf[--len] = '\0';
    snprintf(row->left, sizeof(row->left), "%s", buf);
    if (enabled) { snprintf(row->right, sizeof(row->right), "<===="); }
    else           row->right[0] = '\0';
}

static void GetBandRow(uint16_t index, ListRow *row) {
    snprintf(row->left, sizeof(row->left), "%d:%s", index + 1, BParams[index].BandName);
    if (settings.bandEnabled[index]) { snprintf(row->right, sizeof(row->right), "<====");}
    else                               row->right[0] = '\0';
}

static void GetParametersRow(uint16_t index, ListRow *row) {
    row->right[0] = '\0';
    uint16_t realIndex = index;
    if (Light_Mode) {realIndex = lightModeMenuMapping[index];
}
    switch (realIndex) {
        case PARAM_RSSI_DELAY:
            snprintf(row->left,  sizeof(row->left),  "RSSI Delay:");
            snprintf(row->right, sizeof(row->right), "%sms", DelayRssiText[IndexDelayRssi]);
            break;
        case PARAM_SPECTRUM_DELAY:
            snprintf(row->left, sizeof(row->left), "Spectrum Delay:");
            if (SpectrumDelay < 65000)
                snprintf(row->right, sizeof(row->right), "%us", SpectrumDelay / 1000);
            else
                strncpy(row->right, "OFF", sizeof(row->right) - 1);
            break;
        case PARAM_MAX_LISTEN_TIME:
            snprintf(row->left,  sizeof(row->left),  "MaxListenTime:");
            snprintf(row->right, sizeof(row->right), "%s", labels[IndexMaxLT]);
            break;
        case PARAM_RANGE_START: {
            char tmp[12];
            snprintf(tmp, sizeof(tmp), "%u.%05u",
                     RangeStart / 100000, RangeStart % 100000);
            snprintf(row->left,  sizeof(row->left),  "Fstart:");
            snprintf(row->right, sizeof(row->right), "%s", tmp);
            break;
        }
        case PARAM_RANGE_STOP: {
            char tmp[12];
            snprintf(tmp, sizeof(tmp), "%u.%05u",
                     RangeStop / 100000, RangeStop % 100000);
           // RemoveTrailZeros(tmp);
            snprintf(row->left,  sizeof(row->left),  "Fstop:");
            snprintf(row->right, sizeof(row->right), "%s", tmp);
            break;
        }
        case PARAM_SCAN_STEP: {
            uint32_t step = GetScanStep();
            snprintf(row->left, sizeof(row->left), "Step:");
            snprintf(row->right, sizeof(row->right),
                     step % 100 ? "%uk%02u" : "%uk", step / 100, step % 100);
            break;
        }
        case PARAM_LISTEN_BW:
            snprintf(row->left,  sizeof(row->left),  "Listen BW:");
            snprintf(row->right, sizeof(row->right), "%s", bwNames[settings.listenBw]);
            break;
        case PARAM_MODULATION:
            snprintf(row->left,  sizeof(row->left),  "Modulation:");
            snprintf(row->right, sizeof(row->right), "%s", gModulationStr[settings.modulationType]);
            break;
        case PARAM_POWER_SAVE:
            snprintf(row->left,  sizeof(row->left),  "Power Save:");
            snprintf(row->right, sizeof(row->right), "%s", labelsPS[IndexPS]);
            break;
        case PARAM_NOISE_LEVEL_OFF:
            snprintf(row->left,  sizeof(row->left),  "Nois LVL OFF:");
            snprintf(row->right, sizeof(row->right), "%d", Noislvl_OFF);
            break;
        case PARAM_OSD_POPUP:
            snprintf(row->left, sizeof(row->left), "Popups:");
            if (osdPopupSetting) {
                uint8_t sec = osdPopupSetting / 1000;
                uint8_t dec = (osdPopupSetting % 1000) / 100;
                if (dec) snprintf(row->right, sizeof(row->right), "%d.%ds", sec, dec);
                else     snprintf(row->right, sizeof(row->right), "%ds", sec);
            } else {
                strncpy(row->right, "OFF", sizeof(row->right) - 1);
            }
            break;
        case PARAM_RECORD_TRIGGER:
            snprintf(row->left,  sizeof(row->left),  "Record Trig:");
            snprintf(row->right, sizeof(row->right), "%d", UOO_trigger);
            break;
        case PARAM_AUTO_KEYLOCK:
            if (AUTO_KEYLOCK) {
                snprintf(row->left,  sizeof(row->left),  "Keylock:");
                snprintf(row->right, sizeof(row->right), "%ds", durations[AUTO_KEYLOCK] / 2);
            } else {
                snprintf(row->left, sizeof(row->left), "Key Unlocked");
            }
            break;
        case PARAM_GLITCH_MAX:
            snprintf(row->left,  sizeof(row->left),  "GlitchMax:");
            snprintf(row->right, sizeof(row->right), "%d", GlitchMax);
            break;
        case PARAM_SOUND_BOOST:
            snprintf(row->left, sizeof(row->left), "SoundBoost:");
            strncpy(row->right, SoundBoost ? "ON" : "OFF", sizeof(row->right) - 1);
            break;
        case PARAM_PTT_EMISSION:
            snprintf(row->left, sizeof(row->left), "PTT:");
            switch (PttEmission) {
            case 0: 
                strncpy(row->right, "VFO FREQ", sizeof(row->right) - 1);
                break;
            case 1:
                strncpy(row->right, "NINJA",    sizeof(row->right) - 1);
                break;
            case 2:
                strncpy(row->right, "LAST RX",  sizeof(row->right) - 1);
                break;
            case 3: 
                strncpy(row->right, "AUTO ROGER 10s",  sizeof(row->right) - 1);
                gAutoPtt_Time = 10;
                break;
            case 4: 
                strncpy(row->right, "AUTO ROGER 30s",  sizeof(row->right) - 1);
                gAutoPtt_Time = 30;
                break;
            case 5: 
                strncpy(row->right, "AUTO ROGER 2m",  sizeof(row->right) - 1);
                gAutoPtt_Time = 120;
                break;
            case 6: 
                strncpy(row->right, "AUTO ROGER 5m",  sizeof(row->right) - 1);
                gAutoPtt_Time = 300;
                break;
            case 7: 
                strncpy(row->right, "AUTO ROGER 10m",  sizeof(row->right) - 1);
                gAutoPtt_Time = 600;
                break;
            case 8: 
                strncpy(row->right, "AUTO ROGER 30m",  sizeof(row->right) - 1);
                gAutoPtt_Time = 1800;
                break;
            }
            break;
        case PARAM_MONITOR_SCAN:
            snprintf(row->left, sizeof(row->left), "Monitor SL");
            if (gMonitorScan) snprintf(row->right, sizeof(row->right), "ON");
            else snprintf(row->right, sizeof(row->right), "OFF");
            break;
        case PARAM_RESET_DEFAULT:
            snprintf(row->left, sizeof(row->left), "Reset Default");
            strncpy(row->right, ">", sizeof(row->right) - 1);
            break;
        case PARAM_LIGHT_MODE:
            strncpy(row->left, Light_Mode ? "Advanced Menu" : "Light Menu", sizeof(row->left) - 1);
            break;
        default:
            row->left[0] = '\0';
            break;
    }
}

// ============================================================
// SECTION: List screen render functions
// ============================================================

static void RenderScanListSelect() {
    if (refreshScanListName) {
        BuildValidScanListIndices(); 
        refreshScanListName = false;
    }
    uint8_t selectedCount = 0;
    for (uint8_t i = 0; i < validScanListCount; i++) {
        if (settings.scanListEnabled[validScanListIndices[i]]) selectedCount++;
    }
    char title[24];
    snprintf(title, sizeof(title), "SCANLISTS: %u/%u", selectedCount, validScanListCount);
    RenderUnifiedList(title, false, validScanListCount, scanListSelectedIndex,
                      scanListScrollOffset, true, GetScanListRow);
}

static void RenderParametersSelect() {
    RenderUnifiedList("PARAMETERS:", false, GetMaxVisualRows(), parametersSelectedIndex,
                      parametersScrollOffset, true, GetParametersRow);
}

void RenderBandSelect() {
    RenderUnifiedList("BANDS:", false, bandCount, bandListSelectedIndex,
                      bandListScrollOffset, true, GetBandRow);
}

static void RenderHistoryList() {
    uint16_t count = CountValidHistoryItems();
    char title[32];
    sprintf(title, "HISTORY: %d", count);

    // Only preload channels if the scroll offset has changed to optimize performance
    if (historyScrollOffset != lastHistoryScrollOffset) {
        PreloadHistoryChannels(historyScrollOffset, count);
        lastHistoryScrollOffset = historyScrollOffset; // Update tracking variable
    }

    RenderUnifiedList(title, false, count, historyListIndex,
                      historyScrollOffset, true, GetHistoryRow);
}
