/* Copyright 2023 Dual Tachyon / Robzyl KA52
 * Licensed under the Apache License, Version 2.0
 */

#include <string.h>
#include "nav_invert.h"

#include "app/action.h"
#include "app/app.h"
#include "app/common.h"
#ifdef ENABLE_FMRADIO
    #include "app/fm.h"
#endif
#include "app/generic.h"
#include "app/main.h"

#ifdef ENABLE_SPECTRUM
#include "app/spectrum.h"
#endif


#include "board.h"
#include "driver/bk4819.h"
#include "frequencies.h"
#include "misc.h"
#include "radio.h"
#include "settings.h"
#include "driver/py25q16.h"
#include "driver/st7565.h"
#include "driver/system.h"
#include "ui/helper.h"

// ── Save current VFO frequency to first free memory channel ──────────────────
static void SaveFreqToFreeChannel(void)
{
    if (!IS_FREQ_CHANNEL(gTxVfo->CHANNEL_SAVE))
        return;

    uint32_t f = gTxVfo->pRX->Frequency;
    if (f < 1000000)
        return;

    int freeCh = -1;
    int firstEmptyCh = -1;

    // Une seule boucle pour trouver soit un doublon, soit le premier canal vide
    for (uint16_t i = MR_CHANNEL_FIRST; i <= MR_CHANNEL_LAST; i++) {
        uint32_t chf = SETTINGS_FetchChannelFrequency(i);
        
        // Si la fréquence existe déjà, c'est notre cible prioritaire (on écrase)
        if (chf == f) {
            freeCh = (int)i;
            break; 
        }
        
        // On mémorise le premier canal vide trouvé (au cas où la fréquence n'existe nulle part)
        if (firstEmptyCh == -1 && (chf == 0xFFFFFFFF || chf == 0)) {
            firstEmptyCh = (int)i;
        }
    }

    // Si la fréquence n'a pas été trouvée, on utilise le premier canal vide mémorisé
    if (freeCh == -1) {
        freeCh = firstEmptyCh;
    }

    // Sauvegarde et affichage du Pop-up
    if (freeCh >= 0) {
        SETTINGS_SaveChannel((uint16_t)freeCh, gEeprom.TX_VFO, gTxVfo, 2);
        MR_InvalidateChannelAttributesCache();
        
        char chStr[16];
        sprintf(chStr, "SAVE CH:%d", freeCh + 1);
        UI_DisplayPopup(chStr);
        ST7565_BlitLine(2);
        ST7565_BlitLine(3);
        SYSTEM_DelayMs(800);
    } else {
        UI_DisplayPopup("MEM FULL");
        ST7565_BlitLine(2);
        ST7565_BlitLine(3);
        SYSTEM_DelayMs(600);
    }
    
    gRequestDisplayScreen = DISPLAY_MAIN;
    gUpdateDisplay = true;
}

#include "ui/inputbox.h"
#include "ui/ui.h"
#include <stdlib.h>

// ── Delete current MR channel with M confirmation ───────────────────────────
static void DeleteChannelWithConfirm(void)
{
    if (!IS_MR_CHANNEL(gTxVfo->CHANNEL_SAVE))
        return;

    uint16_t ch = gTxVfo->CHANNEL_SAVE;

    // Inverted 2-line popup: line1=DEL CH:N, line2=[M]=YES
    char line1[14];
    sprintf(line1, "DEL CH:%d?", ch + 1);
    // Draw inverted popup on rows 2-3 (line1) and 4-5 ([M]=YES)
    // Row 2-3: channel name, inverted
    memset(gFrameBuffer[2], 0x00, LCD_WIDTH);
    memset(gFrameBuffer[3], 0x00, LCD_WIDTH);
    UI_PrintString(line1, 12, 116, 2, 8);
    for (uint8_t x = 0; x < LCD_WIDTH; x++) {
        gFrameBuffer[2][x] ^= 0xFF;
        gFrameBuffer[3][x] ^= 0xFF;
    }
    // Row 4-5: confirm hint, inverted
    memset(gFrameBuffer[4], 0x00, LCD_WIDTH);
    memset(gFrameBuffer[5], 0x00, LCD_WIDTH);
    UI_PrintString("[M]=YES", 12, 116, 4, 8);
    for (uint8_t x = 0; x < LCD_WIDTH; x++) {
        gFrameBuffer[4][x] ^= 0xFF;
        gFrameBuffer[5][x] ^= 0xFF;
    }
    ST7565_BlitLine(2);
    ST7565_BlitLine(3);
    ST7565_BlitLine(4);
    ST7565_BlitLine(5);

    // Wait for M (confirm) or any other key (cancel)
    // Poll for up to ~3 seconds
    bool confirmed = false;
    for (int t = 0; t < 300; t++) {
        SYSTEM_DelayMs(10);
        KEY_Code_t k = KEYBOARD_Poll();
        if (k == KEY_MENU) { confirmed = true; break; }
        if (k != KEY_INVALID) break; // any other key = cancel
    }

    if (confirmed) {
        SETTINGS_UpdateChannel(ch, NULL, false, false, true);
        MR_InvalidateChannelAttributesCache();
        gVfoConfigureMode = VFO_CONFIGURE_RELOAD;
        gFlagResetVfos    = true;
        UI_DisplayPopup("DELETED");
        ST7565_BlitLine(2);
        ST7565_BlitLine(3);
        SYSTEM_DelayMs(600);
    } 
    gRequestDisplayScreen = DISPLAY_MAIN;
    gUpdateDisplay = true;
}

// ── Scanlist toggle ──────────────────────────────────────────────────────────

static void toggle_chan_scanlist(void)
{
    if (!IS_MR_CHANNEL(gTxVfo->CHANNEL_SAVE)) {
        return;
    }

    ChannelAttributes_t *att = MR_GetChannelAttributes(gTxVfo->CHANNEL_SAVE);

    if (att->exclude == true) {
        att->exclude = false;
        MR_SaveChannelAttributesToFlash(gTxVfo->CHANNEL_SAVE, att);
    } else {
        uint8_t scanlist = gTxVfo->SCANLIST_PARTICIPATION;
        scanlist++;
        if (scanlist > MR_CHANNELS_LIST + 1)
            scanlist = 0;
        gTxVfo->SCANLIST_PARTICIPATION = scanlist;
        SETTINGS_UpdateChannel(gTxVfo->CHANNEL_SAVE, gTxVfo, true, true, true);
    }

    gVfoConfigureMode = VFO_CONFIGURE;
    gFlagResetVfos    = true;
}

// ── F-key functions ──────────────────────────────────────────────────────────

static void processFKeyFunction(const KEY_Code_t Key, const bool beep)
{
    if (gScreenToDisplay == DISPLAY_MENU) {
        return;
    }
    switch (Key) {
        case KEY_0:
#ifdef ENABLE_FMRADIO
            ACTION_FM();
#endif
            break;

        case KEY_1:
            const uint8_t Vfo1 = gEeprom.TX_VFO;
            if (IS_MR_CHANNEL(gTxVfo->CHANNEL_SAVE)) {
                uint32_t mrFreq = gTxVfo->pRX->Frequency;
                uint8_t  mrBand = gTxVfo->Band;
                uint16_t freqCh = FREQ_CHANNEL_FIRST + mrBand;
                gEeprom.ScreenChannel[Vfo1] = freqCh;
                gEeprom.FreqChannel[Vfo1]   = freqCh;
                RADIO_SelectVfos();            
                gTxVfo->pRX->Frequency = mrFreq;
                gTxVfo->pTX->Frequency = mrFreq;
                gTxVfo->Band           = mrBand;
                SETTINGS_SaveChannel(freqCh, Vfo1, gTxVfo, 2); 
                RADIO_ConfigureSquelchAndOutputPower(gTxVfo);
                RADIO_SetupRegisters(true);
                gVfoConfigureMode     = VFO_CONFIGURE_RELOAD;
                gRequestDisplayScreen = DISPLAY_MAIN;
            } 
            break;

        case KEY_2:
            if (IS_MR_CHANNEL(gTxVfo->CHANNEL_SAVE)) {
                // F+2 in MR mode: delete with sync confirmation popup
                DeleteChannelWithConfirm();
            } else {
                // F+2 in VFO mode: save frequency to nearest free channel
                SaveFreqToFreeChannel();
            }
            break;

        case KEY_3:
        case KEY_4:
        case KEY_5:
            break;

        case KEY_6:
            APP_RunSpectrumMode(2);
            gRequestDisplayScreen = DISPLAY_MAIN;
            break;

        case KEY_7:
            APP_RunSpectrumMode(1);
            gRequestDisplayScreen = DISPLAY_MAIN;
            break;

        case KEY_8:
            APP_RunSpectrumMode(3);
            gRequestDisplayScreen = DISPLAY_MAIN;
            break;

        case KEY_9:
            APP_RunSpectrumMode(0);
            gRequestDisplayScreen = DISPLAY_MAIN;
            break;

        case KEY_UP:
            gEeprom.SQUELCH_LEVEL = (gEeprom.SQUELCH_LEVEL < 9) ? gEeprom.SQUELCH_LEVEL + 1 : 9;
            gVfoConfigureMode     = VFO_CONFIGURE;
            gWasFKeyPressed       = false;
            break;

        case KEY_DOWN:
            gEeprom.SQUELCH_LEVEL = (gEeprom.SQUELCH_LEVEL > 0) ? gEeprom.SQUELCH_LEVEL - 1 : 0;
            gVfoConfigureMode     = VFO_CONFIGURE;
            gWasFKeyPressed       = false;
            break;

        case KEY_SIDE1: {
            uint8_t a = FREQUENCY_GetSortedIdxFromStepIdx(gTxVfo->STEP_SETTING);
            if (a < STEP_N_ELEM - 1)
                gTxVfo->STEP_SETTING = FREQUENCY_GetStepIdxFromSortedIdx(a + 1);
            if (IS_FREQ_CHANNEL(gTxVfo->CHANNEL_SAVE))
                gRequestSaveChannel = 1;
            gVfoConfigureMode = VFO_CONFIGURE;
            gWasFKeyPressed   = false;
            break;
        }

        case KEY_SIDE2: {
            uint8_t b = FREQUENCY_GetSortedIdxFromStepIdx(gTxVfo->STEP_SETTING);
            if (b > 0)
                gTxVfo->STEP_SETTING = FREQUENCY_GetStepIdxFromSortedIdx(b - 1);
            if (IS_FREQ_CHANNEL(gTxVfo->CHANNEL_SAVE))
                gRequestSaveChannel = 1;
            gVfoConfigureMode = VFO_CONFIGURE;
            gWasFKeyPressed   = false;
            break;
        }

        default:
            gUpdateStatus   = true;
            gWasFKeyPressed = false;
            break;
    }
}

// ── Channel move ─────────────────────────────────────────────────────────────

void channelMove(uint16_t Channel)
{
    const uint8_t Vfo = gEeprom.TX_VFO;

    if (!RADIO_CheckValidChannel(Channel, false, 0)) {
        return;
    }

    gEeprom.MrChannel[Vfo]     = (uint16_t)Channel;
    gEeprom.ScreenChannel[Vfo] = (uint16_t)Channel;
    gVfoConfigureMode           = VFO_CONFIGURE_RELOAD;

    RADIO_ConfigureChannel(gEeprom.TX_VFO, gVfoConfigureMode);
    SETTINGS_SaveVfoIndices();
}

void channelMoveSwitch(void)
{
    if (!IS_MR_CHANNEL(gTxVfo->CHANNEL_SAVE))
        return;

    uint16_t Channel = 0;
    for (uint8_t i = 0; i < gInputBoxIndex; i++)
        Channel = (Channel * 10) + gInputBox[i];

    if ((Channel == 0) && (gInputBoxIndex != 4))
        return;

    if (gInputBoxIndex == 4) {
        gInputBoxIndex     = 0;
        gKeyInputCountdown = 1;
    }

    channelMove(Channel - 1);
    SETTINGS_SaveVfoIndices();
}

// ── Digit keys ───────────────────────────────────────────────────────────────

static void MAIN_Key_DIGITS(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld)
{
    if (bKeyHeld) {
        if (bKeyPressed) {
            if (gScreenToDisplay == DISPLAY_MAIN) {
                if (gInputBoxIndex > 0) {
                    gInputBoxIndex        = 0;
                    gRequestDisplayScreen = DISPLAY_MAIN;
                }

                HideFKeyIcon();

                // Долгое 7: фонарик при RX
                if (Key == KEY_7) {
                    gEeprom.FlashlightOnRX = !gEeprom.FlashlightOnRX;
                    gRequestSaveSettings   = true;
                    gUpdateStatus          = true;
                    gRequestDisplayScreen  = DISPLAY_MAIN;
                    return;
                }
                // Долгое 9: подсветка
                if (Key == KEY_9) {
                    if (gBackLight)
                        ACTION_BackLight();
                    else
                        ACTION_BackLightOnDemand();
                    return;
                }
                // Долгое 0: модуляция
                if (Key == KEY_0) {
                    ACTION_SwitchDemodul();
                    gRequestDisplayScreen = DISPLAY_MAIN;
                    return;
                }
                if (Key == KEY_2 && gEeprom.DUAL_WATCH != DUAL_WATCH_OFF) {
                    gVfoConfigureMode = VFO_CONFIGURE;
                    COMMON_SwitchVFOs();
                    return;
                }
                if (Key == KEY_3) {
                    gVfoConfigureMode = VFO_CONFIGURE;
                    COMMON_SwitchVFOMode();
                    return;
                }
                if (Key == KEY_4) {
                    ACTION_Wn();
                    gRequestDisplayScreen = DISPLAY_MAIN;
                    return;
                }
                if (Key == KEY_5) {
                    if (IS_MR_CHANNEL(gTxVfo->CHANNEL_SAVE)) {
                        toggle_chan_scanlist();
                        gRequestDisplayScreen = DISPLAY_MAIN;
                    } else {
                        uint8_t a = FREQUENCY_GetSortedIdxFromStepIdx(gTxVfo->STEP_SETTING);
                        a = (a < STEP_N_ELEM - 1) ? (a + 1) : 0;
                        gTxVfo->STEP_SETTING  = FREQUENCY_GetStepIdxFromSortedIdx(a);
                        gTxVfo->StepFrequency = gStepFrequencyTable[gTxVfo->STEP_SETTING];
                        // Save and apply immediately
                        SETTINGS_SaveChannel(gTxVfo->CHANNEL_SAVE, gEeprom.TX_VFO, gTxVfo, 2);
                        gVfoConfigureMode     = VFO_CONFIGURE;
                        gUpdateStatus         = true;
                        gUpdateDisplay        = true;
                        gRequestDisplayScreen = DISPLAY_MAIN;
                    }
                    gWasFKeyPressed = false;
                    return;
                }

                // Долгое 6: цикл мощности L→M→H→U→L
                if (Key == KEY_6) {
                    ACTION_Power();
                    gRequestDisplayScreen = DISPLAY_MAIN;
                    return;
                }

                // Долгое 8: ничего (спектр бендов только по F+8)
                if (Key == KEY_8) {
                    return;
                }

                // Остальные долгие (1) → processFKeyFunction(Key, true)
                processFKeyFunction(Key, true);
            }
        }
        return;
    }

    if (bKeyPressed) {
        return;
    }

    if (gWasFKeyPressed) {
        // F+digit: route to processFKeyFunction
        gWasFKeyPressed = false;
        HideFKeyIcon();
        processFKeyFunction(Key, true);
        return;
    }

    if (!gWasFKeyPressed) {
        const uint8_t Vfo = gEeprom.TX_VFO;
        INPUTBOX_Append(Key);
        gKeyInputCountdown = key_input_timeout_500ms;

        channelMoveSwitch();
        gRequestDisplayScreen = DISPLAY_MAIN;

        if (IS_MR_CHANNEL(gTxVfo->CHANNEL_SAVE)) {
            gKeyInputCountdown = key_input_timeout_500ms / 4;
            return;
        }

        if (IS_FREQ_CHANNEL(gTxVfo->CHANNEL_SAVE)) {
            uint8_t totalDigits = 6;
            if (gTxVfo->pRX->Frequency >= _1GHz_in_KHz)
                totalDigits = 7;

            if (gInputBoxIndex == 0)
                return;

            gKeyInputCountdown = (gInputBoxIndex >= totalDigits)
                ? (key_input_timeout_500ms / 16)
                : (key_input_timeout_500ms / 3);

            if (gInputBoxIndex > totalDigits) {
                gInputBoxIndex = totalDigits;
                return;
            }

            const char *inputStr  = INPUTBOX_GetAscii();
            uint8_t     inputLen  = gInputBoxIndex;
            uint32_t    inputFreq = StrToUL(inputStr);
            for (uint8_t i = 0; i < (totalDigits - inputLen); i++)
                inputFreq *= 10;

            uint32_t Frequency = inputFreq * 100;

            if (Frequency < frequencyBandTable[0].lower)
                Frequency = frequencyBandTable[0].lower;
            else if (Frequency >= BX4819_band1.upper && Frequency < BX4819_band2.lower) {
                const uint32_t center = (BX4819_band1.upper + BX4819_band2.lower) / 2;
                Frequency = (Frequency < center) ? BX4819_band1.upper : BX4819_band2.lower;
            } else if (Frequency > frequencyBandTable[BAND_N_ELEM - 1].upper)
                Frequency = frequencyBandTable[BAND_N_ELEM - 1].upper;

            const FREQUENCY_Band_t band = FREQUENCY_GetBand(Frequency);
            if (gTxVfo->Band != band) {
                gTxVfo->Band               = band;
                gEeprom.ScreenChannel[Vfo] = band + FREQ_CHANNEL_FIRST;
                gEeprom.FreqChannel[Vfo]   = band + FREQ_CHANNEL_FIRST;
                SETTINGS_SaveVfoIndices();
                RADIO_ConfigureChannel(Vfo, VFO_CONFIGURE_RELOAD);
            }

            Frequency = FREQUENCY_RoundToStep(Frequency, gTxVfo->StepFrequency);
            if (Frequency >= BX4819_band1.upper && Frequency < BX4819_band2.lower) {
                const uint32_t center = (BX4819_band1.upper + BX4819_band2.lower) / 2;
                Frequency = (Frequency < center)
                    ? BX4819_band1.upper - gTxVfo->StepFrequency
                    : BX4819_band2.lower;
            }
            gTxVfo->freq_config_RX.Frequency = Frequency;
            gRequestSaveChannel = 1;
            return;
        }

        gRequestDisplayScreen = DISPLAY_MAIN;
        return;
    }

    // F + цифра
    HideFKeyIcon();
    processFKeyFunction(Key, true);  // beep=true = F-key press (not long press)
}

// ── EXIT key ─────────────────────────────────────────────────────────────────

static void MAIN_Key_EXIT(bool bKeyPressed, bool bKeyHeld)
{
    if (!bKeyHeld && bKeyPressed) {
        return;
    }

    if (bKeyHeld) {
        if (bKeyPressed) {
            gInputBoxIndex        = 0;
            gRequestDisplayScreen = DISPLAY_MAIN;
        }
        return;
    }

    // released
#ifdef ENABLE_FMRADIO
    if (!gFmRadioMode)
#endif
    {
            if (gInputBoxIndex == 0)
                return;
            gInputBox[--gInputBoxIndex] = 10;
            gKeyInputCountdown = key_input_timeout_500ms;

        gRequestDisplayScreen = DISPLAY_MAIN;
        return;
    }

#ifdef ENABLE_FMRADIO
    ACTION_FM();
#endif
}

// ── MENU key ─────────────────────────────────────────────────────────────────

static void MAIN_Key_MENU(bool bKeyPressed, bool bKeyHeld)
{
    if (bKeyHeld) {
        if (bKeyPressed) {
            gWasFKeyPressed = false;
            if (gScreenToDisplay == DISPLAY_MAIN) {
                if (gInputBoxIndex > 0) {
                    gInputBoxIndex        = 0;
                    gRequestDisplayScreen = DISPLAY_MAIN;
                }
                gUpdateStatus = true;
                ACTION_Handle(KEY_MENU, bKeyPressed, bKeyHeld);
            }
        }
        return;
    }

    if (!bKeyPressed) {
        gKeyInputCountdown = 1;
        channelMoveSwitch();
        const bool bFlag = !gInputBoxIndex;
        gInputBoxIndex   = 0;

        if (bFlag) {
            gFlagRefreshSetting   = true;
            gRequestDisplayScreen = DISPLAY_MENU;
        } else {
            gRequestDisplayScreen = DISPLAY_MAIN;
        }
    }
}


// ── UP/DOWN keys ─────────────────────────────────────────────────────────────

static void MAIN_Key_UP_DOWN(bool bKeyPressed, bool bKeyHeld, int8_t Direction)
{
    if (!gEeprom.SET_NAV)
        Direction = -Direction;

#ifdef ENABLE_FEAT_F4HWN
    if (gWasFKeyPressed) {
        processFKeyFunction(Direction == 1 ? KEY_UP : KEY_DOWN, true);
        return;
    }
#endif

    uint16_t Channel = gEeprom.ScreenChannel[gEeprom.TX_VFO];

    if (bKeyHeld || !bKeyPressed) {
        if (gInputBoxIndex > 0)
            return;
        if (!bKeyPressed) {
            if (!bKeyHeld || IS_FREQ_CHANNEL(Channel))
                return;
            return;
        }
    } else {
        if (gInputBoxIndex > 0) {
            return;
        }
    }
        {
            if (IS_FREQ_CHANNEL(Channel)) {
                const uint32_t frequency = APP_SetFrequencyByStep(gTxVfo, Direction);
                if (RX_freq_check(frequency) < 0) {
                    return;
                }
                gTxVfo->freq_config_RX.Frequency = frequency;
                BK4819_SetFrequency(frequency);
                BK4819_RX_TurnOn();
                gRequestSaveChannel = 1;
                return;
            }

            uint16_t Next = RADIO_FindNextChannel(Channel + Direction, Direction, false, 0);
            if (Next == 0xFFFF)
                return;
            if (Channel == Next)
                return;
            gEeprom.MrChannel[gEeprom.TX_VFO]    = Next;
            gEeprom.ScreenChannel[gEeprom.TX_VFO] = Next;
            if (!bKeyHeld) {
            }
        }
        gRequestSaveVFO   = true;
        gVfoConfigureMode = VFO_CONFIGURE_RELOAD;
        return;
    
    gPttWasReleased        = true;
}

// ── Main dispatcher ──────────────────────────────────────────────────────────

void MAIN_ProcessKeys(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld)
{
#ifdef ENABLE_FMRADIO
    if (gFmRadioMode && Key != KEY_PTT && Key != KEY_EXIT) {
        if (!bKeyHeld && bKeyPressed)
        return;
    }
#endif

    switch (Key) {
#ifdef ENABLE_FEAT_F4HWN
        case KEY_SIDE1:
        case KEY_SIDE2:
#endif
        case KEY_0 ... KEY_9:
            MAIN_Key_DIGITS(Key, bKeyPressed, bKeyHeld);
            break;
        case KEY_MENU:
            MAIN_Key_MENU(bKeyPressed, bKeyHeld);
            break;
        case KEY_UP:
        case KEY_DOWN:
            MAIN_Key_UP_DOWN(bKeyPressed, bKeyHeld, NAV_DIR(Key == KEY_UP ? 1 : -1));
            break;
        case KEY_EXIT:
            MAIN_Key_EXIT(bKeyPressed, bKeyHeld);
            break;
        case KEY_STAR:
            break;
        case KEY_F:
            GENERIC_Key_F(bKeyPressed, bKeyHeld);
            break;
        case KEY_PTT:
            GENERIC_Key_PTT(bKeyPressed);
            break;
        default:
            break;
    }
}
