# Sonic Firmware manual V77

◀️  Join the SonicFw Telegram [https://t.me/SonicFw](https://t.me/SonicFw)

📺 Youtube Channel [https://www.youtube.com/@robby_69400](https://www.youtube.com/@robby_69400)

💾 DOWNLOAD select RAW file [DOWNLOAD_HERE](https://github.com/Robby69400/Sonic/tree/main/.DOWNLOAD_HERE)

🙏 Many thanks to Sonic Team : Zylka, Kolyan, Toni, Yves, Francois and Adam.

This software is a fork of Armel F4HWN firmware with code from NTOIVOLA, EGZUMER and DUAL TACHYON.

## EXPLANATION OF BUTTON SYMBOLS:

**1 –** Simple press.

**L1 –** Long press.

**F1–** Press after the F button.

## MR MAIN SCREEN (CHANNELS) | VFO (FREQUENCY)

**L0 -** (MR) **–** Transfer the channel frequency to VFO mode.

**L1 –** Increase Squelch level

**L2 –** (MR and VFO) **–** Bandwidth change.

**L3 –** AM/FM/USB Modulation Change

**L4 –** (MR & VFO) **–** Power Change (L/M/H).

**L5 –** (MR and VFO) **–** Step change.

**L6 –** Change the MR/VFO mode.

**L7 –** (MR) **–** Change the list number for the channel.

**L8 –** (MR and VFO) **–** On/Off flashlight indication when receiving a signal.

**L9 –** (MR and VFO) **–** On/Off constant backlight.

**F2 –** (MR) **–** Delete Channel. (VFO) **–** Preserve the frequency.

**F6 –** Spectrum analyzer in scanrange mode.

**F7 –** Spectrum analyzer in scanlist mode.

**F8 –** Band Spectrum Analyzer.

**F9 –** Launch Close Call

**F0 –** Radio broadcasting (FM radio).

**SK1** and **SK2**—Side keys, assignable buttons.

**<=** and **=>** – Navigation buttons.

**MA** – Menu.

**EXIT** – Exit.

**#F** – Functional "F".

## OPERATION IN SPECTRUM MODES

In this mode the scan speed is around 300 ch/s.

The scan is interlaced: the step is minimal 25k and the spectrum is swept with different start frequencies to cover all the frequencies.

For example 125.100, 125.125,… then 125.10833,125.133333…then 125.11666,125.3666…

This to improve detection.

Different types of spectrum are called by the quick buttons

**F6 -** Range, Freq start, stop and step set in **\[5\]** menu.

Scans between 2 selected frequencies with the given step

**F7 -** Scanlists

Scans the frequencies in scanlists that can be defined in VFO or Chirp\*.

50 lists are available with their name.

**F8 -** Bands

Scans ranges that are defined in bands, can be defined in Chirp\* or in channels

**1** – Skip the frequency in the scan mode.

**2** – Toggles the backlight activation as follow:

- When active, it will use BLTime from the main menu to maintain light ON.

It will switch light ON when RX and pressed keys.

- When not active, it will stay always OFF until 2 is pressed again in spectrum display.

**3** – Toggle Fast or Slow scan speed, result visible on lower line with speed in ch/s

**4** – List of scanned lists or bands.\
In scan Range: enter START  and STOP frequencies

**5** – Options menu. Navigation through the menu **<** and **>**. Change the values with buttons **1** and **3**.

**6** – Changing the modes of the spectrum analyzer Scanlists / Bands / Range.

In range mode enter START and STOP frequencies

**7** - Save Settings. Save after configuring and selecting the lists and the options will be applied once enabled.

**8** – Change the display screen between spectrum RAW **OR** Smooth and ULTRA WATCH.

**Smooth Spectrum   Raw Spectrum**

![](<Sonic Manual_images/image1.png>)    ![](<Sonic Manual_images/image2.png>)

**9** – Change of AM/FM/USB modulation.

**0** – Switch to the history mode. Displays the RX frequencies, channels, code and time in s.

**\*** and **#**  DYNSQL **Level setting**  - RSSI spike threshold  for instant peak capture. For weak and fuzzy signals, increase or disable.

**L\*** and **L#** DYNSQL OFF This will fill the history, but won’t stop.

**SK1** – Frequency Lock (FL) then monitor and back to normal mode.

**FL** locks the frequency, opens listening on receive

**Monitor**, keeps the frequency locked and  open listening

**SK2** – Blacklist a frequency, it is visible in history with a **#**

## OPERATION IN ULTRA WATCH MODE

In this mode the scan speed is around 600 ch/s, in spectrum it is around 300ch/s

Keys are the same as for spectrum.

**UW in Scanlist mode**

| ![](<Sonic Manual_images/image3.png>) | Frequency is displayed in big<br>Transmit or received DCS and CTSS codes are displayed in small<br>RX displays last received channel.<br>RX> when receiving<br>in PTT Channel mode:<br>TX to show current selected channel, change with <> keys into the selected scanlists<br>In PTT LastRx mode:<br>TX shows last received channel, ready for TX.<br>TX+ or TX- when an offset is used<br>TX> when transmitting with PTT |
| --- | --- |

**UW Band mode**

| ![](<Sonic Manual_images/image4.png>) | Frequency is displayed in big<br>DCS and CTSS codes are displayed in small<br>Bx shows current band<br>Here we receive so RX> is displayed with an smeter below.<br>TX shows the last received frequency, ready to use for TX. In band mode, no channel setting is used. |
| --- | --- |

## OPERATION IN EXPERIMENTAL CLOSE CALL MODE

Key to launch the close call app: F+9.\
**MENU**: open the frequency bands selection\
**UP**: adjust the squelch threshold: raise it (ignore weaker signals). \
**DOWN**: adjust the squelch threshold: lower it (allow weaker signals).\
**1 -** Open the history list (newest first).\
**9 -** Clear the temporary block list.\
**SIDE2 -**  BLOCK: never re-capture the shown/locked frequency this session, then resume scanning.\
**F -** Cycle the screen backlight: full -> half -> off -> full.\
**\* -**  Jump straight back to live scanning and clear the browse view.

## Keys in history

This mode lists received signals, with a time counter. Up to 200 items are stored.

**<,>** - Move to previous / next item, automatically set to frequency lock to the selected item

**1** – Scans history entries.

**2** – Saves the frequency to a free channel (The first empty found).

**3** – Delete the entry.

**5** – History menu

Clear history : erase the history

Clear blacklist – Clears all black listed entries.

Clear non blacklist - Clears all non black listed entries.

Save history – Saves the history.

Save hist to ch : Saves all the history items to free channels. Tip: try to avoid holes.

**8** - Save history to flash.

**EXIT** : go back to UW or SPECTRUM

## Keys in band selection

**<,>** - Move to previous / next item

**1,3** – Select a preset

**4** – Activate or deactivate one band.

**5** – Activate one band and deactivate any other

**7** – Save current preset.

**MENU** -  activate selected item and start scanning

**EXIT** : go back to SCAN or SPECTRUM

## Keys in scanlist selection

**<,>** - Move to previous / next item

**4** – Activate or deactivate one scanlist.

**5** – Activate one scanlist and deactivate any other

**MENU** -  activate selected item and start scanning

**EXIT** : go back to SCAN or SPECTRUM

## LED Behavior

<mark>RED</mark> when TX

<mark>GREEN</mark> when RX

<mark>YELLOW</mark> blink at 5s frequency when scanning (UW or Spectrum)

<mark>YELLOW</mark> blink at 1s frequency when close call.

Remark: when USB is connected a <mark>BLUE</mark> led is added.

## PARAMETERS OF THE SPECTRUM ANALYZER

**PTT** – What to do when pressing PTT: LAST VFO /NINJA /LAST RECEIVED/AUTO ROGER .

- NINJA chooses a random frequency in the current scanlist.
- AUTO ROGER :Choose your Roger in vfo menu, select a frequency or a channel in vfo, in spectrum select ptt auto Roger in the menu, it will send a Roger beep every duration you select.

**SPECTRUMDELAY** – Pause time after signal detection (in ms) – if 0, immediately resumes scanning.

**MAXLISTENTIME** – Maximum time to listen to the signal (in seconds) – after that, the scan continues.

**STEP** – Frequency scan step (in kHz): 0.01; 0.05; 0.1; 0.5; 1.0; 2.0; 5.0; 6.25; 10.0; 12.5; 20.0; 25.0; 50.0; 100.0.

**LISTENBW** – Bandwidth: **Wide** (25 kHz) / **Narrow** (12.5 kHz)/ **AIR** (8.33kHz).

**MODULATION** – Modulation type: **FM/AM/USB**.

**POWERSAVE** – Power saving mode: Off /200ms /500ms /1s /2s /5s.

**KEY UNLOCKED** – THE TIME BEFORE THE KEYPAD IS LOCKED.

**NOIS LVL OFF** – Noise level for disabling reception (above this – the signal is counted).

**POPUPS** – Popup display time (in ms) – 0 = disabled.

**RECORD TRIG** – An additional threshold for recording in history. The higher the threshold, the stronger the signal will be recorded.

**SOUND BOOST** – Amplify sound. Increase bass frequencies.\
**AM AUDIO** – Select between STOCK, OPEN and SHARP

**MONITOR SL – Includes the** "MON" **priority list in all scan lists**.

**SPECTRUM** – SMOOTH or RAW

**RESET DEFAULT** – Reset all spectrum parameters to factory settings.

## DETAILED MONITORING IN SPECTRUM MODE VIA THE "MENU" BUTTON

Move through the low-level register menu with buttons **2** and **8**. Change values with the **<=** and **=> buttons**

**LNAS** – Low Noise Amplifier (LNA) – signal amplification stages in front of the mixer.

**LNA** – Gain of the first LNA stage – higher = stronger signal but more interference.

**PGA** – Programmable Gain Amplifier (after LNA). – Balance between sensitivity and distortion.

**MIX** – Mixer Gain (Frequency Mixing) – Higher = Better Dynamic Range but More Noise.

**XTAL F MODE SELECT** – Quartz filter (XTAL) operating mode. Filtering (reduces noise, but can distort).

**OFF AF RX DE-EMP** – Disable demphasis (pre-distortion correction) during reception.

**GAIN AFTER FM DEMOD** – Adjusts the reception volume.

**RF TX DEVIATION** – Transmission frequency deviation (Fm deviation) – affects the transmission bandwidth (higher = louder).

**COMPRESS AF TX RATIO** – Compression ratio of audio during transmission – compresses the dynamic range.

**COMPRESS AF TX 0 DB** – Compression level 0 db during transmission – the compression start point.

**COMPRESS AF TX NOISE** – Noise reduction in quiet areas.

**MIC AGC DISABLE** – Disable automatic microphone gain control (AGC).

**AFC RANGE SELECT** – Operating range of AFC (Automatic Frequency Tuning).

**AFC DISABLE –** Disable AFC.

**AFC SPEED** – AFC Speed.

## OPERATION IN BROADCAST RADIO MODE "FM"

The firmware has a simplified FM radio. Use **the <** and **> buttons**  to search for the nearest available broadcasting frequency in automatic mode.  The **\*** button will change the **Auto mode** to **manual** search.

Press key 1 to 9 to select the corresponding memory.

Long press key 1 to 9 to save a frequency to that memory.

Press 0 to toggle receiving the VFO frequency, RX MON displayed or removed.

Press Menu key to toggle name or memory displays.

You can name radios frequencies using chirp, see below.

## MAIN MENU

THERE IS NO HIDDEN MENU IN THE FIRMWARE

|   | **Step** | The frequency step (in kHz) |
| --- | --- | --- |
|   | **SQL** | The level of noise cancellation (Squelch) is from 0 (always open) to 9 (very strict). Set for both bands (A and B) |
|   | **Mode** | Demodulation mode, default - FM, AM and USB can only be used for listening |
|   | **RxDCS** | Setting Digital Coded Squelch, if enabled, the squelch will only open after receiving that subtone, applicable for FM modulation only. |
|   | **RxCTCS** | Setting an analog subtone to reception (Continuous Tone Coded Squelch), if the function is enabled, the noise canceler will open only after receiving this subtone, applicable only for FM modulation. |
|   | **TxDCS** | Setting the digital encoded digital subtone to transmit, if the function is enabled, the transceiver will transmit this tone, applicable only for FM modulation. |
|   | **TxCTCS** | Setting the analog subtone to transmit, if the function is enabled, the transceiver will transmit this tone, applicable for FM modulation only. |
|   | **TxOffs** | Transmitter frequency offset value in MHz, transmit/receive at different frequencies. |
|   | **TxODir** | Direction (+/-) of transmitter frequency offset when transmitting/receiving at different frequencies |
|   | **W/N** | The bandwidth used by the transceiver (WIDE (25 kHz), NARROW (12.5 kHz). |
|   | **SetNFM** | Sets Narrow FM to Narrow or Narrower |
|   | **Scramb** | Selects Scrambling type |
|   | **Compnd** | Compander - Soft audio filter for clearer sound. Amplifies the received desired signal even when the signal is weak at the transmitting microphone, improving the signal-to-noise ratio. To work, it must be turned on on both walkie-talkies. |
|   | **Roger** | On/off "roger beep" (end-of-pass tone). |
|   | **STE** | Noise suppressor, eliminates noise at the end of the transmission. When PTT is released, a short 55Hz tone is emitted, signaling to other transceivers to mute |
|   | **RP STE** | Noise Cancelling Tail Eliminator Repeater |
|   | **ChList** | Set Scanlist for current channel |
|   | **ChSave** | Saves the frequency to a chosen memory |
|   | **ChDele** | Deletes a channel memory |
|   | **ChName** | Name of the memory channel. |
|   | **F1Shrt** | Function assignment by short pressing the F1 side key. |
|   | **F1Long** | Function assignment when the F1 side key is pressed for a long time. |
|   | **F2Shrt** | Function assignment by short pressing the F2 side key. |
|   | **F2Long** | Assign a function when you press the F2 side key for a long time. |
|   | **M Long** | Assign a function when you long press the M key |
|   | **KeyLck** | Automatically lock the keypad after a set time |
|   | **TxOut** | Transmit Timeout - The amount of time that PTT can be held during transmission (in seconds). Protection against accidental clamping of PTT. |
|   | **BatSav** | Battery Save: OFF / 1:1 / 1:2 / 1:4 / 1:8. |
|   | **Mic** | Microphone sensitivity (microphone amplifier). |
|   | **POnMsg** | Enable/disable the display of a message when turned on (Power On Message). |
|   | **BLTime** | Screen backlight time when buttons are pressed. |
|   | **BLMin** | Minimum backlight brightness (in percentage). |
|   | **BLMax** | Maximum backlight brightness (in percentage). |
|   | **BLTxRx** | Transmit/Receive Backlight: OFF/TX/RX/TX+RX. |
|   | **SetInv** | Enable walkie-talkie screen color inversion |
|   | **BtnInv** | Reverse the direction of the UP and DOWN buttons |
|   | **SetLck** | Setting the keypad lock type with or without PTT. |
|   | **BatTxt** | Battery Display Type, %, V or disabled. |
|   | **SetTmr** | Set the display of RX/TX timers on the screen |
|   | **SetRXA** | Select AM or FM filtering |
|   | **F Lock** | Setting up Frequency Lock. What frequencies are allowed or prohibited to work on. |
|   | **BatCal** | Battery Calibration. |
|   | **BatTyp** | Choice of battery type (Li-ion, NiMH, etc.) and capacity. |
|   | **Reset** | Channel Preservation Reset (NOCH) or Full Factory Reset including Saved Channels (ALL) |

# Installation

RS232 version uses the kenwood cable.

USB version uses USB-C cable.

![](<Sonic Manual_images/image5.png>) ![Can the UV-K1 only be programmed through the USB-C port?](<Sonic Manual_images/image6.jpeg>)

RS232 Kenwood cable USB-C cable

use the same procedure as F4HWN, after that :

- If necessary, you can do a full reset of the radio. press **the PTT** and **Fn2 buttons**  , turn on the radio. A warning message will be displayed on the screen. hold the buttons for 4 seconds before resetting.

In case of issues to install, try to first install N7SIX firmware to reset the memory.

## CHIRP Driver

Chirp_Sonic.py in [DOWNLOAD_HERE](https://github.com/Robby69400/Sonic/tree/main/.DOWNLOAD_HERE)

This driver also allows

- customization of the 50 bands on channels 976 to 1024.

Select start frequency  and offset as stop frequency, step and modulation

## Chirp Configuration

- you can give names to the scanlists like so:

![](<Sonic Manual_images/image7.png>)

- Memory channels 975 to 1024 store custom band settings for <mark>Sonic 1K.</mark>
- Memory channels 4047 to 4096 store custom band settings for <mark>Sonic 4K.</mark>
- For each, define:
- Band Name.
- Start and End frequency (via offset).
- ![](<Sonic Manual_images/image8.png>)Modulation & Frequency Step.
- ![](<Sonic Manual_images/image9.png>)Scanlist Monitor setting Select up to 20ch to Monitor scanlist anywhere in the channels.
- FM Broadcast names\
  Enter channels frequencies in first section

Enter frequencies and Names in the second section.

![](<Sonic Manual_images/image10.png>)

## Flash mapping considerations:

Here is a very simplified explanation of the regions in Flash memory:

Sonic 1K uses his slot, or the one set with option SetCfg for his parameters.\
Sonic 4K uses a fixed region for his channels. If you install multiple Sonic 4K they will share the same region.

When first installing Sonic it is recommended to RESET ALL with PTT+SK2 at boot.

LOGO and CALIBRATION in Sonic are not available from UvStudio, please use F4HWN to manage them.

The RESET feature is not available in UvStudio.

The apps are not available in Sonic.
