# Sonic POCSAG (SPOC) Firmware manual

◀️  Join the SonicFw Telegram [https://t.me/SonicFw](https://t.me/SonicFw)

📺 Youtube Channel [https://www.youtube.com/@robby_69400](https://www.youtube.com/@robby_69400)

💾 DOWNLOAD select RAW file [DOWNLOAD_HERE](https://github.com/Robby69400/Sonic/tree/main/.DOWNLOAD_HERE)

This software is a fork of Armel F4HWN firmware with code from NTOIVOLA, EGZUMER and DUAL TACHYON.

# Installation

**USB version only use USB-C cable.**

use the same procedure as F4HWN, after that :

- **Do a full reset** of the radio. press **the PTT** and **Fn2 buttons**  , turn on the radio.

A warning message will be displayed on the screen.

hold the buttons for 4 seconds before resetting.

# Chirp Configuration

- Use Sonic 1K driver for POCSAG in  [DOWNLOAD_HERE](https://github.com/Robby69400/Sonic/tree/main/.DOWNLOAD_HERE).
- SPOC, a POCSAG scanlist 1 is required.
- In memory mode simply press 7 to toggle between POCSAG scanlist or OFF.

# POCSAG use

## Overview

Sonic POCSAG is a standalone POCSAG paging receiver. It decodes POCSAG, displays messages on screen, and keeps the last 200 pages in external flash memory.

The decoder supports 512 and 1200 baud

Configure a channel to AM to use 512 bauds

Configure a channel to FM to use 1200 bauds

Scanning is fast: "POCSAG" scanlist channels are read on the fly with immediate RSSI-based skipping, reaching about 150 channels per second on empty channels.

It is recommended to select the minimal channels to listen possible for best performance.

## Power-On and Main Screen

Pocsag Scan starts automatically, when exit to VFO, return to Pocsag scan with F+7. \
The receiver starts scanning the channels programmed in the POCSAG scanlist.

Scan screen:

\- Names or Frequencies of the channel currently being scanned, select with key \*

When a page is received, it is displayed with the frequency in the header. Scanning resumes automatically after the transmission ends (no new batches for the end-of-transmission delay). Pages remain displayed while scanning continues; a new page turns the screen back on if it was off.

## Message Filtering

To avoid cluttering the screen and memory:

Messages shorter than 10 characters are ignored (including tone-only, which have zero length).

Pure numeric messages (after applying the \`preferAlpha\` heuristic, which catches alpha messages erroneously sent on func=0) are **rejected**.\
2 similar messages received consecutively are filtered to enhance caracters and keep only one.

## Message Size and Storage

Maximum message size: 254 characters.

External flash storage (24,576 bytes), around 200 messages with a 100 characters mean length.

## Display

Two fonts are available:

Large (default) 18 columns × 6 lines

Small 32 columns × 8 lines

The \`\*\` key toggles between the two fonts.

## Controls

EXIT Exits the current mode

UP / DOWN Navigates between pages; holding the key scrolls with repeat; wraps at the end

\`\*\` Toggles large / small font

\`2\` Clears all the messages

The RSSI skip threshold is automatic (calibrated floor + margin 6); no manual intervention is needed.
