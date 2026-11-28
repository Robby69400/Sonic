# Sonic POCSAG (SPOC) Firmware manual

# Installation

**USB version only use USB-C cable.**
use the same procedure as F4HWN, after that :
- **Do a full reset** of the radio. press **the PTT** and **FN2 buttons**  , turn on the radio.
A warning message will be displayed on the screen.
hold the buttons for 4 seconds to reset.

# Chirp Configuration

- Use **Sonic 1K** driver for POCSAG in  [DOWNLOAD_HERE](https://github.com/Robby69400/Sonic/tree/main/.DOWNLOAD_HERE).
- SPOC, a POCSAG scanlist 1 is required.
- In memory mode simply press 7 to toggle between POCSAG scanlist and OFF.

# POCSAG use

Sonic POCSAG is a standalone POCSAG paging receiver. 
It decodes POCSAG, displays messages on screen, and keeps the last 200 pages in external flash memory.
The decoder supports 512 and 1200 baud.
Configure a channel to AM to use 512 bauds, it won't affect pocsag decode.
Configure a channel to FM to use 1200 bauds
It will scan up to 30 channels.

## Power-On and Main Screen

Pocsag Scan starts automatically, when exit to VFO, return to Pocsag scan with **F+7**.
The receiver starts scanning the channels programmed in the POCSAG scanlist.

When a page is received, it is displayed with the frequency in the header. Scanning resumes automatically after the transmission ends. Pages remain displayed while scanning continues; a new page turns the screen backlight on if it was off.

## Message Filtering
With :four: activate / deactivate filtering :
When Filtered: 
- Messages shorter than 10 characters are ignored (including tone-only, which have zero length).
- Pure numeric messages (after applying the \`preferAlpha\` heuristic, which catches alpha messages erroneously sent on func=0) are **rejected**.\
- 2 similar messages received consecutively are filtered to enhance caracters and keep only one.

## Message Size and Storage

Maximum message size: 254 characters.
External flash storage (24,576 bytes), around 200 messages with a 100 characters mean length.

## Display
**Scan:**
key * toggles name or frequencies display.

**Messages:** 
:star: toggles small or large font.
:two: Clears all the messages

## Leds
Slow blinking in SCAN
Fast blinking when a message is received.

## Controls

EXIT Exits the current mode

:arrow_backward: :arrow_forward: Navigates between pages; holding the key scrolls with repeat; wraps at the end


The RSSI skip threshold is automatic (calibrated floor + margin 6); no manual intervention is needed.
