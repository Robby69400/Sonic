# POCSAG TX (Falcon app, 0.4)

Sends POCSAG test pages to check the receivers (POCSAG RX, POCSAG Scan,
Sonic's scan) with a second radio. Direct FSK through the BK4819's FSK
modem, at the lowest power, **only between 433.050 and 434.790 MHz**.

| Key | Action |
|---|---|
| left / right | channel of the scan list named POCSAG (else the VFO's TX frequency; always the VFO on F4HWN) |
| 1 to 5 | message: alpha, numeric, two pages, long, tone only |
| 0 | auto repeat: OFF, 10 s, 30 s, 60 s |
| * | 1200 / 512 bauds (512: untested register value) |
| 9 | byte order in the modem's FIFO words (shown "SWAP"): for the first tests |
| PTT | send once |
| EXIT | quit |

Expected on the receiver: the pages of Armel's POCSAG RX test files
(`1234567` `F4HWN: POCSAG RX test on 433.650 MHz`, ...).

The stream built by the C code was checked on a computer: decoded by
`test/pocsag.py`, the five messages come out exactly, in both byte orders.
On the air (0.2, key 9): decoded by POCSAG RX 0.7 on a second UV-K1 at
433.650 MHz once the FIFO words carried the first byte in their low half;
0.3 makes that the only order.

    ./make.sh       # -> POCSAGTX.app (make.sh, not build.sh: kept out of the release apps)

Runs on Falcon 3.1+ and on F4HWN Labs 6.3.0 (API level 3: same table
offsets for every service used; checked against Armel's app_api.h). On
F4HWN the memory list is not readable: the app sends on the VFO frequency.
