<img width="1024" height="572" alt="image" src="https://github.com/user-attachments/assets/c3440fd8-dcbc-4071-bfc6-8ad4ee3f5fc2" />

# [◀️  Join the SonicFw Telegram](https://t.me/SonicFw)
# [📺 Youtube Channel](https://www.youtube.com/@robby_69400)
# [💾 DOWNLOAD (select RAW file)](https://github.com/Robby69400/Sonic/tree/main/.DOWNLOAD_HERE)
## 🙏 Many thanks to Sonic Team : Zylka, Kolyan, Toni, Yves, Francois and Adam

This software is a fork of Armel F4HWN firmware with code from NTOIVOLA, EGZUMER and DUAL TACHYON.

# [Sonic Manual](Sonic_Manual.md)

# [Sonic POCSAG Manual](Sonic_Pocsag_Manual.md)

## Multiboot and dynamic profiles

Sonic supports the same five-slot multiboot layout and host protocol as F4HWN
Fusion. On the first boot after a normal firmware flash, Sonic saves the running
image as the protected **Main** backup (slot 0). Hold **MENU** while powering on
to open the selector; slots 1 through 4 can be managed by a compatible host tool.

Each slot owns a separate 64 KiB settings profile. Channels, names, VFO state,
Spectrum history and Sonic settings therefore follow the selected firmware,
while calibration and the boot logo remain shared. The active-profile state is
stored redundantly and tied to the expected firmware CRC so a normal firmware
flash is detected and adopted safely.

The USB and RS232 builds expose the Fusion-compatible commands `0x0720` through
`0x0729` for slot info/erase/write/validation and profile reset. Slot 0 cannot be
overwritten or erased by these host commands.

Build artifacts follow the shared metadata convention
`f4hwn.<name>.<variant>.v<version>.bin`: Sonic V52b is emitted as
`usb.V52b.bin` or `rs232.V52b.bin`. UVStudio derives the
slot name from `<name>` and stores `v<version>` in the slot header.

## POCSAG scan firmware

The normal Sonic spectrum scanner can be replaced at build time with POCSAG
scan mode:

```powershell
cmake --preset S1K -DENABLE_POCSAG=ON
cmake --build --preset S1K
```

Create a memory scanlist named `POCSAG` and add up to 20 channels to it. The
firmware scans those channels and runs the POCSAG decoder when it detects a
carrier. Leave `ENABLE_POCSAG` off for the normal Sonic spectrum scanner.
