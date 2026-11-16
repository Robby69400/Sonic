#!/usr/bin/env bash
# POCSAG Scan: Armel's POCSAG RX with the Falcon scan of the list named POCSAG.
# make.sh, not build.sh: tools/release.sh leaves it out of the Falcon apps.
set -euo pipefail
cd "$(dirname "$0")"
python3 gen_assets.py pocsagrx_assets.bin pocsagrx_assets.h
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -Oz -fno-jump-tables -std=gnu11 -ffreestanding -fno-builtin -fno-common -fomit-frame-pointer -ffunction-sections -fdata-sections -Wall -nostdlib -nostartfiles -T app.ld -Wl,--gc-sections -Wl,--build-id=none -Wl,--no-warn-rwx-segments pocsagrx_app.c -o pocsagrx.elf
arm-none-eabi-objcopy -O binary pocsagrx.elf pocsagrx.bin
arm-none-eabi-size pocsagrx.elf
python3 ../pack_app.py pocsagrx.bin POCSAGScan.app --name "POCSAG Scan" --ver "0.7F" --api-min 3 --vma 0x20000280 --assets pocsagrx_assets.bin
