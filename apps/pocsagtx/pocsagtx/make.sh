#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
python3 gen_assets.py pocsagtx_assets.bin pocsagtx_assets.h
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -Oz -std=gnu11 -ffreestanding -fno-builtin -fno-common -fomit-frame-pointer -fno-jump-tables -ffunction-sections -fdata-sections -Wall -Wextra -nostdlib -nostartfiles -T app.ld -Wl,--gc-sections -Wl,-Map=pocsagtx.map -Wl,--build-id=none -Wl,--no-warn-rwx-segments pocsagtx_app.c -o pocsagtx.elf
arm-none-eabi-objcopy -O binary pocsagtx.elf pocsagtx.bin
arm-none-eabi-size pocsagtx.elf
python3 ../pack_app.py pocsagtx.bin POCSAGTX.app --name "POCSAG TX" --ver "0.4" --api-min 3 --vma 0x20000280 --assets pocsagtx_assets.bin
