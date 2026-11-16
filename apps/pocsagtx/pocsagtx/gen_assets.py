#!/usr/bin/env python3
# POCSAG TX read-only assets: the screen texts, the numeric character set and
# the test messages (the pages of Armel's POCSAG RX test files, so the result
# expected on the receiving radio is known).
#
# A message is a fixed-stride record: page count, then per page the RIC (u32
# little endian), the function (0 = numeric) and the text length, then the text.
#
#   ./gen_assets.py pocsagtx_assets.bin pocsagtx_assets.h
import os, struct, sys
sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from app_assets import Assets

TITLE = "POCSAG TX"
NUMERIC = "0123456789*U -)("        # 4-bit numeric code, value = index
MESSAGES = [                        # name, pages (ric, function, text)
    ("ALPHA",     [(1234567, 3, "F4HWN: POCSAG RX test on 433.650 MHz")]),
    ("NUMERIC",   [(1234560, 0, "0145 67-89")]),
    ("TWO PAGES", [(2504, 3, "DAPNET time 2026-10-13 21:07"),
                   (165856, 3, "Second page, same transmission.")]),
    ("LONG",      [(1000001, 3, "The quick brown fox jumps over the lazy dog 0123456789 times, then again.")]),
    ("TONE ONLY", [(1234567, 1, "")]),
]
MSG_STRIDE = 96

UI = [
    ("T_TITLE", TITLE),
    ("T_AUTO",  "AUTO "),
    ("T_OFF",   "OFF"),
    ("T_SENT",  "  sent "),
    ("T_NEXT",  "  next "),
    ("T_ONLY",  "433 MHz ONLY"),
    ("T_VFO",   "VFO"),
    ("T_BD",    "bd"),
]

a = Assets("POCSAGTX")
ui_size = 0
for name, s in UI:
    pad = -(len(s) + 1) % 4                 # keep every offset word-aligned
    a.text(name, s + "\0" * pad)
    ui_size += len(s) + 1 + pad
a.const("UI_SIZE", ui_size)
a.const("T_TITLE_CHARS", len(TITLE))
a.table("T_MSG", [m[0] for m in MESSAGES])
a.raw("NUMSET", NUMERIC.encode() + b"\0" * 0)
recs = b""
for name, pages in MESSAGES:
    r = bytes([len(pages)])
    for ric, func, text in pages:
        assert all(c in NUMERIC for c in text) if func == 0 else all(32 <= ord(c) < 127 for c in text)
        r += struct.pack("<IBB", ric, func, len(text)) + text.encode()
    assert len(r) <= MSG_STRIDE, name
    recs += r + b"\0" * (MSG_STRIDE - len(r))
a.raw("MSGS", recs)
a.const("MSG_STRIDE", MSG_STRIDE)
a.const("MSG_COUNT", len(MESSAGES))
if __name__ == "__main__":
    a.main()
