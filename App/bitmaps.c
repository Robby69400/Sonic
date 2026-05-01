
#include "bitmaps.h"

// all these images are on their right sides
// turn your monitor 90-deg anti-clockwise to see the images

const uint8_t gFontPowerSave[2][6] =
{
    {0x00, 0x7f, 0x9, 0x9, 0x9, 0x6},
    {0x00, 0x26, 0x49, 0x49, 0x49, 0x32},
};

const uint8_t gFontPttOnePush[2][6] =
{
    {0x00, 0x3e, 0x41, 0x41, 0x41, 0x3e},
    {0x00, 0x7f, 0x9, 0x9, 0x9, 0x6},
};

const uint8_t gFontPttClassic[2][6] =
{
    {0x00, 0x3e, 0x41, 0x41, 0x41, 0x22},
    {0x00, 0x7f, 0x40, 0x40, 0x40, 0x40},
};

const uint8_t gFontF[8] =
{
    0b01111111,
    0b00000000,
    0b01110110,
    0b01110110,
    0b01110110,
    0b01110110,
    0b01111110,
    0b01111111
};

const uint8_t gFontS[6] =
{
    0x26, 0x49, 0x49, 0x49, 0x49, 0x32 // 'S'
};

const uint8_t gFontKeyLock[9] =
{
    0x7c, 0x46, 0x45, 0x45, 0x45, 0x45, 0x45, 0x46, 0x7c
};

const uint8_t gFontLight[9] =
{
    0b00001100,
    0b00010010,
    0b00100001,
    0b01101101,
    0b01111001,
    0b01101101,
    0b00100001,
    0b00010010,
    0b00001100,
};

const uint8_t gFontMute[12] =
{
    0b00011100,
    0b00011100,
    0b00010100,
    0b00100010,
    0b01000001,
    0b01111111,
    0b00000000,
    0b00100010,
    0b00010100,
    0b00001000,
    0b00010100,
    0b00100010,
};

const uint8_t gFontXB[2][6] =
{   // "XB"
    {0x00, 0x63, 0x14, 0x8, 0x14, 0x63},
    {0x00, 0x7f, 0x49, 0x49, 0x49, 0x36},
};

const uint8_t gFontMO[2][6] =
{   // "MO"
    {0x00, 0x7f, 0x2, 0x1c, 0x2, 0x7f},
    {0x00, 0x3e, 0x41, 0x41, 0x41, 0x3e},
};

const uint8_t gFontDWR[3][6] =
{   // "DWR"

    {0x00, 0x7f, 0x41, 0x41, 0x41, 0x3e},
    {0x00, 0x3f, 0x40, 0x38, 0x40, 0x3f},
    {0x00, 0x7f, 0x9, 0x19, 0x29, 0x46},
};

const uint8_t gFontHold[2][5] =
{   // "><" .. DW on hold
    {0x00, 0x41, 0x22, 0x14, 0x8},
    {0x00, 0x8, 0x14, 0x22, 0x41},
};

const uint8_t BITMAP_BatteryLevel[2] =
{
    0b01011101,
    0b01011101
};

// Quansheng way (+ pole to the left)
const uint8_t BITMAP_BatteryLevel1[17] =
{
    0b00000000,
    0b00111110,
    0b00100010,
    0b01000001,
    0b01000001,
    0b01000001,
    0b01000001,
    0b01000001,
    0b01000001,
    0b01000001,
    0b01000001,
    0b01000001,
    0b01000001,
    0b01000001,
    0b01000001,
    0b01000001,
    0b01111111
};

const uint8_t BITMAP_VFO_Lock[7] =
{
    0b01111100,
    0b01000110,
    0b01000101,
    0b01000101,
    0b01000101,
    0b01000110,
    0b01111100,
};

// Compact filled arrow (F4HWN style)
const uint8_t BITMAP_VFO_Default[7] =
{
    0b00111110,
    0b00111110,
    0b00011100,
    0b00011100,
    0b00001000,
    0b00001000,
    0b00000000
};

// Compact hollow arrow (F4HWN style)
const uint8_t BITMAP_VFO_NotDefault[7] =
{
    0b00100010,
    0b00100010,
    0b00010100,
    0b00010100,
    0b00001000,
    0b00001000,
    0b00000000
};

// Empty (for blinking)
const uint8_t BITMAP_VFO_Empty[7] =
{
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000
};

const uint8_t BITMAP_compand[6] =
{
    0b00000000,
    0b00111100,
    0b01000010,
    0b01000010,
    0b01000010,
    0b00100100
};

const uint8_t BITMAP_PowerUser[3] =
{   // 'arrow' symbol
    0b00111110,
    0b00011100,
    0b00001000,
};

const uint8_t BITMAP_CurrentIndicator[8] = {
    0xFF,
    0xFF,
    0x7E,
    0x7E,
    0x3C,
    0x3C,
    0x18,
    0x18
};
