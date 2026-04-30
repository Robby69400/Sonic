#include "driver/iwdg.h"
#include "py32f071x8.h"   /* IWDG, RCC – supplied by build-system include path */

/* IWDG key register magic values */
#define IWDG_KEY_RELOAD   0xAAAAU
#define IWDG_KEY_ENABLE   0xCCCCU
#define IWDG_KEY_WRITE    0x5555U

void IWDG_Init(void)
{
    /* Enable write access to PR and RLR */
    IWDG->KR = IWDG_KEY_WRITE;

    /*
     * Prescaler /64  →  PR[2:0] = 4
     * LSI ~40 kHz → tick = 40000/64 = 625 Hz
     */
    IWDG->PR = 4U;

    /*
     * Reload = 3000
     * Timeout = 3000 / 625 Hz = 4.8 s
     * Comfortable margin above the longest normal operation
     * (spectrum full sweep at widest step ≈ 1-2 s).
     */
    IWDG->RLR = 3000U;

    /* Kick once to latch the new settings */
    IWDG->KR = IWDG_KEY_RELOAD;

    /* Start the watchdog */
    IWDG->KR = IWDG_KEY_ENABLE;
}

void IWDG_Feed(void)
{
    IWDG->KR = IWDG_KEY_RELOAD;
}
