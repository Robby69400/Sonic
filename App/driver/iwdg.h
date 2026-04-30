#ifndef DRIVER_IWDG_H
#define DRIVER_IWDG_H

/*
 * IWDG (Independent Watchdog) driver for PY32F071
 *
 * Clocked by internal LSI (~40 kHz).
 * Timeout = (4 * 2^prescaler * reload) / LSI_Hz
 *
 * Settings used:
 *   Prescaler = /64  (PR = 4)
 *   Reload    = 3000
 *   Timeout   = (4 * 64 * 3000) / 40000 = ~7.68 s
 *
 * IWDG_Feed() must be called at least once per ~7 s.
 * It is called every iteration of the main while(true) loop,
 * so any true hang (scan/spectrum deadlock, boot-screen spin)
 * will cause an automatic hardware reset.
 */

#include <stdint.h>

void IWDG_Init(void);
void IWDG_Feed(void);

#endif /* DRIVER_IWDG_H */
