#ifndef POCSAG_H
#define POCSAG_H

#include <stdint.h>
#include <stdbool.h>

void APP_RunPocsagScan(void);
extern volatile bool g_pocsagScanStop;

#endif