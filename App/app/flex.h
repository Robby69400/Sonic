#ifndef APP_FLEX_H
#define APP_FLEX_H

#include <stdbool.h>

extern volatile bool g_flexScanStop;

void APP_RunFlexScan(void);

#endif