/* Copyright 2026
 *
 * Licensed under the Apache License, Version 2.0.
 */

#ifndef APP_HISTORY_LOG_H
#define APP_HISTORY_LOG_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/keyboard.h"
#include "functions.h"
#include "radio.h"

#ifdef ENABLE_FEAT_F4HWN_HISTORY_LOG

#define HISTORY_LOG_VISIBLE_COUNT 512

// Field order mirrors the leading fields of HISTORY_LogFlashEntry_t
// (HISTORY_log.c) so both layouts match byte-for-byte up to and including
// battVolt, copied in one pass. Scan-only fields (sequence) sit past the
// copied prefix in the flash layout and are not cached in RAM.
// The channel name is not stored in flash: it is resolved from `channel` when
// exporting the K5Viewer packet.
typedef struct {
    uint32_t frequency;
    uint32_t trafficSeq;
    uint16_t durationSeconds;
    uint16_t channel;
    uint8_t  flags;
    uint8_t  sMeter;
    uint8_t  battVolt;
} HISTORY_LogEntry_t;

void HISTORY_LOG_Init(void);
void HISTORY_LOG_BeginRx(const VFO_Info_t *vfo, FUNCTION_Type_t function);
void HISTORY_LOG_BeginTx(const VFO_Info_t *vfo);
void HISTORY_LOG_EndActive(void);
void HISTORY_LOG_Task10ms(void);
void HISTORY_LOG_Tick500ms(void);
const char *HISTORY_LOG_GetFilterName(void);

void ACTION_RxTxLog(void);
void HISTORY_LOG_ProcessKeys(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld);
void UI_DisplayRxTxLog(void);

#endif

#endif
