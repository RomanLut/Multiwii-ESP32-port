#pragma once

#include "HX_ESPNOW_RC_Slave.h"

extern void HXRCInit();
extern void HXRCLoop();
extern int HXRCReadRawRC(int ch);

extern uint16_t HXRCSerialAvailable();
extern uint8_t HXRCSerialRead();

extern uint16_t HXRCSerialAvailableForWrite();
extern void HXRCSerialWrite(uint8_t c);

extern bool HXRCSerialTxFree();

extern int HXRCRSSI();