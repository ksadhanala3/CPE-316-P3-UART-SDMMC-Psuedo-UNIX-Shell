#ifndef SD_SDIO_H
#define SD_SDIO_H

#include "stm32l476xx.h"
#include "ff.h"     // for BYTE, DWORD etc.

BYTE SD_InitializeLowLevel(void);
int SD_ReadBlock (BYTE *buff, DWORD sector);
int SD_WriteBlock(const BYTE *buff, DWORD sector);

#endif /* SD_SDIO_H */
