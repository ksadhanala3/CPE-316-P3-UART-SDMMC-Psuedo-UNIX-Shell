#include <string.h>
#include "ff_gen_drv.h"
#include "diskio.h"
#include "sd_sdio.h"

extern void UART_print(char *str);
extern void UART_printf(const char *fmt, ...);

/* disk status */
static volatile DSTATUS Stat = STA_NOINIT;

/* Prototypes */
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status     (BYTE pdrv);
DRESULT USER_read       (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
DRESULT USER_write      (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif
#if _USE_IOCTL == 1
DRESULT USER_ioctl      (BYTE pdrv, BYTE cmd, void *buff);
#endif

Diskio_drvTypeDef USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE == 1
  USER_write,
#endif
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif
};

DSTATUS USER_initialize (BYTE pdrv)
{
  if (pdrv != 0) {
    return STA_NOINIT;  /* only drive 0 supported */
  }

  BYTE res = SD_InitializeLowLevel();

  if (res == 0) {
    Stat &= ~STA_NOINIT;
  } else {
    Stat  = STA_NOINIT;
  }

  return Stat;
}

DSTATUS USER_status (BYTE pdrv)
{
  if (pdrv != 0) {
    return STA_NOINIT;
  }
  return Stat;
}

DRESULT USER_read (
  BYTE pdrv,
  BYTE *buff,
  DWORD sector,
  UINT count
)
{
  if (pdrv != 0 || !count) {
    return RES_PARERR;
  }
  if (Stat & STA_NOINIT) {
    return RES_NOTRDY;
  }

  for (UINT i = 0; i < count; i++) {
    DWORD s = sector + i;
    if (!SD_ReadBlock(buff + (512U * i), s)) {
      // UART_printf("USER_read: SD_ReadBlock failed at sector %lu\r\n",
      //             (unsigned long)s);
      return RES_ERROR;
    }
  }

  return RES_OK;
}

#if _USE_WRITE == 1
DRESULT USER_write (
  BYTE pdrv,
  const BYTE *buff,
  DWORD sector,
  UINT count
)
{
  if (pdrv != 0 || !count) {
    return RES_PARERR;
  }
  if (Stat & STA_NOINIT) {
    return RES_NOTRDY;
  }

  for (UINT i = 0; i < count; i++) {
    DWORD s = sector + i;
    if (!SD_WriteBlock(buff + (512U * i), s)) {
      // UART_printf("USER_write: SD_WriteBlock failed at sector %lu\r\n",
      //             (unsigned long)s);
      return RES_ERROR;
    }
  }

  return RES_OK;
}
#endif /* _USE_WRITE == 1 */

#if _USE_IOCTL == 1
DRESULT USER_ioctl (
  BYTE pdrv,
  BYTE cmd,
  void *buff
)
{
  if (pdrv != 0) {
    return RES_PARERR;
  }

  switch (cmd) {
  case CTRL_SYNC:
    return RES_OK;

  case GET_SECTOR_SIZE:
    *(WORD*)buff = 512;
    return RES_OK;

  case GET_BLOCK_SIZE:
    *(DWORD*)buff = 1;  /* erase block size in sectors (dummy) */
    return RES_OK;

  case GET_SECTOR_COUNT:
    /* optional: implement via CSD; not needed for basic logging */
    return RES_ERROR;

  default:
    return RES_PARERR;
  }
}
#endif /* _USE_IOCTL == 1 */
