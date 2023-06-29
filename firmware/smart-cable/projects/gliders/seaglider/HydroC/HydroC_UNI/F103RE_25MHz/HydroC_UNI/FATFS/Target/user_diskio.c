/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver skeleton to be completed by the user.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
 /* USER CODE END Header */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/*
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future.
 * Kept to ensure backward compatibility with previous CubeMx versions when
 * migrating projects.
 * User code previously added there should be copied in the new user sections before
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"
#include "sd.h"
#include "string.h"
#include "main.h"



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

extern sd_info_ptr* sdinfo;
extern UART_HandleTypeDef huart1;

/* Disk status */
DSTATUS* Stat;

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS SD0_initialize (BYTE pdrv);
DSTATUS SD1_initialize (BYTE pdrv);
DSTATUS SD2_initialize (BYTE pdrv);
DSTATUS SD3_initialize (BYTE pdrv);

DSTATUS USER_status (BYTE pdrv);
DSTATUS SD0_status (BYTE pdrv);
DSTATUS SD1_status (BYTE pdrv);
DSTATUS SD2_status (BYTE pdrv);
DSTATUS SD3_status (BYTE pdrv);

DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
DRESULT SD0_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
DRESULT SD1_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
DRESULT SD2_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
DRESULT SD3_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);

#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
  DRESULT SD0_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
  DRESULT SD1_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
  DRESULT SD2_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
  DRESULT SD3_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
  DRESULT SD0_ioctl (BYTE pdrv, BYTE cmd, void *buff);
  DRESULT SD1_ioctl (BYTE pdrv, BYTE cmd, void *buff);
  DRESULT SD2_ioctl (BYTE pdrv, BYTE cmd, void *buff);
  DRESULT SD3_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

  /*
Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif
};
*/

Diskio_drvTypeDef  SD0_Driver =
{
  SD0_initialize,
  SD0_status,
  SD0_read,
#if  _USE_WRITE
  SD0_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  SD0_ioctl,
#endif /* _USE_IOCTL == 1 */
};

Diskio_drvTypeDef  SD1_Driver =
{
  SD1_initialize,
  SD1_status,
  SD1_read,
#if  _USE_WRITE
  SD1_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  SD1_ioctl,
#endif /* _USE_IOCTL == 1 */
};

Diskio_drvTypeDef  SD2_Driver =
{
  SD2_initialize,
  SD2_status,
  SD2_read,
#if  _USE_WRITE
  SD2_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  SD2_ioctl,
#endif /* _USE_IOCTL == 1 */
};

Diskio_drvTypeDef  SD3_Driver =
{
  SD3_initialize,
  SD3_status,
  SD3_read,
#if  _USE_WRITE
  SD3_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  SD3_ioctl,
#endif /* _USE_IOCTL == 1 */
};



/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS SD0_initialize (BYTE pdrv)
{
	sd_ss_set_active(0);
	return USER_initialize(pdrv);
}
DSTATUS SD1_initialize (BYTE pdrv)
{
	sd_ss_set_active(1);
	return USER_initialize(pdrv);
}
DSTATUS SD2_initialize (BYTE pdrv)
{
	sd_ss_set_active(2);
	return USER_initialize(pdrv);
}
DSTATUS SD3_initialize (BYTE pdrv)
{
	sd_ss_set_active(3);
	return USER_initialize(pdrv);
}

DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
	//SD_PowerOn();
	SS_SD_SELECT();
	if(sd_ini()==0) {*Stat &= ~STA_NOINIT;} // STA_NOINIT
	SS_SD_DESELECT();
    return *Stat;
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS SD0_status (BYTE pdrv)
{
	sd_ss_set_active(0);
	return USER_status(pdrv);
}
DSTATUS SD1_status (BYTE pdrv)
{
	sd_ss_set_active(1);
	return USER_status(pdrv);
}
DSTATUS SD2_status (BYTE pdrv)
{
	sd_ss_set_active(2);
	return USER_status(pdrv);
}
DSTATUS SD3_status (BYTE pdrv)
{
	sd_ss_set_active(3);
	return USER_status(pdrv);
}

DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
	SS_SD_SELECT();
	//if (pdrv) return STA_NOINIT;
	SS_SD_DESELECT();
    return *Stat;
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DSTATUS SD0_read (BYTE pdrv,BYTE *buff,DWORD sector,UINT count)
{
	sd_ss_set_active(0);
	return USER_read(pdrv,buff,sector,count);
}
DSTATUS SD1_read (BYTE pdrv,BYTE *buff,DWORD sector,UINT count)
{
	sd_ss_set_active(1);
	return USER_read(pdrv,buff,sector,count);
}
DSTATUS SD2_read (BYTE pdrv,BYTE *buff,DWORD sector,UINT count)
{
	sd_ss_set_active(2);
	return USER_read(pdrv,buff,sector,count);
}
DSTATUS SD3_read (BYTE pdrv,BYTE *buff,DWORD sector,UINT count)
{
	sd_ss_set_active(3);
	return USER_read(pdrv,buff,sector,count);
}

DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */

	SS_SD_SELECT();
	if (pdrv || !count) return RES_PARERR;
	if (*Stat & STA_NOINIT) return RES_NOTRDY;
	if (!(sdinfo->type & 4)) sector *= 512; /* Convert to byte address if needed */
	if (count == 1) /* Single block read */
	{
	  SD_Read_Block(buff,sector); //Ð¡Ñ‡Ð¸Ñ‚Ð°ÐµÐ¼ Ð±Ð»Ð¾Ðº Ð² Ð±ÑƒÑ„ÐµÑ€
	  count = 0;
	}
	else /* Multiple block read */
	{
		do{
			SD_Read_Block((BYTE*)buff,sector);
			if (!(sdinfo->type & 4)){
				sector+=512;
			}
			else sector++;
			buff+=512;
			count--;
		}while(count>0);
	}
	SPI_Release();
	SS_SD_DESELECT();
	return count ? RES_ERROR : RES_OK;
    return RES_OK;
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1

DSTATUS SD0_write (BYTE pdrv,const BYTE *buff,DWORD sector,UINT count)
{
	sd_ss_set_active(0);
	return USER_write(pdrv,buff,sector,count);
}
DSTATUS SD1_write (BYTE pdrv,const BYTE *buff,DWORD sector,UINT count)
{
	sd_ss_set_active(1);
	return USER_write(pdrv,buff,sector,count);
}
DSTATUS SD2_write (BYTE pdrv,const BYTE *buff,DWORD sector,UINT count)
{
	sd_ss_set_active(2);
	return USER_write(pdrv,buff,sector,count);
}
DSTATUS SD3_write (BYTE pdrv,const BYTE *buff,DWORD sector,UINT count)
{
	sd_ss_set_active(3);
	return USER_write(pdrv,buff,sector,count);
}


DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
  /* USER CODE HERE */

	uint8_t res;
	SS_SD_SELECT();
	if (pdrv || !count) return RES_PARERR;
	if (*Stat & STA_NOINIT) return RES_NOTRDY;
	if (*Stat & STA_PROTECT) return RES_WRPRT;
	if (!(sdinfo->type & 4)) sector *= 512; /* Convert to byte address if needed */
  	res=SD_Write_Blocks((BYTE*)buff,sector,count);
	SS_SD_DESELECT();

	return res ? RES_ERROR : RES_OK;
  /* USER CODE END WRITE */
}

#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DSTATUS SD0_ioctl (BYTE pdrv,BYTE cmd,void *buff)
{
	sd_ss_set_active(0);
	return USER_ioctl(pdrv,cmd,buff);
}
DSTATUS SD1_ioctl (BYTE pdrv,BYTE cmd,void *buff)
{
	sd_ss_set_active(1);
	return USER_ioctl(pdrv,cmd,buff);
}
DSTATUS SD2_ioctl (BYTE pdrv,BYTE cmd,void *buff)
{
	sd_ss_set_active(2);
	return USER_ioctl(pdrv,cmd,buff);
}
DSTATUS SD3_ioctl (BYTE pdrv,BYTE cmd,void *buff)
{
	sd_ss_set_active(3);
	return USER_ioctl(pdrv,cmd,buff);
}

DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
	DRESULT res;

	SS_SD_SELECT();
	if (pdrv) return RES_PARERR;
	if (*Stat & STA_NOINIT) return RES_NOTRDY;
	res = RES_ERROR;
	switch (cmd)
	{
	  case CTRL_SYNC : /* Flush dirty buffer if present */
	    //SS_SD_SELECT();
	    if (SPI_wait_ready() == 0xFF)
	    res = RES_OK;
	    break;
	  case GET_SECTOR_SIZE : /* Get sectors on the disk (WORD) */
	    *(WORD*)buff = 512;
	    res = RES_OK;
	    break;
	  default:
	    res = RES_PARERR;
	}
	SPI_Release();
	SS_SD_DESELECT();
    return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
