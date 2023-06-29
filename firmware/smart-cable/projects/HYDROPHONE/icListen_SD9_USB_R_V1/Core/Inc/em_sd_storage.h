/*
 * em_sd_storage.h
 *
 *  Created on: 19 мая 2022 г.
 *      Author: admin
 */

#ifndef INC_EM_SD_STORAGE_H_
#define INC_EM_SD_STORAGE_H_

#include "fatfs.h"
#include "sd.h"
#include "main.h"
#include "string.h"
#include "system_definitions.h"

#define       SD_STORAGE_NUM_DISKS 4

typedef enum{
	DISK_PRESENT=0,
	DISK_ABSENT
}DISK_STATUS;

typedef struct{
  FATFS       fs;
  DISK_STATUS status;
  uint32_t    size;        //KB
  uint32_t    free_space;  //KB

}sd_disk_t;

typedef struct{
	sd_disk_t disks[SD_STORAGE_NUM_DISKS];
}sd_storage_t;


F_RES sd_storage_init(sd_storage_t* self_object);
F_RES sd_storage_write(sd_storage_t* self_object);
F_RES sd_storage_read(sd_storage_t* self_object);
F_RES sd_storage_link_ss(sd_storage_t* self_object,uint8_t disk_num, uint16_t sd_ss_pin,GPIO_TypeDef* sd_ss_port);


#endif /* INC_EM_SD_STORAGE_H_ */
