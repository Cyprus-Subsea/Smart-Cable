/*
 * mcu_flash.h
 *
 *  Created on: 13 апр. 2021 г.
 *      Author: admin
 */

#ifndef INC_MCU_FLASH_H_
#define INC_MCU_FLASH_H_

#include "cmsis_os.h"

#define FLASH_PAGE_SIZE     2048
#define FLASH_BUFFER_SIZE   10240

#define MCU_FLASH_CLEAN_FLAG  0xFA1205AB
#define MCU_FLASH_DIRTY_FLAG  0xFFFFFFFF



#pragma pack (push, 1)
typedef struct
{
 uint32_t flag;
 uint32_t write_indx;
}flash_state_str;
#pragma pack (pop)



#pragma pack (push, 1)
typedef struct
{
   flash_state_str flash_state;
   uint32_t  sys_page_addr;
   uint32_t  data_pages_addr;
   uint32_t  num_of_pages;
   uint8_t   buffer[FLASH_BUFFER_SIZE];
} mcu_flash;
#pragma pack (pop)



void mcu_flash_write(mcu_flash* mcu_flash_obj,uint8_t* data, uint32_t datalen);
void mcu_flash_init(mcu_flash* mcu_flash_obj,uint32_t start_page);
void mcu_flash_open(mcu_flash* mcu_flash_obj);
void mcu_flash_close(mcu_flash* mcu_flash_obj,uint32_t flag);

#endif /* INC_MCU_FLASH_H_ */
