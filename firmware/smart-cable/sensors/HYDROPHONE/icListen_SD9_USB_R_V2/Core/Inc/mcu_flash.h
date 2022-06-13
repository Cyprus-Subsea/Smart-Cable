/*
 * mcu_flash.h
 *
 *  Created on: 13 апр. 2021 г.
 *      Author: admin
 */

#ifndef INC_MCU_FLASH_H_
#define INC_MCU_FLASH_H_

#include  "cmsis_os.h"
#include  "main.h"
#include  "string.h"
#include  "crc.h"
#include  "system_definitions.h"

#define FLASH_DATA_SIZE                   16
#define FLASH_DATA_CRC_SIZE                2

#if defined(STM32F405xx)
 #define FLASH_NUM_OF_BLOCKS               12
#endif
#if defined(STM32F107xx)
 #define FLASH_NUM_OF_BLOCKS              256
#endif



#pragma pack (push, 1)
typedef struct
{
   uint8_t   raw_data[FLASH_DATA_SIZE];
   uint16_t  crc;
} mcu_flash_data_typedef;
#pragma pack (pop)


#pragma pack (push, 1)
typedef struct
{
   uint32_t         first_block_addr;
   uint32_t         first_block_num;
   uint32_t            num_of_blocks;
   mcu_flash_data_typedef      data;
} mcu_flash_typedef;
#pragma pack (pop)



void mcu_flash_init(mcu_flash_typedef* mcu_flash_obj,uint32_t first_block);
F_RES mcu_flash_read(mcu_flash_typedef* mcu_flash_obj);
void mcu_flash_save(mcu_flash_typedef* mcu_flash_obj);


#endif /* INC_MCU_FLASH_H_ */
