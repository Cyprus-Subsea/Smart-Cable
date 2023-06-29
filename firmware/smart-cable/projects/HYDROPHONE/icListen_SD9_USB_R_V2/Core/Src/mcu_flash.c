/*
 * mcu_flash.c
 *
 *  Created on: 13 апр. 2021 г.
 *      Author: admin
 */

#include "mcu_flash.h"

#if defined(STM32F405xx)  //KB
uint32_t  flash_block_size[]={16,16,16,16,64,128,128,128,128,128,128,128};
#endif
#if defined(STM32F205xx)  //KB
uint32_t  flash_block_size[]={16,16,16,16,64,128,128,128,128,128,128,128};
#endif

void mcu_flash_init(mcu_flash_typedef* mcu_flash_obj,uint32_t first_block)
{
	uint32_t temp=0;
	int32_t temp2=0;
	if(first_block<FLASH_NUM_OF_BLOCKS){
      mcu_flash_obj->first_block_num=first_block;
      for(int i=0;i<first_block;i++) temp+=flash_block_size[i];
	  mcu_flash_obj->first_block_addr=FLASH_BASE+(temp*1024);

	  temp2=FLASH_DATA_SIZE+FLASH_DATA_CRC_SIZE;
	  mcu_flash_obj->num_of_blocks=0;
	  for(int i=mcu_flash_obj->first_block_num;i<FLASH_NUM_OF_BLOCKS&&temp2>0;i++) {
		  temp2-=flash_block_size[i];
		  mcu_flash_obj->num_of_blocks++;
	  }
	}
}

F_RES mcu_flash_read(mcu_flash_typedef* mcu_flash_obj)
{
	 memcpy((uint8_t*)&(mcu_flash_obj->data),(uint8_t*)mcu_flash_obj->first_block_addr,FLASH_DATA_SIZE+FLASH_DATA_CRC_SIZE);
	 if(mcu_flash_obj->data.crc==get_crc16_arc(mcu_flash_obj->data.raw_data,FLASH_DATA_SIZE)){
		 return F_OK;
	 }
	 else{
		 return F_ERR;
	 }

}
void mcu_flash_save(mcu_flash_typedef* mcu_flash_obj)
{
	uint32_t i=0;
	uint16_t tmp;
	uint32_t pgerr = 0;
	uint8_t* data=(uint8_t*)&mcu_flash_obj->data;

	mcu_flash_obj->data.crc=get_crc16_arc(mcu_flash_obj->data.raw_data,FLASH_DATA_SIZE);

    HAL_FLASH_Unlock();
    #if defined(STM32F405xx)
	FLASH_EraseInitTypeDef erase_info = {
		.TypeErase = FLASH_TYPEERASE_SECTORS,
		.Sector = mcu_flash_obj->first_block_num,
		.NbSectors = mcu_flash_obj->num_of_blocks,
	};
    #endif

    #if defined(STM32F107xx)
    FLASH_EraseInitTypeDef erase_info = {
	.TypeErase = FLASH_TYPEERASE_PAGES,
	.PageAddress = mcu_flash_obj->first_block_addr,
	.NbPages = mcu_flash_obj->num_of_blocks,
    };
    #endif


	HAL_FLASHEx_Erase(&erase_info, &pgerr);

	if(pgerr != 0xFFFFFFFFul)
	{
		HAL_FLASH_Lock();
		return ;
	}

	while(i<(FLASH_DATA_SIZE+FLASH_DATA_CRC_SIZE))
	{
        tmp=(tmp&0x0000)|(data[i]&0x00FF);
		if((i+1)<(FLASH_DATA_SIZE+FLASH_DATA_CRC_SIZE)) tmp=tmp|(data[i+1]<<8&0xFF00);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,mcu_flash_obj->first_block_addr+i,tmp);
		i=i+2;
	}

	HAL_FLASH_Lock();
}



