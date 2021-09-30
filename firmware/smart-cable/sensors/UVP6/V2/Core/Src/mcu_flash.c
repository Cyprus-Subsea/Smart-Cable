/*
 * mcu_flash.c
 *
 *  Created on: 13 апр. 2021 г.
 *      Author: admin
 */

#include "mcu_flash.h"
#include  "mbcrc.h"
#include  "string.h"


//debug
#include "main.h"
extern UART_HandleTypeDef huart1;
//HAL_UART_Transmit(&huart1,msg,strlen(msg),100)


void mcu_flash_write(mcu_flash* mcu_flash_obj,uint8_t* data, uint32_t datalen)
{
	if((mcu_flash_obj->flash_state.write_indx+datalen)<FLASH_BUFFER_SIZE)
	{
		memcpy(mcu_flash_obj->buffer+mcu_flash_obj->flash_state.write_indx,data,datalen);
		mcu_flash_obj->flash_state.write_indx=mcu_flash_obj->flash_state.write_indx+datalen;
	}
}



void mcu_flash_init(mcu_flash* mcu_flash_obj,uint32_t start_page)
{
	mcu_flash_obj->sys_page_addr=FLASH_BASE+(start_page*FLASH_PAGE_SIZE);
	mcu_flash_obj->data_pages_addr=mcu_flash_obj->sys_page_addr+FLASH_PAGE_SIZE;
	mcu_flash_obj->num_of_pages=(FLASH_BUFFER_SIZE/FLASH_PAGE_SIZE)+1;
	mcu_flash_obj->flash_state.flag=MCU_FLASH_CLEAN_FLAG;
	mcu_flash_obj->flash_state.write_indx=0;
}

void mcu_flash_open(mcu_flash* mcu_flash_obj)
{

	memcpy((uint8_t*)&(mcu_flash_obj->flash_state),(uint8_t*)mcu_flash_obj->sys_page_addr,sizeof(flash_state_str));
	HAL_UART_Transmit(&huart1,&(mcu_flash_obj->flash_state.flag),4,100);
	HAL_UART_Transmit(&huart1,&(mcu_flash_obj->flash_state.write_indx),4,100);
	if(mcu_flash_obj->flash_state.flag!=MCU_FLASH_CLEAN_FLAG)
	{
	  mcu_flash_obj->flash_state.flag=MCU_FLASH_CLEAN_FLAG;
	  mcu_flash_obj->flash_state.write_indx=0;
	}
	else
	{
	 memcpy(mcu_flash_obj->buffer,(uint8_t*)mcu_flash_obj->data_pages_addr,mcu_flash_obj->flash_state.write_indx);
	}


}
void mcu_flash_close(mcu_flash* mcu_flash_obj,uint32_t flag)
{
	mcu_flash_obj->flash_state.flag=flag;
	uint32_t start_addr=mcu_flash_obj->sys_page_addr;
    HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef erase_info = {
		.TypeErase = FLASH_TYPEERASE_PAGES,
		.PageAddress = start_addr ,
		.NbPages = mcu_flash_obj->num_of_pages,
	};

	uint32_t pgerr = 0;
	HAL_FLASHEx_Erase(&erase_info, &pgerr);

	if(pgerr != 0xFFFFFFFFul)
	{
		HAL_FLASH_Lock();
		return ;
	}

	uint32_t i=0;
	uint16_t tmp;
	uint8_t* data=(uint8_t*)&(mcu_flash_obj->flash_state);
    uint32_t datalen=sizeof(flash_state_str);
	while(i<datalen)
	{
        tmp=(tmp&0x0000)|(data[i]&0x00FF);
		if((i+1)<datalen) tmp=tmp|(data[i+1]<<8&0xFF00);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,start_addr+i,tmp);
		i=i+2;
	}

	i=0;
	start_addr=mcu_flash_obj->data_pages_addr;
	data=mcu_flash_obj->buffer;
	datalen=mcu_flash_obj->flash_state.write_indx;
	while(i<datalen)
	{
        tmp=(tmp&0x0000)|(data[i]&0x00FF);
		if((i+1)<datalen) tmp=tmp|(data[i+1]<<8&0xFF00);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,start_addr+i,tmp);
		i=i+2;
	}


	HAL_FLASH_Lock();
}

