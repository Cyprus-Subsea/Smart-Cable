/*
 * em_sd_storage.c
 *
 *  Created on: 19 мая 2022 г.
 *      Author: admin
 */

#include "em_sd_storage.h"

extern ss_pp sd_cards_ss[4];
extern UART_HandleTypeDef huart1;

F_RES sd_storage_init(sd_storage_t* self_object)
{
  char tt[10];
  char t5[10];
  FRESULT res;
  DWORD fre_clust, fre_sect, tot_sect;
  FATFS*  fs;

  MX_FATFS_Init();

  for(int i=0;i<SD_STORAGE_NUM_DISKS;i++)
  {
	  sprintf(tt,"%d:",i);
	  if(f_mount(&self_object->disks[i].fs,tt,1)==FR_OK){
		  self_object->disks[i].status=DISK_PRESENT;
		  sprintf(t5,"ID:%d\n",i);
		  HAL_UART_Transmit(&huart1,t5,strlen(t5),100);
  		  if(f_getfree(tt, &fre_clust, &fs)==FR_OK){
		    tot_sect = (self_object->disks[i].fs.n_fatent - 2) * self_object->disks[i].fs.csize;
			fre_sect = fre_clust * self_object->disks[i].fs.csize;
			self_object->disks[i].size=tot_sect/2;
			self_object->disks[i].free_space=fre_sect/2;
		  }
	  }
	  else
	  {
		 self_object->disks[i].status=DISK_ABSENT;
	  }
  }
  return F_OK;
}

F_RES sd_storage_link_ss(sd_storage_t* self_object,uint8_t disk_num, uint16_t sd_ss_pin,GPIO_TypeDef* sd_ss_port)
{
	  sd_cards_ss[disk_num].sd_ss_pin=sd_ss_pin;
	  sd_cards_ss[disk_num].sd_ss_port=sd_ss_port;
	  return F_OK;
}

F_RES sd_storage_write(sd_storage_t* self_object)
{

}
F_RES sd_storage_read(sd_storage_t* self_object)
{

}
