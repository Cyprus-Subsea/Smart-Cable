/*
 * em_sd_storage.c
 *
 *  Created on: 19 мая 2022 г.
 *      Author: admin
 */

#include "em_sd_storage.h"
#include "stdio.h"
#include "diskio.h"

extern ss_pp sd_cards_ss[SD_STORAGE_NUM_SLOTS];

F_RES sd_storage_disk_init(FATFS* fs,char* path)
{
	if(f_mount(fs,path,1)==FR_OK) return F_OK;
	else return F_ERR;
}

F_RES sd_storage_init(sd_storage_t* self_object)
{
  char tt[10];
  FRESULT res;
  DWORD fre_clust, fre_sect, tot_sect;
  FATFS*  fs;
  self_object->active_disk_indx=0;
  self_object->num_of_discs=0;
  self_object->status=STORAGE_NOT_INITTIALIZED;

  for(int i=0;i<SD_STORAGE_NUM_DISKS;i++)
  {
   sd_cards_ss[i].Stat=STA_NOINIT;
   self_object->disks[i].fs.fs_type=0;
   sprintf(tt,"%d:",i);
   if(sd_storage_disk_init(&self_object->disks[i].fs,tt)==F_OK){
		  self_object->disks[i].status=DISK_PRESENT;
		  self_object->num_of_discs++;
  		  if(f_getfree(tt, &fre_clust, &fs)==FR_OK){
		    tot_sect = (self_object->disks[i].fs.n_fatent - 2) * self_object->disks[i].fs.csize;
			fre_sect = fre_clust * self_object->disks[i].fs.csize;
			self_object->disks[i].size=tot_sect/2;
			self_object->disks[i].free_space=fre_sect/2;
		  }
   }
   else self_object->disks[i].status=DISK_ABSENT;
  }

  for(int i=0;i<SD_STORAGE_NUM_DISKS;i++)
  {
   if(self_object->disks[i].status==DISK_PRESENT){
	   self_object->active_disk_indx=i;
	   self_object->status=STORAGE_INITTIALIZED;
	   return F_OK;
   }
  }
  return F_ERR;
}

F_RES sd_storage_link_ss(sd_storage_t* self_object,uint8_t disk_num, uint16_t sd_ss_pin,GPIO_TypeDef* sd_ss_port)
{
	  sd_cards_ss[disk_num].sd_ss_pin=sd_ss_pin;
	  sd_cards_ss[disk_num].sd_ss_port=sd_ss_port;
	  sd_cards_ss[disk_num].Stat=STA_NOINIT;
	  if(sd_ss_pin!=NULL&&sd_ss_port!=NULL) HAL_GPIO_WritePin(sd_ss_port, sd_ss_pin, GPIO_PIN_SET);
	  return F_OK;
}

F_RES sd_storage_write(sd_storage_t* self_object)
{

}
F_RES sd_storage_read(sd_storage_t* self_object)
{

}
F_RES sd_storage_set_next_disk(sd_storage_t* self_object)
{
 int i=self_object->active_disk_indx+1;
 for(;i<SD_STORAGE_NUM_DISKS;i++)
 {
   if(self_object->disks[i].status==DISK_PRESENT){
	 self_object->active_disk_indx=i;
	 return F_OK;
   }
 }
 return F_ERR;
}
