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

F_RES sd_storage_init(sd_storage_t* self_object,osSemaphoreId microsd_media_sem)
{
  self_object->microsd_media_sem=microsd_media_sem;
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

F_RES read_raw_data_crc16(sd_storage_t* self_object,uint8_t* data,uint32_t size,char* filename)
{
	FIL data_file;
	int16_t CRC16_from_disk;
	uint32_t bytes_readed;
	char tmp_str[SD_FILENAME_LEN+2];
	osSemaphoreWait(self_object->microsd_media_sem, osWaitForever);
	sprintf(tmp_str,"%u:%s",(unsigned int)self_object->active_disk_indx,filename);
	if(f_open(&data_file,tmp_str,FA_READ)==FR_OK){
	  if(f_read(&data_file,(uint8_t*)data,size,(UINT*)&bytes_readed)==FR_OK){
	   if(bytes_readed!=size){
		   f_close(&data_file);
		   osSemaphoreRelease(self_object->microsd_media_sem);
		   return F_ERR;
	   }
	   if(f_read(&data_file,(uint8_t*)&CRC16_from_disk,2,(UINT*)&bytes_readed)==FR_OK) f_close(&data_file);
	   if(bytes_readed!=2)return F_ERR;
	   int16_t CRC16_calculated=usMBCRC16((uint8_t*)data,size);
	   if(CRC16_calculated==CRC16_from_disk) {
		  osSemaphoreRelease(self_object->microsd_media_sem);
		  return F_OK;
	   }
	   osSemaphoreRelease(self_object->microsd_media_sem);
	   return F_ERR;
	  }
	  osSemaphoreRelease(self_object->microsd_media_sem);
	  return F_ERR;
	}
	osSemaphoreRelease(self_object->microsd_media_sem);
	return F_ERR;
}

F_RES save_raw_data_crc16(sd_storage_t* self_object,uint8_t* data,uint32_t size,char* filename)
{
	uint32_t byteswritten;
	FIL data_file;
	char tmp_str[SD_FILENAME_LEN+2];
	osSemaphoreWait(self_object->microsd_media_sem, osWaitForever);
	sprintf(tmp_str,"%u:%s",(unsigned int)self_object->active_disk_indx,filename);

	if(f_open(&data_file,tmp_str,FA_CREATE_ALWAYS|FA_WRITE)==FR_OK){
	  int16_t CRC16_calculated=usMBCRC16((uint8_t*)data,size);
	  if(f_write(&data_file,(uint8_t*)data,size,(UINT*)&byteswritten)==FR_OK){
		  if(byteswritten==size){
			if(f_write(&data_file,(uint8_t*)&CRC16_calculated,2,(UINT*)&byteswritten)==FR_OK){
			 if(byteswritten==2){
			  f_close(&data_file);
			  osSemaphoreRelease(self_object->microsd_media_sem);
			  return F_OK;
			 }
			}
		  }
		  osSemaphoreRelease(self_object->microsd_media_sem);
		  return F_ERR;
	  }
	  osSemaphoreRelease(self_object->microsd_media_sem);
	  return F_ERR;
	}
	osSemaphoreRelease(self_object->microsd_media_sem);
	return F_ERR;
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
