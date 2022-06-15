#include "wav.h"
#include "string.h"
#include "system_definitions.h"


extern UART_HandleTypeDef huart1;

F_RES wav_file_open(wav_file_typedef* self_object,char* filename)
{
  memcpy(self_object->header.chunk.prefix,"RIFF",4);
  memcpy(self_object->header.chunk.postfix,"WAVE",4);
  memcpy(self_object->header.subchunk_fmt.prefix,"fmt ",4);
  memcpy(self_object->header.subchunk_data.prefix,"data",4);
  self_object->data_counter=0;
  if(f_open(&self_object->media.file,filename,FA_CREATE_ALWAYS|FA_WRITE)==FR_OK){
   f_lseek(&self_object->media.file, sizeof(self_object->header));
   return F_OK;
  }
  return F_ERR;
}

F_RES wav_file_close(wav_file_typedef* self_object)
{
	self_object->header.subchunk_data.size=self_object->data_counter;
	self_object->header.chunk.size=36+self_object->data_counter;
 	f_lseek(&self_object->media.file, 0);
 	if(f_write(&self_object->media.file,(uint8_t*)&self_object->header,sizeof(self_object->header),(void*)&self_object->media.byteswritten)==FR_OK){
 	  f_close(&self_object->media.file);
 	  return F_OK;
 	}
 	return F_ERR;
}

F_RES wav_file_write(wav_file_typedef* self_object,uint8_t* data,uint32_t length)
{
	FRESULT res;
	self_object->data_counter+=length;
	res=f_write(&self_object->media.file,data,length,(void*)&self_object->media.byteswritten);
	HAL_UART_Transmit(&huart1,(uint8_t*)&res,1,100);
	if(res==FR_OK) return F_OK;
	return F_ERR;
}

void readDir(char* dir_name)
{
	//DWORD fre_clust, fre_sect, tot_sect;
	//uint32_t byteswritten,bytesread;
	//FRESULT res;
	char *fn;
	DIR dir;
	FILINFO fileInfo;
	uint8_t sect[512];
	uint8_t result;

	//fileInfo.fname = (char*)sect;
	fileInfo.fsize = sizeof(sect);
	result = f_opendir(&dir,dir_name);
	if (result == FR_OK)
	{
		while(1)
		  {
			result = f_readdir(&dir, &fileInfo);
			if (result==FR_OK && fileInfo.fname[0]){
			  fn = fileInfo.fname;
			  if(strlen(fn)){
				HAL_UART_Transmit(&huart1,(uint8_t*)fn,strlen(fn),0x1000);
			  }
			  if(fileInfo.fattrib&AM_DIR){
				HAL_UART_Transmit(&huart1,(uint8_t*)" [DIR]",6,0x1000);
			  }
			  HAL_UART_Transmit(&huart1,(uint8_t*)"\n",1,0x1000);
			}
			else break;
		   }
		   f_closedir(&dir);
	 }
}
