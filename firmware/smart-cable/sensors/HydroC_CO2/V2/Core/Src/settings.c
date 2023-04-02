/*
 * settings.c
 *
 *  Created on: Nov 18, 2022
 *      Author: admin
 */


#include  "settings.h"
#include  "main.h"

char settings_filename[]= SETTINGS_FILE;
extern UART_HandleTypeDef huart5;
settings_str run_cfg;

void set_default_settings()
{
	run_cfg.last_file_index=0;
	run_cfg.crc=0;
	run_cfg.sensor_errors.P_in=0;
	run_cfg.sensor_errors.rH_gas=0;
	run_cfg.sensor_errors.T_control=0;
	run_cfg.sensor_errors.P_pump=0;
}

F_RES read_settings()
{
	uint32_t bytesreaded;
	FIL settings_file;
	char tmp_str[FILENAME_LEN+2];
	sprintf(tmp_str,"%u:%s",run_cfg.disk_id,settings_filename);
	if(f_open(&settings_file,tmp_str,FA_READ)==FR_OK){
	  if(f_read(&settings_file,(uint8_t*)&run_cfg,sizeof(run_cfg),(UINT*)&bytesreaded)==FR_OK){
	   f_close(&settings_file);
	   if(bytesreaded!=sizeof(run_cfg))return F_ERR;
	   int16_t CRC16_calculated=usMBCRC16((uint8_t*)&run_cfg,sizeof(run_cfg)-2);
	   if(CRC16_calculated==run_cfg.crc) {
		  return F_OK;
	   }
	   return F_ERR;
	  }
	  return F_ERR;
	}
	return F_ERR;
}



F_RES save_settings()
{
	uint32_t byteswritten;
	FIL settings_file;
	char tmp_str[FILENAME_LEN+2];
	sprintf(tmp_str,"%u:%s",run_cfg.disk_id,settings_filename);

	if(f_open(&settings_file,tmp_str,FA_CREATE_ALWAYS|FA_WRITE)==FR_OK){
	  int16_t CRC16_calculated=usMBCRC16((uint8_t*)&run_cfg,sizeof(run_cfg)-2);
	  run_cfg.crc=CRC16_calculated;
	  if(f_write(&settings_file,(uint8_t*)&run_cfg,sizeof(run_cfg),(UINT*)&byteswritten)==FR_OK){
		  if(byteswritten==sizeof(run_cfg)){
			f_close(&settings_file);
			return F_OK;
		  }
		  return F_ERR;
	  }
	  return F_ERR;
	}
	return F_ERR;
}


