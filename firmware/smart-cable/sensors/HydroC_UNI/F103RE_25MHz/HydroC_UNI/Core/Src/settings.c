/*
 * settings.c
 *
 *  Created on: Nov 18, 2022
 *      Author: admin
 */


#include  "settings.h"
#include  "main.h"
#include  "em_sd_storage.h"

char settings_filename[]= SETTINGS_FILE;
extern sd_storage_t microsd_storage;

settings_str run_cfg;

void set_default_settings()
{
	run_cfg.last_file_index=0;
}

F_RES read_settings()
{
  if( read_raw_data_crc16(&microsd_storage,(uint8_t*)&run_cfg,sizeof(run_cfg),settings_filename)==F_OK){
	return F_OK;
  }
  return F_ERR;
}

F_RES save_settings()
{
  if( save_raw_data_crc16(&microsd_storage,(uint8_t*)&run_cfg,sizeof(run_cfg),settings_filename)==F_OK){
	return F_OK;
  }
  return F_ERR;
}







