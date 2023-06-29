/*
 * settings.h
 *
 *  Created on: Nov 18, 2022
 *      Author: admin
 */

#ifndef INC_SETTINGS_H_
#define INC_SETTINGS_H_

#include  "main.h"

#include  "string.h"
#include  "stdio.h"
#include  "em_sd_storage.h"
#include  "HydroC.h"




#define   DATA_FILE_PREFIX       "HydroC"
#define   ERR_FILE_PREFIX        "HydroC"
#define   DATA_FILE_EXTENSION           ".dat"
#define   ERR_FILE_EXTENSION            ".err"
#define   SETTINGS_FILE      "smart_cable.sys"


#pragma pack (push, 1)
typedef struct{
    uint32_t   last_file_index;

}settings_str;

#pragma pack (pop)



F_RES read_settings();
F_RES save_settings();
void  set_default_settings();

F_RES read_raw_data_crc16(sd_storage_t* self_object,uint8_t* data,uint32_t size,char* filename);
F_RES save_raw_data_crc16(sd_storage_t* self_object,uint8_t* data,uint32_t size,char* filename);




#endif /* INC_SETTINGS_H_ */
