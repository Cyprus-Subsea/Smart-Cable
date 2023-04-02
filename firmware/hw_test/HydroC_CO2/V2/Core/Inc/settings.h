/*
 * settings.h
 *
 *  Created on: Nov 18, 2022
 *      Author: admin
 */

#ifndef INC_SETTINGS_H_
#define INC_SETTINGS_H_

#include  "main.h"
#include  "mbcrc.h"
#include  "string.h"
#include  "stdio.h"
#include  "em_sd_storage.h"
#include  "HydroC.h"


#define   DATA_FILE_PREFIX       "HydroC_CO2_"
#define   ERR_FILE_PREFIX        "HydroC_CO2_"
#define   DATA_FILE_EXTENSION           ".dat"
#define   ERR_FILE_EXTENSION            ".err"
#define   SETTINGS_FILE      "smart_cable.sys"
#define   FILENAME_LEN                      30


#pragma pack (push, 1)
typedef struct{
	uint32_t disk_id;
    uint32_t last_file_index;

	hydroc_errors sensor_errors;

	int16_t crc;  //should be at the end!!!!
}settings_str;

#pragma pack (pop)



F_RES read_settings();
F_RES save_settings();
void set_default_settings();



#endif /* INC_SETTINGS_H_ */
