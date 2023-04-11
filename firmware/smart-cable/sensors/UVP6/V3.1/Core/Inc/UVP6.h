/*
 * UVP6.h
 *
 *  Created on: 12 апр. 2021 г.
 *      Author: admin
 */

#ifndef SRC_UVP6_H_
#define SRC_UVP6_H_


#include "main.h"
#include "string.h"
#include "cmsis_os.h"
#include "events.h"


#define UVP6_DEPTH_LH_PROFILE    100.0 //should be 100.0 !!!
#define UVP6_RESPONSE_TIMEOUT    5000
#define UVP6_CONNECTION_RETRIES     10

#define UVP6_RX_BUFFER_SIZE      2048
#define UVP6_RX_BUFFER_THR       1500

#define UVP6_NUM_OF_CATEGORIES     18
#define UVP6_USER_PARAM_POWER_ON    0
#define UVP6_USER_PARAM_POWER_OFF   1

//media status
#define UVP6_MEDIA_READY     1

//sensor status
#define UVP6_IDLE                  1
#define UVP6_ACQ_H_STARTED         2
#define UVP6_ACQ_L_STARTED         3
#define UVP6_STARTED               4

// functions result
#define UVP6_F_OK      0
#define UVP6_F_ERR     1

//uvp6 messages definitions
#define  UVP6_MSG_NUM_OF_FUNCTIONS           7
#define  UVP6_MSG_HW_CONF                    0
#define  UVP6_MSG_BLACK_DATA                 1
#define  UVP6_MSG_LPM_DATA                   2
#define  UVP6_MSG_ACQ_CONF                   3
#define  UVP6_MSG_START_ACK                  4
#define  UVP6_MSG_START_ERR                  5
#define  UVP6_MSG_STOP_ACK                   6


//uvp6 commands definitions
#define  UVP6_CMD_START_H_ACQ                0
#define  UVP6_CMD_STOP_ACQ                   1
#define  UVP6_CMD_WKUP                       2
#define  UVP6_CMD_START_L_ACQ                3



typedef enum{
	UVP6_PROFILE_L=0,
	UVP6_PROFILE_H
}profile_t;

enum{
	UVP6_POWER_IS_ON=0,
	UVP6_POWER_IS_OFF,
	UVP6_POWER_IS_UNKNOWN
};

#pragma pack (push, 1)
typedef struct
{
	float     pressure;
	char      date[10];  //C-string
	char      time[7];  //C-string
	uint32_t  number_of_images;
	float     temperature;
	uint32_t  data[UVP6_NUM_OF_CATEGORIES];
	uint32_t  grey_levels[UVP6_NUM_OF_CATEGORIES];
} lpm_data_str;
#pragma pack (pop)

#pragma pack (push, 1)
typedef struct
{
   osMessageQId* media_rx_messages_q;
   osMessageQId* media_tx_q;
   osMessageQId* events_q;
   uint8_t       media_rx_byte;

   uint8_t       power_status;
   uint8_t       media_status;
   lpm_data_str  lpm_data;
   uint32_t      start_error;
   profile_t     profile_zone;


   uint8_t  rx_buffer[UVP6_RX_BUFFER_SIZE];
   uint16_t rx_buffer_indx;
   uint16_t rx_buffer_new_string_indx;


} uvp6;
#pragma pack (pop)


//functions

void uvp6_loop(uvp6* uvp6_obj);
void uvp6_init(uvp6* uvp6_obj,osMessageQId events_q_Handle);
void uvp6_media_process_byte(uvp6* uvp6_obj,uint8_t rx_byte);
uint8_t uvp6_media_get_byte(uvp6* uvp6_obj,uint8_t* tx_byte);

void uvp6_send_cmd(uvp6* uvp6_obj,uint8_t cmd_id,void* arg);

uint8_t uvp6_get_event(uvp6* uvp6_obj,uint8_t* event);

int uvp6_parse_message(uvp6* uvp6_obj,uint8_t* msg);
void uvp6_messages_init(uvp6* uvp6_obj);
int UVP6_MSG_HW_CONF_f(uvp6* uvp6_obj,uint8_t* msg);
int UVP6_MSG_BLACK_DATA_f(uvp6* uvp6_obj,uint8_t* msg);
int UVP6_MSG_LPM_DATA_f(uvp6* uvp6_obj,uint8_t* msg);
int UVP6_MSG_ACQ_CONF_f(uvp6* uvp6_obj,uint8_t* msg);
int UVP6_MSG_START_ACK_f(uvp6* uvp6_obj,uint8_t* msg);
int UVP6_MSG_START_ERR_f(uvp6* uvp6_obj,uint8_t* msg);
int UVP6_MSG_STOP_ACK_f(uvp6* uvp6_obj,uint8_t* msg);

#endif /* SRC_UVP6_H_ */
