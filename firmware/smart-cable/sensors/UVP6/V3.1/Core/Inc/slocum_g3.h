/*
 * slocum.h
 *
 *  Created on: 12 апр. 2021 г.
 *      Author: admin
 */

#ifndef SLOCUM_H_
#define SLOCUM_H_

#include "main.h"
#include "string.h"
#include "cmsis_os.h"
#include "events.h"

#define SLOCUM_CONNECTION_RETRIES        1
#define SLOCUM_RESPONSE_TIMEOUT         50
#define SLOCUM_DIVE_COUNTER_MAX        999
#define SLOCUM_FILE_WRITE_LEN          180


#define SLOCUM_RX_BUFFER_SIZE         1024
#define SLOCUM_RX_BUFFER_THR           768

#define SLOCUM_BASE64_BUFFER_SIZE     1000
#define SLOCUM_SND_BUFFER_SIZE        1000

typedef enum{
	SLOCUM_X_GLIDER_DOS_OFF=0,
	SLOCUM_X_GLIDER_DOS_ON
}x_glider_dos_t;


typedef enum{
 GLIDER_FINAL_DEPTH_AT_SURFACE=0,
 GLIDER_FINAL_DEPTH_DIVINIG=1,
 GLIDER_FINAL_DEPTH_CLIMBING=2
}cc_final_depth_state_mode_t;

typedef enum{
 GLIDER_STATE_DIVING=0,
 GLIDER_STATE_CLIMBING,
 GLIDER_STATE_AT_SURFACE
}dcs_state_t;

#define  SLOCUM_F_OK                 0
#define  SLOCUM_F_ERR                1

#define  SLOCUM_MEDIA_READY          10

// slocum messages definitions
#define  SLOCUM_MSG_NUM_OF_FUNCTIONS           1
#define  SLOCUM_MSG_SD                         0


// slocum commands definitions
#define  SLOCUM_CMD_PROMPT                     0
#define  SLOCUM_CMD_SEND_DATA                  1
#define  SLOCUM_CMD_OPEN_FILE_W                2
#define  SLOCUM_CMD_WRITE_FILE_DATA            3
#define  SLOCUM_CMD_CLOSE_FILE                 4
#define  SLOCUM_CMD_SEND_EVNT                  5

#define  SLOCUM_MSG_OPEN_FILE_W                1
#define  SLOCUM_MSG_EOL                        2
#define  SLOCUM_MSG_SLOCUM_CMD_WRITE_FILE_DATA 3
#define  SLOCUM_MSG_CLOSE_FILE                 4
#define  SLOCUM_MSG_SEND_EVNT                  5

#pragma pack (push, 1)
typedef struct
{
  void*  start_addr;
  uint32_t size;
} memory_region_pointer;
#pragma pack (pop)

#pragma pack (push, 1)
typedef struct
{
   osMessageQId* media_rx_messages_q;
   osMessageQId* media_tx_q;
   osMessageQId* events_q;
   osSemaphoreId out_q_sem;
   uint8_t       media_rx_byte;
   uint8_t       media_status;

   float        last_depth;
   float        prev_depth;
   float        gps_lon;
   float        gps_lat;
   float        dive_target_depth;
   char         date[8];  //YYYYMMDD
   char         time[6];  //HHMMSS
   cc_final_depth_state_mode_t  final_depth_state;
   x_glider_dos_t  x_glider_dos;
   dcs_state_t  dcs_state;

   uint32_t     timestamp;
   char         mission_id[8];
   uint32_t     dive_climb_counter;

   uint32_t    user_param_power_ctrl;
   float       user_param_min_depth;

   uint8_t  rx_buffer[SLOCUM_RX_BUFFER_SIZE];
   uint16_t rx_buffer_indx;
   uint16_t rx_buffer_new_string_indx;

} slocum;
#pragma pack (pop)

void slocum_loop(slocum* slocum_obj);
void slocum_init(slocum* slocum_obj,osMessageQId events_q_Handle,osSemaphoreId out_q_sem);

void slocum_schedule_for_tx(slocum* slocum_obj,uint8_t* message,uint32_t size);
void slocum_media_process_byte(slocum* slocum_obj,uint8_t rx_byte);
uint8_t slocum_media_get_byte(slocum* slocum_obj,uint8_t* tx_byte);

int slocum_parse_message(slocum* slocum_obj,uint8_t* msg);
void slocum_messages_init(slocum* slocum_obj);

void slocum_send_cmd(slocum* slocum_obj,uint8_t cmd_id,void* arg);
void slocum_send_evnt(slocum* slocum_obj,uint32_t evnt);
uint8_t slocum_get_event(slocum* slocum_obj,uint8_t* event);

int SLOCUM_MSG_SD_f(slocum* slocum_obj,uint8_t* msg);

#endif /* slocum_H_ */
