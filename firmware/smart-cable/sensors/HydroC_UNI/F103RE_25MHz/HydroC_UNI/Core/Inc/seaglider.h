/*
 * seaglider.h
 *
 *  Created on: 12 апр. 2021 г.
 *      Author: admin
 */

#ifndef SEAGLIDER_H_
#define SEAGLIDER_H_

#include "string.h"
#include "cmsis_os.h"
#include "events.h"

#define SEAGLIDER_SND_BUFFER_SIZE  1024


#define SEAGLIDER_RX_BUFFER_SIZE   1024
#define SEAGLIDER_RX_BUFFER_THR     768


#define SEAGLIDER_STATUS_DIVE      1
#define SEAGLIDER_STATUS_CLIMB     2
#define SEAGLIDER_STATUS_UNKNOWN   0




#define SEAGLIDER_F_OK      0
#define SEAGLIDER_F_ERR     1

#define SEAGLIDER_MEDIA_READY     10

// seaglider messages definitions
#define  SEAGLIDER_MSG_NUM_OF_FUNCTIONS           12
#define  SEAGLIDER_MSG_DEPTH                      0
#define  SEAGLIDER_MSG_STOP                       1
#define  SEAGLIDER_MSG_SEND_TXT_FILE              2
#define  SEAGLIDER_MSG_START                      3
#define  SEAGLIDER_MSG_SEND_INFO                  4
#define  SEAGLIDER_MSG_RESET                      5
#define  SEAGLIDER_MSG_TEST                       6
#define  SEAGLIDER_MSG_CLOCK                      7
#define  SEAGLIDER_MSG_WAKEUP                     8
#define  SEAGLIDER_MSG_CLEAR                      9
#define  SEAGLIDER_MSG_POFF                       10
#define  SEAGLIDER_MSG_ERRORS                     11

// seaglider commands definitions
#define  SEAGLIDER_CMD_PROMPT                     0
#define  SEAGLIDER_CMD_SEND_DATA                  1

#define  SEAGLIDER_MSG_PROMPT                     0
#define  SEAGLIDER_MSG_EVNT                       1

#define  SEAGLIDER_STOP_PUMP_FLAG_ACTIVATED      1
#define  SEAGLIDER_STOP_PUMP_FLAG_DEACTIVATED    0


#pragma pack (push, 1)
typedef struct
{
  uint32_t start_addr;
  uint32_t size;
} memory_region_pointer;
#pragma pack (pop)


#pragma pack (push, 1)
typedef struct
{
   osMessageQId media_rx_messages_q;
   osMessageQId media_tx_q;
   osMessageQId events_q;
   osSemaphoreId out_q_sem;
   uint8_t       media_rx_byte;
   uint8_t       media_status;

   uint8_t      status;
   uint8_t      dive_status;   // a-dive b-climb
   uint8_t      stop_pump_flag;

   float        last_depth;
   float        prev_depth;
   char         date[8];  //YYYYMMDD
   char         time[8];  //HHMMSS

   uint8_t      rx_buffer[SEAGLIDER_RX_BUFFER_SIZE];
   uint16_t     rx_buffer_indx;
   uint16_t     rx_buffer_new_string_indx;

   uint32_t     param_x;
   uint32_t     param_y;
   uint32_t     param_z;

} seaglider;
#pragma pack (pop)


void seaglider_loop(seaglider* seaglider_obj);
void seaglider_init(seaglider* seaglider_obj,osMessageQId events_q_Handle,osSemaphoreId out_q_sem);


void seaglider_media_process_byte(seaglider* seaglider_obj,uint8_t rx_byte);
uint8_t seaglider_media_get_byte(seaglider* seaglider_obj,uint8_t* tx_byte);

int seaglider_parse_message(seaglider* seaglider_obj,uint8_t* msg);
void seaglider_messages_init(seaglider* seaglider_obj);

void seaglider_send_evnt(seaglider* seaglider_obj,uint32_t event_id);
void seaglider_send_cmd(seaglider* seaglider_obj,uint8_t cmd_id,void* arg);
uint8_t seaglider_get_event(seaglider* seaglider_obj,uint8_t* event);
void seaglider_schedule_for_tx(seaglider* seaglider_obj,uint8_t* message,uint32_t size);


int SEAGLIDER_MSG_DEPTH_f(seaglider* seaglider_obj,uint8_t* msg);
int SEAGLIDER_MSG_STOP_f(seaglider* seaglider_obj,uint8_t* msg);
int SEAGLIDER_MSG_START_f(seaglider* seaglider_obj,uint8_t* msg);
int SEAGLIDER_MSG_SEND_TXT_FILE_f(seaglider* seaglider_obj,uint8_t* msg);
int SEAGLIDER_MSG_SEND_INFO_f(seaglider* seaglider_obj,uint8_t* msg);
int SEAGLIDER_MSG_RESET_f(seaglider* seaglider_obj,uint8_t* msg);
int SEAGLIDER_MSG_TEST_f(seaglider* seaglider_obj,uint8_t* msg);
int SEAGLIDER_MSG_CLOCK_f(seaglider* seaglider_obj,uint8_t* msg);
int SEAGLIDER_MSG_WAKEUP_f(seaglider* seaglider_obj,uint8_t* msg);
int SEAGLIDER_MSG_CLEAR_f(seaglider* seaglider_obj,uint8_t* msg);
int SEAGLIDER_MSG_POFF_f(seaglider* seaglider_obj,uint8_t* msg);
int SEAGLIDER_MSG_ERRORS_f(seaglider* seaglider_obj,uint8_t* msg);

#endif /* SEAGLIDER_H_ */
