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

#define SEAGLIDER_RX_BUFFER_SIZE  1024
#define SEAGLIDER_RX_BUFFER_THR   768


#define SEAGLIDER_F_OK      0
#define SEAGLIDER_F_ERR     1

#define SEAGLIDER_MEDIA_READY     10

//seaglider events def

#define SEAGLIDER_EVNT_START_RCVD                0
#define SEAGLIDER_EVNT_STOP_RCVD                 1
#define SEAGLIDER_EVNT_TEST_RCVD                 2
#define SEAGLIDER_EVNT_SEND_TXT_FILE_RCVD        3
#define SEAGLIDER_EVNT_CLOCK_RCVD                4
#define SEAGLIDER_EVNT_WAKEUP_RCVD               5
#define SEAGLIDER_EVNT_DEPTH_RCVD                6

// seaglider messages definitions
#define  SEAGLIDER_MSG_NUM_OF_FUNCTIONS           9
#define  SEAGLIDER_MSG_DEPTH                      0
#define  SEAGLIDER_MSG_STOP                       1
#define  SEAGLIDER_MSG_SEND_TXT_FILE              2
#define  SEAGLIDER_MSG_START                      3
#define  SEAGLIDER_MSG_SEND_INFO                  4
#define  SEAGLIDER_MSG_RESET                      5
#define  SEAGLIDER_MSG_TEST                       6
#define  SEAGLIDER_MSG_CLOCK                      7
#define  SEAGLIDER_MSG_WAKEUP                     8

// seaglider commands definitions
#define  SEAGLIDER_CMD_PROMPT                     0
#define  SEAGLIDER_CMD_SEND_DATA                  1

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
   osMessageQId* media_rx_messages_q;
   osMessageQId* media_tx_q;
   osMessageQId* events_q;
   uint8_t       media_rx_byte;
   uint8_t       media_status;


   uint8_t      status;
   float        last_depth;
   float        prev_depth;
   char         date[8];  //YYYYMMDD
   char         time[6];  //HHMMSS

   uint8_t  rx_buffer[SEAGLIDER_RX_BUFFER_SIZE];
   uint16_t rx_buffer_indx;
   uint16_t rx_buffer_new_string_indx;



} seaglider;
#pragma pack (pop)


void seaglider_loop(seaglider* seaglider_obj);
void seaglider_init(seaglider* seaglider_obj);


void seaglider_media_process_byte(seaglider* seaglider_obj,uint8_t rx_byte);
uint8_t seaglider_media_get_byte(seaglider* seaglider_obj,uint8_t* tx_byte);


int ((*seaglider_functions[SEAGLIDER_MSG_NUM_OF_FUNCTIONS]))(seaglider* seaglider_obj,uint8_t* msg);
char*  seaglider_messages_strings[SEAGLIDER_MSG_NUM_OF_FUNCTIONS];

int seaglider_parse_message(seaglider* seaglider_obj,uint8_t* msg);
void seaglider_messages_init(seaglider* seaglider_obj);

void seaglider_send_cmd(seaglider* seaglider_obj,uint8_t cmd_id,void* arg);
uint8_t seaglider_get_event(seaglider* seaglider_obj,uint8_t* event);

int SEAGLIDER_MSG_DEPTH_f(seaglider* seaglider_obj,uint8_t* msg);
int SEAGLIDER_MSG_STOP_f(seaglider* seaglider_obj,uint8_t* msg);
int SEAGLIDER_MSG_START_f(seaglider* seaglider_obj,uint8_t* msg);
int SEAGLIDER_MSG_SEND_TXT_FILE_f(seaglider* seaglider_obj,uint8_t* msg);
int SEAGLIDER_MSG_SEND_INFO_f(seaglider* seaglider_obj,uint8_t* msg);
int SEAGLIDER_MSG_RESET_f(seaglider* seaglider_obj,uint8_t* msg);
int SEAGLIDER_MSG_TEST_f(seaglider* seaglider_obj,uint8_t* msg);
int SEAGLIDER_MSG_CLOCK_f(seaglider* seaglider_obj,uint8_t* msg);
int SEAGLIDER_MSG_WAKEUP_f(seaglider* seaglider_obj,uint8_t* msg);


#endif /* SEAGLIDER_H_ */
