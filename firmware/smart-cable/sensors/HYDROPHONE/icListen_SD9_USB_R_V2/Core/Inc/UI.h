/*
 * UI.h
 *
 *  Created on: 12 апр. 2021 г.
 *      Author: admin
 */

#ifndef UI_H_
#define UI_H_

#include "string.h"
#include "cmsis_os.h"
#include "main.h"
#include "system_definitions.h"

#define UI_RX_BUFFER_SIZE  1024
#define UI_RX_BUFFER_THR   768


#define UI_F_OK      0
#define UI_F_ERR     1

#define UI_MEDIA_READY     10

//UI events def

#define UI_EVNT_SET                                     0
#define UI_EVNT_SHOW                                    1
#define UI_EVNT_RESET                                   2
#define UI_EVNT_HELP                                    3

// UI messages definitions
#define  UI_MSG_NUM_OF_FUNCTIONS           4

#define  UI_MSG_SET                        0
#define  UI_MSG_SHOW                       1
#define  UI_MSG_RESET                      2
#define  UI_MSG_HELP                       3

// UI commands definitions
#define  UI_CMD_SEND_FROM_PREDEFINED       0
#define  UI_CMD_SEND_DATA                  1

#pragma pack (push, 1)
typedef struct
{
  uint8_t* start_addr;
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


   //uint8_t      status;

   uint8_t  rx_buffer[UI_RX_BUFFER_SIZE];
   uint16_t rx_buffer_indx;
   uint16_t rx_buffer_new_string_indx;
} UI_typedef;
#pragma pack (pop)


void UI_loop(UI_typedef* UI_obj);
void UI_init(UI_typedef* UI_obj);


void UI_media_process_byte(UI_typedef* UI_obj,uint8_t rx_byte);
uint8_t UI_media_get_byte(UI_typedef* UI_obj,uint8_t* tx_byte);

int UI_parse_message(UI_typedef* UI_obj,uint8_t* msg);
void UI_messages_init(UI_typedef* UI_obj);

void UI_send_msg(UI_typedef* UI_obj,uint8_t cmd_id,void* arg);
uint8_t UI_get_event(UI_typedef* UI_obj,uint8_t* event);

int UI_MSG_SET_f(UI_typedef* UI_obj,uint8_t* msg);
int UI_MSG_SHOW_f(UI_typedef* UI_obj,uint8_t* msg);
int UI_MSG_RESET_f(UI_typedef* UI_obj,uint8_t* msg);
int UI_MSG_HELP_f(UI_typedef* UI_obj,uint8_t* msg);

#endif /* UI_H_ */
