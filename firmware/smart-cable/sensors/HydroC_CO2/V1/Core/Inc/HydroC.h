/*
 * HYDROC.h
 *
 *      Author: admin
 */

#ifndef SRC_HYDROC_H_
#define SRC_HYDROC_H_


#include "cmsis_os.h"


#define HYDROC_WARMUP_DELAY               30

#define HYDROC_RX_BUFFER_SIZE             1024
#define HYDROC_RX_BUFFER_THR              768

//media status
#define HYDROC_MEDIA_READY                1


//sensor status
#define HYDROC_WARMUP_FINISHED            0
#define HYDROC_WARMUP_START_RECEIVED      1
#define HYDROC_WARMUP_IN_PROGRESS         2


//sensor status
#define HYDROC_READY                      1
#define HYDROC_WAIT_DATA                  2
#define HYDROC_IDLE                       3

// functions result
#define HYDROC_F_OK                       0
#define HYDROC_F_ERR                      1

//hydroc messages definitions
#define  HYDROC_MSG_NUM_OF_FUNCTIONS           2

#define  HYDROC_MSG_COSIM                      0
#define  HYDROC_MSG_CODS4                      1

//hydroc commands definitions
#define  HYDROC_CMD_ENTER_CFG                  0
#define  HYDROC_CMD_EXIT_CFG                   1
#define  HYDROC_CMD_SET_REAL_TIME              2
#define  HYDROC_CMD_ENABLE_PUMP                3
#define  HYDROC_CMD_DISABLE_PUMP               4
#define  HYDROC_CMD_SET_MODE                   5
#define  HYDROC_CMD_SET_ZERO_MODE              6
#define  HYDROC_CMD_SET_MEASURE_MODE           7


//events definitions
#define  HYDROC_EVNT_BOOTED                    0
#define  HYDROC_EVNT_CODS4                     1

#pragma pack (push, 1)
typedef struct
{
   osMessageQId* media_rx_messages_q;
   osMessageQId* media_tx_q;
   osMessageQId* events_q;
   uint8_t       media_rx_byte;

   uint8_t       status;
   uint8_t       warmup_status;
   uint8_t       media_status;

   uint8_t  rx_buffer[HYDROC_RX_BUFFER_SIZE];
   uint16_t rx_buffer_indx;
   uint16_t rx_buffer_new_string_indx;

   uint8_t  ds4_data[120];
} hydroc;
#pragma pack (pop)


//functions

void    hydroc_loop(hydroc* hydroc_obj);
void    hydroc_init(hydroc* hydroc_obj);
void    hydroc_media_process_byte(hydroc* hydroc_obj,uint8_t rx_byte);
uint8_t hydroc_media_get_byte(hydroc* hydroc_obj,uint8_t* tx_byte);

void    hydroc_send_cmd(hydroc* hydroc_obj,uint8_t cmd_id,void* arg);

uint8_t hydroc_get_event(hydroc* hydroc_obj,uint8_t* event);

int hydroc_parse_message(hydroc* hydroc_obj,uint8_t* msg);
void hydroc_messages_init(hydroc* hydroc_obj);


int HYDROC_MSG_COSIM_f(hydroc* hydroc_obj,uint8_t* msg);
int HYDROC_MSG_CODS4_f(hydroc* hydroc_obj,uint8_t* msg);

#endif /* SRC_HYDROC_H_ */
