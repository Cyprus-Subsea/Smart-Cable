/*
 * slocum.h
 *
 *  Created on: 12 апр. 2021 г.
 *      Author: admin
 */

#ifndef SLOCUM_H_
#define SLOCUM_H_

#include "string.h"
#include "cmsis_os.h"

#define SLOCUM_RX_BUFFER_SIZE  1024
#define SLOCUM_RX_BUFFER_THR   768

#define SLOCUM_BASE64_BUFFER_SIZE     1000
#define SLOCUM_SND_BUFFER_SIZE        1000


typedef enum{
 GLIDER_FSM_NONE=0,
 GLIDER_FSM_CLK_UNSYNCED=1,
 GLIDER_FSM_CLK_SYNCED=2,
 GLIDER_FSM_ACTIVE=3,
 GLIDER_FSM_WAIT_CLIMB=4,
 GLIDER_FSM_WAIT_DIVE=5,
 GLIDER_FSM_START_CLIMB=6,
 GLIDER_FSM_START_DIVE=7,
 GLIDER_FSM_START_PROFILE=8
}fsm_status_t;

typedef enum{
 GLIDER_DICL_DIVE=0,
 GLIDER_DICL_CLIMB=1,

}dicl_status_t;


typedef enum{
 GLIDER_BEHAVIOR_NONE=-1,  // None
 GLIDER_BEHAVIOR_AI=0,     //actively inflecting or in the surface interval
 GLIDER_BEHAVIOR_DV=1,     //dive activated this cycle
 GLIDER_BEHAVIOR_CL=2,     //climb activated this cycle
 GLIDER_BEHAVIOR_HVR=3,    //hover activated this cycle
 GLIDER_BEHAVIOR_NTR=4,    //not transitioning, active dive/climb/hover
 GLIDER_BEHAVIOR_SAC=5,    //surface activated this cycle
 GLIDER_BEHAVIOR_SA=6,     //surface active
 GLIDER_BEHAVIOR_IGNORE=99 //ignore
}cc_final_behavior_state_t;




#define  SLOCUM_F_OK             0
#define  SLOCUM_F_ERR            1

#define  SLOCUM_MEDIA_READY     10

//slocum events def

#define  SLOCUM_EVNT_STATUS_RCVD                1
#define  SLOCUM_EVNT_DEPTH_RCVD                 2
#define  SLOCUM_EVNT_CLK_RCVD                   3
#define  SLOCUM_EVNT_MISSION_ID__RCVD           4

// slocum messages definitions
#define  SLOCUM_MSG_NUM_OF_FUNCTIONS           1
#define  SLOCUM_MSG_SD                         0


// slocum commands definitions
#define  SLOCUM_CMD_PROMPT                     0
#define  SLOCUM_CMD_SEND_DATA                  1
#define  SLOCUM_CMD_OPEN_FILE_W                2
#define  SLOCUM_CMD_WRITE_FILE_DATA            3
#define  SLOCUM_CMD_CLOSE_FILE                 4

#define  SLOCUM_MSG_OPEN_FILE_W                1
#define  SLOCUM_MSG_EOL                        2
#define  SLOCUM_MSG_SLOCUM_CMD_WRITE_FILE_DATA 3
#define  SLOCUM_MSG_CLOSE_FILE                 4

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
   uint8_t       media_rx_byte;
   uint8_t       media_status;

   fsm_status_t fsm_status;
   dicl_status_t dicl_status;
   float        last_depth;
   float        prev_depth;
   char         date[8];  //YYYYMMDD
   char         time[6];  //HHMMSS
   cc_final_behavior_state_t  behavior_state;
   uint32_t     timestamp;
   char         mission_id[8];

   uint8_t  rx_buffer[SLOCUM_RX_BUFFER_SIZE];
   uint16_t rx_buffer_indx;
   uint16_t rx_buffer_new_string_indx;

} slocum;
#pragma pack (pop)

void slocum_loop(slocum* slocum_obj);
void slocum_init(slocum* slocum_obj);

void slocum_media_process_byte(slocum* slocum_obj,uint8_t rx_byte);
uint8_t slocum_media_get_byte(slocum* slocum_obj,uint8_t* tx_byte);

int slocum_parse_message(slocum* slocum_obj,uint8_t* msg);
void slocum_messages_init(slocum* slocum_obj);

void slocum_send_cmd(slocum* slocum_obj,uint8_t cmd_id,void* arg);
uint8_t slocum_get_event(slocum* slocum_obj,uint8_t* event);

int SLOCUM_MSG_SD_f(slocum* slocum_obj,uint8_t* msg);

#endif /* slocum_H_ */
