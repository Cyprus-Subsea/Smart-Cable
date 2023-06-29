/*
 * HYDROC.h
 *
 *      Author: admin
 */

#ifndef SRC_HYDROC_H_
#define SRC_HYDROC_H_


#include "cmsis_os.h"
#include "events.h"


#define HYDROC_WARMUP_DELAY               40

#define HYDROC_RX_BUFFER_SIZE             2048
#define HYDROC_RX_BUFFER_THR              1600

#define HYDROC_CMD_TIMEOUT                5000

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
#define  HYDROC_MSG_NUM_OF_FUNCTIONS           9
enum{
 HYDROC_MSG_COSIM=0,
 HYDROC_MSG_CODS4,
 HYDROC_MSG_COTS1,
 HYDROC_MSG_COCFG,
 HYDROC_MSG_CODPV,
 HYDROC_MSG_COPEX,
 HYDROC_MSG_CORTS,
 HYDROC_MSG_COEXT,
 HYDROC_MSG_COERR
};

//hydroc commands definitions
#define  HYDROC_CMD_ENTER_CFG                  0
#define  HYDROC_CMD_EXIT_CFG                   1
#define  HYDROC_CMD_SET_REAL_TIME              2
#define  HYDROC_CMD_ENABLE_PUMP                3
#define  HYDROC_CMD_DISABLE_PUMP               4
#define  HYDROC_CMD_SET_MODE                   5
#define  HYDROC_CMD_SET_ZERO_MODE              6
#define  HYDROC_CMD_SET_MEASURE_MODE           7
#define  HYDROC_CMD_CLEAR                      8
#define  HYDROC_CMD_TEST                       9
#define  HYDROC_CMD_READ_INVENTORY             10

#define HYDROC_CMD_TEST_STRING                 9

#define HYDROC_PROFILE_0                       0
#define HYDROC_PROFILE_1                       1
#define HYDROC_PROFILE_2                       2
#define HYDROC_PROFILE_3                       3

typedef enum{
	HYDROC_TYPE_UNKNOWN=0,
	HYDROC_TYPE_CO2,
	HYDROC_TYPE_CH4
}hydroc_type_t;

typedef enum{
	HYDROC_PUMP_ENABLED=0,
	HYDROC_PUMP_DISABLED,
	HYDROC_PUMP_UNKNOWN
}hydroc_pump_status_t;


typedef struct
{
  hydroc_type_t type_id;
  char   type_name[10];
} hydroc_model_specific_t;


typedef struct
{
  char date[15];
  char time[15];
  char weekday[15];
  char pump_pwr[15];
  char P_SENSOR[15];   //  sensor-NDIR/TDLAS
  char P_IN[15];
  char I_total[15];
  char U_total[15];
  char status_zeroing[5];
  char status_flush[5];
  char status_pump[5];
  char runtime[15];
  char signal_raw[15];
  char signal_ref[15];
  char T_sensor[15];
  char signal_proc[15];
  char conc_estimate[15];
  char pGas_corr[15];
  char xGas_corr[15];
} hydroc_ds4_parameters;

typedef struct
{
  char date[15];
  char time[15];
  char  weekday[15];
  char  T_control[15];
  char  T_gas[15];
  char rH_gas[15];
} hydroc_ts1_parameters;

typedef struct
{
  uint32_t  P_in;
  uint32_t  rH_gas;
  uint32_t  T_control;
  uint32_t  P_pump;
} hydroc_errors;


#pragma pack (push, 1)
typedef struct
{
   osMessageQId  media_rx_messages_q;
   osMessageQId  media_tx_q;
   osMessageQId  events_q;
   osSemaphoreId out_q_sem;

   uint8_t       media_rx_byte;

   uint8_t       status;
   uint8_t       warmup_status;
   uint8_t       media_status;

   uint8_t       rx_buffer[HYDROC_RX_BUFFER_SIZE];
   uint16_t      rx_buffer_indx;
   uint16_t      rx_buffer_new_string_indx;

   hydroc_ds4_parameters ds4;
   hydroc_ts1_parameters ts1;
   hydroc_errors         errors;
   uint32_t      data_profile_id;

   uint32_t      inventory_PCB_num;
   uint32_t      inventory_FW_ver;
   uint32_t      inventory_HW_id;
   char          inventory_serial_number[19];

   hydroc_model_specific_t* model_specific;

   hydroc_pump_status_t pump_status;


} hydroc;
#pragma pack (pop)

//functions

void    hydroc_loop(hydroc* hydroc_obj);
void    hydroc_init(hydroc* hydroc_obj,osMessageQId events_q_Handle,osSemaphoreId out_q_sem);
void    hydroc_media_process_byte(hydroc* hydroc_obj,uint8_t rx_byte);
uint8_t hydroc_media_get_byte(hydroc* hydroc_obj,uint8_t* tx_byte);

void    hydroc_schedule_for_tx(hydroc* hydroc_obj,uint8_t* message,uint32_t size);
void    hydroc_send_cmd(hydroc* hydroc_obj,uint8_t cmd_id,void* arg);

uint8_t hydroc_get_event(hydroc* hydroc_obj,uint8_t* event);

int hydroc_parse_message(hydroc* hydroc_obj,uint8_t* msg);
void hydroc_messages_init(hydroc* hydroc_obj);


int HYDROC_MSG_COSIM_f(hydroc* hydroc_obj,uint8_t* msg);
int HYDROC_MSG_CODS4_f(hydroc* hydroc_obj,uint8_t* msg);
int HYDROC_MSG_COTS1_f(hydroc* hydroc_obj,uint8_t* msg);
int HYDROC_MSG_COCFG_f(hydroc* hydroc_obj,uint8_t* msg);
int HYDROC_MSG_CODPV_f(hydroc* hydroc_obj,uint8_t* msg);
int HYDROC_MSG_COPEX_f(hydroc* hydroc_obj,uint8_t* msg);
int HYDROC_MSG_CORTS_f(hydroc* hydroc_obj,uint8_t* msg);
int HYDROC_MSG_COEXT_f(hydroc* hydroc_obj,uint8_t* msg);
int HYDROC_MSG_COERR_f(hydroc* hydroc_obj,uint8_t* msg);




#endif /* SRC_HYDROC_H_ */
