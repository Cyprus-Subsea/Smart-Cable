/*
 * PUCK.h
 *
 *  Created on: 11 сент. 2021 г.
 *      Author: admin
 */

#ifndef INC_PUCK_H_
#define INC_PUCK_H_

#include   "main.h"
#include   "smac.h"
#include   "stdlib.h"
#include   "stdio.h"
#include   "string.h"
#include   "timed_callbacks.h"



#define PUCK_MSG_BUFFER_SIZE     50
#define PUCK_TYPE                "000"
#define PUCK_VERSION             "v1.4"
#define PUCK_DEFAULT_BAUDRATE    115200
#define PUCK_DATASHEET_SIZE      96


#define PUCK_MAX_WRITE_BYTES     32
#define PUCK_MAX_READ_BYTES      1024

#define PUCKWM_TIMEOUT           4000
#define PUCK_TIMEOUT             120000


#define PUCK_CMD_LEN             6
#define PUCK_NUM_OF_COMMANDS     13







typedef enum {
	PUCK_MSG_COMPLETE,
	PUCK_MSG_INCOMPLETE
}puck_message_status;

typedef enum {
	PUCK_ERR_ERR020=0,
	PUCK_ERR_ERR021,
	PUCK_ERR_ERR022,
	PUCK_ERR_ERR023,
	PUCK_ERR_ERR024,
	PUCK_ERR_ERR010,
	PUCK_ERR_NO_ERR
}puck_err_code;

typedef enum {
	PUCK_CMD_PUCKRM=0,
	PUCK_CMD_PUCKWM,
	PUCK_CMD_PUCKFM,
	PUCK_CMD_PUCKEM,
	PUCK_CMD_PUCKGA,
	PUCK_CMD_PUCKSA,
	PUCK_CMD_PUCKSZ,
	PUCK_CMD_PUCK,
	PUCK_CMD_PUCKTY,
	PUCK_CMD_PUCKVR,
	PUCK_CMD_PUCKIM,
	PUCK_CMD_PUCKVB,
	PUCK_CMD_PUCKSB,
	PUCK_CMD_UNKNOWN
}puck_func_index;

typedef enum {
	PUCK_MSG_PUCKRDY=0,

}puck_message_index;

typedef enum {
	PUCK_RX_ASCII_MODE,
	PUCK_RX_BIN_MODE
}puck_rx_mode;


typedef enum {
	PUCK_ANALYZER_STAGE1=0,
	PUCK_ANALYZER_STAGE1_DELAY,
	PUCK_ANALYZER_STAGE2,
	PUCK_ANALYZER_STAGE2_DELAY,
}puck_analyzer_stage;


#pragma pack(push, 1)
typedef struct {
  char UUID[16];
  uint16_t  version;
  uint16_t  size;
  uint32_t  manufacture_id;
  uint16_t  manufacture_model;
  uint16_t  manufacture_version;
  uint32_t  serial;
  char name[64];
} puck_datasheet_type;
#pragma pack(pop)


typedef struct{
	uint32_t size;
	uint32_t data_pointer_offset;
	uint8_t* start_addr;
}puck_memory_pointer_str;


typedef struct {
  puck_analyzer_stage stage;
  char stage1_msg[6];
  char stage2_msg[6];
  uint32_t stage1_delay;
  uint32_t stage2_delay;
  uint32_t byte_counter;
  timed_callback_t* stage1_timed_callback;
  timed_callback_t* stage2_timed_callback;

}puck_analyzer_str;

typedef struct {
	smac_controller* smac;
	puck_memory_pointer_str datasheet;
	osMessageQId rxQ;
	osMessageQId instrument_rxQ;
	uint8_t rx_message[PUCK_MSG_BUFFER_SIZE];
	uint32_t rx_message_last_byte_index;
    uint8_t rx_mode;
	puck_err_code ((*cmd_functions[PUCK_NUM_OF_COMMANDS]))();
	char* rx_msg_params[PUCK_CMD_LEN];

	puck_analyzer_str analyzer;

}puck_str;


void puck_init( smac_controller* smac,osMessageQId rxQ,osMessageQId instrument_rxQ);
void puck_loop( );
analyzer_err_code puck_analyzer(uint8_t new_byte);



err_code puck_commands_init(void);
puck_message_status puck_update_rx_message(uint8_t new_byte);
err_code puck_new_byte_processing(uint8_t new_byte);
err_code puck_split_rx_message(char** s_array,char* delimiter);
puck_func_index puck_get_cmd_pattern_index(char* cmd_string);
err_code puck_write_datasheet(void);
void puck_stage1_timed_callback();
void puck_stage2_timed_callback();
void puck_event_callback(smac_event event_id);

void puck_send_integer(uint32_t msg);
void puck_send_raw(uint8_t* msg,uint32_t len,uint8_t mem_free_flag);
void puck_send_string(char* msg,uint8_t mem_free_flag);


puck_err_code PUCKRM_f(void);
puck_err_code PUCKWM_f(void);
puck_err_code PUCKFM_f(void);
puck_err_code PUCKEM_f(void);
puck_err_code PUCKGA_f(void);
puck_err_code PUCKSA_f(void);
puck_err_code PUCKSZ_f(void);
puck_err_code PUCK_f(void);
puck_err_code PUCKTY_f(void);
puck_err_code PUCKVR_f(void);
puck_err_code PUCKIM_f(void);
puck_err_code PUCKVB_f(void);
puck_err_code PUCKSB_f(void);

#endif /* INC_PUCK_H_ */
