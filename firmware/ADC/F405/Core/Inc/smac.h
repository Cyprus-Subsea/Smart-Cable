/*
 * smac_controller.h
 *
 *  Created on: 11 сент. 2021 г.
 *      Author: admin
 */

#ifndef INC_SMAC_H_
#define INC_SMAC_H_

#include "main.h"
#include "system_definitions.h"
#include "cmsis_os.h"

#include "interrupts.h"

#define NUM_OF_SMAC_CONTROLLERS  3
#define NUM_OF_RX_OUT_QUEUES        3


enum tx_media_statuses
{
  TX_MEDIA_READY,
  TX_MEDIA_BUSY
};
enum memory_clear_flag
{
	FREE_MEMORY_FLAG,
	KEEP_MEMORY_FLAG
};
enum Q_status
{
	ACTIVE_QUEUE_FLAG=10,
	INACTIVE_QUEUE_FLAG=20
};

typedef enum
{
	SMAC_ANALYZER_OK,
	SMAC_ANALYZER_ERR,
	SMAC_ANALYZER_STOP_SENDING

} analyzer_err_code;

typedef struct
{
 uint8_t* data_pointer;
 uint16_t data_size;
 uint8_t  free_after_use;

}memory_block_ptr;

typedef enum
{
  SMAC_EVNT_NONE=0,
  SMAC_EVNT_RX_OUT_Q_CHANGED=1
}smac_event;

typedef struct
{
  UART_HandleTypeDef*  huart;

  osMessageQId         rx_inQ;
  uint8_t              rx_byte_raw;

  osMessageQId         tx_outQ;
  memory_block_ptr*    last_tx_message;
  uint8_t              tx_media_status;

  osMessageQId         rx_outQ[NUM_OF_RX_OUT_QUEUES];
  void    (*event_callback[NUM_OF_RX_OUT_QUEUES])(smac_event event_id);
  osMessageQId         active_rx_outQ;
  uint8_t              new_rx_outQ_index;

  analyzer_err_code (*analyzer)(uint8_t new_byte);
  smac_event event_status;
}smac_controller;


typedef struct {
 smac_controller* items[NUM_OF_SMAC_CONTROLLERS];
 uint8_t new_smac_index;
} smac_list;


void smac_list_init();
smac_controller* smac_create_new(UART_HandleTypeDef* huart,osMessageQId uart_rxQHandle,osMessageQId uart_txQHandle);
void smac_loop(smac_controller* self_object);
err_code smac_send(smac_controller* self_object,void* mem_ptr,uint16_t mem_size, uint8_t mem_free_after_use_flag);

err_code smac_register_rx_outQ(smac_controller* self_object,osMessageQId   rx_outQ, uint8_t activeQ_flag,void (*event_callback)(smac_event event_id));

#endif /* INC_SMAC_H_ */
