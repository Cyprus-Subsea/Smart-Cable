/*
 * SSP.h
 *
 *  Created on: 12 сент. 2021 г.
 *      Author: admin
 */

#ifndef INC_SSP_H_
#define INC_SSP_H_

#include "system_definitions.h"
#include "cmsis_os.h"
#include "smac.h"
#include "lwjson.h"
#include "string.h"


#define SSP_MSG_BUFFER_SIZE  50

typedef enum {
	SSP_MSG_COMPLETE,
	SSP_MSG_INCOMPLETE
}ssp_message_status;

typedef struct {
	smac_controller* smac;
	osMessageQId rxQ;

	uint8_t  rx_message[SSP_MSG_BUFFER_SIZE];
	uint32_t rx_message_last_byte_index;
    uint8_t  rx_mode;

    LwJsonMsg jsonMsg;
}ssp_str;


void ssp_init(smac_controller* smac,osMessageQId rxQ);
void ssp_event_callback(smac_event event_id);
void ssp_loop();
err_code ssp_new_byte_processing(uint8_t new_byte);
ssp_message_status ssp_update_rx_message(uint8_t new_byte);
void ssp_send_data();


#endif /* INC_SSP_H_ */
