/*
 * SSP.c
 *
 *  Created on: 12 сент. 2021 г.
 *      Author: admin
 */
#include "main.h"
#include "SSP.h"
#include "ADC.h"

ssp_str ssp_main;
extern adc_object adc_main;

void ssp_init(smac_controller* smac,osMessageQId rxQ)
{
	ssp_main.smac=smac;                                                                              // standard initializations steps
	ssp_main.rxQ=rxQ;                                                                                // standard initializations steps
	smac_register_rx_outQ(ssp_main.smac,ssp_main.rxQ,ACTIVE_QUEUE_FLAG,&ssp_event_callback);         // standard initializations steps
}


void ssp_event_callback(smac_event event_id)
{
	switch(event_id)
	{
	  case SMAC_EVNT_RX_OUT_Q_CHANGED:
		  ssp_main.rx_message_last_byte_index=0;
		  ssp_main.rx_message[0]=0;
		  xQueueReset(ssp_main.rxQ);
	  break;
	};
}


void ssp_loop()
{
	uint8_t b;
	while(1)
    {
	  while(xQueueReceive(ssp_main.rxQ,&b,1)==pdTRUE){
			ssp_new_byte_processing(b);
	  }
      osDelay(1);
    }
}


err_code ssp_new_byte_processing(uint8_t new_byte)
{
	char *path[2];
	path[0] = "action";
	path[1] = NULL;

	int action_type=0;

	if(ssp_update_rx_message(new_byte)==SSP_MSG_COMPLETE){
		ssp_main.jsonMsg.string=ssp_main.rx_message;
		ssp_main.jsonMsg.len = strlen(ssp_main.jsonMsg.string);
		lwJsonGetInt(path, &ssp_main.jsonMsg, &action_type);
		if(action_type==ACTION_DATA_REQUEST){
			ssp_send_data();
		}
	}
	return RES_OK;
}

char json_buffer[50];

void ssp_send_data()
{


	ssp_main.jsonMsg.string = json_buffer;
	ssp_main.jsonMsg.len = 45;
	json_buffer[0]=0;
	lwJsonWriteStart(&ssp_main.jsonMsg);
	lwJsonStartObject(&ssp_main.jsonMsg);
	lwJsonAddIntToObject(&ssp_main.jsonMsg, "channel",1);
	lwJsonAddIntToObject(&ssp_main.jsonMsg, "value",adc_main.adc_ch1_value);
	lwJsonCloseObject(&ssp_main.jsonMsg);
	lwJsonWriteEnd(&ssp_main.jsonMsg);
    strcat(json_buffer,"\r");
	smac_send(ssp_main.smac,json_buffer,strlen(json_buffer),KEEP_MEMORY_FLAG);json_buffer[49]=0;
}


ssp_message_status ssp_update_rx_message(uint8_t new_byte)
{

		if(new_byte==0x0D){ //end of string detected "\r", replace it with 0x00
			ssp_main.rx_message[ssp_main.rx_message_last_byte_index]=0x00;
			ssp_main.rx_message_last_byte_index=0;
			return SSP_MSG_COMPLETE;
		}

		ssp_main.rx_message[ssp_main.rx_message_last_byte_index]=new_byte;
		ssp_main.rx_message_last_byte_index++;
		ssp_main.rx_message_last_byte_index %= SSP_MSG_BUFFER_SIZE;
		return SSP_MSG_INCOMPLETE;

}
