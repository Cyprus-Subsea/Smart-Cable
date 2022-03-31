/*
 * seaglider.c
 *
 *  Created on: 12 апр. 2021 г.
 *      Author: admin
 */


#include "seaglider.h"
#include "string.h"
#include "stdlib.h"

//debug
#include "main.h"
extern UART_HandleTypeDef huart1;
//HAL_UART_Transmit(&huart1,msg,strlen(msg),100);

int ((*seaglider_functions[SEAGLIDER_MSG_NUM_OF_FUNCTIONS]))(seaglider* seaglider_obj,uint8_t* msg);
char*  seaglider_messages_strings[SEAGLIDER_MSG_NUM_OF_FUNCTIONS];


const char* seaglider_commands_strings[]={"UV>\r"};


void seaglider_init(seaglider* seaglider_obj)
{
	seaglider_obj->last_depth=0.0;
	seaglider_obj->prev_depth=0.0;
	seaglider_messages_init(seaglider_obj);


	osMessageQDef(seaglider_events_q, 20, uint8_t);
	seaglider_obj->events_q= osMessageCreate(osMessageQ(seaglider_events_q), NULL);

	osMessageQDef(seaglider_media_rx_q, 20, uint16_t);
	seaglider_obj->media_rx_messages_q= osMessageCreate(osMessageQ(seaglider_media_rx_q), NULL);

	osMessageQDef(seaglider_media_tx_q, 200, uint8_t);
	seaglider_obj->media_tx_q = osMessageCreate(osMessageQ(seaglider_media_tx_q), NULL);

	osThreadDef(seaglider_task, seaglider_loop, osPriorityNormal, 0, 256);
	osThreadCreate(osThread(seaglider_task), seaglider_obj);


}


void seaglider_loop(seaglider* seaglider_obj)
{
	 uint16_t msg_indx;
	 for(;;)
	 {
		//test loopback
		if(xQueueReceive(seaglider_obj->media_rx_messages_q,&msg_indx,0))
		{   uint8_t* msg=seaglider_obj->rx_buffer+msg_indx;
			seaglider_parse_message(seaglider_obj,msg);
		}
		//osDelay(1);
	 }

}


void seaglider_media_process_byte(seaglider* seaglider_obj,uint8_t rx_byte)
{
	if(seaglider_obj->media_status==SEAGLIDER_MEDIA_READY && rx_byte!=0x00)
	{
		seaglider_obj->rx_buffer[seaglider_obj->rx_buffer_indx]=rx_byte;
		if(rx_byte=='\r')
		{
			seaglider_obj->rx_buffer[seaglider_obj->rx_buffer_indx]=0x00;
			osMessagePut(seaglider_obj->media_rx_messages_q,seaglider_obj->rx_buffer_new_string_indx,1);
			if(seaglider_obj->rx_buffer_indx>SEAGLIDER_RX_BUFFER_THR) seaglider_obj->rx_buffer_indx=0;
			else  seaglider_obj->rx_buffer_indx++;
			seaglider_obj->rx_buffer_new_string_indx=seaglider_obj->rx_buffer_indx;
		}
		else seaglider_obj->rx_buffer_indx++;
		if(seaglider_obj->rx_buffer_indx==SEAGLIDER_RX_BUFFER_SIZE)
		{
			seaglider_obj->rx_buffer_indx=0;
			seaglider_obj->rx_buffer_new_string_indx=seaglider_obj->rx_buffer_indx;
		}
	}
	else
	{
		seaglider_obj->rx_buffer_indx=0;
		seaglider_obj->rx_buffer_new_string_indx=seaglider_obj->rx_buffer_indx;
	}
}

uint8_t seaglider_media_get_byte(seaglider* seaglider_obj,uint8_t* tx_byte)
{
	osEvent res=osMessageGet(seaglider_obj->media_tx_q,0);
	if(res.status==osEventMessage)
    {
		*tx_byte=res.value.v;
		return SEAGLIDER_F_OK;
    }
  return SEAGLIDER_F_ERR;
}


void seaglider_send_cmd(seaglider* seaglider_obj,uint8_t cmd_id,void* arg)
{
 memory_region_pointer* ptr1;
 switch(cmd_id)
 {
     case SEAGLIDER_CMD_PROMPT:
  	   for(int i=0;i<strlen(seaglider_commands_strings[SEAGLIDER_CMD_PROMPT]);i++)
  	   {
  		   osMessagePut(seaglider_obj->media_tx_q,*(seaglider_commands_strings[SEAGLIDER_CMD_PROMPT]+i),1);
  	   }
	 break;
     case SEAGLIDER_CMD_SEND_DATA:
       ptr1=arg;
       for(int i=0;i<ptr1->size;i++)
	   {
		   osMessagePut(seaglider_obj->media_tx_q,*((uint8_t*)(ptr1->start_addr)+i),osWaitForever);
	   }
     break;
 }

}

uint8_t seaglider_get_event(seaglider* seaglider_obj,uint8_t* event)
{
	if(xQueueReceive(seaglider_obj->events_q,event,1))
	{
     return SEAGLIDER_F_OK;
	}
	return SEAGLIDER_F_ERR;
}


int seaglider_parse_message(seaglider* seaglider_obj,uint8_t* msg)
{
 uint8_t* tmp_ptr;
 for(int i=0;i<SEAGLIDER_MSG_NUM_OF_FUNCTIONS;i++)
 {
	  tmp_ptr=0;
	  tmp_ptr=strstr(msg,seaglider_messages_strings[i]);
	  if(tmp_ptr)
		{
			if(seaglider_functions[i](seaglider_obj,msg)==SEAGLIDER_F_OK) return SEAGLIDER_F_OK;
      else return SEAGLIDER_F_ERR;
		}

 }
 return SEAGLIDER_F_ERR;
}

void seaglider_messages_init(seaglider* seaglider_obj)
{
	seaglider_functions[SEAGLIDER_MSG_DEPTH] = SEAGLIDER_MSG_DEPTH_f;
	seaglider_functions[SEAGLIDER_MSG_STOP] = SEAGLIDER_MSG_STOP_f;
	seaglider_functions[SEAGLIDER_MSG_SEND_TXT_FILE] = SEAGLIDER_MSG_SEND_TXT_FILE_f;
	seaglider_functions[SEAGLIDER_MSG_START] = SEAGLIDER_MSG_START_f;
	seaglider_functions[SEAGLIDER_MSG_SEND_INFO] =SEAGLIDER_MSG_SEND_INFO_f;
	seaglider_functions[SEAGLIDER_MSG_RESET] =SEAGLIDER_MSG_RESET_f;
	seaglider_functions[SEAGLIDER_MSG_TEST] =SEAGLIDER_MSG_TEST_f;
	seaglider_functions[SEAGLIDER_MSG_CLOCK] =SEAGLIDER_MSG_CLOCK_f;
	seaglider_functions[SEAGLIDER_MSG_WAKEUP] =SEAGLIDER_MSG_WAKEUP_f;


	seaglider_messages_strings[SEAGLIDER_MSG_DEPTH] = "DEPTH";
	seaglider_messages_strings[SEAGLIDER_MSG_STOP] = "STOP";
	seaglider_messages_strings[SEAGLIDER_MSG_SEND_TXT_FILE] = "SEND_TXT_FILE";
	seaglider_messages_strings[SEAGLIDER_MSG_START] = "START";
	seaglider_messages_strings[SEAGLIDER_MSG_SEND_INFO] ="SEND_INFO";
	seaglider_messages_strings[SEAGLIDER_MSG_RESET] ="RESET";
	seaglider_messages_strings[SEAGLIDER_MSG_TEST] ="TEST";
	seaglider_messages_strings[SEAGLIDER_MSG_CLOCK] ="CLOCK";
	seaglider_messages_strings[SEAGLIDER_MSG_WAKEUP] ="WAKEUP";




}


int SEAGLIDER_MSG_DEPTH_f(seaglider* seaglider_obj,uint8_t* msg)
{

	char * pch;
	pch = strtok (msg,":");//header
	pch = strtok (NULL,",");//depth
	seaglider_obj->prev_depth=seaglider_obj->last_depth;
	seaglider_obj->last_depth=strtof(pch,NULL);
	pch = strtok (NULL,",");//date
	memcpy(seaglider_obj->date,pch,8);
	pch = strtok (NULL,",");//time
	memcpy(seaglider_obj->time,pch,6);

	osMessagePut(seaglider_obj->events_q,SEAGLIDER_EVNT_DEPTH_RCVD,1);
	return SEAGLIDER_F_OK;
}
int SEAGLIDER_MSG_STOP_f(seaglider* seaglider_obj,uint8_t* msg)
{
	osMessagePut(seaglider_obj->events_q,SEAGLIDER_EVNT_STOP_RCVD,1);
	return SEAGLIDER_F_OK;
}
int SEAGLIDER_MSG_START_f(seaglider* seaglider_obj,uint8_t* msg)
{
	char * pch;
	pch = strtok (msg,":");//header
	pch = strtok (NULL,",");//depth
	seaglider_obj->prev_depth=seaglider_obj->last_depth;
	seaglider_obj->last_depth=strtof(pch,NULL);
	pch = strtok (NULL,",");//date
	memcpy(seaglider_obj->date,pch,8);
	pch = strtok (NULL,",");//time
	memcpy(seaglider_obj->time,pch,6);

	osMessagePut(seaglider_obj->events_q,SEAGLIDER_EVNT_START_RCVD,1);
	return SEAGLIDER_F_OK;
}
int SEAGLIDER_MSG_SEND_TXT_FILE_f(seaglider* seaglider_obj,uint8_t* msg)
{
	osMessagePut(seaglider_obj->events_q,SEAGLIDER_EVNT_SEND_TXT_FILE_RCVD,1);
	return SEAGLIDER_F_OK;
}
int SEAGLIDER_MSG_SEND_INFO_f(seaglider* seaglider_obj,uint8_t* msg)
{
	return SEAGLIDER_F_OK;
}
int SEAGLIDER_MSG_RESET_f(seaglider* seaglider_obj,uint8_t* msg)
{
	return SEAGLIDER_F_OK;
}
int SEAGLIDER_MSG_TEST_f(seaglider* seaglider_obj,uint8_t* msg)
{
	osMessagePut(seaglider_obj->events_q,SEAGLIDER_EVNT_TEST_RCVD,1);
	return SEAGLIDER_F_OK;
}
int SEAGLIDER_MSG_CLOCK_f(seaglider* seaglider_obj,uint8_t* msg)
{
	char * pch;
	pch = strtok (msg," :");//header
	pch = strtok (NULL,",:");//date
	memcpy(seaglider_obj->date,pch,8);
	pch = strtok (NULL,",:");//time
	memcpy(seaglider_obj->time,pch,6);

	osMessagePut(seaglider_obj->events_q,SEAGLIDER_EVNT_CLOCK_RCVD,1);
	return SEAGLIDER_F_OK;
}

int SEAGLIDER_MSG_WAKEUP_f(seaglider* seaglider_obj,uint8_t* msg)
{
	osMessagePut(seaglider_obj->events_q,SEAGLIDER_EVNT_WAKEUP_RCVD,1);
	return SEAGLIDER_F_OK;
}
