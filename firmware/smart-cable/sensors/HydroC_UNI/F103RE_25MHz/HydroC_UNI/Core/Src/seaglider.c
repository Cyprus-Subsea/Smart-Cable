/*
 * seaglider.c
 *
 *  Created on: 12 апр. 2021 г.
 *      Author: admin
 */
#include "main.h"
#include "seaglider.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"

int ((*seaglider_functions[SEAGLIDER_MSG_NUM_OF_FUNCTIONS]))(seaglider* seaglider_obj,uint8_t* msg);
char*  seaglider_messages_strings[SEAGLIDER_MSG_NUM_OF_FUNCTIONS];


const char* seaglider_commands_strings[]={"CD>\r","DBG:"};


void seaglider_init(seaglider* seaglider_obj,osMessageQId events_q_Handle,osSemaphoreId out_q_sem)
{
	seaglider_obj->last_depth=0.0;
	seaglider_obj->prev_depth=0.0;
	seaglider_obj->dive_status=SEAGLIDER_STATUS_UNKNOWN;

	seaglider_obj->stop_pump_flag=SEAGLIDER_STOP_PUMP_FLAG_DEACTIVATED;
	seaglider_messages_init(seaglider_obj);

	seaglider_obj->events_q = events_q_Handle;
	seaglider_obj->out_q_sem=out_q_sem;

	//osMessageQDef(seaglider_events_q, 20, uint8_t);
	//seaglider_obj->events_q= osMessageCreate(osMessageQ(seaglider_events_q), NULL);

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

void seaglider_send_evnt(seaglider* seaglider_obj,uint32_t event_id)
{
  char* snd_buffer=(char*)malloc(SEAGLIDER_SND_BUFFER_SIZE);
  sprintf( snd_buffer,"%s%u",
		    seaglider_commands_strings[SEAGLIDER_MSG_EVNT],
			event_id
			);
  seaglider_schedule_for_tx(seaglider_obj,(uint8_t*)snd_buffer,strlen(snd_buffer));
  free(snd_buffer);
}

void seaglider_send_cmd(seaglider* seaglider_obj,uint8_t cmd_id,void* arg)
{
 memory_region_pointer* ptr1;
 switch(cmd_id)
 {
     case SEAGLIDER_CMD_PROMPT:
       seaglider_schedule_for_tx(seaglider_obj,(uint8_t*)seaglider_commands_strings[SEAGLIDER_MSG_PROMPT],strlen(seaglider_commands_strings[SEAGLIDER_MSG_PROMPT]));
	 break;
     case SEAGLIDER_CMD_SEND_DATA:
       ptr1=arg;
       seaglider_schedule_for_tx(seaglider_obj,(uint8_t*)ptr1->start_addr,ptr1->size);
     break;
 }
}

void seaglider_schedule_for_tx(seaglider* seaglider_obj,uint8_t* message,uint32_t size)
{
 osSemaphoreWait(seaglider_obj->out_q_sem,osWaitForever);
 for(int i=0;i<size;i++)
 {
   osMessagePut(seaglider_obj->media_tx_q,*(message+i),osWaitForever);
 }
 osSemaphoreRelease(seaglider_obj->out_q_sem);
}

uint8_t seaglider_get_event(seaglider* seaglider_obj,uint8_t* event)
{
 if(xQueueReceive(seaglider_obj->events_q,event,1)){
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
	seaglider_functions[SEAGLIDER_MSG_CLEAR] =SEAGLIDER_MSG_CLEAR_f;
	seaglider_functions[SEAGLIDER_MSG_POFF] =SEAGLIDER_MSG_POFF_f;
	seaglider_functions[SEAGLIDER_MSG_ERRORS] =SEAGLIDER_MSG_ERRORS_f;

	seaglider_messages_strings[SEAGLIDER_MSG_DEPTH] = "DEPTH";
	seaglider_messages_strings[SEAGLIDER_MSG_STOP] = "STOP";
	seaglider_messages_strings[SEAGLIDER_MSG_SEND_TXT_FILE] = "SEND_TXT_FILE";
	seaglider_messages_strings[SEAGLIDER_MSG_START] = "START";
	seaglider_messages_strings[SEAGLIDER_MSG_SEND_INFO] ="SEND_INFO";
	seaglider_messages_strings[SEAGLIDER_MSG_RESET] ="RESET";
	seaglider_messages_strings[SEAGLIDER_MSG_TEST] ="TEST";
	seaglider_messages_strings[SEAGLIDER_MSG_CLOCK] ="CLOCK";
	seaglider_messages_strings[SEAGLIDER_MSG_WAKEUP] ="WAKEUP";
	seaglider_messages_strings[SEAGLIDER_MSG_CLEAR] ="CLEAR";
	seaglider_messages_strings[SEAGLIDER_MSG_POFF] ="POFF";
	seaglider_messages_strings[SEAGLIDER_MSG_ERRORS] ="ERRORS";
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
	char * pch;
	pch = strtok (msg,":");//header
	pch = strtok (NULL,",");//dive-climb
	if(*pch=='a'){
	  seaglider_obj->dive_status=SEAGLIDER_STATUS_DIVE;
	}
	else if(*pch=='b'){
	  seaglider_obj->dive_status=SEAGLIDER_STATUS_CLIMB;
	}
	osMessagePut(seaglider_obj->events_q,SEAGLIDER_EVNT_STOP_RCVD,1);
	return SEAGLIDER_F_OK;
}
int SEAGLIDER_MSG_START_f(seaglider* seaglider_obj,uint8_t* msg)
{
	char * pch;
	pch = strtok (msg,":");//header
	pch = strtok (NULL,",");//param_x  pump stop after zero
	seaglider_obj->stop_pump_flag=strtol(pch,NULL,10);
	pch = strtok (NULL,",");//param_y  zero after dive climb
	seaglider_obj->param_y=strtol(pch,NULL,10);
	pch = strtok (NULL,",");//param_z  profile id
	seaglider_obj->param_z=strtol(pch,NULL,10);
	pch = strtok (NULL,",");//dive-climb
	if(*pch=='a'){
	  seaglider_obj->dive_status=SEAGLIDER_STATUS_DIVE;
	}
	else if(*pch=='b'){
	  seaglider_obj->dive_status=SEAGLIDER_STATUS_CLIMB;
	}

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
	pch = strtok (msg,":");//header
	pch = strtok (NULL,":");//date
	memcpy(seaglider_obj->date,pch,8);
	pch = strtok (NULL,":");//time
	memcpy(seaglider_obj->time,pch,8);

	osMessagePut(seaglider_obj->events_q,SEAGLIDER_EVNT_CLOCK_RCVD,1);
	return SEAGLIDER_F_OK;
}

int SEAGLIDER_MSG_WAKEUP_f(seaglider* seaglider_obj,uint8_t* msg)
{
	osMessagePut(seaglider_obj->events_q,SEAGLIDER_EVNT_WAKEUP_RCVD,1);
	return SEAGLIDER_F_OK;
}

int SEAGLIDER_MSG_CLEAR_f(seaglider* seaglider_obj,uint8_t* msg)
{
	osMessagePut(seaglider_obj->events_q,SEAGLIDER_EVNT_CLEAR_RCVD,1);
	return SEAGLIDER_F_OK;
}

int SEAGLIDER_MSG_POFF_f(seaglider* seaglider_obj,uint8_t* msg)
{
	osMessagePut(seaglider_obj->events_q,SEAGLIDER_EVNT_POFF_RCVD,1);
	return SEAGLIDER_F_OK;
}
int SEAGLIDER_MSG_ERRORS_f(seaglider* seaglider_obj,uint8_t* msg)
{
	osMessagePut(seaglider_obj->events_q,SEAGLIDER_EVNT_ERRORS_RCVD,1);
	return SEAGLIDER_F_OK;
}
