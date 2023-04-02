/*
 * slocum.c
 *
 *  Created on: 12 апр. 2021 г.
 *      Author: admin
 */


#include <SLOCUM_g3.h>
#include "string.h"
#include "stdlib.h"
#include "base64.h"

//debug
#include "main.h"
//extern UART_HandleTypeDef huart5;
//HAL_UART_Transmit(&huart1,msg,strlen(msg),100);

int ((*slocum_functions[SLOCUM_MSG_NUM_OF_FUNCTIONS]))(slocum* slocum_obj,uint8_t* msg);
char*  slocum_messages_strings[SLOCUM_MSG_NUM_OF_FUNCTIONS];


const char* slocum_commands_strings[]={"UV>","$FW,","\r\n","$FO,","$FC"};
char base64_buffer[SLOCUM_BASE64_BUFFER_SIZE];
uint32_t base64_buffer_size;
char snd_buffer[SLOCUM_SND_BUFFER_SIZE];


void slocum_init(slocum* slocum_obj)
{
	slocum_obj->last_depth=0.0;
	slocum_obj->prev_depth=0.0;
	slocum_obj->behavior_state=GLIDER_BEHAVIOR_NONE;
	slocum_obj->timestamp=0;
	slocum_obj->fsm_status=GLIDER_FSM_NONE;

	slocum_messages_init(slocum_obj);


	osMessageQDef(SLOCUM_events_q, 20, uint8_t);
	slocum_obj->events_q= osMessageCreate(osMessageQ(SLOCUM_events_q), NULL);

	osMessageQDef(SLOCUM_media_rx_q, 20, uint16_t);
	slocum_obj->media_rx_messages_q= osMessageCreate(osMessageQ(SLOCUM_media_rx_q), NULL);

	osMessageQDef(SLOCUM_media_tx_q, 200, uint8_t);
	slocum_obj->media_tx_q = osMessageCreate(osMessageQ(SLOCUM_media_tx_q), NULL);

	osThreadDef(SLOCUM_task, slocum_loop, osPriorityNormal, 0, 512);
	osThreadCreate(osThread(SLOCUM_task), slocum_obj);


}


void slocum_loop(slocum* slocum_obj)
{
	 uint16_t msg_indx;
	 for(;;)
	 {
		//test loopback
		if(xQueueReceive(slocum_obj->media_rx_messages_q,&msg_indx,0))
		{   uint8_t* msg=slocum_obj->rx_buffer+msg_indx;
		    slocum_parse_message(slocum_obj,msg);
		}
		//osDelay(1);
	 }

}


void slocum_media_process_byte(slocum* slocum_obj,uint8_t rx_byte)
{
	if(slocum_obj->media_status==SLOCUM_MEDIA_READY && rx_byte!=0x00)
	{
		slocum_obj->rx_buffer[slocum_obj->rx_buffer_indx]=rx_byte;
		if(rx_byte=='\r')
		{
			slocum_obj->rx_buffer[slocum_obj->rx_buffer_indx]=0x00;
			osMessagePut(slocum_obj->media_rx_messages_q,slocum_obj->rx_buffer_new_string_indx,1);
			if(slocum_obj->rx_buffer_indx>SLOCUM_RX_BUFFER_THR) slocum_obj->rx_buffer_indx=0;
			else  slocum_obj->rx_buffer_indx++;
			slocum_obj->rx_buffer_new_string_indx=slocum_obj->rx_buffer_indx;
		}
		else slocum_obj->rx_buffer_indx++;
		if(slocum_obj->rx_buffer_indx==SLOCUM_RX_BUFFER_SIZE)
		{
			slocum_obj->rx_buffer_indx=0;
			slocum_obj->rx_buffer_new_string_indx=slocum_obj->rx_buffer_indx;
		}
	}
	else
	{
		slocum_obj->rx_buffer_indx=0;
		slocum_obj->rx_buffer_new_string_indx=slocum_obj->rx_buffer_indx;
	}
}

uint8_t slocum_media_get_byte(slocum* slocum_obj,uint8_t* tx_byte)
{
	osEvent res=osMessageGet(slocum_obj->media_tx_q,0);
	if(res.status==osEventMessage)
    {
		*tx_byte=res.value.v;
		return SLOCUM_F_OK;
    }
  return SLOCUM_F_ERR;
}


void slocum_send_cmd(slocum* slocum_obj,uint8_t cmd_id,void* arg)
{
 memory_region_pointer* ptr1;
 uint8_t xor;

 switch(cmd_id)
 {
     case SLOCUM_CMD_PROMPT:
  	   for(int i=0;i<strlen(slocum_commands_strings[SLOCUM_CMD_PROMPT]);i++)
  	   {
  		   osMessagePut(slocum_obj->media_tx_q,*(slocum_commands_strings[SLOCUM_CMD_PROMPT]+i),osWaitForever);
  	   }

       for(int i=0;i<strlen(slocum_commands_strings[SLOCUM_MSG_EOL]);i++)
       {
    	   osMessagePut(slocum_obj->media_tx_q,*(slocum_commands_strings[SLOCUM_MSG_EOL]+i),osWaitForever);
       }
	 break;

     case SLOCUM_CMD_SEND_DATA:
       ptr1=arg;
       for(int i=0;i<ptr1->size;i++)
	   {
		   osMessagePut(slocum_obj->media_tx_q,*((uint8_t*)(ptr1->start_addr)+i),osWaitForever);
	   }
     break;

     case SLOCUM_CMD_OPEN_FILE_W:
       ptr1=arg;

       sprintf(snd_buffer,"%s%s",
    		    slocum_commands_strings[SLOCUM_MSG_OPEN_FILE_W],
				ptr1->start_addr
				);
       xor=calc_XOR(snd_buffer+1,strlen(snd_buffer+1));
       sprintf(snd_buffer,"%s*%02x%s",
    		    snd_buffer,
				xor,
				slocum_commands_strings[SLOCUM_MSG_EOL]);

       for(int i=0;i<strlen(snd_buffer);i++)
       {
    	   osMessagePut(slocum_obj->media_tx_q,*(snd_buffer+i),osWaitForever);
       }
     break;

     case SLOCUM_CMD_CLOSE_FILE:

       sprintf(snd_buffer,"%s",
    		    slocum_commands_strings[SLOCUM_MSG_CLOSE_FILE]
				);
       xor=calc_XOR(snd_buffer+1,strlen(snd_buffer+1));
       sprintf(snd_buffer,"%s*%02x%s",
    		    snd_buffer,
				xor,
				slocum_commands_strings[SLOCUM_MSG_EOL]);

       for(int i=0;i<strlen(snd_buffer);i++)
       {
    	   osMessagePut(slocum_obj->media_tx_q,*(snd_buffer+i),osWaitForever);
       }
     break;

     case SLOCUM_CMD_WRITE_FILE_DATA:
       ptr1=arg;
       base64encode((char*)ptr1->start_addr,(uint32_t)ptr1->size,base64_buffer,SLOCUM_BASE64_BUFFER_SIZE);
       sprintf( snd_buffer,"%s%s",
    		    slocum_commands_strings[SLOCUM_MSG_SLOCUM_CMD_WRITE_FILE_DATA],
				base64_buffer
				//ptr1->start_addr
				);
       xor=calc_XOR(snd_buffer+1,strlen(snd_buffer+1));
       sprintf( snd_buffer,"%s*%02x%s",
    		    snd_buffer,
				xor,
				slocum_commands_strings[SLOCUM_MSG_EOL]);

       for(int i=0;i<strlen(snd_buffer);i++)
       {
    	   osMessagePut(slocum_obj->media_tx_q,*(snd_buffer+i),osWaitForever);
       }
     break;
 }

}

uint8_t slocum_get_event(slocum* slocum_obj,uint8_t* event)
{
	if(xQueueReceive(slocum_obj->events_q,event,1))
	{
     return SLOCUM_F_OK;
	}
	return SLOCUM_F_ERR;
}


int slocum_parse_message(slocum* slocum_obj,uint8_t* msg)
{
 uint8_t* tmp_ptr;
 for(int i=0;i<SLOCUM_MSG_NUM_OF_FUNCTIONS;i++)
 {
	  tmp_ptr=0;
	  tmp_ptr=strstr(msg,slocum_messages_strings[i]);
	  if(tmp_ptr)
		{
			if(slocum_functions[i](slocum_obj,msg)==SLOCUM_F_OK) return SLOCUM_F_OK;
      else return SLOCUM_F_ERR;
		}
 }
 return SLOCUM_F_ERR;
}

void slocum_messages_init(slocum* slocum_obj)
{
	slocum_functions[SLOCUM_MSG_SD] = SLOCUM_MSG_SD_f;
	slocum_messages_strings[SLOCUM_MSG_SD] = "$SD";
}

int SLOCUM_MSG_SD_f(slocum* slocum_obj,uint8_t* msg)
{
	char* pch;
	char* params_str[20];
	uint8_t crc;

	char tt[30];
	uint32_t ssd;

	pch = strtok (msg,"*");//msg
	msg=(uint8_t*)pch;

	pch = strtok (NULL,",");//crc
	crc=*pch;

	uint8_t msg_xor=calc_XOR(msg+1,strlen(msg+1));
	uint8_t e_xor=strtol(crc,NULL,16);
	//if(msg_xor!=e_xor) return;

	pch = strtok (msg,",");//header

	memset(params_str,0,20);
	for(int i=0;i<20;i++){
	 pch = strtok (NULL,",");
	 if(pch) params_str[i]=pch;
	 else break;
	}

	for(int i=0;i<20;i++){
	 if(params_str[i]){
	  pch = strtok (params_str[i],":");
	  if(*pch=='3'){
		  pch = strtok (NULL,",");
		  if(pch){
			slocum_obj->prev_depth=slocum_obj->last_depth;
			slocum_obj->last_depth=strtof(pch,NULL);
			osMessagePut(slocum_obj->events_q,SLOCUM_EVNT_DEPTH_RCVD,1);
		  }
	  }
	  else if(*pch=='4'){
		  pch = strtok (NULL,",");
		  if(pch){
		    slocum_obj->behavior_state=strtol(pch,NULL,10);
			osMessagePut(slocum_obj->events_q,SLOCUM_EVNT_STATUS_RCVD,1);
		  }
	  }
	  else if(*pch=='2'){
		  pch = strtok (NULL,".");
		  if(pch){
			slocum_obj->timestamp=(uint32_t)strtol(pch,NULL,10);
  			osMessagePut(slocum_obj->events_q,SLOCUM_EVNT_CLK_RCVD,1);
		  }
	  }
	  else if(*pch=='5'){
		  pch = strtok (NULL,",");
		  if(pch){
			strcpy(slocum_obj->mission_id,pch);
  			osMessagePut(slocum_obj->events_q,SLOCUM_EVNT_MISSION_ID__RCVD,1);
		  }
	  }
	 }
	 else break;
	}
	return SLOCUM_F_OK;
}

