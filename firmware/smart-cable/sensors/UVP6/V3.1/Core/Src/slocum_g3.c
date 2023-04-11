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

#include "disp_proc.h"
#include "fsm.h"
//extern UART_HandleTypeDef huart5;
//HAL_UART_Transmit(&huart1,msg,strlen(msg),100);

int ((*slocum_functions[SLOCUM_MSG_NUM_OF_FUNCTIONS]))(slocum* slocum_obj,uint8_t* msg);
char*  slocum_messages_strings[SLOCUM_MSG_NUM_OF_FUNCTIONS];


const char* slocum_commands_strings[]={"UV>","$FW,","\r\n","$FO,","$FC","$SW,0:"};



void slocum_init(slocum* slocum_obj,osMessageQId events_q_Handle,osSemaphoreId out_q_sem)
{
	slocum_obj->last_depth=0.0;
	slocum_obj->prev_depth=0.0;
	slocum_obj->final_depth_state=GLIDER_FINAL_DEPTH_AT_SURFACE;
	slocum_obj->timestamp=0;
	slocum_obj->dive_climb_counter=0;
	slocum_obj->dcs_state=GLIDER_STATE_AT_SURFACE;
	slocum_obj->x_glider_dos=SLOCUM_X_GLIDER_DOS_ON;
	slocum_messages_init(slocum_obj);

	//osMessageQDef(SLOCUM_events_q, 20, uint8_t);
	//slocum_obj->events_q= osMessageCreate(osMessageQ(SLOCUM_events_q), NULL);
	slocum_obj->events_q = events_q_Handle;
	slocum_obj->out_q_sem=out_q_sem;

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
  if(res.status==osEventMessage){
	*tx_byte=res.value.v;
	return SLOCUM_F_OK;
  }
  return SLOCUM_F_ERR;
}

void slocum_send_evnt(slocum* slocum_obj,uint32_t evnt)
{
  uint8_t xor;

  char* snd_buffer=malloc(SLOCUM_SND_BUFFER_SIZE);
  sprintf( snd_buffer,"%s%u",
		    slocum_commands_strings[SLOCUM_MSG_SEND_EVNT],
			evnt
			);
  xor=calc_XOR(snd_buffer+1,strlen(snd_buffer+1));
  sprintf( snd_buffer,"%s*%02x%s",
		    snd_buffer,
			xor,
			slocum_commands_strings[SLOCUM_MSG_EOL]);
  slocum_schedule_for_tx(slocum_obj,(uint8_t*)snd_buffer,strlen(snd_buffer));
  free(snd_buffer);
}

void slocum_send_cmd(slocum* slocum_obj,uint8_t cmd_id,void* arg)
{
 memory_region_pointer* ptr1;
 uint8_t xor;
 char* base64_buffer=malloc(SLOCUM_BASE64_BUFFER_SIZE);
 char* snd_buffer=malloc(SLOCUM_SND_BUFFER_SIZE);
 uint32_t base64_buffer_size;

 switch(cmd_id)
 {
     case SLOCUM_CMD_PROMPT:
    	 slocum_schedule_for_tx(slocum_obj,(uint8_t*)slocum_commands_strings[SLOCUM_CMD_PROMPT],strlen(slocum_commands_strings[SLOCUM_CMD_PROMPT]));
    	 slocum_schedule_for_tx(slocum_obj,(uint8_t*)slocum_commands_strings[SLOCUM_MSG_EOL],strlen(slocum_commands_strings[SLOCUM_MSG_EOL]));
	 break;

     case SLOCUM_CMD_SEND_DATA:
         ptr1=arg;
  	     slocum_schedule_for_tx(slocum_obj,(uint8_t*)ptr1->start_addr,ptr1->size);
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

  	   slocum_schedule_for_tx(slocum_obj,(uint8_t*)snd_buffer,strlen(snd_buffer));
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

       slocum_schedule_for_tx(slocum_obj,(uint8_t*)snd_buffer,strlen(snd_buffer));
     break;

     case SLOCUM_CMD_WRITE_FILE_DATA:
       ptr1=arg;
       uint32_t len;
       uint32_t size=(uint32_t)ptr1->size;
       char* start_addr=(char*)ptr1->start_addr;
       for(int i=0;size>0;i++)
       {
         start_addr=start_addr+(i*SLOCUM_FILE_WRITE_LEN);
    	 if(size>SLOCUM_FILE_WRITE_LEN){
    		 len=SLOCUM_FILE_WRITE_LEN;
    	 }
    	 else{
    		 len=size;
    	 }

    	 size=size-len;
    	 base64encode(start_addr,len,base64_buffer,SLOCUM_BASE64_BUFFER_SIZE);
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
         slocum_schedule_for_tx(slocum_obj,(uint8_t*)snd_buffer,strlen(snd_buffer));

       }
     break;

 }
  free(snd_buffer);
  free(base64_buffer);
}

void slocum_schedule_for_tx(slocum* slocum_obj,uint8_t* message,uint32_t size)
{
 osSemaphoreWait(slocum_obj->out_q_sem,osWaitForever);
 for(int i=0;i<size;i++)
 {
   osMessagePut(slocum_obj->media_tx_q,*(message+i),osWaitForever);
 }
 osSemaphoreRelease(slocum_obj->out_q_sem);
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

	  if(strcmp(pch,"2")==0){
		  pch = strtok (NULL,".");
		  if(pch){
			slocum_obj->timestamp=(uint32_t)strtol(pch,NULL,10);
  			osMessagePut(slocum_obj->events_q,SLOCUM_EVNT_CLK_RCVD,1);
		  }
	  }
	  else if(strcmp(pch,"3")==0){
		  pch = strtok (NULL,",");
		  if(pch){
			slocum_obj->prev_depth=slocum_obj->last_depth;
			slocum_obj->last_depth=strtof(pch,NULL);
			osMessagePut(slocum_obj->events_q,SLOCUM_EVNT_DEPTH_RCVD,1);
		  }
	  }
	  else if(strcmp(pch,"4")==0){
		  pch = strtok (NULL,",");
		  if(pch){
			uint32_t final_depth_state=strtol(pch,NULL,10);
			if(final_depth_state==GLIDER_FINAL_DEPTH_AT_SURFACE
					               ||final_depth_state==GLIDER_FINAL_DEPTH_DIVINIG
								   ||final_depth_state==GLIDER_FINAL_DEPTH_CLIMBING){
		       slocum_obj->final_depth_state=final_depth_state;
		       if(slocum_obj->final_depth_state==GLIDER_FINAL_DEPTH_AT_SURFACE)
		    	                osMessagePut(slocum_obj->events_q,SLOCUM_EVNT_FINAL_DEPTH_AT_SURFACE_RCVD,1);
		       else if(slocum_obj->final_depth_state==GLIDER_FINAL_DEPTH_DIVINIG)
		    	                osMessagePut(slocum_obj->events_q,SLOCUM_EVNT_FINAL_DEPTH_DIVING_RCVD,1);
		       else if(slocum_obj->final_depth_state==GLIDER_FINAL_DEPTH_CLIMBING)
		    	                osMessagePut(slocum_obj->events_q,SLOCUM_EVNT_FINAL_DEPTH_CLIMBING_RCVD,1);
		    }
		  }
	  }

	  else if(strcmp(pch,"5")==0){
		  pch = strtok (NULL,",");
		  if(pch){
			strcpy(slocum_obj->mission_id,pch);
  			osMessagePut(slocum_obj->events_q,SLOCUM_EVNT_MISSION_ID_RCVD,1);
		  }
	  }
	  else if(strcmp(pch,"6")==0){
		  pch = strtok (NULL,",");
		  if(pch){
			slocum_obj->gps_lon=strtof(pch,NULL);
			osMessagePut(slocum_obj->events_q,SLOCUM_EVNT_GPS_LON_RCVD,1);
		  }
	  }
	  else if(strcmp(pch,"7")==0){
		  pch = strtok (NULL,",");
		  if(pch){
			slocum_obj->gps_lat=strtof(pch,NULL);
			osMessagePut(slocum_obj->events_q,SLOCUM_EVNT_GPS_LAT_RCVD,1);
		  }
	  }
	  else if(strcmp(pch,"8")==0){
		  pch = strtok (NULL,",");
		  if(pch){
			slocum_obj->dive_target_depth=strtof(pch,NULL);
			osMessagePut(slocum_obj->events_q,SLOCUM_EVNT_DIVE_TARGET_DEPTH_RCVD,1);
		  }
	  }
	  else if(strcmp(pch,"9")==0){
		  pch = strtok (NULL,",");
		  if(pch){
			uint32_t new_power_status=strtol(pch,NULL,10);
			if(new_power_status==UVP6_USER_PARAM_POWER_ON||new_power_status==UVP6_USER_PARAM_POWER_OFF)
			{   slocum_obj->user_param_power_ctrl=new_power_status;
				if(slocum_obj->user_param_power_ctrl==UVP6_USER_PARAM_POWER_ON)osMessagePut(slocum_obj->events_q,SLOCUM_EVNT_USER_PARAM_PWR_ON_RCVD,1);
				else if(slocum_obj->user_param_power_ctrl==UVP6_USER_PARAM_POWER_OFF)osMessagePut(slocum_obj->events_q,SLOCUM_EVNT_USER_PARAM_PWR_OFF_RCVD,1);
			}
		  }
	  }
	  else if(strcmp(pch,"10")==0){
		  pch = strtok (NULL,",");
		  if(pch){
			slocum_obj->user_param_min_depth=strtof(pch,NULL);
			osMessagePut(slocum_obj->events_q,SLOCUM_EVNT_USER_PARAM_MIN_DEPTH_RCVD,1);
		  }
	  }
	  else if(strcmp(pch,"11")==0){
		  pch = strtok (NULL,",");
		  if(pch){
			uint32_t x_glider_dos=strtol(pch,NULL,10);
			if(x_glider_dos==SLOCUM_X_GLIDER_DOS_ON||x_glider_dos==SLOCUM_X_GLIDER_DOS_OFF){
                  slocum_obj->x_glider_dos=x_glider_dos;
				  if(slocum_obj->x_glider_dos==SLOCUM_X_GLIDER_DOS_ON)osMessagePut(slocum_obj->events_q,SLOCUM_EVNT_DOS_ON,1);
				  else if(slocum_obj->x_glider_dos==SLOCUM_X_GLIDER_DOS_OFF)osMessagePut(slocum_obj->events_q,SLOCUM_EVNT_DOS_OFF,1);
			}
		   }
	  }

	 }
	 else break;
	}
	return SLOCUM_F_OK;
}
