/*
 * UVP6.c
 *
 *  Created on: 12 апр. 2021 г.
 *      Author: admin
 */

#include "UVP6.h"
#include "string.h"
#include "main.h"
#include "stdlib.h"


//debug
extern UART_HandleTypeDef huart1;
//HAL_UART_Transmit(&huart1,msg,strlen(msg),100);

//$start:ACQ_CSCS_002H,20211404,120000;\n

int (*uvp6_functions[UVP6_MSG_NUM_OF_FUNCTIONS])(uvp6* uvp6_obj,uint8_t* msg);
char*  uvp6_messages_strings[UVP6_MSG_NUM_OF_FUNCTIONS];
const char* uvp6_commands_strings[]={"$start:ACQ_CSCS_002H","$stop;\n","$stop;\n","$start:ACQ_CSCS_052L"};

void uvp6_init(uvp6* uvp6_obj,osMessageQId events_q_Handle)
{
	uvp6_messages_init(uvp6_obj);

	uvp6_obj->power_status=UVP6_POWER_IS_UNKNOWN;

	//osMessageQDef(uvp6_events_q, 20, uint8_t);
	//uvp6_obj->events_q= osMessageCreate(osMessageQ(uvp6_events_q), NULL);

	uvp6_obj->events_q = events_q_Handle;

	osMessageQDef(uvp6_media_rx_q, 20, uint16_t);
	uvp6_obj->media_rx_messages_q= osMessageCreate(osMessageQ(uvp6_media_rx_q), NULL);

	osMessageQDef(uvp6_media_tx_q, 400, uint8_t);
	uvp6_obj->media_tx_q = osMessageCreate(osMessageQ(uvp6_media_tx_q), NULL);

	osThreadDef(uvp6_task, uvp6_loop, osPriorityNormal, 0, 512);
	osThreadCreate(osThread(uvp6_task), uvp6_obj);
}

void uvp6_media_process_byte(uvp6* uvp6_obj,uint8_t rx_byte)
{
	if(uvp6_obj->media_status==UVP6_MEDIA_READY)
	{
		uvp6_obj->rx_buffer[uvp6_obj->rx_buffer_indx]=rx_byte;
		if(rx_byte=='\n')
		{
			uvp6_obj->rx_buffer[uvp6_obj->rx_buffer_indx]=0x00;
			osMessagePut(uvp6_obj->media_rx_messages_q,uvp6_obj->rx_buffer_new_string_indx,1);
			if(uvp6_obj->rx_buffer_indx>UVP6_RX_BUFFER_THR) uvp6_obj->rx_buffer_indx=0;
			else  uvp6_obj->rx_buffer_indx++;
			uvp6_obj->rx_buffer_new_string_indx=uvp6_obj->rx_buffer_indx;
		}
		else uvp6_obj->rx_buffer_indx++;
		if(uvp6_obj->rx_buffer_indx==UVP6_RX_BUFFER_SIZE)
		{
			uvp6_obj->rx_buffer_indx=0;
			uvp6_obj->rx_buffer_new_string_indx=uvp6_obj->rx_buffer_indx;
		}
	}
	else
	{
		uvp6_obj->rx_buffer_indx=0;
		uvp6_obj->rx_buffer_new_string_indx=uvp6_obj->rx_buffer_indx;
	}
}

void uvp6_loop(uvp6* uvp6_obj)
{
 uint16_t msg_indx;
 for(;;)
 {
	if(xQueueReceive(uvp6_obj->media_rx_messages_q,&msg_indx,0))
	{
		uint8_t* msg=uvp6_obj->rx_buffer+msg_indx;
		uvp6_parse_message(uvp6_obj,msg);
	}
	osDelay(1);
 }

}


uint8_t uvp6_media_get_byte(uvp6* uvp6_obj,uint8_t* tx_byte)
{
	osEvent res=osMessageGet(uvp6_obj->media_tx_q,0);
	if(res.status==osEventMessage)
     {
		*tx_byte=res.value.v;
		return UVP6_F_OK;
     }
   return UVP6_F_ERR;
}


void uvp6_send_cmd(uvp6* uvp6_obj,uint8_t cmd_id,void* arg)
{
 char tmp_cmd[40];
 switch(cmd_id)
 {
     case UVP6_CMD_START_H_ACQ:
       tmp_cmd[0]=0x00;
       strcat(tmp_cmd,uvp6_commands_strings[UVP6_CMD_START_H_ACQ]);
       strcat(tmp_cmd,(char*)arg);
       strcat(tmp_cmd,";\n");
  	   for(int i=0;i<strlen(tmp_cmd);i++)
  	   {
  		   osMessagePut(uvp6_obj->media_tx_q,tmp_cmd[i],0);
  	   }

	 break;
     case UVP6_CMD_START_L_ACQ:
       tmp_cmd[0]=0x00;
       strcat(tmp_cmd,uvp6_commands_strings[UVP6_CMD_START_L_ACQ]);
       strcat(tmp_cmd,(char*)arg);
       strcat(tmp_cmd,";\n");
  	   for(int i=0;i<strlen(tmp_cmd);i++)
  	   {
  		   osMessagePut(uvp6_obj->media_tx_q,tmp_cmd[i],0);
  	   }

	 break;
     case UVP6_CMD_STOP_ACQ:
	   for(int i=0;i<strlen(uvp6_commands_strings[UVP6_CMD_STOP_ACQ]);i++)
	   {
		   osMessagePut(uvp6_obj->media_tx_q,*(uvp6_commands_strings[UVP6_CMD_STOP_ACQ]+i),0);
	   }

	 break;
 }

}


uint8_t uvp6_get_event(uvp6* uvp6_obj,uint8_t* event)
{
	if(xQueueReceive(uvp6_obj->events_q,event,1))
	{
     return UVP6_F_OK;
	}
	return UVP6_F_ERR;
}



int uvp6_parse_message(uvp6* uvp6_obj,uint8_t* msg)
{
	char * pch;
	pch = strtok (msg,":,;");//header
	uint8_t* tmp_ptr;
	for(int i=0;i<UVP6_MSG_NUM_OF_FUNCTIONS;i++)
	{
	   if(strlen(pch)==strlen(uvp6_messages_strings[i]))
	   {
		  tmp_ptr=0;
		  tmp_ptr=strstr(pch,uvp6_messages_strings[i]);
		  if(tmp_ptr)
		   {
			 if(uvp6_functions[i](uvp6_obj,msg+strlen(tmp_ptr)+1)==UVP6_F_OK) return UVP6_F_OK;
	         else return UVP6_F_ERR;
		   }
		}
	 }
	 return UVP6_F_ERR;

}

void uvp6_messages_init(uvp6* uvp6_obj)
{

	uvp6_functions[UVP6_MSG_HW_CONF] = UVP6_MSG_HW_CONF_f;
	uvp6_functions[UVP6_MSG_BLACK_DATA] = UVP6_MSG_BLACK_DATA_f;
	uvp6_functions[UVP6_MSG_LPM_DATA] = UVP6_MSG_LPM_DATA_f;
	uvp6_functions[UVP6_MSG_ACQ_CONF] = UVP6_MSG_ACQ_CONF_f;
	uvp6_functions[UVP6_MSG_START_ACK] = UVP6_MSG_START_ACK_f;
	uvp6_functions[UVP6_MSG_START_ERR] = UVP6_MSG_START_ERR_f;
	uvp6_functions[UVP6_MSG_STOP_ACK] = UVP6_MSG_STOP_ACK_f;

	uvp6_messages_strings[UVP6_MSG_HW_CONF] = "HW_CONF";
	uvp6_messages_strings[UVP6_MSG_BLACK_DATA] = "BLACK_DATA";
	uvp6_messages_strings[UVP6_MSG_LPM_DATA] = "LPM_DATA";
	uvp6_messages_strings[UVP6_MSG_ACQ_CONF] = "ACQ_CONF";
	uvp6_messages_strings[UVP6_MSG_START_ACK] = "$startack";
	uvp6_messages_strings[UVP6_MSG_START_ERR] = "$starterr";
	uvp6_messages_strings[UVP6_MSG_STOP_ACK] = "$stopack";
}


int UVP6_MSG_HW_CONF_f(uvp6* uvp6_obj,uint8_t* msg)
{
   osMessagePut(uvp6_obj->events_q,UVP6_EVNT_BOOTED,1);
   return UVP6_F_OK;
}
int UVP6_MSG_BLACK_DATA_f(uvp6* uvp6_obj,uint8_t* msg)
{
	return UVP6_F_OK;
}
int UVP6_MSG_LPM_DATA_f(uvp6* uvp6_obj,uint8_t* msg)
{
	char * pch;
	pch = strtok (msg,",");//pressure
	uvp6_obj->lpm_data.pressure=strtof(pch,NULL);
	pch = strtok (NULL,",");//date
	strcpy(uvp6_obj->lpm_data.date,pch);
	pch = strtok (NULL,",");//time
	strcpy(uvp6_obj->lpm_data.time,pch);
	pch = strtok (NULL,",");//num_of_images
	uvp6_obj->lpm_data.number_of_images=strtoul(pch,NULL,0);
	pch = strtok (NULL,",");//temperature
	uvp6_obj->lpm_data.temperature=strtof(pch,NULL);
	//data
	for(int i=0;i<UVP6_NUM_OF_CATEGORIES;i++)
	{
		if(pch = strtok (NULL,","))//data i
		{
		 uvp6_obj->lpm_data.data[i]=strtoul(pch,NULL,0);
		}

	}
	//grey levels
	for(int i=0;i<UVP6_NUM_OF_CATEGORIES;i++)
	{
		if(pch = strtok (NULL,",;"))//grey level i
		{
		 uvp6_obj->lpm_data.grey_levels[i]=strtoul(pch,NULL,0);
		}
	}

	osMessagePut(uvp6_obj->events_q,UVP6_EVNT_LPM_DATA_RCVD,1);
	return UVP6_F_OK;
}

int UVP6_MSG_ACQ_CONF_f(uvp6* uvp6_obj,uint8_t* msg)
{
	osMessagePut(uvp6_obj->events_q,UVP6_EVNT_ACQ_CONF_RCVD,1);
	//uvp6_obj->status=UVP6_IDLE;
	return UVP6_F_OK;
}


int UVP6_MSG_START_ACK_f(uvp6* uvp6_obj,uint8_t* msg)
{
	osMessagePut(uvp6_obj->events_q,UVP6_EVNT_START_ACK_RCVD,1);
	return UVP6_F_OK;
}

int UVP6_MSG_START_ERR_f(uvp6* uvp6_obj,uint8_t* msg)
{
	char * pch;
	pch = strtok (msg,";");//error num
	uvp6_obj->start_error=strtol(pch,NULL,10);
	osMessagePut(uvp6_obj->events_q,UVP6_EVNT_START_ERR_RCVD,1);
	return UVP6_F_OK;
}

int UVP6_MSG_STOP_ACK_f(uvp6* uvp6_obj,uint8_t* msg)
{
	osMessagePut(uvp6_obj->events_q,UVP6_EVNT_STOP_ACK_RCVD,1);
	return UVP6_F_OK;
}

