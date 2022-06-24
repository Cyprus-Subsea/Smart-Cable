/*
 * UI.c
 *
 *  Created on: 12 апр. 2021 г.
 *      Author: admin
 */


#include "UI.h"
#include "string.h"
#include "stdlib.h"
#include "icListen.h"
#include "mcu_flash.h"
#include "em_sd_storage.h"
#include "rtc.h"


extern UART_HandleTypeDef huart1;
extern icListen_object_typedef icListen;
extern mcu_flash_typedef mcu_flash;
extern sd_storage_t microsd_storage;
extern rtc_typedef rtc;

int ((*UI_functions[UI_MSG_NUM_OF_FUNCTIONS]))(UI_typedef* UI_obj,uint8_t* msg);
char*  UI_messages_strings[UI_MSG_NUM_OF_FUNCTIONS];

memory_region_pointer temp_ptr;
char temp_array[400];
const char* UI_commands_strings[]={"help here\r"};


void UI_init(UI_typedef* UI_obj)
{
	UI_messages_init(UI_obj);


	osMessageQDef(UI_events_q, 20, uint8_t);
	UI_obj->events_q= osMessageCreate(osMessageQ(UI_events_q), NULL);

	osMessageQDef(UI_media_rx_q, 20, uint16_t);
	UI_obj->media_rx_messages_q= osMessageCreate(osMessageQ(UI_media_rx_q), NULL);

	osMessageQDef(UI_media_tx_q, 200, uint8_t);
	UI_obj->media_tx_q = osMessageCreate(osMessageQ(UI_media_tx_q), NULL);

	osThreadDef(UI_task, UI_loop, osPriorityNormal, 0, 256);
	osThreadCreate(osThread(UI_task), UI_obj);
}


void UI_loop(UI_typedef* UI_obj)
{
	 uint16_t msg_indx;
	 for(;;)
	 {
		if(xQueueReceive(UI_obj->media_rx_messages_q,&msg_indx,osWaitForever))
		{   uint8_t* msg=UI_obj->rx_buffer+msg_indx;
			UI_parse_message(UI_obj,msg);
		}

	 }

}


void UI_media_process_byte(UI_typedef* UI_obj,uint8_t rx_byte)
{
	if(UI_obj->media_status==UI_MEDIA_READY && rx_byte!=0x00)
	{
		UI_obj->rx_buffer[UI_obj->rx_buffer_indx]=rx_byte;
		if(rx_byte=='\r')
		{
			UI_obj->rx_buffer[UI_obj->rx_buffer_indx]=0x00;
			osMessagePut(UI_obj->media_rx_messages_q,UI_obj->rx_buffer_new_string_indx,0);
			if(UI_obj->rx_buffer_indx>UI_RX_BUFFER_THR) UI_obj->rx_buffer_indx=0;
			else  UI_obj->rx_buffer_indx++;
			UI_obj->rx_buffer_new_string_indx=UI_obj->rx_buffer_indx;
		}
		else UI_obj->rx_buffer_indx++;
		if(UI_obj->rx_buffer_indx==UI_RX_BUFFER_SIZE)
		{
			UI_obj->rx_buffer_indx=0;
			UI_obj->rx_buffer_new_string_indx=UI_obj->rx_buffer_indx;
		}
	}
	else
	{
		UI_obj->rx_buffer_indx=0;
		UI_obj->rx_buffer_new_string_indx=UI_obj->rx_buffer_indx;
	}
}

uint8_t UI_media_get_byte(UI_typedef* UI_obj,uint8_t* tx_byte)
{
	osEvent res=osMessageGet(UI_obj->media_tx_q,0);
	if(res.status==osEventMessage)
    {
		*tx_byte=res.value.v;
		return UI_F_OK;
    }
  return UI_F_ERR;
}


void UI_send_msg(UI_typedef* UI_obj,uint8_t cmd_id,void* arg)
{
 memory_region_pointer*  ptr=arg;

 switch(cmd_id)
 {
     case UI_CMD_SEND_FROM_PREDEFINED:
       //for(int i=0;i<strlen(UI_commands_strings[ptr->start_addr]);i++)
  	   //{
  		   //osMessagePut(UI_obj->media_tx_q,*(UI_commands_strings[ptr->start_addr]+i),1);
  	   //}
	 break;
     case UI_CMD_SEND_DATA:

       for(int i=0;i<ptr->size;i++)
	   {
		   osMessagePut(UI_obj->media_tx_q,*((uint8_t*)(ptr->start_addr)+i),osWaitForever);
	   }
     break;
 }

}

uint8_t UI_get_event(UI_typedef* UI_obj,uint8_t* event)
{
	if(xQueueReceive(UI_obj->events_q,event,1))
	{
     return UI_F_OK;
	}
	return UI_F_ERR;
}


int UI_parse_message(UI_typedef* UI_obj,uint8_t* msg)
{
 char* pch;
 pch=strtok(msg," ");
 for(int i=0;i<UI_MSG_NUM_OF_FUNCTIONS;i++)
 {
	  if(strcmp(pch,UI_messages_strings[i])==0)
		{
			if(UI_functions[i](UI_obj,pch+strlen(pch)+1)==UI_F_OK) return UI_F_OK;
      else return UI_F_ERR;
		}

 }
 return UI_F_ERR;
}

void UI_messages_init(UI_typedef* UI_obj)
{
	UI_functions[UI_MSG_SET] = UI_MSG_SET_f;
	UI_functions[UI_MSG_SHOW] = UI_MSG_SHOW_f;
	UI_functions[UI_MSG_RESET] = UI_MSG_RESET_f;
	UI_functions[UI_MSG_HELP] = UI_MSG_HELP_f;

	UI_messages_strings[UI_MSG_SET] = "set";
	UI_messages_strings[UI_MSG_SHOW] = "show";
	UI_messages_strings[UI_MSG_RESET] = "reset";
	UI_messages_strings[UI_MSG_HELP] = "help";
}

int UI_MSG_HELP_f(UI_typedef* UI_obj,uint8_t* msg)
{
	char * pch;
	memory_region_pointer ptr;

	pch = strtok (NULL," ");//subcomand
	osMessagePut(UI_obj->events_q,UI_EVNT_HELP,1);
	return UI_F_OK;
}

int UI_MSG_RESET_f(UI_typedef* UI_obj,uint8_t* msg)
{
	char * pch;
	memory_region_pointer ptr;

	pch = strtok (NULL," ");//subcomand
	if(strcmp(pch,"settings")==0){
		icListen.settings->wav_sample_rate=ICLISTEN_DEFAULT_WAV_SAMPLE_RATE;
		icListen.settings->wav_sample_bit_depth=ICLISTEN_DEFAULT_WAV_SAMPLE_BIT_DEPTH;
		icListen.settings->file_duration=ICLISTEN_DEFAULT_FILE_DURATION;
		mcu_flash_save(&mcu_flash);
	}
	osMessagePut(UI_obj->events_q,UI_EVNT_RESET,1);
	return UI_F_OK;
}

int UI_MSG_SHOW_f(UI_typedef* UI_obj,uint8_t* msg)
{
	char * pch;
	pch = strtok (NULL," ");//subcomand

	if(strcmp(pch,"sensor")==0){
		sprintf(temp_array,"Device type: %d\rSerial num: %d\rFW version: %s\rBuild date: %s\rStatus: %d\rFile duration: %d\rWAV sample depth: %d\rWAV sample rate: %d\r",icListen.device_type,icListen.serial_number,icListen.firmware_version,icListen.build_date,icListen.status,icListen.settings->file_duration,icListen.settings->wav_sample_bit_depth,icListen.settings->wav_sample_rate);
		temp_ptr.start_addr=temp_array;
		temp_ptr.size=strlen(temp_array);
		UI_send_msg(UI_obj,UI_CMD_SEND_DATA,&temp_ptr);
	}
	else if(strcmp(pch,"storage")==0){
		temp_array[0]=0x00;
		for(int i=0;i<SD_STORAGE_NUM_DISKS;i++)
		{
		 sprintf(temp_array+strlen(temp_array),"Disk: %d\rStatus: %d\rSize: %d KB\rFree space: %d KB\r",i,microsd_storage.disks[i].status,microsd_storage.disks[i].size,microsd_storage.disks[i].free_space);
		}
		temp_ptr.start_addr=temp_array;
		temp_ptr.size=strlen(temp_array);
		UI_send_msg(UI_obj,UI_CMD_SEND_DATA,&temp_ptr);
	}
	else if(strcmp(pch,"clock")==0){
		temp_array[0]=0x00;
		read_time(&rtc);
        sprintf(temp_array,"Clock: %d:%d:%d  %d/%d/%d %d\r",rtc.time.Hours,rtc.time.Minutes,rtc.time.Seconds,rtc.date.Date,rtc.date.Month,rtc.date.Year,(uint32_t)rtc.timestamp);
		temp_ptr.start_addr=temp_array;
		temp_ptr.size=strlen(temp_array);
		UI_send_msg(UI_obj,UI_CMD_SEND_DATA,&temp_ptr);
	}
	else{
		sprintf(temp_array,"sensor\rstorage\rclock\r");
		temp_ptr.start_addr=temp_array;
		temp_ptr.size=strlen(temp_array);
		UI_send_msg(UI_obj,UI_CMD_SEND_DATA,&temp_ptr);
	}
	osMessagePut(UI_obj->events_q,UI_EVNT_SHOW,1);
	return UI_F_OK;
}

int UI_MSG_SET_f(UI_typedef* UI_obj,uint8_t* msg)
{
	char * pch;
	pch = strtok (NULL," ");//subcomand
	if(strcmp(pch,"clock")==0){
		pch = strtok (NULL,":");//hours
		rtc.time.Hours=atol(pch);
		pch = strtok (NULL,":");//minutes
		rtc.time.Minutes=atol(pch);
		pch = strtok (NULL," ");//seconds
		rtc.time.Seconds=atol(pch);
		pch = strtok (NULL,"/");//day
		rtc.date.Date=atol(pch);
		pch = strtok (NULL,"/");//month
		rtc.date.Month=atol(pch);
		pch = strtok (NULL," ");//year
		rtc.date.Year=atol(pch);
		rtc.date.WeekDay=RTC_WEEKDAY_MONDAY;
		set_time(&rtc);
	}

	osMessagePut(UI_obj->events_q,UI_EVNT_SET,1);
	return UI_F_OK;
}
