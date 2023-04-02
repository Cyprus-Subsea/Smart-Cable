/*
 * HYDROC.c
 *
 *      Author: admin
 */

#include "HydroC.h"
#include "string.h"
#include "main.h"

//debug
extern UART_HandleTypeDef huart1;

int (*hydroc_functions[HYDROC_MSG_NUM_OF_FUNCTIONS])(hydroc* hydroc_obj,uint8_t* msg);
char*  hydroc_messages_strings[HYDROC_MSG_NUM_OF_FUNCTIONS];
const char* hydroc_commands_strings[]={"$COCFG,0,0,W,1\r\n","$COCFG,0,0,W,0\r\n","$CORTS,0,0,W,1000,","$COPEX,0,0,W,1,0\r\n","$COPEX,0,0,W,0,0\r\n","$COMDI,0,0,W,","$","$","$CODBC,0,0,W,1\r\n","TEST\r\n"};

void hydroc_init(hydroc* hydroc_obj)
{
	hydroc_messages_init(hydroc_obj);

	osMessageQDef(hydroc_events_q, 20, uint8_t);
	hydroc_obj->events_q= osMessageCreate(osMessageQ(hydroc_events_q), NULL);

	osMessageQDef(hydroc_media_rx_q, 20, uint16_t);
	hydroc_obj->media_rx_messages_q= osMessageCreate(osMessageQ(hydroc_media_rx_q), NULL);

	osMessageQDef(hydroc_media_tx_q, 400, uint8_t);
	hydroc_obj->media_tx_q = osMessageCreate(osMessageQ(hydroc_media_tx_q), NULL);

	osThreadDef(hydroc_task, hydroc_loop, osPriorityNormal, 0, 256);
	osThreadCreate(osThread(hydroc_task), hydroc_obj);

	hydroc_obj->rx_buffer_indx=0;

	hydroc_obj->errors.P_in=0;
	hydroc_obj->errors.P_pump=0;
	hydroc_obj->errors.T_control=0;
	hydroc_obj->errors.rH_gas=0;
}

void hydroc_media_process_byte(hydroc* hydroc_obj,uint8_t rx_byte)
{

	if(hydroc_obj->media_status==HYDROC_MEDIA_READY)
	{
		hydroc_obj->rx_buffer[hydroc_obj->rx_buffer_indx]=rx_byte;
		if(rx_byte=='\r'|| rx_byte=='\n')
		{
			hydroc_obj->rx_buffer[hydroc_obj->rx_buffer_indx]=0x00;
			osMessagePut(hydroc_obj->media_rx_messages_q,hydroc_obj->rx_buffer_new_string_indx,1);
			if(hydroc_obj->rx_buffer_indx>HYDROC_RX_BUFFER_THR) hydroc_obj->rx_buffer_indx=0;
			else  hydroc_obj->rx_buffer_indx++;
			hydroc_obj->rx_buffer_new_string_indx=hydroc_obj->rx_buffer_indx;
		}
		else hydroc_obj->rx_buffer_indx++;
		if(hydroc_obj->rx_buffer_indx==HYDROC_RX_BUFFER_SIZE)
		{
			hydroc_obj->rx_buffer_indx=0;
			hydroc_obj->rx_buffer_new_string_indx=hydroc_obj->rx_buffer_indx;
		}
	}
	else
	{
		hydroc_obj->rx_buffer_indx=0;
		hydroc_obj->rx_buffer_new_string_indx=hydroc_obj->rx_buffer_indx;
	}

}

void hydroc_loop(hydroc* hydroc_obj)
{
 uint16_t msg_indx;
 for(;;)
 {
	if(xQueueReceive(hydroc_obj->media_rx_messages_q,&msg_indx,1))
	{
		uint8_t* msg=hydroc_obj->rx_buffer+msg_indx;
		hydroc_parse_message(hydroc_obj,msg);
	}
	osDelay(1);
 }

}


uint8_t hydroc_media_get_byte(hydroc* hydroc_obj,uint8_t* tx_byte)
{
	osEvent res=osMessageGet(hydroc_obj->media_tx_q,1);
	if(res.status==osEventMessage)
     {
		*tx_byte=res.value.v;
		return HYDROC_F_OK;
     }
   return HYDROC_F_ERR;
}


void hydroc_send_cmd(hydroc* hydroc_obj,uint8_t cmd_id,void* arg)
{

 char tmp_cmd[50];
 osDelay(20);
 switch(cmd_id)
 {
     case HYDROC_CMD_TEST:
      tmp_cmd[0]=0x00;
      strcat(tmp_cmd,hydroc_commands_strings[HYDROC_CMD_TEST_STRING]);
	  for(int i=0;i<strlen(tmp_cmd);i++)
	  {
		   osMessagePut(hydroc_obj->media_tx_q,tmp_cmd[i],0);
	  }
	 break;
     case HYDROC_CMD_ENTER_CFG:
       tmp_cmd[0]=0x00;
       strcat(tmp_cmd,hydroc_commands_strings[HYDROC_CMD_ENTER_CFG]);
  	   for(int i=0;i<strlen(tmp_cmd);i++)
  	   {
  		   osMessagePut(hydroc_obj->media_tx_q,tmp_cmd[i],0);
  	   }
	 break;
     case HYDROC_CMD_EXIT_CFG:
       tmp_cmd[0]=0x00;
       strcat(tmp_cmd,hydroc_commands_strings[HYDROC_CMD_EXIT_CFG]);
  	   for(int i=0;i<strlen(tmp_cmd);i++)
  	   {
  		   osMessagePut(hydroc_obj->media_tx_q,tmp_cmd[i],0);
  	   }
	 break;
     case HYDROC_CMD_SET_REAL_TIME:
       tmp_cmd[0]=0x00;
       strcat(tmp_cmd,hydroc_commands_strings[HYDROC_CMD_SET_REAL_TIME]);
       strcat(tmp_cmd,(char*)arg);
       strcat(tmp_cmd,"\r\n");
  	   for(int i=0;i<strlen(tmp_cmd);i++)
  	   {
  		   osMessagePut(hydroc_obj->media_tx_q,tmp_cmd[i],0);
  	   }
	 break;
     case HYDROC_CMD_ENABLE_PUMP:
       tmp_cmd[0]=0x00;
       strcat(tmp_cmd,hydroc_commands_strings[HYDROC_CMD_ENABLE_PUMP]);
  	   for(int i=0;i<strlen(tmp_cmd);i++)
  	   {
  		   osMessagePut(hydroc_obj->media_tx_q,tmp_cmd[i],0);
  	   }
	 break;
     case HYDROC_CMD_SET_ZERO_MODE:
       tmp_cmd[0]=0x00;
       strcat(tmp_cmd,hydroc_commands_strings[HYDROC_CMD_SET_MODE]);
       strcat(tmp_cmd,"1,120,2,2,2\r\n");
  	   for(int i=0;i<strlen(tmp_cmd);i++)
  	   {
  		   osMessagePut(hydroc_obj->media_tx_q,tmp_cmd[i],0);
  	   }
	 break;
     case HYDROC_CMD_SET_MEASURE_MODE:
       tmp_cmd[0]=0x00;
       strcat(tmp_cmd,hydroc_commands_strings[HYDROC_CMD_SET_MODE]);
       strcat(tmp_cmd,"3,3500000,");
       strcat(tmp_cmd,(char*)arg);
       strcat(tmp_cmd,"\r\n");
  	   for(int i=0;i<strlen(tmp_cmd);i++)
  	   {
  		   osMessagePut(hydroc_obj->media_tx_q,tmp_cmd[i],0);
  	   }
	 break;
     case HYDROC_CMD_CLEAR:
       tmp_cmd[0]=0x00;
       strcat(tmp_cmd,hydroc_commands_strings[HYDROC_CMD_CLEAR]);
  	   for(int i=0;i<strlen(tmp_cmd);i++)
  	   {
  		   osMessagePut(hydroc_obj->media_tx_q,tmp_cmd[i],0);
  	   }
  	 break;
     case HYDROC_CMD_DISABLE_PUMP:
       tmp_cmd[0]=0x00;
       strcat(tmp_cmd,hydroc_commands_strings[HYDROC_CMD_DISABLE_PUMP]);
       for(int i=0;i<strlen(tmp_cmd);i++)
       {
         osMessagePut(hydroc_obj->media_tx_q,tmp_cmd[i],0);
       }
     break;
 }

}


uint8_t hydroc_get_event(hydroc* hydroc_obj,uint8_t* event)
{
	if(xQueueReceive(hydroc_obj->events_q,event,1))
	{
     return HYDROC_F_OK;
	}
	return HYDROC_F_ERR;
}



int hydroc_parse_message(hydroc* hydroc_obj,uint8_t* msg)
{
	char * pch;
	pch = strtok (msg,",");//header
	uint8_t* tmp_ptr;
	for(int i=0;i<HYDROC_MSG_NUM_OF_FUNCTIONS;i++)
	{
	   if(strlen(pch)==strlen(hydroc_messages_strings[i]))
	   {
		  tmp_ptr=0;
		  tmp_ptr=strstr(pch,hydroc_messages_strings[i]);
		  if(tmp_ptr)
		   {
			 if(hydroc_functions[i](hydroc_obj,msg+strlen(tmp_ptr)+1)==HYDROC_F_OK) return HYDROC_F_OK;
	         else return HYDROC_F_ERR;
		   }
		}
	 }
	 return HYDROC_F_ERR;

}

void hydroc_messages_init(hydroc* hydroc_obj)
{
	hydroc_functions[HYDROC_MSG_COSIM] = HYDROC_MSG_COSIM_f;
	hydroc_functions[HYDROC_MSG_CODS4] = HYDROC_MSG_CODS4_f;
	hydroc_functions[HYDROC_MSG_COTS1] = HYDROC_MSG_COTS1_f;

	hydroc_messages_strings[HYDROC_MSG_COSIM] = "$COSIM";
	hydroc_messages_strings[HYDROC_MSG_CODS4] = "$CODS4";
	hydroc_messages_strings[HYDROC_MSG_COTS1] = "$COTS1";


}


int HYDROC_MSG_COSIM_f(hydroc* hydroc_obj,uint8_t* msg)
{
   //osMessagePut(hydroc_obj->events_q,HYDROC_EVNT_BOOTED,1);
   hydroc_obj->status=HYDROC_READY;
   return HYDROC_F_OK;
}

int HYDROC_MSG_COTS1_f(hydroc* hydroc_obj,uint8_t* msg)
{
   char * pch;
   if(hydroc_obj->status==HYDROC_WAIT_DATA)
   {
	 pch = strtok ((char*)msg,",");//destination
	 pch = strtok (NULL,",");//source
	 pch = strtok (NULL,",");//data
	 pch = strtok (NULL,",");//format
	 pch = strtok (NULL,",");//LDN
	 pch = strtok (NULL,",");//date
	 hydroc_obj->ts1.date[0]=0x00;
	 strcpy(hydroc_obj->ts1.date,pch);
	 pch = strtok (NULL,",");//time
	 hydroc_obj->ts1.time[0]=0x00;
	 strcpy(hydroc_obj->ts1.time,pch);
	 pch = strtok (NULL,",");//weekday
	 pch = strtok (NULL,",");//T_control
	 hydroc_obj->ts1.T_control[0]=0x00;
	 strcpy(hydroc_obj->ts1.T_control,pch);
	 pch = strtok (NULL,",");//xz
	 pch = strtok (NULL,",");//T_gas
	 hydroc_obj->ts1.T_gas[0]=0x00;
	 strcpy(hydroc_obj->ts1.T_gas,pch);
	 pch = strtok (NULL,",");//xz
	 pch = strtok (NULL,",");//xz
	 pch = strtok (NULL,",");//rH_gas
	 hydroc_obj->ts1.rH_gas[0]=0x00;
	 strcpy(hydroc_obj->ts1.rH_gas,pch);
	 osMessagePut(hydroc_obj->events_q,HYDROC_EVNT_COTS1,1);
   }
   return HYDROC_F_OK;
}

int HYDROC_MSG_CODS4_f(hydroc* hydroc_obj,uint8_t* msg)
{
   char * pch;
   if(hydroc_obj->status==HYDROC_WAIT_DATA)
   {
	 pch = strtok ((char*)msg,",");//destination
	 pch = strtok (NULL,",");//source
	 pch = strtok (NULL,",");//data
	 pch = strtok (NULL,",");//format
	 pch = strtok (NULL,",");//LDN
	 pch = strtok (NULL,",");//date
	 hydroc_obj->ds4.date[0]=0x00;
	 strcpy(hydroc_obj->ds4.date,pch);
	 pch = strtok (NULL,",");//time
	 hydroc_obj->ds4.time[0]=0x00;
	 strcpy(hydroc_obj->ds4.time,pch);
	 //pch = strtok (NULL,",");//msec
	 pch = strtok (NULL,",");//weekday
	 hydroc_obj->ds4.weekday[0]=0x00;
	 strcpy(hydroc_obj->ds4.weekday,pch);
	 pch = strtok (NULL,",");//power_pump
	 hydroc_obj->ds4.pump_pwr[0]=0x00;
	 strcpy(hydroc_obj->ds4.pump_pwr,pch);
	 //pch = strtok (NULL,",");//analog_1
	 pch = strtok (NULL,",");//p_ndir
	 hydroc_obj->ds4.P_NDIR[0]=0x00;
	 strcpy(hydroc_obj->ds4.P_NDIR,pch);
	 pch = strtok (NULL,",");//p_in
	 hydroc_obj->ds4.P_IN[0]=0x00;
	 strcpy(hydroc_obj->ds4.P_IN,pch);
	 //pch = strtok (NULL,",");//analog_4
	 //pch = strtok (NULL,",");//analog_5
	 pch = strtok (NULL,",");//I_total
	 hydroc_obj->ds4.I_total[0]=0x00;
	 strcpy(hydroc_obj->ds4.I_total,pch);
	 pch = strtok (NULL,",");//U_total
	 hydroc_obj->ds4.U_total[0]=0x00;
	 strcpy(hydroc_obj->ds4.U_total,pch);
	 pch = strtok (NULL,",");//state 1
	 hydroc_obj->ds4.status_zeroing[0]=0x00;
	 strcpy(hydroc_obj->ds4.status_zeroing,pch);
	 pch = strtok (NULL,",");//state 2
	 hydroc_obj->ds4.status_flush[0]=0x00;
	 strcpy(hydroc_obj->ds4.status_flush,pch);
	 pch = strtok (NULL,",");//state 3
	 hydroc_obj->ds4.status_pump[0]=0x00;
	 strcpy(hydroc_obj->ds4.status_pump,pch);
	 //pch = strtok (NULL,",");//xz
	 //pch = strtok (NULL,",");//xz
	 //pch = strtok (NULL,",");//xz
	 //pch = strtok (NULL,",");//xz
	 pch = strtok (NULL,",");//runtime
	 hydroc_obj->ds4.runtime[0]=0x00;
	 strcpy(hydroc_obj->ds4.runtime,pch);
	 pch = strtok (NULL,",");//signal raw
	 hydroc_obj->ds4.signal_raw[0]=0x00;
	 strcpy(hydroc_obj->ds4.signal_raw,pch);
	 pch = strtok (NULL,",");//signal ref
	 hydroc_obj->ds4.signal_ref[0]=0x00;
	 strcpy(hydroc_obj->ds4.signal_ref,pch);
	 pch = strtok (NULL,",");//t sensor
	 hydroc_obj->ds4.T_sensor[0]=0x00;
	 strcpy(hydroc_obj->ds4.T_sensor,pch);
	 pch = strtok (NULL,",");//signal proc
	 hydroc_obj->ds4.signal_proc[0]=0x00;
	 strcpy(hydroc_obj->ds4.signal_proc,pch);
	 pch = strtok (NULL,",");//conc_estimate
	 hydroc_obj->ds4.conc_estimate[0]=0x00;
	 strcpy(hydroc_obj->ds4.conc_estimate,pch);
	 pch = strtok (NULL,",");//pCO2_corr
	 hydroc_obj->ds4.pCO2_corr[0]=0x00;
	 strcpy(hydroc_obj->ds4.pCO2_corr,pch);
	 pch = strtok (NULL,"*");//xCO2_corr
	 hydroc_obj->ds4.xCO2_corr[0]=0x00;
	 strcpy(hydroc_obj->ds4.xCO2_corr,pch);

     osMessagePut(hydroc_obj->events_q,HYDROC_EVNT_CODS4,1);
     //strcpy(hydroc_obj->ds4_data,msg);
   }
   return HYDROC_F_OK;
}

