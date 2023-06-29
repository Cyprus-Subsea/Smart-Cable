/*
 * PUCK.c
 *
 *  Created on: 11 сент. 2021 г.
 *      Author: admin
 */
#include "PUCK.h"


const puck_datasheet_type sensor_default_datasheet={{0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10},3,96,0,0,0,1,"Smart-Cable/ADC"};
puck_str puck_main;



const  char* puck_cmd_patterns[] ={"PUCKRM","PUCKWM","PUCKFM","PUCKEM","PUCKGA","PUCKSA","PUCKSZ","PUCK","PUCKTY","PUCKVR","PUCKIM","PUCKVB","PUCKSB"};
const  char* puck_error_messages[] ={"ERR 0020\r","ERR 0021\r","ERR 0022\r","ERR 0023\r","ERR 0024\r","ERR 0010\r"};
const  char* puck_messages[] ={"PUCKRDY\r"};


/*
  p = (uint8_t*)malloc(sizeof(uint8_t));
  *p=b;
 if(smac_send(puck_main.smac,p,1,FREE_MEMORY_FLAG)!=RES_OK)
 {
	 free(p);
 }
 */

void puck_event_callback(smac_event event_id)
{
	switch(event_id)
	{
	  case SMAC_EVNT_RX_OUT_Q_CHANGED:
		  puck_main.rx_message[0]=0;
		  puck_main.rx_message_last_byte_index=0;
		  xQueueReset(puck_main.rxQ);
		  PUCK_f();
	  break;
	};
}

analyzer_err_code puck_analyzer(uint8_t new_byte)
{
  switch(puck_main.analyzer.stage)
  {
     case PUCK_ANALYZER_STAGE1:
    	 if(puck_main.analyzer.stage1_msg[puck_main.analyzer.byte_counter]==new_byte){
    		 puck_main.analyzer.byte_counter++;
    		 if(puck_main.analyzer.byte_counter==PUCK_CMD_LEN){
    			 puck_main.analyzer.byte_counter=0;
    			 puck_main.analyzer.stage=PUCK_ANALYZER_STAGE1_DELAY;
    			 timed_callback_set(puck_main.analyzer.stage1_timed_callback,75);
    		 }
    	 }
    	 else {
    		 if(puck_main.analyzer.byte_counter!=0) puck_main.analyzer.byte_counter=0;
    	 }
    	break;
    case PUCK_ANALYZER_STAGE1_DELAY:
    	timed_callback_off(puck_main.analyzer.stage1_timed_callback);
    	puck_main.analyzer.byte_counter=0;
    	puck_main.analyzer.stage=PUCK_ANALYZER_STAGE1;
    	break;
    case PUCK_ANALYZER_STAGE2:
		 if(puck_main.analyzer.stage2_msg[puck_main.analyzer.byte_counter]==new_byte){
			 puck_main.analyzer.byte_counter++;
			 if(puck_main.analyzer.byte_counter==PUCK_CMD_LEN){
				 puck_main.analyzer.byte_counter=0;
				 puck_main.analyzer.stage=PUCK_ANALYZER_STAGE2_DELAY;
				 timed_callback_set(puck_main.analyzer.stage2_timed_callback,50);
			 }
		 }
		 else {
			 if(puck_main.analyzer.byte_counter!=0) puck_main.analyzer.byte_counter=0;
		 }
    	break;
    case PUCK_ANALYZER_STAGE2_DELAY:
    	timed_callback_off(puck_main.analyzer.stage2_timed_callback);
    	puck_main.analyzer.byte_counter=0;
    	puck_main.analyzer.stage=PUCK_ANALYZER_STAGE1;
    	break;
  };
  return SMAC_ANALYZER_OK;
}

void puck_stage1_timed_callback()
{
  if(puck_main.analyzer.stage==PUCK_ANALYZER_STAGE1_DELAY) {
	  puck_main.analyzer.stage=PUCK_ANALYZER_STAGE2;
	  puck_main.analyzer.byte_counter=0;
  }
}

void puck_stage2_timed_callback()
{
  if(puck_main.analyzer.stage==PUCK_ANALYZER_STAGE2_DELAY) {
	puck_main.analyzer.stage=PUCK_ANALYZER_STAGE1;
	puck_main.analyzer.byte_counter=0;
	puck_main.smac->active_rx_outQ=puck_main.rxQ;
	puck_main.smac->event_status=SMAC_EVNT_RX_OUT_Q_CHANGED;
  }
}


void puck_init( smac_controller* smac,osMessageQId rxQ,osMessageQId instrument_rxQ)
{
	puck_commands_init();

	puck_main.smac=smac;                                                                                  // standard initializations steps
	puck_main.rxQ=rxQ;                                                                                    // standard initializations steps
	smac_register_rx_outQ(puck_main.smac,puck_main.rxQ,INACTIVE_QUEUE_FLAG,&puck_event_callback);         // standard initializations steps

	puck_main.rx_message_last_byte_index=0;
	puck_main.rx_mode=PUCK_RX_ASCII_MODE;
	puck_main.instrument_rxQ=instrument_rxQ;
	puck_main.smac->analyzer=&puck_analyzer;


	puck_main.analyzer.stage1_delay=750;
	puck_main.analyzer.stage1_delay=500;
	memcpy(puck_main.analyzer.stage1_msg,"@@@@@@",6);
	memcpy(puck_main.analyzer.stage2_msg,"!!!!!!",6);

	puck_main.analyzer.stage=PUCK_ANALYZER_STAGE1;
	puck_main.analyzer.byte_counter=0;

	puck_main.analyzer.stage1_timed_callback=timed_callback_register_new(&puck_stage1_timed_callback);
	puck_main.analyzer.stage2_timed_callback=timed_callback_register_new(&puck_stage2_timed_callback);

}


err_code puck_commands_init(void){


	puck_main.cmd_functions[0] =PUCKRM_f;
	puck_main.cmd_functions[1] =PUCKWM_f;
	puck_main.cmd_functions[2] =PUCKFM_f;
	puck_main.cmd_functions[3] =PUCKEM_f;
	puck_main.cmd_functions[4] =PUCKGA_f;
	puck_main.cmd_functions[5] =PUCKSA_f;
	puck_main.cmd_functions[6] =PUCKSZ_f;
	puck_main.cmd_functions[7] =PUCK_f;
	puck_main.cmd_functions[8] =PUCKTY_f;
	puck_main.cmd_functions[9] =PUCKVR_f;
	puck_main.cmd_functions[10] =PUCKIM_f;
	puck_main.cmd_functions[11] =PUCKVB_f;
	puck_main.cmd_functions[12] =PUCKSB_f;

	puck_main.datasheet.start_addr=(uint8_t*)&sensor_default_datasheet;
	puck_main.datasheet.size=PUCK_DATASHEET_SIZE;
	puck_main.datasheet.data_pointer_offset=0;

	return RES_OK;
  }


void puck_loop()
{
  uint8_t b;
  while(1)
  {
	  if(xQueueReceive(puck_main.rxQ,&b,1)==pdTRUE){
		  switch(puck_main.rx_mode)
		  {
		   case PUCK_RX_ASCII_MODE:
		    puck_new_byte_processing(b);
		   break;
		  };
	  }
	  osDelay(1);
  }
}



err_code puck_new_byte_processing(uint8_t new_byte)
{
	int func_index;

	if(puck_update_rx_message(new_byte)==PUCK_MSG_COMPLETE){
		for(int i=0;i<PUCK_CMD_LEN;i++) puck_main.rx_msg_params[i]=0x00;
		puck_split_rx_message(puck_main.rx_msg_params," ");
		func_index=puck_get_cmd_pattern_index(puck_main.rx_msg_params[0]);
		if(func_index!=PUCK_CMD_UNKNOWN)
		{
			puck_err_code error_num=puck_main.cmd_functions[func_index]();
            if(error_num!=PUCK_ERR_NO_ERR)
            {
    	      //puck_err_patterns[puck_err_code];
             }
		    //uart_send_str("PUCKRDY\r");
		}
	}
	return 0;
}

puck_message_status puck_update_rx_message(uint8_t new_byte)
{

		if(new_byte==0x0D){ //end of string detected "\n" or "\r", replace it with 0x00
			puck_main.rx_message[puck_main.rx_message_last_byte_index]=0x00;
			puck_main.rx_message_last_byte_index=0;
			return PUCK_MSG_COMPLETE;
		}

		puck_main.rx_message[puck_main.rx_message_last_byte_index]=new_byte;
		puck_main.rx_message_last_byte_index++;
		puck_main.rx_message_last_byte_index %= PUCK_MSG_BUFFER_SIZE;
		return PUCK_MSG_INCOMPLETE;

}


err_code puck_split_rx_message(char** s_array,char* delimiter)
{
    char* p = strtok ((char*)puck_main.rx_message,delimiter);
	  s_array[0]=p;
	for(int i=1;p!= NULL;i++){
    p = strtok (NULL,delimiter);
	  s_array[i]=p;
	}
	return 0;
}


puck_func_index puck_get_cmd_pattern_index(char* cmd_string){

  for(int i=0;i<PUCK_NUM_OF_COMMANDS;i++) if (strcmp(puck_cmd_patterns[i],cmd_string)==0) return i;
  return PUCK_CMD_UNKNOWN;

}


void puck_send_integer(uint32_t msg)
{
	char* tt=(char*)malloc(10);
    sprintf(tt,"%d\r",msg);

    if(smac_send(puck_main.smac,tt,strlen(tt),FREE_MEMORY_FLAG)!=RES_OK)
    {
    	free(tt);
    }
}
void puck_send_raw(uint8_t* msg,uint32_t len,uint8_t mem_free_flag)
{
	err_code res=smac_send(puck_main.smac,msg,len,mem_free_flag);
    if(res!=RES_OK && mem_free_flag==FREE_MEMORY_FLAG)
    {
    	free(msg);
    }
}
void puck_send_string(char* msg,uint8_t mem_free_flag)
{
	err_code res=smac_send(puck_main.smac,msg,strlen(msg), mem_free_flag);
    if(res!=RES_OK && mem_free_flag==FREE_MEMORY_FLAG)
    {
    	free(msg);
    }

}



puck_err_code PUCKRM_f()
{
	unsigned long num_of_rx_bytes=atol(puck_main.rx_msg_params[1]);
	if(num_of_rx_bytes<=PUCK_MAX_READ_BYTES){
         uint8_t* new_ptr=(uint8_t*)malloc(num_of_rx_bytes+2);
         new_ptr[0]='[';
         new_ptr[num_of_rx_bytes+1]=']';
         if(new_ptr){
        	 for(int i=1;i<=num_of_rx_bytes;i++){
        		 new_ptr[i]=*(puck_main.datasheet.start_addr+puck_main.datasheet.data_pointer_offset);
        		 puck_main.datasheet.data_pointer_offset++;
        		 puck_main.datasheet.data_pointer_offset%=PUCK_DATASHEET_SIZE;
        	 }
         }
         puck_send_raw(new_ptr,num_of_rx_bytes+2,FREE_MEMORY_FLAG);
	}
	else{
		puck_send_string(puck_error_messages[PUCK_ERR_ERR020],KEEP_MEMORY_FLAG);
	}
	PUCK_f();
    return PUCK_ERR_NO_ERR;
}
puck_err_code PUCKWM_f()
{
  return PUCK_ERR_NO_ERR;
}
puck_err_code PUCKFM_f()
{
  return PUCK_ERR_NO_ERR;
}
puck_err_code PUCKEM_f()
{
  return PUCK_ERR_NO_ERR;
}
puck_err_code PUCKGA_f()
{
  puck_send_integer(puck_main.datasheet.data_pointer_offset);
  PUCK_f();
  return PUCK_ERR_NO_ERR;
}
puck_err_code PUCKSA_f()
{
	unsigned long new_ptr=atol(puck_main.rx_msg_params[1]);
	if(new_ptr>=PUCK_DATASHEET_SIZE){
		puck_send_string(puck_error_messages[PUCK_ERR_ERR021],KEEP_MEMORY_FLAG);
	}
	else{
		puck_main.datasheet.data_pointer_offset=new_ptr;
	}
	PUCK_f();
	return PUCK_ERR_NO_ERR;
}
puck_err_code PUCKSZ_f()
{
    puck_send_integer(puck_main.datasheet.size);
	PUCK_f();
	return PUCK_ERR_NO_ERR;
}
puck_err_code PUCK_f()//done
{
	puck_send_string(puck_messages[PUCK_MSG_PUCKRDY],KEEP_MEMORY_FLAG);
    return PUCK_ERR_NO_ERR;
}
puck_err_code PUCKTY_f()
{
  return PUCK_ERR_NO_ERR;
}
puck_err_code PUCKVR_f()
{
  return PUCK_ERR_NO_ERR;
}
puck_err_code PUCKIM_f()
{
  return PUCK_ERR_NO_ERR;
}
puck_err_code PUCKVB_f()
{
  /*
  unsigned long speed=atol(substr[1]);
  if(speed==1200||speed==2400||speed==4800||speed==9600||speed==19200||speed==38400||speed==57600||speed==74880||speed==115200)
  {
    uart_send_str("YES\r");
  }
  else
  {
   uart_send_str("NO\r");
  }
  return 0;
  */
  return PUCK_ERR_NO_ERR;
}
puck_err_code PUCKSB_f()
{
  /*
  unsigned long speed=atol(substr[1]);
  if(speed==1200||speed==2400||speed==4800||speed==9600||speed==19200||speed==38400||speed==57600||speed==74880||speed==115200)
  {
   HAL_UART_DeInit(&huart1);
   huart1.Init.BaudRate = speed;
	 HAL_UART_Init(&huart1);
	 HOST_UART_Handle=&huart1;
	 HAL_UART_Receive_IT(HOST_UART_Handle,&buffer1_rx,1);
  }
  else
  {
    // return error 0010
    return 6;
  }
  return 0;
  */
  return PUCK_ERR_NO_ERR;
}
