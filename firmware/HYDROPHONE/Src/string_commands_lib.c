#include "string_commands_lib.h"
#include "cmsis_os.h"
#include "iclisten.h"

#define NUM_OF_COMMANDS   6
#define BUFFER_SIZE       100


extern UART_HandleTypeDef huart1;
extern osMessageQId ETH_client_qHandle;
extern uint32_t epoch_counter[5];

char uart_Rx_buffer[BUFFER_SIZE];
int  current_buffer_pointer=0;
char* substr[]={NULL,NULL,NULL,NULL,NULL,NULL};


int ((*cmd_functions[NUM_OF_COMMANDS]))();
char* cmd_patterns[]={"GET_EPOCH_EVENTS","READ_DATETIME","SET_DATETIME","SAMPLE","START","STOP"};


/**
  * @brief  Add new byte to buffer and make all checks
  *
  * @note   
  *
  * @param  new_byte: one byte of data
  * @retval 1: error
  * 		0: no errors
  */
int new_byte_processing(uint8_t new_byte)
{
	
	//char index[20];
	int func_index;
	if(update_buffer(new_byte)){
		
		for(int i=0;i<6;i++)substr[i]=0x00;
		split_buffer(substr,"=");

		//HAL_UART_Transmit(&huart1,substr[0],strlen(substr[0]),10);
		//HAL_UART_Transmit(&huart1,substr[1],strlen(substr[1]),10);
		//HAL_UART_Transmit(&huart1,"\r\n",2,10);
		
		func_index=get_cmd_pattern_index(substr[0]);
		if(func_index>=0)	cmd_functions[func_index]();
		
	}
	return 0;
}

/**
  * @brief  Initialize all string functions
  *
  * @note   
  *
  * @param  
  * @retval 1: error
  * 		0: no errors
  */
int string_commands_init(void){


  cmd_functions[0] =GET_EPOCH_f;
  cmd_functions[1] =READ_DATETIME_f;
  cmd_functions[2] =SET_DATETIME_f;
  cmd_functions[3] =SAMPLE_f;
  cmd_functions[4] =START_f;
  cmd_functions[5] =STOP_f;
	return 1;
  }

  /**
  * @brief  Convert ASCII string to unixtime
  *
  * @note   format %H%M%S%d%m%Y
  *
  * @param  datetime_string: 1 byte of data for add to buffer
  * @retval unixtime
  * 		
  */
uint32_t get_unixtime_from_str(char* datetime_string)
{
  //struct date_time_str dt_str;
  char time_string_buffer[20];
   
  time_t timestamp;
  struct tm currTime;
  
  memcpy(time_string_buffer+15,datetime_string+10,strlen(datetime_string+10)+1);
	datetime_string[10]=0x00;
	memcpy(time_string_buffer+12,datetime_string+8,strlen(datetime_string+8)+1);
	datetime_string[8]=0x00;
	memcpy(time_string_buffer+9,datetime_string+6,strlen(datetime_string+6)+1);
	datetime_string[6]=0x00;
	memcpy(time_string_buffer+6,datetime_string+4,strlen(datetime_string+4)+1);
	datetime_string[4]=0x00;
	memcpy(time_string_buffer+3,datetime_string+2,strlen(datetime_string+2)+1);
  datetime_string[2]=0x00;
	memcpy(time_string_buffer,datetime_string,strlen(datetime_string)+1);
  

  
  
  currTime.tm_hour=atoi(time_string_buffer+0);
  currTime.tm_min=atoi(time_string_buffer+3);
  currTime.tm_sec=atoi(time_string_buffer+6);
  currTime.tm_mday=atoi(time_string_buffer+9);
  currTime.tm_mon=atoi(time_string_buffer+12)-1;
  currTime.tm_year=atoi(time_string_buffer+15)-1900;
	
	
	
 
 
  timestamp = mktime(&currTime);
 
  
  return timestamp;
}  
  
/**
  * @brief  Updates cyclic Rx buffer with new byte
  *
  * @note   After <CR> is detected it will replace with 0x00 for conversion to C-string
  *
  * @param  new_byte: 1 byte of data for add to buffer
  * @retval 1: <CR> symbol detected
  * 		0: no events
  */
int update_buffer(uint8_t new_byte)
{
  
		if(new_byte==0x0D||new_byte==0x0A){ //end of string detected "\n" or "\r", replace it with 0x00
			uart_Rx_buffer[current_buffer_pointer]=0x00;
			current_buffer_pointer=0;
			return 1;
		}

		uart_Rx_buffer[current_buffer_pointer]=new_byte;
		current_buffer_pointer++;
		current_buffer_pointer=current_buffer_pointer%BUFFER_SIZE;
		return 0;
     
}

/**
  * @brief  Splits cyclic Rx onto strings
  *
  * @note   Each delimiter symbol will replaced with 0x00
  *
  * @param  delimiter: delimiter symbol
* 							array: array where will places pointer on strings
  * @retval 1: <CR> symbol detected
  * 				0: no events
  */
int split_buffer(char** s_array,char* delimiter)
{
    char* p = strtok (uart_Rx_buffer,delimiter);
	  s_array[0]=p;
	for(int i=1;p!= NULL;i++){   
    p = strtok (NULL,delimiter);  
	  s_array[i]=p;
	}
	return 0;
}

/**
  * @brief  Compares string with commands array
  *
  *
  * @param  cmd_string: pointer to string for compare
  * @retval >=0: index of command
  * 		 -1: not found
  */
int get_cmd_pattern_index(char* cmd_string){

  for(int i=0;i<NUM_OF_COMMANDS;i++) if (strcmp(cmd_patterns[i],cmd_string)==0) return i;
  return -1;
 
}

/**
  * @brief  Send C-string via UART
  *
  *
  * @param  str: C-string for sending
  * @retval 0: no error
  * 		other: error
  */
int uart_send(char* str){

	HAL_UART_Transmit_DMA(&huart1,str,strlen(str));
 return 1;
}

int GET_EPOCH_f(void)
 {  
	char buffer[50];
	char cnt_buffer[10];
	cnt_buffer[0]=0x00;
	buffer[0]=0x00;
	strcat(buffer,"HP>epoch_stat:");
	for(int i=0;i<EPOCH_COUNTER_SIZE;i++)
	{
	 if(i==EPOCH_COUNTER_SIZE-1) sprintf(cnt_buffer,"%d.\0",epoch_counter[i]);
	 else sprintf(cnt_buffer,"%d,\0",epoch_counter[i]);
	 
	 strcat(buffer,cnt_buffer);
	 
	}
	strcat(buffer,"\r\nHP>");
	buffer[26]=0x00;
	uart_send(buffer);
	
	return 0;
 }
  
int READ_DATETIME_f(void)
 {
   uart_send("HP>  read_time_stub\n");
   return 0;
  }
  
int SET_DATETIME_f(void)
 {

			uint32_t  unixtime;
			char index[20];
			unixtime=get_unixtime_from_str(substr[1]);
		
      osMessagePut(ETH_client_qHandle,unixtime,10);
			uart_send("HP>"); 
 
       return 0;
  }
  
int SAMPLE_f(void)
 {
   uart_send("HP>sample\n");
   return 0;
  }
  
int START_f(void)
 {
   
   uart_send("HP>start\n");
   return 0;
  }

int STOP_f(void)
 {
   uart_send("HP>stop\n");
   return 0;
  }



