#ifndef __STRING_COMMANDS_H
#define __STRING_COMMANDS_H

#include "stdint.h"
#include "string.h"
#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "stdlib.h"
#include "time.h"






int new_byte_processing(uint8_t new_byte);
int string_commands_init(void);
int update_buffer(uint8_t new_byte);
int split_buffer(char** s_array,char* delimiter);
int get_cmd_pattern_index(char* cmd_string);
uint32_t get_unixtime_from_str(char* datetime_string);

int uart_send(char* str);


int GET_EPOCH_f(void);
int READ_DATETIME_f(void);
int SET_DATETIME_f(void);
int SAMPLE_f(void);
int START_f(void);
int STOP_f(void);

#endif /* __STRING_PARSING_H */

