/*
 * interrupts.h
 *
 *  Created on: 11 сент. 2021 г.
 *      Author: admin
 */

#ifndef INC_INTERRUPTS_H_
#define INC_INTERRUPTS_H_

#include <smac.h>
#include "main.h"
#include "cmsis_os.h"
#include  "stdlib.h"
#include  "stdio.h"



void uart_rx_it(UART_HandleTypeDef *huart);
void uart_tx_it(UART_HandleTypeDef *huart);
void uart_start_rx_it(UART_HandleTypeDef* huart,uint8_t* rx_byte_ptr);


#endif /* INC_INTERRUPTS_H_ */
