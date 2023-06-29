/*
 * interrupts.c
 *
 *  Created on: 11 сент. 2021 г.
 *      Author: admin
 */

#include "interrupts.h"


extern smac_list smac_main_list;



void uart_start_rx_it(UART_HandleTypeDef* huart,uint8_t* rx_byte_ptr)
{

   HAL_UART_Receive_IT(huart,rx_byte_ptr,1);
}




void uart_rx_it(UART_HandleTypeDef *huart)
{
	for(int i=0;i<smac_main_list.new_smac_index;i++){
		if(huart==smac_main_list.items[i]->huart){
			osMessagePut(smac_main_list.items[i]->rx_inQ,smac_main_list.items[i]->rx_byte_raw,1);
			uart_start_rx_it(smac_main_list.items[i]->huart,&smac_main_list.items[i]->rx_byte_raw);
			return;
		}
	}

}

void uart_tx_it(UART_HandleTypeDef *huart)
{
	for(int i=0;i<smac_main_list.new_smac_index;i++){
		if(huart==smac_main_list.items[i]->huart){
			if(smac_main_list.items[i]->last_tx_message->free_after_use == 	FREE_MEMORY_FLAG) {
			  free(smac_main_list.items[i]->last_tx_message->data_pointer);
			}
			free(smac_main_list.items[i]->last_tx_message);
			smac_main_list.items[i]->tx_media_status=TX_MEDIA_READY;
			return;
		}
	}
}


