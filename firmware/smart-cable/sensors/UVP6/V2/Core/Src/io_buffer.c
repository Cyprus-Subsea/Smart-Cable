/*
 * media_io.c
 *
 *  Created on: 14 апр. 2021 г.
 *      Author: admin
 */
#include "io_buffer.h"



void io_buffer_init(io_buffer* io_buffer_obj,char* EOL_symbols_string)
{

	io_buffer_obj->EOL_symbols[0]=0;
	strcat(io_buffer_obj->EOL_symbols,EOL_symbols_string);

	osMessageQDef(io_buffer_rx_messages_q, IO_BUFFER_RX_Q_SIZE, mem_ptr);
	io_buffer_obj->rx_messages_q= osMessageCreate(osMessageQ(io_buffer_rx_messages_q), NULL);

	osMessageQDef(io_buffer_tx_messages_q, IO_BUFFER_TX_Q_SIZE, mem_ptr);
	io_buffer_obj->tx_messages_q= osMessageCreate(osMessageQ(io_buffer_tx_messages_q), NULL);

	/*
	osThreadDef(media_serial_txt_task, media_serial_txt_loop, osPriorityNormal, 0, 128);
	osThreadCreate(osThread(media_serial_txt_task), media_io_obj);
   */

}

void io_buffer_media_read(io_buffer* io_buffer_obj,uint8_t* data,uint32_t size)
{
	for(uint32_t i=0;i<size;i++)
	{
		io_buffer_obj->rx_prebuffer[]=data[i];
		
	}



}