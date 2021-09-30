/*
 * media_io.h
 *
 *  Created on: 14 апр. 2021 г.
 *      Author: admin
 */

#ifndef INC_IO_BUFFER_H_
#define INC_IO_BUFFER_H_

#include "cmsis_os.h"
#include "string.h"


#define IO_BUFFER_TX_BUFFER_SIZE              1024

#define IO_BUFFER_RX_PREBUFFER_SIZE      200
#define IO_BUFFER_RX_BUFFER_SIZE         1024
#define IO_BUFFER_RX_BUFFER_THR          768

#define IO_BUFFER_RX_Q_SIZE                 20
#define IO_BUFFER_TX_Q_SIZE                 20

#pragma pack (push, 1)
typedef struct
{
  uint32_t start_addr;
  uint32_t size;
} mem_ptr;
#pragma pack (pop)


#pragma pack (push, 1)
typedef struct
{

   //API variables
   osMessageQId* rx_messages_q;
   osMessageQId* tx_messages_q;
   uint8_t       rx_prebuffer[IO_BUFFER_RX_PREBUFFER_SIZE];
   uint8_t       rx_buffer[IO_BUFFER_RX_BUFFER_SIZE];
   uint8_t       tx_buffer[IO_BUFFER_TX_BUFFER_SIZE];

   //private variables
   char          EOL_symbols[10];
   uint8_t       rx_byte;
   uint8_t       buffer_status;
   uint32_t      rx_prebuffer_indx;
   uint32_t      rx_buffer_indx;
   uint32_t      rx_buffer_new_string_indx;

} io_buffer;
#pragma pack (pop)



void io_buffer_init(io_buffer* io_buffer_obj,char* EOL_symbols_string);
void io_buffer_media_read(io_buffer* io_buffer_obj,uint8_t* data,uint32_t size);

#endif /* INC_IO_BUFFER_H_ */
