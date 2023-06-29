#include <smac.h>
#include  "stdlib.h"
#include  "stdio.h"

smac_list smac_main_list;

extern smac_controller* uart1_smac;

//timer1=chronometer_register_new();
//chronometer_set(timer1,500);

/*
if(timer1->status==TIMER_TIMED_OUT){
chronometer_set(timer1,1000);
HAL_UART_Transmit(&huart1,"t1",2,100);
}
*/

void smac_list_init()
{
	smac_main_list.new_smac_index=0;
}



smac_controller* smac_create_new(UART_HandleTypeDef* huart,osMessageQId uart_rxQHandle,osMessageQId uart_txQHandle)
{
	if(smac_main_list.new_smac_index<NUM_OF_SMAC_CONTROLLERS){
	  smac_controller* new_dispatcher = (smac_controller*) malloc(sizeof(smac_controller));
	  smac_main_list.items[smac_main_list.new_smac_index]=new_dispatcher;

	  new_dispatcher->huart=huart;
	  new_dispatcher->rx_inQ=uart_rxQHandle;
	  new_dispatcher->tx_outQ=uart_txQHandle;
	  new_dispatcher->last_tx_message=0;
	  new_dispatcher->tx_media_status=TX_MEDIA_READY;
	  new_dispatcher->new_rx_outQ_index=0;
	  new_dispatcher->active_rx_outQ=0;
	  new_dispatcher->analyzer=0;
	  new_dispatcher->event_status=SMAC_EVNT_NONE;
	  smac_main_list.new_smac_index++;


	  uart_start_rx_it(new_dispatcher->huart,&new_dispatcher->rx_byte_raw);

      return new_dispatcher;
	}
	return 0;

}


void smac_loop(smac_controller* self_object)
{
   uint8_t b;
   while(1)
   {
	  //Events
	  if(self_object->event_status!=SMAC_EVNT_NONE){
		  for(int i=0;i<self_object->new_rx_outQ_index;i++){
			  self_object->event_callback[i](self_object->event_status);
		  }
		  self_object->event_status=SMAC_EVNT_NONE;
	  }

	  //RX part
      while(xQueueReceive(self_object->rx_inQ,&b,1)==pdTRUE){
    	  if(self_object->analyzer!=0) self_object->analyzer(b);
    	  osMessagePut(self_object->active_rx_outQ,b,1);
	  }

      //TX part
      while(uxQueueMessagesWaiting(self_object->tx_outQ)>0){
        if(self_object->tx_media_status==TX_MEDIA_READY){
          if(xQueueReceive(self_object->tx_outQ,&self_object->last_tx_message,1)==pdTRUE){
    	       self_object->tx_media_status=TX_MEDIA_BUSY;
               HAL_UART_Transmit_IT(self_object->huart,self_object->last_tx_message->data_pointer,self_object->last_tx_message->data_size);
           }
         }
      }

     osDelay(1);
   }
}

err_code smac_send(smac_controller* self_object,void* mem_ptr,uint16_t mem_size, uint8_t mem_free_after_use_flag)
{
   if(mem_size>0){
	memory_block_ptr* new_memory_block_pointer=(memory_block_ptr*) malloc(sizeof(memory_block_ptr));

	new_memory_block_pointer->data_pointer=mem_ptr;
	new_memory_block_pointer->data_size=mem_size;
	new_memory_block_pointer->free_after_use=mem_free_after_use_flag;

	if(osMessagePut(self_object->tx_outQ,(uint32_t)new_memory_block_pointer,1)==osOK){
	 return RES_OK;
	}
	else{
	 free(new_memory_block_pointer);
	 return RES_ERR;
	}
   }
   else{
		return RES_ERR;
   }
}

err_code smac_register_rx_outQ(smac_controller* self_object, osMessageQId   rx_outQ, uint8_t activeQ_flag, void (*event_callback)(smac_event event_id))
{
	if(self_object->new_rx_outQ_index<NUM_OF_RX_OUT_QUEUES){
		 self_object->rx_outQ[self_object->new_rx_outQ_index]=rx_outQ;
		 if(activeQ_flag==ACTIVE_QUEUE_FLAG) self_object->active_rx_outQ=rx_outQ;
		 self_object->event_callback[self_object->new_rx_outQ_index]=event_callback;
		 self_object->new_rx_outQ_index++;
	     return RES_OK;
	}
	return RES_ERR;
}







