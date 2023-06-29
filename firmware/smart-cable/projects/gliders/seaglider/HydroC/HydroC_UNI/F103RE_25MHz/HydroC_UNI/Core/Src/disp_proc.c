/*
 * disp_core.c
 *
 *  Created on: Mar 23, 2023
 *      Author: admin
 */

#include "disp_proc.h"

#define  SENSOR_UART             huart1
#define  GLIDER_UART             huart5
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;

F_RES disp_proc_get_event(proc_dispatcher* self_object,uint32_t* event)
{
	osEvent res=osMessageGet(self_object->events_q_Handle,1);
	if(res.status==osEventMessage){
		*event=res.value.v;
		return F_OK;
	}
	else  if(res.status==osEventTimeout)return F_TIMEOUT;
}

F_RES disp_proc_send_event(proc_dispatcher* self_object,osMessageQId out_Q,uint32_t event)
{
	if(osMessagePut(out_Q,(uint32_t)event,1))
	{
     return F_OK;
	}
	return F_ERR;
}




void disp_proc_init(proc_dispatcher* self_object,osMessageQId workers_cmd_q_Handle,osMessageQId events_q_Handle)
{
	self_object->workers_cmd_q_Handle=workers_cmd_q_Handle;
	self_object->events_q_Handle=events_q_Handle;
	fsm_init();
}

void disp_proc_init_func(proc_dispatcher* self_object,uint32_t proc_ID,void* func_ptr,
		osMessageQId inQ, proc_queue_gen_flag in_gen_flag , void* proc_self_object,uint32_t events_num,uint32_t* evnt)
{

	if(in_gen_flag==PROC_CREATE_NEW_Q){
		osMessageQDef(proc_func_in_q, DISP_PROC_Q_LEN, uint32_t);
		self_object->processes[proc_ID].proc_arg.inQ_handle = osMessageCreate(osMessageQ(proc_func_in_q), NULL);
	}
	else{
		self_object->processes[proc_ID].proc_arg.inQ_handle = inQ;
	}

	self_object->processes[proc_ID].proc_arg.outQ_handle = self_object->events_q_Handle;

	self_object->processes[proc_ID].proc_arg.func_self_object=proc_self_object;
	self_object->processes[proc_ID].proc_func_ptr=func_ptr;
	self_object->processes[proc_ID].events.size=events_num;
	self_object->processes[proc_ID].events.evnt=evnt;
	self_object->processes[proc_ID].state=PROC_STOPPED;
}

F_RES  disp_proc_start(proc_dispatcher* self_object,uint32_t proc_ID,void* func_args)
{
	if(self_object->processes[proc_ID].state!=PROC_STARTED&&self_object->processes[proc_ID].state!=PROC_START_PENDING){
	  self_object->processes[proc_ID].proc_arg.func_args=func_args;
	  self_object->processes[proc_ID].state=PROC_START_PENDING;
	  osMessagePut(self_object->workers_cmd_q_Handle,proc_ID,1);
	  return F_OK;
	}
	return F_ERR;
}

void disp_proc_execute(proc_dispatcher* self_object,uint32_t proc_ID)
{
	self_object->processes[proc_ID].state=PROC_STARTED;
	self_object->processes[proc_ID].proc_func_ptr(&self_object->processes[proc_ID].proc_arg);
    //move dependent event from in to out here
	self_object->processes[proc_ID].state=PROC_STOPPED;
}

void disp_proc_set_EA_table(proc_dispatcher* self_object,EA_table_t*  EA_table)
{
	self_object->EA_table=EA_table;
}

extern UART_HandleTypeDef huart5;
void disp_proc_loop(proc_dispatcher* self_object)
{
    uint32_t last_event;
    char tt[100];

    for(;;)
    {
     if(disp_proc_get_event(self_object,&last_event)==F_OK){

      for(int i=0;i<DISP_NUM_OF_PROCESSES;i++)
      {
        if(self_object->processes[i].state==PROC_STARTED){
        	if(disp_proc_check_event(last_event,&self_object->processes[i].events)==F_OK){
        		disp_proc_send_event(self_object,self_object->processes[i].proc_arg.inQ_handle,last_event);
        	}
        }
      }
      //EA table check
      //sprintf(tt,"Event:%d\n",last_event);
      //HAL_UART_Transmit(&huart5,tt,strlen(tt),100);
      for(int y=0;y<(self_object->EA_table->num_of_events);y++)
      {
          //sprintf(tt,"Event:%d\n",self_object->EA_table->EAT[y].event);
          //HAL_UART_Transmit(&huart5,tt,strlen(tt),100);
    	  if(last_event==self_object->EA_table->EAT[y].event) {
    		  self_object->EA_table->EAT[y].action();
    	  }
      }
     }
    }
}

F_RES disp_proc_check_event(uint32_t event,proc_events_t* events)
{
	for(int i=0;i<events->size;i++){
		if(events->evnt[i]==event) return F_OK;
	}
	return F_ERR;
}
