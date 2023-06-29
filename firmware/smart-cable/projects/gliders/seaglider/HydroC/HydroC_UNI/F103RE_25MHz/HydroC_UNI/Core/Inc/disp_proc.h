/*
 * disp_core.h
 *
 *  Created on: Mar 23, 2023
 *      Author: admin
 */

#ifndef INC_DISP_PROC_H_
#define INC_DISP_PROC_H_

#include "main.h"
#include "cmsis_os.h"
#include "system_definitions.h"
#include "fsm.h"

#define  DISP_NUM_OF_PROCESSES          13
#define  DISP_PROC_Q_LEN                10


typedef struct{
	uint32_t event;
	void (*action)();
}EA_record_t;


typedef struct{
	uint32_t     num_of_events;
	EA_record_t* EAT;
}EA_table_t;

typedef enum{
 PROC_CREATE_NEW_Q=0,
 PROC_LINK_Q
} proc_queue_gen_flag;

typedef enum{
 PROC_STOPPED=0,
 PROC_STARTED,
 PROC_START_PENDING,
 PROC_STOP_PENDING
} proc_state;

typedef struct{
    void* func_self_object;
	osMessageQId inQ_handle;
	osMessageQId outQ_handle;
	void* func_args;

}proc_arg_t;

typedef struct{
	uint32_t  size;
	uint32_t* evnt;
}proc_events_t;

typedef struct{

	void (*proc_func_ptr)(proc_arg_t* proc_arg);
	proc_arg_t proc_arg;
	proc_state state;
	proc_events_t events;

}proc_data_t;


typedef struct{
	proc_data_t  processes[DISP_NUM_OF_PROCESSES];
	osMessageQId workers_cmd_q_Handle;
	osMessageQId events_q_Handle;
	EA_table_t*  EA_table;
}proc_dispatcher;


F_RES disp_proc_get_event(proc_dispatcher* self_object,uint32_t* event);
F_RES disp_proc_send_event(proc_dispatcher* self_object,osMessageQId out_Q,uint32_t event);

void disp_proc_init(proc_dispatcher* self_object,osMessageQId workers_cmd_q_Handle,osMessageQId events_q_Handle);
void disp_proc_init_func(proc_dispatcher* self_object,uint32_t proc_ID,void* func_ptr,
		osMessageQId inQ, proc_queue_gen_flag in_gen_flag , void* proc_self_object,uint32_t events_num,uint32_t* evnt);
F_RES disp_proc_start(proc_dispatcher* self_object,uint32_t proc_ID,void* func_args);
void disp_proc_execute(proc_dispatcher* self_object,uint32_t proc_ID);

void disp_proc_loop();

void disp_proc_set_EA_table(proc_dispatcher* self_object,EA_table_t*  EA_table);

F_RES disp_proc_check_event(uint32_t event,proc_events_t* events);


#endif /* INC_DISP_PROC_H_ */
