/*
 * fsm.c
 *
 *  Created on: Mar 26, 2023
 *      Author: admin
 */

#include "fsm.h"


extern proc_dispatcher dispatcher1;
extern uvp6 uvp6_sensor1;
extern slocum glider1;

extern osMessageQId proc_inQ1Handle;
extern osMessageQId proc_inQ2Handle;
extern osMessageQId proc_inQ3Handle;
extern osMessageQId proc_inQ4Handle;
extern osMessageQId proc_inQ5Handle;
extern osMessageQId proc_inQ6Handle;
extern osMessageQId proc_inQ7Handle;
extern osMessageQId proc_inQ8Handle;
extern osMessageQId proc_inQ9Handle;
extern osMessageQId proc_inQ10Handle;
extern osMessageQId proc_inQ11Handle;


//----------------  VAR definition BEGIN  ----------------------

uint32_t     lpm_buffer_num_of_msgs=0;
lpm_data_str lpm_messages_buffer;
char         lmp_data_bloc_msg[500];

float lpm_bloc_depth_size=0.0;
float lpm_bloc_depth_start=0.0;
float bloc_GPS_lat=0.0;
float bloc_GPS_lon=0.0;



//----------------  VAR definition END -------------------------

//-------------------BEGIN PROC FUNCTIONS--------------------------------

uint32_t  uvp6_power_on_events[]={0};
#define   uvp6_power_on_events_num                                             0

void uvp6_power_on_proc(proc_arg_t* proc_arg)
{
 uvp6* uvp6_obj=proc_arg->func_self_object;
 if(uvp6_obj->power_status!=UVP6_POWER_IS_ON){
   HAL_GPIO_WritePin(SENSOR_PWR_CTRL_GPIO_Port, SENSOR_PWR_CTRL_Pin, GPIO_PIN_SET);
   fsm_generate_event(proc_arg->outQ_handle,UVP6_EVNT_POWERED_ON);
   uvp6_obj->power_status=UVP6_POWER_IS_ON;
 }
}

uint32_t  uvp6_power_off_events[]={0};
#define   uvp6_power_off_events_num                                            0

void uvp6_power_off_proc(proc_arg_t* proc_arg)
{
 uvp6* uvp6_obj=proc_arg->func_self_object;
 if(uvp6_obj->power_status!=UVP6_POWER_IS_OFF){
  HAL_GPIO_WritePin(SENSOR_PWR_CTRL_GPIO_Port, SENSOR_PWR_CTRL_Pin, GPIO_PIN_RESET);
  fsm_generate_event(proc_arg->outQ_handle,UVP6_EVNT_POWERED_OFF);
  uvp6_obj->power_status=UVP6_POWER_IS_OFF;
 }
}

uint32_t  uvp6_stop_events[]={UVP6_EVNT_STOP_ACK_RCVD,
		                      UVP6_EVNT_START_ERR_RCVD};
#define   uvp6_stop_events_num                 2

void uvp6_stop_proc(proc_arg_t* proc_arg)
{
	uvp6* uvp6_obj=proc_arg->func_self_object;
	uint8_t retry_timeout_num=UVP6_CONNECTION_RETRIES;
	uint8_t retry_busy_num=UVP6_CONNECTION_RETRIES;
	uint32_t in_event;
	F_RES    res;

	do{
	 uvp6_send_cmd(uvp6_obj,UVP6_CMD_STOP_ACQ,NULL);
	 res=fsm_take_event(proc_arg->inQ_handle,&in_event,UVP6_RESPONSE_TIMEOUT);
	 if(res==F_OK){
		  if( in_event==UVP6_EVNT_STOP_ACK_RCVD)break;
		  else if( in_event==UVP6_EVNT_START_ERR_RCVD){
			  retry_busy_num--;
			  retry_timeout_num=UVP6_CONNECTION_RETRIES;
		  }
	 }
	 else if(res==F_TIMEOUT) retry_timeout_num--;
	}while(retry_timeout_num!=0 && retry_busy_num!=0);

	if(retry_busy_num==0){
		fsm_generate_event(proc_arg->outQ_handle,UVP6_EVNT_ERR_BUSY);
		fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_UVP6_NOT_STOPPED);
	}
	if(retry_timeout_num==0){
		fsm_generate_event(proc_arg->outQ_handle,UVP6_EVNT_ERR_NO_RESPONSE);
		fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_UVP6_NOT_STOPPED);
	}
	else fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_UVP6_STOPPED);
}

uint32_t  uvp6_startL_events[]={UVP6_EVNT_START_ACK_RCVD,
		                        UVP6_EVNT_START_ERR_RCVD};
#define   uvp6_startL_events_num                     2

void uvp6_startL_proc(proc_arg_t* proc_arg)
{
	uvp6* uvp6_obj=proc_arg->func_self_object;
	uint8_t retry_timeout_num=UVP6_CONNECTION_RETRIES;
	uint8_t retry_busy_num=UVP6_CONNECTION_RETRIES;
	uint32_t in_event;
	F_RES    res;

	do{
	 uvp6_send_cmd(uvp6_obj,UVP6_CMD_START_L_ACQ,NULL);
	 res=fsm_take_event(proc_arg->inQ_handle,&in_event,UVP6_RESPONSE_TIMEOUT);
	 if(res==F_OK){
		  if( in_event==UVP6_EVNT_START_ACK_RCVD)break;
		  else if( in_event==UVP6_EVNT_START_ERR_RCVD){
			  retry_busy_num--;
			  retry_timeout_num=UVP6_CONNECTION_RETRIES;
			  // here should be more detailed evnt
		  }
	 }
	 else if(res==F_TIMEOUT) retry_timeout_num--;
	}while(retry_timeout_num!=0 && retry_busy_num!=0);

	if(retry_busy_num==0){
		fsm_generate_event(proc_arg->outQ_handle,UVP6_EVNT_ERR_BUSY);
		fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_UVP6_NOT_STARTED);
	}
	if(retry_timeout_num==0){
		fsm_generate_event(proc_arg->outQ_handle,UVP6_EVNT_ERR_NO_RESPONSE);
		fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_UVP6_NOT_STARTED);
	}
	else fsm_generate_event(proc_arg->outQ_handle,UVP6_EVNT_STARTED_LOW_PROFILE);
}


uint32_t  uvp6_startH_events[]={UVP6_EVNT_START_ACK_RCVD,
		                        UVP6_EVNT_START_ERR_RCVD};
#define   uvp6_startH_events_num                      2

void uvp6_startH_proc(proc_arg_t* proc_arg)
{
	uvp6* uvp6_obj=proc_arg->func_self_object;
	uint8_t retry_timeout_num=UVP6_CONNECTION_RETRIES;
	uint8_t retry_busy_num=UVP6_CONNECTION_RETRIES;
	uint32_t in_event;
	F_RES    res;

	do{
	 uvp6_send_cmd(uvp6_obj,UVP6_CMD_START_H_ACQ,NULL);
	 res=fsm_take_event(proc_arg->inQ_handle,&in_event,UVP6_RESPONSE_TIMEOUT);
	 if(res==F_OK){
		  if( in_event==UVP6_EVNT_START_ACK_RCVD)break;
		  else if( in_event==UVP6_EVNT_START_ERR_RCVD){
			  retry_busy_num--;
			  retry_timeout_num=UVP6_CONNECTION_RETRIES;
			  // here should be more detailed evnt
		  }
	 }
	 else if(res==F_TIMEOUT) retry_timeout_num--;
	}while(retry_timeout_num!=0 && retry_busy_num!=0);

	if(retry_busy_num==0){
		fsm_generate_event(proc_arg->outQ_handle,UVP6_EVNT_ERR_BUSY);
		fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_UVP6_NOT_STARTED);
	}
	if(retry_timeout_num==0){
		fsm_generate_event(proc_arg->outQ_handle,UVP6_EVNT_ERR_NO_RESPONSE);
		fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_UVP6_NOT_STARTED);
	}
	else fsm_generate_event(proc_arg->outQ_handle,UVP6_EVNT_STARTED_HIGH_PROFILE);
}


uint32_t  uvp6_lpm_data_agg_events[]={UVP6_EVNT_LPM_DATA_RCVD,
		                              UVP6_EVNT_LPM_DATA_BLOC_END_REACHED,
									  FSM_EVNT_READY_FOR_LPM_DATA};
#define   uvp6_lpm_data_agg_events_num                3

void uvp6_lpm_data_agg_proc(proc_arg_t* proc_arg)
{
	uvp6* uvp6_obj=proc_arg->func_self_object;
	uint32_t in_event;

	while(1)
	{
	 if(fsm_take_event(proc_arg->inQ_handle,&in_event,osWaitForever)==F_OK){
	   if(in_event==UVP6_EVNT_LPM_DATA_RCVD){
		 lpm_buffer_num_of_msgs++;
		 if(lpm_buffer_num_of_msgs==1){
		   bloc_GPS_lat=glider1.gps_lat;
		   bloc_GPS_lon=glider1.gps_lon;
		   memcpy(&lpm_messages_buffer,&uvp6_obj->lpm_data,sizeof(uvp6_obj->lpm_data));
		   for(int y=0;y<UVP6_NUM_OF_CATEGORIES;y++){
			lpm_messages_buffer.grey_levels[y]=(lpm_messages_buffer.grey_levels[y]*lpm_messages_buffer.data[y]);
		   }
		 }
		 else lpm_sum_messages(uvp6_obj,&lpm_messages_buffer);
		 if(lpm_bloc_depth_size==0.0)
		 {
			if(lpm_aggregate_and_close_bloc(uvp6_obj,lmp_data_bloc_msg,&lpm_messages_buffer,&lpm_buffer_num_of_msgs)==F_OK){
			    fsm_generate_event(proc_arg->outQ_handle,UVP6_EVNT_LPM_AGG_DATA_READY);
			}
		 }
	   }
	   else if(in_event==UVP6_EVNT_LPM_DATA_BLOC_END_REACHED){
		    if(lpm_buffer_num_of_msgs>0){
			  if(lpm_aggregate_and_close_bloc(uvp6_obj,lmp_data_bloc_msg,&lpm_messages_buffer,&lpm_buffer_num_of_msgs)==F_OK){
				fsm_generate_event(proc_arg->outQ_handle,UVP6_EVNT_LPM_AGG_DATA_READY);
			  }
		    }
		    else fsm_generate_event(proc_arg->outQ_handle,UVP6_EVNT_LPM_AGG_NOTHING_TO_SEND);
	   }
	   else if(in_event==FSM_EVNT_READY_FOR_LPM_DATA){
		   lpm_buffer_num_of_msgs=0;
		   fsm_generate_event(proc_arg->outQ_handle,UVP6_EVNT_LPM_BUFFER_CLEARED);
	   }
	 }
	}
}

uint32_t  slocum_open_file_events[]={SLOCUM_EVNT_FILE_CREATE_ERR_RCVD};
#define   slocum_open_file_events_num                                 1

void slocum_open_file_proc(proc_arg_t* proc_arg)
{
	slocum* slocum_obj=proc_arg->func_self_object;
	uint8_t retry_num=SLOCUM_CONNECTION_RETRIES;

	memory_region_pointer ptr1;
	char filename[30];
	uint32_t in_event;
	F_RES res;

	sprintf(filename,"%s%u.uv6",slocum_obj->mission_id+2,slocum_obj->dive_climb_counter);
    ptr1.start_addr=filename;
	ptr1.size=strlen(filename);

	do{
	   slocum_send_cmd(slocum_obj,SLOCUM_CMD_OPEN_FILE_W,&ptr1);

	   res=fsm_take_event(proc_arg->inQ_handle,&in_event,SLOCUM_RESPONSE_TIMEOUT);

	   if(res==F_OK){
	    if( in_event==SLOCUM_EVNT_FILE_CREATE_ERR_RCVD){
	    	fsm_generate_event(proc_arg->outQ_handle,SLOCUM_EVNT_FILE_OPEN_ERR);
	    	return;
	    }
	   }
	   else if(res==F_TIMEOUT) retry_num--;

	}while(retry_num>0);

	if(retry_num==0) fsm_generate_event(proc_arg->outQ_handle,SLOCUM_EVNT_FILE_OPENED);
}


uint32_t  slocum_close_file_events[]={SLOCUM_EVNT_FILE_CREATE_ERR_RCVD};
#define   slocum_close_file_events_num                                 1

void slocum_close_file_proc(proc_arg_t* proc_arg)
{
	slocum* slocum_obj=proc_arg->func_self_object;
	uint8_t retry_num=SLOCUM_CONNECTION_RETRIES;

	// close file FC closed by FW to empty file, glider's bug

	memory_region_pointer ptr1;
	char filename[30];
	uint32_t in_event;
	F_RES res;

	sprintf(filename,"fcstub.tmp");
    ptr1.start_addr=filename;
	ptr1.size=strlen(filename);

	do{
	   slocum_send_cmd(slocum_obj,SLOCUM_CMD_OPEN_FILE_W,&ptr1);

	   res=fsm_take_event(proc_arg->inQ_handle,&in_event,SLOCUM_RESPONSE_TIMEOUT);

	   if(res==F_OK){
	    if( in_event==SLOCUM_EVNT_FILE_CREATE_ERR_RCVD){
	    	fsm_generate_event(proc_arg->outQ_handle,SLOCUM_EVNT_FILE_CLOSE_ERR);
	    	return;
	    }
	   }
	   else if(res==F_TIMEOUT) retry_num--;

	}while(retry_num>0);

	if(retry_num==0) fsm_generate_event(proc_arg->outQ_handle,SLOCUM_EVNT_FILE_CLOSED);
}

uint32_t  slocum_write_to_file_events[]={SLOCUM_EVNT_FILE_WRITE_S_ERR_RCVD};
#define   slocum_write_to_file_events_num                                 1
void slocum_write_to_file_proc(proc_arg_t* proc_arg)
{
	slocum* slocum_obj=proc_arg->func_self_object;

    uint32_t in_event;
    memory_region_pointer ptr1;
    F_RES res;

    ptr1.start_addr=lmp_data_bloc_msg;
	ptr1.size=strlen(lmp_data_bloc_msg);
  	slocum_send_cmd(slocum_obj,SLOCUM_CMD_WRITE_FILE_DATA,&ptr1);

  	res=fsm_take_event(proc_arg->inQ_handle,&in_event,SLOCUM_RESPONSE_TIMEOUT);
    if(res==F_OK){
       if( in_event==SLOCUM_EVNT_FILE_WRITE_S_ERR_RCVD){
      	fsm_generate_event(proc_arg->outQ_handle,SLOCUM_EVNT_FILE_WRITE_ERR);
      	return;
       }
    }
    else if(res==F_TIMEOUT) fsm_generate_event(proc_arg->outQ_handle,SLOCUM_EVNT_FILE_DATA_WRITTEN);
}

uint32_t slocum_depth_analyzer_events[]={SLOCUM_EVNT_DEPTH_RCVD,
		                                 UVP6_EVNT_LPM_AGG_DATA_READY,
										 UVP6_EVNT_LPM_BUFFER_CLEARED};
#define slocum_depth_analyzer_events_num                      3


void slocum_depth_analyzer_proc(proc_arg_t* proc_arg){

	slocum* slocum_obj=proc_arg->func_self_object;
	uint32_t in_event;
	uvp6_sensor1.profile_zone=UVP6_PROFILE_H;

	while(1)
	{
	 if(fsm_take_event(proc_arg->inQ_handle,&in_event,osWaitForever)==F_OK){
	   if(in_event==SLOCUM_EVNT_DEPTH_RCVD){
		   if(slocum_obj->prev_depth>UVP6_DEPTH_LH_PROFILE&&slocum_obj->last_depth<=UVP6_DEPTH_LH_PROFILE){
			   uvp6_sensor1.profile_zone=UVP6_PROFILE_H;
			   fsm_generate_event(proc_arg->outQ_handle,UVP6_EVNT_DEPTH_LH_REACHED_FROM_BOTTOM);
		   }

		   else if(slocum_obj->prev_depth<UVP6_DEPTH_LH_PROFILE&&slocum_obj->last_depth>=UVP6_DEPTH_LH_PROFILE){
			   uvp6_sensor1.profile_zone=UVP6_PROFILE_L;
			   fsm_generate_event(proc_arg->outQ_handle,UVP6_EVNT_DEPTH_LH_REACHED_FROM_TOP);
		   }

          if(glider1.last_depth<lpm_bloc_depth_start){
            if(glider1.last_depth<(lpm_bloc_depth_start-lpm_bloc_depth_size)||glider1.last_depth<2.0){
              lpm_bloc_depth_start=glider1.last_depth;

       	      if(lpm_bloc_depth_start>1000.0) lpm_bloc_depth_size=20.0;
       	      else if(lpm_bloc_depth_start>500.0) lpm_bloc_depth_size=20.0;
       	      else if(lpm_bloc_depth_start>100.0) lpm_bloc_depth_size=10.0;
       	      else if(lpm_bloc_depth_start>2.0) lpm_bloc_depth_size=5.0;
       	      else lpm_bloc_depth_size=0.0;
       	      fsm_generate_event(proc_arg->outQ_handle,UVP6_EVNT_LPM_DATA_BLOC_END_REACHED);
            }
          }
          else if (glider1.last_depth>lpm_bloc_depth_start){
            if(glider1.last_depth>(lpm_bloc_depth_start+lpm_bloc_depth_size)){
              lpm_bloc_depth_start=glider1.last_depth;

       	      if(lpm_bloc_depth_start>1000.0) lpm_bloc_depth_size=20.0;
       	      else if(lpm_bloc_depth_start>500.0) lpm_bloc_depth_size=20.0;
       	      else if(lpm_bloc_depth_start>100.0) lpm_bloc_depth_size=10.0;
       	      else if(lpm_bloc_depth_start>2.0) lpm_bloc_depth_size=5.0;
       	      else lpm_bloc_depth_size=0.0;
       	      fsm_generate_event(proc_arg->outQ_handle,UVP6_EVNT_LPM_DATA_BLOC_END_REACHED);
            }
          }

	   }
	   else if(in_event==UVP6_EVNT_LPM_AGG_DATA_READY ||in_event==UVP6_EVNT_LPM_BUFFER_CLEARED){
		   lpm_bloc_depth_start=glider1.last_depth;
		   if(lpm_bloc_depth_start>1000.0) lpm_bloc_depth_size=20.0;
		   else if(lpm_bloc_depth_start>500.0) lpm_bloc_depth_size=20.0;
		   else if(lpm_bloc_depth_start>100.0) lpm_bloc_depth_size=10.0;
		   else if(lpm_bloc_depth_start>2.0) lpm_bloc_depth_size=5.0;
		   else lpm_bloc_depth_size=0.0;
	   }


	 }
	}
}

uint32_t slocum_final_depth_state_analyzer_events[]={SLOCUM_EVNT_FINAL_DEPTH_AT_SURFACE_RCVD,
		                                             SLOCUM_EVNT_FINAL_DEPTH_DIVING_RCVD,
													 SLOCUM_EVNT_FINAL_DEPTH_CLIMBING_RCVD};
#define slocum_final_depth_state_analyzer_events_num                      3


void slocum_final_depth_state_analyzer_proc(proc_arg_t* proc_arg){

	slocum* slocum_obj=proc_arg->func_self_object;
	uint32_t in_event;


	while(1)
	{
	 if(fsm_take_event(proc_arg->inQ_handle,&in_event,osWaitForever)==F_OK){
	   if(in_event==SLOCUM_EVNT_FINAL_DEPTH_AT_SURFACE_RCVD){
		   if(slocum_obj->dcs_state==GLIDER_STATE_CLIMBING){
			   slocum_obj->dcs_state=GLIDER_STATE_AT_SURFACE;
			   fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_CLIMB_TO_SURFACE);
		   }
	   }
	   else if(in_event==SLOCUM_EVNT_FINAL_DEPTH_DIVING_RCVD){
		   if(slocum_obj->dcs_state==GLIDER_STATE_AT_SURFACE){
			   slocum_obj->dcs_state=GLIDER_STATE_DIVING;
			   fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_SURFACE_TO_DIVE);
		   }
		   else if(slocum_obj->dcs_state==GLIDER_STATE_CLIMBING){
			   slocum_obj->dcs_state=GLIDER_STATE_DIVING;
			   fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_CLIMB_TO_DIVE);
		   }
	   }
	   else if(in_event==SLOCUM_EVNT_FINAL_DEPTH_CLIMBING_RCVD){
		   if(slocum_obj->dcs_state==GLIDER_STATE_DIVING){
			   slocum_obj->dcs_state=GLIDER_STATE_CLIMBING;
			   fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_DIVE_TO_CLIMB);
		   }

	   }
	 }
	}
}


uint32_t  slocum_send_evnt_events[]={  	FSM_CHANGE_STATE_TO_S0,
		                                FSM_CHANGE_STATE_TO_S1,
		                                FSM_CHANGE_STATE_TO_S2,
		                                FSM_CHANGE_STATE_TO_S3,
		                                FSM_CHANGE_STATE_TO_S4,
		                                FSM_CHANGE_STATE_TO_S5,
		                                FSM_CHANGE_STATE_TO_S6,
		                                FSM_CHANGE_STATE_TO_S7,
										FSM_EVNT_ERR_SENSOR_NOT_RESPOND,
										FSM_EVNT_UVP6_NOT_STARTED,
										FSM_EVNT_UVP6_NOT_STOPPED,
										FSM_EVNT_UVP6_STOPPED,
										FSM_EVNT_READY_FOR_PRE_CONFIG,
										FSM_EVNT_PRE_CONFIG_FINISHED,
										FSM_EVNT_READY_FOR_LPM_DATA,
										FSM_EVNT_WAIT_FOR_AGG_DATA,
										FSM_EVNT_SURFACED,
										FSM_EVNT_CLIMB_TO_SURFACE,
										FSM_EVNT_SURFACE_TO_DIVE,
										FSM_EVNT_CLIMB_TO_DIVE,
										FSM_EVNT_DIVE_TO_CLIMB,
										FSM_EVNT_SYS_ERROR,
										UVP6_EVNT_DEPTH_LH_REACHED_FROM_TOP,
										UVP6_EVNT_DEPTH_LH_REACHED_FROM_BOTTOM,
										UVP6_EVNT_STOP_ACK_RCVD,
										UVP6_EVNT_START_ERR_RCVD,
										UVP6_EVNT_START_ACK_RCVD

                                  };
#define   slocum_send_evnt_events_num                                 27
void slocum_send_evnt_proc(proc_arg_t* proc_arg)
{
	slocum* slocum_obj=proc_arg->func_self_object;
    uint32_t in_event;
    F_RES res;
    while(1)
    {
  	 res=fsm_take_event(proc_arg->inQ_handle,&in_event,osWaitForever);
     if(res==F_OK){
    	 slocum_send_evnt(slocum_obj,in_event);
     }
    }
}
//-------------------END PROC FUNCTIONS--------------------------------


//---------------- EA tables description BEGIN ---------------------
EA_record_t EAT_S0[]={{SLOCUM_EVNT_DOS_OFF,&start_mission},
		              {SLOCUM_EVNT_USER_PARAM_PWR_ON_RCVD,&set_power_on},
		              {SLOCUM_EVNT_USER_PARAM_PWR_OFF_RCVD,&set_power_off}
                     };

EA_record_t EAT_S1[]={{SLOCUM_EVNT_USER_PARAM_PWR_ON_RCVD,&set_power_on},
                      {SLOCUM_EVNT_USER_PARAM_PWR_OFF_RCVD,&set_power_off},
					  {FSM_EVNT_SURFACE_TO_DIVE,&S1_wait_mission_id},
					  {SLOCUM_EVNT_DOS_ON,&go_dos}
                     };

EA_record_t EAT_S3[]={{SLOCUM_EVNT_MISSION_ID_RCVD,&S3_start_pre_conf},
		              {SLOCUM_EVNT_DOS_ON,&go_dos}
                     };

EA_record_t EAT_S4[]={{FSM_EVNT_READY_FOR_PRE_CONFIG,&open_file},
		              {SLOCUM_EVNT_FILE_OPENED,&clear_lpm_data},
					  {UVP6_EVNT_LPM_BUFFER_CLEARED,&config_sensor},
					  {UVP6_EVNT_STARTED_LOW_PROFILE,&start_measurement},
					  {UVP6_EVNT_STARTED_HIGH_PROFILE,&start_measurement},
                      {FSM_EVNT_UVP6_NOT_STARTED,&stop_and_report},
					  {SLOCUM_EVNT_DOS_ON,&stop_and_go_dos}
                     };

EA_record_t EAT_S5[]={{UVP6_EVNT_LPM_AGG_DATA_READY,&write_data},
					  {UVP6_EVNT_DEPTH_LH_REACHED_FROM_TOP,&uvp6_stop},
					  {UVP6_EVNT_DEPTH_LH_REACHED_FROM_BOTTOM,&uvp6_stop},
					  {FSM_EVNT_UVP6_STOPPED,&config_sensor},
					  {FSM_EVNT_DIVE_TO_CLIMB,&stop_measurements},
					  {FSM_EVNT_CLIMB_TO_DIVE,&stop_measurements},
					  {FSM_EVNT_CLIMB_TO_SURFACE,&stop_measurements},
					  {FSM_EVNT_UVP6_NOT_STARTED,&stop_and_report},
					  {FSM_EVNT_UVP6_NOT_STOPPED,&stop_and_report},
					  {SLOCUM_EVNT_DOS_ON,&stop_and_go_dos}
                     };

EA_record_t EAT_S6[]={{FSM_EVNT_UVP6_STOPPED,&save_data},
		              {FSM_EVNT_UVP6_NOT_STOPPED,&stop_and_report},
		              {UVP6_EVNT_LPM_AGG_DATA_READY,&write_data},
					  {SLOCUM_EVNT_FILE_DATA_WRITTEN,&close_file},
					  {UVP6_EVNT_LPM_AGG_NOTHING_TO_SEND,&close_file},
					  {SLOCUM_EVNT_FILE_CLOSED,&S6_select_state},
					  {SLOCUM_EVNT_DOS_ON,&stop_and_go_dos}
                     };

EA_record_t EAT_S7[]={{FSM_EVNT_UVP6_STOPPED,&close_file},
		              {FSM_EVNT_UVP6_NOT_STOPPED,&close_file},
					  {SLOCUM_EVNT_FILE_CLOSED,&go_dos},
					  {SLOCUM_EVNT_FILE_CLOSE_ERR,&go_dos}
                     };


EA_table_t fsm_S0_dos_active={(3),EAT_S0};
EA_table_t fsm_S1_mission_start={(4),EAT_S1};
EA_table_t fsm_S3_mission_id={(2),EAT_S3};
EA_table_t fsm_S4_pre_conf={(7),EAT_S4};
EA_table_t fsm_S5_measure={(10),EAT_S5};
EA_table_t fsm_S6_post_conf={(7),EAT_S6};
EA_table_t fsm_S7_stop_and_go_dos={(4),EAT_S7};

//---------------- EA tables description END ----------------------


//-----------------BEGIN ACTION FUNCTIONS------------------------------
void  init_act()
{
	disp_proc_set_EA_table(&dispatcher1,&fsm_S0_dos_active);
	fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_S0);
	if(disp_proc_start(&dispatcher1, SLOCUM_PROC_DEPTH_ANALYZER, NULL)==F_ERR){/*err processing*/}
	if(disp_proc_start(&dispatcher1, UVP6_PROC_LPM_DATA_AGG, NULL)==F_ERR){/*err processing*/}
	if(disp_proc_start(&dispatcher1, SLOCUM_PROC_EVENTS, NULL)==F_ERR){/*err processing*/}
	if(disp_proc_start(&dispatcher1, SLOCUM_PROC_FINAL_DEPTH_ANALYZER, NULL)==F_ERR){/*err processing*/}
}

void start_mission()
{
	disp_proc_set_EA_table(&dispatcher1,&fsm_S1_mission_start);
	glider1.dcs_state=GLIDER_STATE_AT_SURFACE;  // "fake state" for trigger dive
	fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_S1);
}
void go_dos()
{
	if(disp_proc_start(&dispatcher1, UVP6_PROC_POWER_OFF, NULL)==F_ERR){/*err processing*/}
	disp_proc_set_EA_table(&dispatcher1,&fsm_S0_dos_active);
	fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_S0);
}
void stop_and_go_dos()
{
	disp_proc_set_EA_table(&dispatcher1,&fsm_S7_stop_and_go_dos);
	fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_S7);
	if(disp_proc_start(&dispatcher1, UVP6_PROC_STOP, NULL)==F_ERR){/*err processing*/}
}
//-------
void set_power_on()
{
	if(disp_proc_start(&dispatcher1, UVP6_PROC_POWER_ON, NULL)==F_ERR){/*err processing*/}
}
void  set_power_off()
{
	if(disp_proc_start(&dispatcher1, UVP6_PROC_POWER_OFF, NULL)==F_ERR){/*err processing*/}
}

void save_data()
{
	fsm_generate_event(dispatcher1.events_q_Handle,UVP6_EVNT_LPM_DATA_BLOC_END_REACHED);
}

void open_file()
{
	if(disp_proc_start(&dispatcher1, SLOCUM_PROC_FILE_OPEN, NULL)==F_ERR){/*err processing*/}
}
void config_sensor()
{
	if(uvp6_sensor1.profile_zone==UVP6_PROFILE_H){
	 if(disp_proc_start(&dispatcher1, UVP6_PROC_START_HIGH_PROFILE, NULL)==F_ERR){/*err processing*/}
	}
	else if(uvp6_sensor1.profile_zone==UVP6_PROFILE_L){
	 if(disp_proc_start(&dispatcher1, UVP6_PROC_START_LOW_PROFILE, NULL)==F_ERR){/*err processing*/}
	}
}

void stop_and_report()
{
	glider1.dive_climb_counter++;
	glider1.dive_climb_counter%=SLOCUM_DIVE_COUNTER_MAX;

	close_file();
	set_power_off();

	disp_proc_set_EA_table(&dispatcher1,&fsm_S1_mission_start);
	fsm_generate_event(dispatcher1.events_q_Handle,FSM_EVNT_ERR_SENSOR_NOT_RESPOND);
	fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_S1);

}

void clear_lpm_data()
{

	fsm_generate_event(dispatcher1.events_q_Handle,FSM_EVNT_READY_FOR_LPM_DATA);

}
void write_data()
{
	if(disp_proc_start(&dispatcher1, SLOCUM_PROC_FILE_WRITE, NULL)==F_ERR){/*err processing*/}
}
void uvp6_stop()
{
	if(disp_proc_start(&dispatcher1, UVP6_PROC_STOP, NULL)==F_ERR){/*err processing*/}
}
void uvp6_start_H_profile()
{
	if(disp_proc_start(&dispatcher1, UVP6_PROC_START_HIGH_PROFILE, NULL)==F_ERR){/*err processing*/}
}
void uvp6_start_L_profile()
{
	if(disp_proc_start(&dispatcher1, UVP6_PROC_START_LOW_PROFILE, NULL)==F_ERR){/*err processing*/}
}

//------------

void  S1_powered_on()
{
	disp_proc_set_EA_table(&dispatcher1,&fsm_S1_mission_start);
	fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_S1);
}

void  S1_powered_off()
{
	//send response to glider here
}

//-------

void S1_wait_mission_id()
{
	if(uvp6_sensor1.power_status==UVP6_POWER_IS_ON){
	   disp_proc_set_EA_table(&dispatcher1,&fsm_S3_mission_id);
	   fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_S3);
	}
}

//-------

void S3_start_pre_conf()
{
	disp_proc_set_EA_table(&dispatcher1,&fsm_S4_pre_conf);
	fsm_generate_event(dispatcher1.events_q_Handle,FSM_EVNT_READY_FOR_PRE_CONFIG);
	fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_S4);
}

//-------

void start_measurement()
{
	disp_proc_set_EA_table(&dispatcher1,&fsm_S5_measure);
	fsm_generate_event(dispatcher1.events_q_Handle,FSM_EVNT_PRE_CONFIG_FINISHED);
	fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_S5);
}

//------

void stop_measurements()
{
	if(disp_proc_start(&dispatcher1, UVP6_PROC_STOP, NULL)==F_ERR){/*err processing*/}
	disp_proc_set_EA_table(&dispatcher1,&fsm_S6_post_conf);
	fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_S6);
}

//-------

void close_file()
{
	if(disp_proc_start(&dispatcher1, SLOCUM_PROC_FILE_CLOSE, NULL)==F_ERR){/*err processing*/}
}

void S6_select_state()
{
	glider1.dive_climb_counter++;
	glider1.dive_climb_counter%=SLOCUM_DIVE_COUNTER_MAX;


	if(glider1.dcs_state==GLIDER_STATE_AT_SURFACE){
		disp_proc_set_EA_table(&dispatcher1,&fsm_S1_mission_start);
		fsm_generate_event(dispatcher1.events_q_Handle,FSM_EVNT_SURFACED);
		fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_S1);
	}
	else if(glider1.dcs_state==GLIDER_STATE_DIVING){
		S3_start_pre_conf();
	}
	else if(glider1.dcs_state==GLIDER_STATE_CLIMBING){
		S3_start_pre_conf();
	}

}

//----------------- END  ACTION FUNCTIONS------------------------------




void fsm_init()
{
	disp_proc_init_func(&dispatcher1, UVP6_PROC_POWER_ON,&uvp6_power_on_proc,proc_inQ1Handle,PROC_LINK_Q,&uvp6_sensor1,uvp6_power_on_events_num,uvp6_power_on_events);
	disp_proc_init_func(&dispatcher1, UVP6_PROC_POWER_OFF,&uvp6_power_off_proc,proc_inQ2Handle,PROC_LINK_Q,&uvp6_sensor1,uvp6_power_off_events_num,uvp6_power_off_events);
	disp_proc_init_func(&dispatcher1, UVP6_PROC_STOP,&uvp6_stop_proc,proc_inQ3Handle,PROC_LINK_Q,&uvp6_sensor1,uvp6_stop_events_num,uvp6_stop_events);
	disp_proc_init_func(&dispatcher1, UVP6_PROC_START_LOW_PROFILE,&uvp6_startL_proc,proc_inQ4Handle,PROC_LINK_Q,&uvp6_sensor1,uvp6_startL_events_num,uvp6_startL_events);
	disp_proc_init_func(&dispatcher1, UVP6_PROC_START_HIGH_PROFILE,&uvp6_startH_proc,proc_inQ5Handle,PROC_LINK_Q,&uvp6_sensor1,uvp6_startH_events_num,uvp6_startH_events);
	disp_proc_init_func(&dispatcher1, UVP6_PROC_LPM_DATA_AGG,&uvp6_lpm_data_agg_proc,proc_inQ6Handle,PROC_LINK_Q,&uvp6_sensor1,uvp6_lpm_data_agg_events_num,uvp6_lpm_data_agg_events);
	disp_proc_init_func(&dispatcher1, SLOCUM_PROC_FILE_OPEN,&slocum_open_file_proc,proc_inQ7Handle,PROC_LINK_Q,&glider1,slocum_open_file_events_num,slocum_open_file_events);
	disp_proc_init_func(&dispatcher1, SLOCUM_PROC_FILE_CLOSE,&slocum_close_file_proc,proc_inQ8Handle,PROC_LINK_Q,&glider1,slocum_close_file_events_num,slocum_close_file_events);
	disp_proc_init_func(&dispatcher1, SLOCUM_PROC_FILE_WRITE,&slocum_write_to_file_proc,proc_inQ9Handle,PROC_LINK_Q,&glider1,slocum_write_to_file_events_num,slocum_write_to_file_events);
	disp_proc_init_func(&dispatcher1, SLOCUM_PROC_DEPTH_ANALYZER,&slocum_depth_analyzer_proc,proc_inQ10Handle,PROC_LINK_Q,&glider1,slocum_depth_analyzer_events_num,slocum_depth_analyzer_events);
	disp_proc_init_func(&dispatcher1, SLOCUM_PROC_EVENTS,&slocum_send_evnt_proc,proc_inQ11Handle,PROC_LINK_Q,&glider1,slocum_send_evnt_events_num,slocum_send_evnt_events);
	disp_proc_init_func(&dispatcher1, SLOCUM_PROC_FINAL_DEPTH_ANALYZER,&slocum_final_depth_state_analyzer_proc,NULL,PROC_CREATE_NEW_Q,&glider1,slocum_final_depth_state_analyzer_events_num,slocum_final_depth_state_analyzer_events);




	//start action
	init_act();
}





void fsm_generate_event(osMessageQId out_Q,uint32_t event)
{
	osMessagePut(out_Q,event,osWaitForever);
}

F_RES fsm_take_event(osMessageQId Q_handle,uint32_t* event,uint32_t wait_time)
{
	osEvent res;
	res=osMessageGet(Q_handle,wait_time);
	if(res.status==osEventMessage){
		*event=res.value.v;
		return F_OK;
	}
	else  if(res.status==osEventTimeout)return F_TIMEOUT;
	return F_ERR;
}

