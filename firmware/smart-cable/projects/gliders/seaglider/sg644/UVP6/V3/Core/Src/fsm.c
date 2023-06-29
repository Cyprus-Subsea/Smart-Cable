/*
 * fsm.c
 *
 *  Created on: Mar 26, 2023
 *      Author: admin
 */

#include "fsm.h"


extern seaglider glider1;
extern uvp6 uvp6_sensor1;
extern sd_storage_t microsd_storage;
extern settings_str run_cfg;
extern proc_dispatcher dispatcher1;
extern osSemaphoreId microSD_semHandle;

//----------------  VAR definition BEGIN  ----------------------

FIL  data_file;
FIL  err_file;

uint32_t     lpm_buffer_num_of_msgs=0;
lpm_data_str lpm_messages_buffer;
char         lmp_data_bloc_msg[500];

float lpm_bloc_depth_size=0.0;
float lpm_bloc_depth_start=0.0;
float bloc_GPS_lat=0.0;
float bloc_GPS_lon=0.0;

//----------------  VAR definition END -------------------------

//-------------------BEGIN PROC FUNCTIONS--------------------------------

uint32_t  microsd_init_events[]={0};
#define   microsd_init_events_num                                             0

void microsd_init_proc(proc_arg_t* proc_arg)
{
 MX_FATFS_Init();
 sd_storage_link_ss(&microsd_storage,0,SS_SD1_Pin,GPIOA);

 while(sd_storage_init(&microsd_storage,microSD_semHandle)!=F_OK){
  osDelay(500);
  fsm_generate_event(proc_arg->outQ_handle,MICROSD_EVNT_STORAGE_NOT_INITIALIZED);
 }
 fsm_generate_event(proc_arg->outQ_handle,MICROSD_EVNT_STORAGE_INITIALIZED);
}


uint32_t  load_settings_events[]={0};
#define   load_settings_events_num                                             0

void load_settings_proc(proc_arg_t* proc_arg)
{
  if(read_settings()==F_ERR){
 	  set_default_settings();
 	  save_settings();
 	  fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_DEFAULT_SETTINGS_LOADED);
  }
  else{
    fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_SETTINGS_LOADED);

  }
}


uint32_t  send_data_events[]={
		                     };
#define   send_data_events_num                                             0

void send_data_proc(proc_arg_t* proc_arg)
{
	memory_region_pointer ptr1;
	char tmp_filename[50];
	uint8_t tx_buff[100];
	uint32_t bytesreaded;

	sprintf(tmp_filename,"Data:%s %u\r\n",
				DATA_FILE_PREFIX,(unsigned int)run_cfg.last_file_index);
	ptr1.start_addr=(uint8_t*)tmp_filename;
	ptr1.size=strlen(tmp_filename);
	seaglider_send_cmd(&glider1,SEAGLIDER_CMD_SEND_DATA,&ptr1);

	sprintf(tmp_filename,"%u:%s_%u%s",microsd_storage.active_disk_indx,
			DATA_FILE_PREFIX,(unsigned int)run_cfg.last_file_index,DATA_FILE_EXTENSION);

	osSemaphoreWait(microsd_storage.microsd_media_sem, osWaitForever);
	if(f_open(&data_file,tmp_filename,FA_READ)==FR_OK){

		do{
		   if(f_read(&data_file,tx_buff,50,(UINT*)&bytesreaded)==FR_OK){
		    ptr1.start_addr=tx_buff;
		    ptr1.size=bytesreaded;
		    seaglider_send_cmd(&glider1,SEAGLIDER_CMD_SEND_DATA,&ptr1);
		   }
		}while(bytesreaded==50);
		fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_DATA_SENT);
	}
	else fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_DATA_CANT_BE_SENT);
	f_close(&data_file);
	osSemaphoreRelease(microsd_storage.microsd_media_sem);
}


uint32_t  open_files_events[]={
		                      };
#define   open_files_events_num                                             0

void open_files_proc(proc_arg_t* proc_arg)
{
 char filename[30];
 uint32_t bytesreaded;

 osSemaphoreWait(microsd_storage.microsd_media_sem, osWaitForever);
 if(glider1.dive_status==SEAGLIDER_STATUS_DIVE){

  run_cfg.last_file_index++;
  sprintf(filename,"%u:%s_%u%s",microsd_storage.active_disk_indx,
		  DATA_FILE_PREFIX,(unsigned int)run_cfg.last_file_index,DATA_FILE_EXTENSION);
  if(f_open(&data_file,filename,FA_CREATE_ALWAYS|FA_WRITE)!=FR_OK){
	  fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_FILE_OPEN_ERR);
	  osSemaphoreRelease(microsd_storage.microsd_media_sem);
	  return;
  }

  sprintf(filename,"%u:%s_%u%s",microsd_storage.active_disk_indx,
		  ERR_FILE_PREFIX,(unsigned int)run_cfg.last_file_index,ERR_FILE_EXTENSION);
  if(f_open(&err_file,filename,FA_CREATE_ALWAYS|FA_WRITE)!=FR_OK){
	  fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_FILE_OPEN_ERR);
	  osSemaphoreRelease(microsd_storage.microsd_media_sem);
	  return;
  }

  fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_FILES_CREATED);
 }
 else if(glider1.dive_status==SEAGLIDER_STATUS_CLIMB){

  sprintf(filename,"%u:%s_%u%s",microsd_storage.active_disk_indx,
		  DATA_FILE_PREFIX,(unsigned int)run_cfg.last_file_index,DATA_FILE_EXTENSION);
  if(f_open(&data_file,filename,FA_OPEN_APPEND|FA_WRITE)!=FR_OK) {
	  fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_FILE_OPEN_ERR);
	  osSemaphoreRelease(microsd_storage.microsd_media_sem);
	  return;
  }
  /*
  sprintf(filename,"%u:%s_%u%s",microsd_storage.active_disk_indx,
		  ERR_FILE_PREFIX,(unsigned int)run_cfg.last_file_index,ERR_FILE_EXTENSION);
  if(f_open(&err_file,filename,FA_OPEN_APPEND|FA_WRITE)!=FR_OK){
	  fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_FILE_OPEN_ERR);
	  osSemaphoreRelease(microsd_storage.microsd_media_sem);
	  return;
  }
  if(f_read(&err_file,(uint8_t*)&hydroc_sensor1.errors,sizeof(hydroc_sensor1.errors),(UINT*)&bytesreaded)==FR_OK){
  f_rewind(&err_file);}
  */

  fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_FILES_OPENED);
 }
 osSemaphoreRelease(microsd_storage.microsd_media_sem);
}


uint32_t  close_files_events[]={
		                      };
#define   close_files_events_num                                             0

void close_files_proc(proc_arg_t* proc_arg)
{
 uint32_t byteswritten;

  osSemaphoreWait(microsd_storage.microsd_media_sem, osWaitForever);
  f_close(&data_file);

  /*
  if(f_write(&err_file,(uint8_t*)&hydroc_sensor1.errors,sizeof(hydroc_sensor1.errors),(UINT*)&byteswritten)==FR_OK){
   f_close(&err_file);
  }
  */

  osSemaphoreRelease(microsd_storage.microsd_media_sem);
  save_settings();
  fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_FILES_CLOSED);

}


uint32_t  write_to_file_events[]={};
#define   write_to_file_events_num                                 0
void write_to_file_proc(proc_arg_t* proc_arg)
{
    FRESULT res;
	uint32_t byteswritten=0;

  	res=f_write(&data_file,lmp_data_bloc_msg,strlen(lmp_data_bloc_msg),&byteswritten);


    if(res==FR_OK){
    	fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_FILE_DATA_WRITTEN);
    }
    else{
      	fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_FILE_WRITE_ERR);

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
		   //bloc_GPS_lat=glider1.gps_lat;
		   //bloc_GPS_lon=glider1.gps_lon;
		   memcpy(&lpm_messages_buffer,&uvp6_obj->lpm_data,sizeof(uvp6_obj->lpm_data));
		   for(int y=0;y<UVP6_NUM_OF_CATEGORIES;y++){
			lpm_messages_buffer.grey_levels[y]=(lpm_messages_buffer.grey_levels[y]*lpm_messages_buffer.data[y]);
		   }
		 }
		 else lpm_sum_messages(uvp6_obj,&lpm_messages_buffer);
		 if(lpm_bloc_depth_size==0.0){
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


uint32_t seaglider_depth_analyzer_events[]={SEAGLIDER_EVNT_DEPTH_RCVD,
		                                 UVP6_EVNT_LPM_AGG_DATA_READY,
										 UVP6_EVNT_LPM_BUFFER_CLEARED};
#define seaglider_depth_analyzer_events_num                      3


void seaglider_depth_analyzer_proc(proc_arg_t* proc_arg){

	seaglider* seaglider_obj=proc_arg->func_self_object;
	uint32_t in_event;
	uvp6_sensor1.profile_zone=UVP6_PROFILE_H;

	while(1)
	{
	 if(fsm_take_event(proc_arg->inQ_handle,&in_event,osWaitForever)==F_OK){
	   if(in_event==SEAGLIDER_EVNT_DEPTH_RCVD){
		   if(seaglider_obj->prev_depth>UVP6_DEPTH_LH_PROFILE&&seaglider_obj->last_depth<=UVP6_DEPTH_LH_PROFILE){
			   uvp6_sensor1.profile_zone=UVP6_PROFILE_H;
			   fsm_generate_event(proc_arg->outQ_handle,UVP6_EVNT_DEPTH_LH_REACHED_FROM_BOTTOM);
		   }

		   else if(seaglider_obj->prev_depth<UVP6_DEPTH_LH_PROFILE&&seaglider_obj->last_depth>=UVP6_DEPTH_LH_PROFILE){
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


uint32_t  seaglider_send_evnt_events[]={ FSM_CHANGE_STATE_TO_INIT,
		                                 FSM_CHANGE_STATE_TO_IDLE,
		                                 FSM_CHANGE_STATE_TO_STARTING,
		                                 FSM_CHANGE_STATE_TO_DIVE,
										 FSM_CHANGE_STATE_TO_STOPPING,
										 FSM_EVNT_DEFAULT_SETTINGS_LOADED,
										 FSM_EVNT_SETTINGS_LOADED,
										 FSM_EVNT_FILE_OPEN_ERR,
										 FSM_EVNT_FILES_OPENED,
										 FSM_EVNT_FILES_CREATED,
										 FSM_EVNT_FILES_CLOSED,
										 FSM_EVNT_UVP6_STOPPED,
										 FSM_EVNT_FILE_DATA_WRITTEN,
										 UVP6_EVNT_STOP_ACK_RCVD,
										 UVP6_EVNT_START_ERR_RCVD,
										 UVP6_EVNT_STARTED_LOW_PROFILE,
										 UVP6_EVNT_STARTED_HIGH_PROFILE,
										 UVP6_EVNT_DEPTH_LH_REACHED_FROM_TOP,
										 UVP6_EVNT_DEPTH_LH_REACHED_FROM_BOTTOM,
										 UVP6_EVNT_LPM_AGG_DATA_READY,
										 UVP6_EVNT_LPM_DATA_BLOC_END_REACHED,
										 UVP6_EVNT_LPM_DATA_RCVD,
										 UVP6_EVNT_LPM_AGG_NOTHING_TO_SEND,
										 UVP6_EVNT_LPM_BUFFER_CLEARED,
										 UVP6_EVNT_BOOTED,
										 SEAGLIDER_EVNT_DEPTH_RCVD,
										 SEAGLIDER_EVNT_STOP_RCVD


                                       };
#define   seaglider_send_evnt_events_num                                 27
void seaglider_send_evnt_proc(proc_arg_t* proc_arg)
{
	seaglider* glider_obj=proc_arg->func_self_object;
    uint32_t in_event;
    F_RES res;
    osDelay(100);
    while(1)
    {
  	 res=fsm_take_event(proc_arg->inQ_handle,&in_event,osWaitForever);
     if(res==F_OK){
    	 seaglider_send_evnt(glider_obj,in_event);
     }
    }
}
//-------------------END PROC FUNCTIONS--------------------------------


//---------------- EA tables description BEGIN ---------------------

EA_record_t EAT_INIT[]={{MICROSD_EVNT_STORAGE_INITIALIZED,&read_settings_from_microsd},
		                {FSM_EVNT_DEFAULT_SETTINGS_LOADED,&change_fsm_from_init_to_idle},
		                {FSM_EVNT_SETTINGS_LOADED,&change_fsm_from_init_to_idle},
						{UVP6_EVNT_BOOTED,&microsd_init}
                       };


EA_record_t EAT_IDLE[]={{SEAGLIDER_EVNT_START_RCVD,&change_fsm_from_idle_to_starting},
					    {SEAGLIDER_EVNT_CLOCK_RCVD,&send_prompt},
					    {SEAGLIDER_EVNT_STOP_RCVD,&send_prompt},
					    {SEAGLIDER_EVNT_SEND_TXT_FILE_RCVD,&upload_last_data},
					    {FSM_EVNT_DATA_SENT,&send_prompt},
					    {FSM_EVNT_DATA_CANT_BE_SENT,&send_prompt}
                       };

EA_record_t EAT_STARTING[]={{FSM_EVNT_FILES_CREATED,&clear_lpm_data},
		                   {FSM_EVNT_FILES_OPENED,&clear_lpm_data},
					       {UVP6_EVNT_LPM_BUFFER_CLEARED,&config_sensor},
					       {UVP6_EVNT_STARTED_LOW_PROFILE,&change_fsm_from_starting_to_dive},
					       {UVP6_EVNT_STARTED_HIGH_PROFILE,&change_fsm_from_starting_to_dive},
					       {FSM_EVNT_UVP6_NOT_STARTED,&stop_and_report}
                           };

EA_record_t EAT_DIVE[]={  {SEAGLIDER_EVNT_STOP_RCVD,&change_fsm_from_dive_to_stopping},
		                  {UVP6_EVNT_LPM_AGG_DATA_READY,&write_data},
						  {UVP6_EVNT_DEPTH_LH_REACHED_FROM_TOP,&uvp6_stop},
						  {UVP6_EVNT_DEPTH_LH_REACHED_FROM_BOTTOM,&uvp6_stop},
						  {FSM_EVNT_UVP6_STOPPED,&config_sensor}

                       };

EA_record_t EAT_STOPPING[]={{FSM_EVNT_FILES_CLOSED,&change_fsm_from_stopping_to_idle},
	                       	{FSM_EVNT_UVP6_STOPPED,&save_data},
							{UVP6_EVNT_LPM_AGG_DATA_READY,&write_data},
							{FSM_EVNT_FILE_DATA_WRITTEN,&close_file},
							{FSM_EVNT_FILE_WRITE_ERR,&close_file},
							{FSM_EVNT_UVP6_NOT_STOPPED,&stop_and_report}
                           };


EA_table_t fsm_S0_init={(4),EAT_INIT};
EA_table_t fsm_S1_idle={(6),EAT_IDLE};
EA_table_t fsm_S2_starting={(6),EAT_STARTING};
EA_table_t fsm_S3_dive={(5),EAT_DIVE};
EA_table_t fsm_S4_stopping={(6),EAT_STOPPING};

//---------------- EA tables description END ----------------------


//-----------------BEGIN ACTION FUNCTIONS------------------------------
void  init_act()
{
	change_fsm_to_init();
	if(disp_proc_start(&dispatcher1, REPORT_EVENT, NULL)==F_ERR){/*err processing*/}
	if(disp_proc_start(&dispatcher1, SEAGLIDER_PROC_DEPTH_ANALYZER, NULL)==F_ERR){/*err processing*/}
	if(disp_proc_start(&dispatcher1, UVP6_PROC_LPM_DATA_AGG, NULL)==F_ERR){/*err processing*/}
}

void microsd_init()
{
	if(disp_proc_start(&dispatcher1, MICROSD_INIT, NULL)==F_ERR){/*err processing*/}
}

void  send_prompt()
{
  seaglider_send_cmd(&glider1,SEAGLIDER_CMD_PROMPT,NULL);
}

void  read_settings_from_microsd()
{
  if(disp_proc_start(&dispatcher1, LOAD_SETTINGS, NULL)==F_ERR){/*err processing*/}
}

void upload_last_data()
{
  if(disp_proc_start(&dispatcher1, SEND_DATA, NULL)==F_ERR){/*err processing*/}
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

void start_measurement()
{
	disp_proc_set_EA_table(&dispatcher1,&fsm_S3_dive);
	fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_DIVE);
}

void clear_lpm_data()
{
	fsm_generate_event(dispatcher1.events_q_Handle,FSM_EVNT_READY_FOR_LPM_DATA);
}

void save_data()
{
	fsm_generate_event(dispatcher1.events_q_Handle,UVP6_EVNT_LPM_DATA_BLOC_END_REACHED);
}

void write_data()
{
	if(disp_proc_start(&dispatcher1, FILE_WRITE, NULL)==F_ERR){/*err processing*/}
}
void uvp6_stop()
{
	if(disp_proc_start(&dispatcher1, UVP6_PROC_STOP, NULL)==F_ERR){/*err processing*/}
}

void close_file()
{
	if(disp_proc_start(&dispatcher1, CLOSE_FILES, NULL)==F_ERR){/*err processing*/}
}

void stop_and_report()
{
	fsm_generate_event(dispatcher1.events_q_Handle,FSM_EVNT_UVP6_ERR);
	disp_proc_set_EA_table(&dispatcher1,&fsm_S1_idle);
	fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_IDLE);
	close_file();
}


void  change_fsm_to_init()
{
  disp_proc_set_EA_table(&dispatcher1,&fsm_S0_init);
  fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_INIT);
}

void  change_fsm_from_init_to_idle()
{
  disp_proc_set_EA_table(&dispatcher1,&fsm_S1_idle);
  fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_IDLE);
  send_prompt();
}

void  change_fsm_from_idle_to_starting()
{
  disp_proc_set_EA_table(&dispatcher1,&fsm_S2_starting);
  if(disp_proc_start(&dispatcher1, OPEN_FILES, NULL)==F_ERR){/*err processing*/}
  fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_STARTING);
}

void  change_fsm_from_starting_to_dive()
{
  disp_proc_set_EA_table(&dispatcher1,&fsm_S3_dive);
  fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_DIVE);
  send_prompt();
}

void  change_fsm_from_dive_to_stopping()
{
  disp_proc_set_EA_table(&dispatcher1,&fsm_S4_stopping);
  if(disp_proc_start(&dispatcher1, UVP6_PROC_STOP, NULL)==F_ERR){/*err processing*/}
  fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_STOPPING);
}



void  change_fsm_from_stopping_to_idle()
{
  disp_proc_set_EA_table(&dispatcher1,&fsm_S1_idle);
  fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_IDLE);
  send_prompt();
}

//----------------- END  ACTION FUNCTIONS------------------------------


void fsm_init()
{
	disp_proc_init_func(&dispatcher1, MICROSD_INIT,&microsd_init_proc,NULL,PROC_CREATE_NEW_Q,NULL,microsd_init_events_num,microsd_init_events);
	disp_proc_init_func(&dispatcher1, REPORT_EVENT,&seaglider_send_evnt_proc,NULL,PROC_CREATE_NEW_Q,&glider1,seaglider_send_evnt_events_num,seaglider_send_evnt_events);
	disp_proc_init_func(&dispatcher1, LOAD_SETTINGS,&load_settings_proc,NULL,PROC_CREATE_NEW_Q,&glider1,load_settings_events_num,load_settings_events);
	disp_proc_init_func(&dispatcher1, SEND_DATA,&send_data_proc,NULL,PROC_CREATE_NEW_Q,NULL,send_data_events_num,send_data_events);
	disp_proc_init_func(&dispatcher1, OPEN_FILES,&open_files_proc,NULL,PROC_CREATE_NEW_Q,NULL,open_files_events_num,open_files_events);
	disp_proc_init_func(&dispatcher1, CLOSE_FILES,&close_files_proc,NULL,PROC_CREATE_NEW_Q,NULL,close_files_events_num,close_files_events);
	disp_proc_init_func(&dispatcher1, UVP6_PROC_STOP,&uvp6_stop_proc,NULL,PROC_CREATE_NEW_Q,&uvp6_sensor1,uvp6_stop_events_num,uvp6_stop_events);
	disp_proc_init_func(&dispatcher1, UVP6_PROC_START_LOW_PROFILE,&uvp6_startL_proc,NULL,PROC_CREATE_NEW_Q,&uvp6_sensor1,uvp6_startL_events_num,uvp6_startL_events);
	disp_proc_init_func(&dispatcher1, UVP6_PROC_START_HIGH_PROFILE,&uvp6_startH_proc,NULL,PROC_CREATE_NEW_Q,&uvp6_sensor1,uvp6_startH_events_num,uvp6_startH_events);
	disp_proc_init_func(&dispatcher1, UVP6_PROC_LPM_DATA_AGG,&uvp6_lpm_data_agg_proc,NULL,PROC_CREATE_NEW_Q,&uvp6_sensor1,uvp6_lpm_data_agg_events_num,uvp6_lpm_data_agg_events);
	disp_proc_init_func(&dispatcher1, SEAGLIDER_PROC_DEPTH_ANALYZER,&seaglider_depth_analyzer_proc,NULL,PROC_CREATE_NEW_Q,&glider1,seaglider_depth_analyzer_events_num,seaglider_depth_analyzer_events);
	disp_proc_init_func(&dispatcher1, FILE_WRITE,&write_to_file_proc,NULL,PROC_CREATE_NEW_Q,NULL,write_to_file_events_num,write_to_file_events);


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

