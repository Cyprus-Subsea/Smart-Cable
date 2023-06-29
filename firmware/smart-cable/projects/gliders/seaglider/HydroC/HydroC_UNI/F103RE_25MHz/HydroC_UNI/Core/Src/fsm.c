/*
 * fsm.c
 *
 *  Created on: Mar 26, 2023
 *      Author: admin
 */

#include "fsm.h"


extern seaglider glider1;
extern hydroc hydroc_sensor1;
extern sd_storage_t microsd_storage;
extern settings_str run_cfg;
extern proc_dispatcher dispatcher1;
extern osSemaphoreId microSD_semHandle;





//----------------  VAR definition BEGIN  ----------------------

uint8_t log_ds4_msg[512];
uint8_t log_ts1_msg[512];
FIL  data_file;
FIL  err_file;

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

uint32_t  set_clock_events[]={HYDROC_EVNT_CFG_ENTERED,
		                      HYDROC_EVNT_CFG_EXITED};
#define   set_clock_events_num                                             2

void set_clock_proc(proc_arg_t* proc_arg)
{
  char tmp_str[30];
  uint32_t in_event;

  memcpy(tmp_str,glider1.date,8);
  memcpy(tmp_str+8,",0,",3);
  memcpy(tmp_str+11,glider1.time,8);
  memcpy(tmp_str+19,",0",2);
  tmp_str[21]=0x00;

  hydroc_send_cmd(&hydroc_sensor1,HYDROC_CMD_ENTER_CFG,tmp_str);
  F_RES res=fsm_take_event(proc_arg->inQ_handle,&in_event, HYDROC_CMD_TIMEOUT);
  if(res==F_OK){
	if(in_event==HYDROC_EVNT_CFG_ENTERED){
		hydroc_send_cmd(&hydroc_sensor1,HYDROC_CMD_SET_REAL_TIME,tmp_str);
		osDelay(200);
  	    hydroc_send_cmd(&hydroc_sensor1, HYDROC_CMD_EXIT_CFG, NULL);
		res=fsm_take_event(proc_arg->inQ_handle,&in_event, HYDROC_CMD_TIMEOUT);
		if(res==F_OK){
		  if(in_event==HYDROC_EVNT_CFG_EXITED){
			fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_CLOCK_UPDATED);
		  }
	    }
	}
  }
}


uint32_t  send_errors_events[]={
                               };
#define   send_errors_events_num                                          0

void send_errors_proc(proc_arg_t* proc_arg)
{
	char err_msg[50];
	memory_region_pointer ptr1;
	char tmp_filename[50];

	uint32_t bytesreaded;

	sprintf(tmp_filename,"%u:%s_%s_%u%s",microsd_storage.active_disk_indx,
			ERR_FILE_PREFIX,hydroc_sensor1.model_specific->type_name,(unsigned int)run_cfg.last_file_index,ERR_FILE_EXTENSION);
	osSemaphoreWait(microsd_storage.microsd_media_sem, osWaitForever);
	if(f_open(&err_file,tmp_filename,FA_READ)==FR_OK){
     if(f_read(&err_file,(uint8_t*)&hydroc_sensor1.errors,sizeof(hydroc_sensor1.errors),(UINT*)&bytesreaded)==FR_OK){
      if(sizeof(hydroc_sensor1.errors)==bytesreaded){
    		sprintf(err_msg,"P_in:%u,rH_gas:%u,T_control:%u,P_pump:%u\r",
    				   (unsigned int)hydroc_sensor1.errors.P_in,
					   (unsigned int)hydroc_sensor1.errors.rH_gas,
					   (unsigned int)hydroc_sensor1.errors.T_control,
					   (unsigned int)hydroc_sensor1.errors.P_pump
    				   );
      }
      else sprintf(err_msg,"Errors file corrupted\r");
     }
     else sprintf(err_msg,"Errors file can't be read\r");
	}
	else sprintf(err_msg,"Errors file not found\r");
	f_close(&err_file);
	osSemaphoreRelease(microsd_storage.microsd_media_sem);
	ptr1.start_addr=err_msg;
	ptr1.size=strlen(err_msg);
	seaglider_send_cmd(&glider1,SEAGLIDER_CMD_SEND_DATA,&ptr1);
	fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_ERRORS_SENT);

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

	sprintf(tmp_filename,"Data:%s %s %u\r\n",
				DATA_FILE_PREFIX,hydroc_sensor1.model_specific->type_name,(unsigned int)run_cfg.last_file_index);
	ptr1.start_addr=(uint8_t*)tmp_filename;
	ptr1.size=strlen(tmp_filename);
	seaglider_send_cmd(&glider1,SEAGLIDER_CMD_SEND_DATA,&ptr1);

	sprintf(tmp_filename,"%u:%s_%s_%u%s",microsd_storage.active_disk_indx,
			DATA_FILE_PREFIX,hydroc_sensor1.model_specific->type_name,(unsigned int)run_cfg.last_file_index,DATA_FILE_EXTENSION);

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


uint32_t  hydroc_data_saver_events[]={    FSM_EVNT_DS4_PROCESSED ,
		                                  FSM_EVNT_TS1_PROCESSED
		                             };
#define   hydroc_data_saver_events_num                                             2

void hydroc_data_saver_proc(proc_arg_t* proc_arg)
{
 uint32_t in_event;
 uint32_t byteswritten;

 for(;;){
   if(fsm_take_event(proc_arg->inQ_handle,&in_event, osWaitForever)==F_OK){
	   switch(in_event){
	     case FSM_EVNT_DS4_PROCESSED:
	    	 osSemaphoreWait(microsd_storage.microsd_media_sem, osWaitForever);
	    	 f_write(&data_file,log_ds4_msg,strlen(log_ds4_msg),(UINT*)&byteswritten);
	    	 osSemaphoreRelease(microsd_storage.microsd_media_sem);
		 break;
	     case FSM_EVNT_TS1_PROCESSED:
	    	 osSemaphoreWait(microsd_storage.microsd_media_sem, osWaitForever);
	    	 f_write(&data_file,log_ts1_msg,strlen(log_ts1_msg),(UINT*)&byteswritten);
	    	 osSemaphoreRelease(microsd_storage.microsd_media_sem);
		 break;

	   };
   }
 }
}


uint32_t  open_files_events[]={
		                      };
#define   open_files_events_num                                             0

void open_files_proc(proc_arg_t* proc_arg)
{
 char filename[30];
 uint32_t bytesreaded;

 hydroc_sensor1.data_profile_id=glider1.param_z;

 osSemaphoreWait(microsd_storage.microsd_media_sem, osWaitForever);
 if(glider1.dive_status==SEAGLIDER_STATUS_DIVE){

  run_cfg.last_file_index++;
  sprintf(filename,"%u:%s_%s_%u%s",microsd_storage.active_disk_indx,
		  DATA_FILE_PREFIX,hydroc_sensor1.model_specific->type_name,(unsigned int)run_cfg.last_file_index,DATA_FILE_EXTENSION);
  if(f_open(&data_file,filename,FA_CREATE_ALWAYS|FA_WRITE)!=FR_OK){
	  fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_FILE_OPEN_ERR);
	  osSemaphoreRelease(microsd_storage.microsd_media_sem);
	  return;
  }

  sprintf(filename,"%u:%s_%s_%u%s",microsd_storage.active_disk_indx,
		  ERR_FILE_PREFIX,hydroc_sensor1.model_specific->type_name,(unsigned int)run_cfg.last_file_index,ERR_FILE_EXTENSION);
  if(f_open(&err_file,filename,FA_CREATE_ALWAYS|FA_WRITE)!=FR_OK){
	  fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_FILE_OPEN_ERR);
	  osSemaphoreRelease(microsd_storage.microsd_media_sem);
	  return;
  }

  hydroc_sensor1.errors.P_in=0;
  hydroc_sensor1.errors.rH_gas=0;
  hydroc_sensor1.errors.T_control=0;
  hydroc_sensor1.errors.P_pump=0;

  fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_FILES_CREATED);
 }
 else if(glider1.dive_status==SEAGLIDER_STATUS_CLIMB){

  sprintf(filename,"%u:%s_%s_%u%s",microsd_storage.active_disk_indx,
		  DATA_FILE_PREFIX,hydroc_sensor1.model_specific->type_name,(unsigned int)run_cfg.last_file_index,DATA_FILE_EXTENSION);
  if(f_open(&data_file,filename,FA_OPEN_APPEND|FA_WRITE)!=FR_OK) {
	  fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_FILE_OPEN_ERR);
	  osSemaphoreRelease(microsd_storage.microsd_media_sem);
	  return;
  }
  sprintf(filename,"%u:%s_%s_%u%s",microsd_storage.active_disk_indx,
		  ERR_FILE_PREFIX,hydroc_sensor1.model_specific->type_name,(unsigned int)run_cfg.last_file_index,ERR_FILE_EXTENSION);
  if(f_open(&err_file,filename,FA_OPEN_APPEND|FA_WRITE)!=FR_OK){
	  fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_FILE_OPEN_ERR);
	  osSemaphoreRelease(microsd_storage.microsd_media_sem);
	  return;
  }
  if(f_read(&err_file,(uint8_t*)&hydroc_sensor1.errors,sizeof(hydroc_sensor1.errors),(UINT*)&bytesreaded)==FR_OK){
  f_rewind(&err_file);}

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
  if(f_write(&err_file,(uint8_t*)&hydroc_sensor1.errors,sizeof(hydroc_sensor1.errors),(UINT*)&byteswritten)==FR_OK){
   f_close(&err_file);
  }
  osSemaphoreRelease(microsd_storage.microsd_media_sem);
  save_settings();
  fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_FILES_CLOSED);

}


uint32_t  sensor_post_config_events[]={HYDROC_EVNT_CFG_ENTERED,
		                               HYDROC_EVNT_CFG_EXITED
		                      };
#define   sensor_post_config_events_num                                             2

void sensor_post_config_proc(proc_arg_t* proc_arg)
{
 uint32_t in_event;

 if(hydroc_sensor1.model_specific->type_id==HYDROC_TYPE_CO2){
  if(glider1.dive_status==glider1.param_y){
	  hydroc_send_cmd(&hydroc_sensor1, HYDROC_CMD_ENTER_CFG, NULL);
	  F_RES res=fsm_take_event(proc_arg->inQ_handle,&in_event, HYDROC_CMD_TIMEOUT);
	  if(res==F_OK){
		if(in_event==HYDROC_EVNT_CFG_ENTERED){
		  hydroc_send_cmd(&hydroc_sensor1,HYDROC_CMD_SET_ZERO_MODE,NULL);
		  osDelay(100);
		  if(glider1.stop_pump_flag==SEAGLIDER_STOP_PUMP_FLAG_ACTIVATED){
			  hydroc_send_cmd(&hydroc_sensor1,HYDROC_CMD_DISABLE_PUMP,NULL);
		  }
		  osDelay(100);
		  hydroc_send_cmd(&hydroc_sensor1, HYDROC_CMD_EXIT_CFG, NULL);
		  res=fsm_take_event(proc_arg->inQ_handle,&in_event, HYDROC_CMD_TIMEOUT);
		  if(res==F_OK){
			 if(in_event==HYDROC_EVNT_CFG_EXITED){

			 }
		  }
		}
	  }
  }
 }
 else if(hydroc_sensor1.model_specific->type_id==HYDROC_TYPE_CH4){
  if(glider1.dive_status==SEAGLIDER_STATUS_CLIMB&&glider1.stop_pump_flag==SEAGLIDER_STOP_PUMP_FLAG_ACTIVATED){
	  hydroc_send_cmd(&hydroc_sensor1, HYDROC_CMD_ENTER_CFG, NULL);
	  F_RES res=fsm_take_event(proc_arg->inQ_handle,&in_event, HYDROC_CMD_TIMEOUT);
	  if(res==F_OK){
		if(in_event==HYDROC_EVNT_CFG_ENTERED){
		  hydroc_send_cmd(&hydroc_sensor1,HYDROC_CMD_DISABLE_PUMP,NULL);
		  osDelay(100);
		  hydroc_send_cmd(&hydroc_sensor1, HYDROC_CMD_EXIT_CFG, NULL);
		  res=fsm_take_event(proc_arg->inQ_handle,&in_event, HYDROC_CMD_TIMEOUT);
		  if(res==F_OK){
			 if(in_event==HYDROC_EVNT_CFG_EXITED){

			 }
		  }
		}
	  }
  }

 }

 fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_POST_CONF_FINISHED);
}


uint32_t  sensor_pre_config_events[]={HYDROC_EVNT_CFG_ENTERED,
		                              HYDROC_EVNT_CFG_EXITED
		                      };
#define   sensor_pre_config_events_num                                             2

void sensor_pre_config_proc(proc_arg_t* proc_arg)
{
  uint32_t in_event;

  hydroc_send_cmd(&hydroc_sensor1, HYDROC_CMD_ENTER_CFG, NULL);
  F_RES res=fsm_take_event(proc_arg->inQ_handle,&in_event, HYDROC_CMD_TIMEOUT);
  if(res==F_OK){
	if(in_event==HYDROC_EVNT_CFG_ENTERED){
	  hydroc_send_cmd(&hydroc_sensor1,HYDROC_CMD_ENABLE_PUMP,NULL);
	  osDelay(100);
	  hydroc_send_cmd(&hydroc_sensor1, HYDROC_CMD_EXIT_CFG, NULL);
	  res=fsm_take_event(proc_arg->inQ_handle,&in_event, HYDROC_CMD_TIMEOUT);
	  if(res==F_OK){
		 if(in_event==HYDROC_EVNT_CFG_EXITED){
			fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_PRE_CONF_FINISHED);
		 }
	  }
	}
  }
}

uint32_t  hydroc_data_processor_events[]={FSM_EVNT_DS4_DATA_READY ,
		                                  FSM_EVNT_TS1_DATA_READY
		                                 };
#define   hydroc_data_processor_events_num                                             2

void hydroc_data_processor_proc(proc_arg_t* proc_arg)
{
 uint32_t in_event;

 for(;;){
   if(fsm_take_event(proc_arg->inQ_handle,&in_event, osWaitForever)==F_OK){
    		switch(in_event){
    		 case FSM_EVNT_DS4_DATA_READY:
    			log_ds4_msg[0]=0x00;
			    switch(hydroc_sensor1.data_profile_id){
				  case HYDROC_PROFILE_0:

				  break;
				  case HYDROC_PROFILE_1:
				   sprintf(log_ds4_msg,"DS4:%s,%s,%s\r"
						 ,hydroc_sensor1.ds4.pGas_corr
						 ,hydroc_sensor1.ds4.P_IN
						 ,hydroc_sensor1.ds4.runtime);
				  break;
				  case HYDROC_PROFILE_2:
					  if(hydroc_sensor1.model_specific->type_id==HYDROC_TYPE_CO2){
						   sprintf(log_ds4_msg,"DS4:%s,%s,%s,%s,%s\r"
								 ,hydroc_sensor1.ds4.pGas_corr
								 ,hydroc_sensor1.ds4.P_IN
								 ,hydroc_sensor1.ds4.runtime
								 ,hydroc_sensor1.ds4.signal_raw
								 ,hydroc_sensor1.ds4.signal_ref);
					  }
					  else if(hydroc_sensor1.model_specific->type_id==HYDROC_TYPE_CH4){
						   sprintf(log_ds4_msg,"DS4:%s,%s,%s\r"
								 ,hydroc_sensor1.ds4.pGas_corr
								 ,hydroc_sensor1.ds4.P_IN
								 ,hydroc_sensor1.ds4.runtime);
					  }
				  break;
				  case HYDROC_PROFILE_3:
					  if(hydroc_sensor1.model_specific->type_id==HYDROC_TYPE_CO2){
						   sprintf(log_ds4_msg,"DS4:%s,%s,%s,%s,%s,%s,%s,%s,%s\r"
								 ,hydroc_sensor1.ds4.P_IN
								 ,hydroc_sensor1.ds4.pGas_corr
								 ,hydroc_sensor1.ds4.T_sensor
								 ,hydroc_sensor1.ds4.xGas_corr
								 ,hydroc_sensor1.ds4.runtime
								 ,hydroc_sensor1.ds4.date
								 ,hydroc_sensor1.ds4.time
								 ,hydroc_sensor1.ds4.signal_raw
								 ,hydroc_sensor1.ds4.signal_ref);
					  }
					  else if(hydroc_sensor1.model_specific->type_id==HYDROC_TYPE_CH4){
						   sprintf(log_ds4_msg,"DS4:%s,%s,%s,%s,%s,%s,%s\r"
								 ,hydroc_sensor1.ds4.P_SENSOR
								 ,hydroc_sensor1.ds4.P_IN
								 ,hydroc_sensor1.ds4.pGas_corr
								 //,hydroc_sensor1.ds4.T_sensor
								 ,hydroc_sensor1.ds4.xGas_corr
								 ,hydroc_sensor1.ds4.runtime
								 ,hydroc_sensor1.ds4.date
								 ,hydroc_sensor1.ds4.time);
					  }
				  break;
			    };
	        	uint32_t P_IN=strtol(hydroc_sensor1.ds4.P_IN,NULL,10);
	        	uint32_t pump_pwr=strtol(hydroc_sensor1.ds4.pump_pwr,NULL,10);

	        	if(P_IN>110000) hydroc_sensor1.errors.P_in++;
	       		if(pump_pwr>1600 || pump_pwr<300)hydroc_sensor1.errors.P_pump++;
	       		fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_DS4_PROCESSED);
    		 break;
    		 case FSM_EVNT_TS1_DATA_READY:
    			 log_ts1_msg[0]=0x00;
	             switch(hydroc_sensor1.data_profile_id){
	              case HYDROC_PROFILE_0:

                 break;
	              case HYDROC_PROFILE_1:
                  sprintf(log_ts1_msg,"TS1:%s,%s\r"
					 ,hydroc_sensor1.ts1.T_gas
					 ,hydroc_sensor1.ts1.rH_gas
                      );
                 break;
	              case HYDROC_PROFILE_2:
                  sprintf(log_ts1_msg,"TS1:%s,%s\r"
					 ,hydroc_sensor1.ts1.T_gas
					 ,hydroc_sensor1.ts1.rH_gas
                      );
                 break;
	              case HYDROC_PROFILE_3:
                  sprintf(log_ts1_msg,"TS1:%s,%s,%s\r"
					 ,hydroc_sensor1.ts1.rH_gas
					 ,hydroc_sensor1.ts1.T_gas
					 ,hydroc_sensor1.ts1.T_control
                     );
                 break;
	             };
	        	 uint32_t rH_gas=strtol(hydroc_sensor1.ts1.rH_gas,NULL,10);
	        	 uint32_t T_control=strtol(hydroc_sensor1.ts1.T_control,NULL,10);

	        	 if(rH_gas>85000) hydroc_sensor1.errors.rH_gas++;
	        	 if(hydroc_sensor1.model_specific->type_id==HYDROC_TYPE_CH4){
		       		 if(T_control>21500 || T_control<20500)hydroc_sensor1.errors.T_control++;
		       		 fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_TS1_PROCESSED);
	        	 }
	        	 else if(hydroc_sensor1.model_specific->type_id==HYDROC_TYPE_CO2){
		       		 if(T_control>28500 || T_control<27500)hydroc_sensor1.errors.T_control++;
		       		 fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_TS1_PROCESSED);
	        	 }

    		 break;
    		};

   }
 }
}


uint32_t  read_sensor_type_events[]={HYDROC_EVNT_CFG_ENTERED,
		                             HYDROC_EVNT_INVENTORY_UPDATED,
									 HYDROC_EVNT_CFG_EXITED
                                    };
#define   read_sensor_type_events_num                     3

void read_sensor_type_proc(proc_arg_t* proc_arg)
{
  hydroc* hydroc_obj=proc_arg->func_self_object;
  uint32_t in_event;
  hydroc_send_cmd(hydroc_obj, HYDROC_CMD_ENTER_CFG, NULL);
  F_RES res=fsm_take_event(proc_arg->inQ_handle,&in_event, HYDROC_CMD_TIMEOUT);
  if(res==F_OK){
	if(in_event==HYDROC_EVNT_CFG_ENTERED){
	  hydroc_send_cmd(hydroc_obj, HYDROC_CMD_READ_INVENTORY, NULL);
	  res=fsm_take_event(proc_arg->inQ_handle,&in_event, HYDROC_CMD_TIMEOUT);
	  if(res==F_OK){
		if(in_event==HYDROC_EVNT_INVENTORY_UPDATED){
		   hydroc_send_cmd(hydroc_obj, HYDROC_CMD_EXIT_CFG, NULL);
		   res=fsm_take_event(proc_arg->inQ_handle,&in_event, HYDROC_CMD_TIMEOUT);
		   if(res==F_OK){
			 if(in_event==HYDROC_EVNT_CFG_EXITED){
					fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_SENSOR_INITIALIZED);
			 }
		   }
		   fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_SENSOR_INIT_ERROR);
		}
		fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_SENSOR_INIT_ERROR);
	  }
	  fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_SENSOR_INIT_ERROR);
    }
	fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_SENSOR_INIT_ERROR);
  }
  fsm_generate_event(proc_arg->outQ_handle,FSM_EVNT_SENSOR_INIT_ERROR);
}



uint32_t  seaglider_send_evnt_events[]={ FSM_CHANGE_STATE_TO_S0,
		                                 FSM_CHANGE_STATE_TO_S1,
		                                 FSM_CHANGE_STATE_TO_S2,
		                                 FSM_CHANGE_STATE_TO_S3,
		                                 FSM_CHANGE_STATE_TO_S4,
		                                 FSM_CHANGE_STATE_TO_S5,
		                                 FSM_CHANGE_STATE_TO_S6,
		                                 FSM_CHANGE_STATE_TO_S7,
										 FSM_EVNT_DEFAULT_SETTINGS_LOADED,
										 FSM_EVNT_SETTINGS_LOADED,
										 FSM_EVNT_FILE_OPEN_ERR,
										 FSM_EVNT_FILES_OPENED,
										 FSM_EVNT_FILES_CREATED,
										 FSM_EVNT_FILES_CLOSED,
                                       };
#define   seaglider_send_evnt_events_num                                 14
void seaglider_send_evnt_proc(proc_arg_t* proc_arg)
{
	seaglider* glider_obj=proc_arg->func_self_object;
    uint32_t in_event;
    F_RES res;
    osDelay(5000);
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
EA_record_t EAT_S0[]={{MICROSD_EVNT_STORAGE_INITIALIZED,&read_settings_from_microsd},
		              {FSM_EVNT_DEFAULT_SETTINGS_LOADED,&change_fsm_to_S1},
		              {FSM_EVNT_SETTINGS_LOADED,&change_fsm_to_S1}
                     };

EA_record_t EAT_S1[]={{HYDROC_EVNT_CODS4,&read_sensor_type},
		              {FSM_EVNT_SENSOR_INITIALIZED,&change_fsm_from_S1_to_S2}
                     };

EA_record_t EAT_S2[]={{SEAGLIDER_EVNT_START_RCVD,&pre_config},
                      {SEAGLIDER_EVNT_CLOCK_RCVD,&update_sensor_clock},
					  {FSM_EVNT_CLOCK_UPDATED,&send_prompt},
					  {SEAGLIDER_EVNT_SEND_TXT_FILE_RCVD,&upload_last_data},
					  {FSM_EVNT_DATA_SENT,&send_prompt},
					  {FSM_EVNT_DATA_CANT_BE_SENT,&send_prompt},
                      {SEAGLIDER_EVNT_ERRORS_RCVD,&upload_errors},
					  {FSM_EVNT_ERRORS_SENT,&send_prompt}
                     };
EA_record_t EAT_S3[]={{FSM_EVNT_FILES_CREATED,&sensor_pre_config},
                      {FSM_EVNT_FILES_OPENED,&sensor_pre_config},
					  {FSM_EVNT_PRE_CONF_FINISHED,&change_fsm_to_S4}
                     };
EA_record_t EAT_S4[]={{SEAGLIDER_EVNT_STOP_RCVD,&post_config},
		              {HYDROC_EVNT_CODS4,&process_DS4},
					  {HYDROC_EVNT_COTS1,&process_TS1},
                     };
EA_record_t EAT_S5[]={{FSM_EVNT_FILES_CLOSED,&sensor_post_config},
		              {FSM_EVNT_POST_CONF_FINISHED,&change_fsm_from_S5_to_S2}
                     };


EA_table_t fsm_S0_init_microSD={(3),EAT_S0};
EA_table_t fsm_S1_init_sensor={(2),EAT_S1};
EA_table_t fsm_S2_wait_mission={(8),EAT_S2};
EA_table_t fsm_S3_pre_config={(3),EAT_S3};
EA_table_t fsm_S4_data_processing={(3),EAT_S4};
EA_table_t fsm_S5_post_config={(2),EAT_S5};

//---------------- EA tables description END ----------------------


//-----------------BEGIN ACTION FUNCTIONS------------------------------
void  init_act()
{
	change_fsm_to_S0();
	//if(disp_proc_start(&dispatcher1, REPORT_EVENT, NULL)==F_ERR){/*err processing*/}
	if(disp_proc_start(&dispatcher1, MICROSD_INIT, NULL)==F_ERR){/*err processing*/}
	if(disp_proc_start(&dispatcher1, HYDROC_DATA_PROCESSOR, NULL)==F_ERR){/*err processing*/}
	if(disp_proc_start(&dispatcher1, HYDROC_DATA_SAVER, NULL)==F_ERR){/*err processing*/}
}

void process_DS4()
{
  fsm_generate_event(dispatcher1.events_q_Handle,FSM_EVNT_DS4_DATA_READY);
}

void process_TS1()
{
  fsm_generate_event(dispatcher1.events_q_Handle,FSM_EVNT_TS1_DATA_READY);
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

void upload_errors()
{
  if(disp_proc_start(&dispatcher1, SEND_ERRORS, NULL)==F_ERR){/*err processing*/}
}

void update_sensor_clock()
{
  if(disp_proc_start(&dispatcher1, SET_CLOCK, NULL)==F_ERR){/*err processing*/}
}

void pre_config()
{
  change_fsm_to_S3();
  if(disp_proc_start(&dispatcher1, OPEN_FILES, NULL)==F_ERR){/*err processing*/}
}

void sensor_pre_config()
{
  if(disp_proc_start(&dispatcher1, SENSOR_PRE_CONF, NULL)==F_ERR){/*err processing*/}
}

void post_config()
{
  change_fsm_to_S5();
  if(disp_proc_start(&dispatcher1, CLOSE_FILES, NULL)==F_ERR){/*err processing*/}
}

void sensor_post_config()
{
  if(disp_proc_start(&dispatcher1, SENSOR_POST_CONF, NULL)==F_ERR){/*err processing*/}
}

void  change_fsm_to_S0()
{
  disp_proc_set_EA_table(&dispatcher1,&fsm_S0_init_microSD);
  fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_S0);
}

void  change_fsm_to_S1()
{
  disp_proc_set_EA_table(&dispatcher1,&fsm_S1_init_sensor);
  fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_S1);
}

void  change_fsm_from_S1_to_S2()
{
  disp_proc_set_EA_table(&dispatcher1,&fsm_S2_wait_mission);
  fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_S2);
  send_prompt();
}

void  change_fsm_from_S5_to_S2()
{
  disp_proc_set_EA_table(&dispatcher1,&fsm_S2_wait_mission);
  fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_S2);
  send_prompt();
}

void  change_fsm_to_S3()
{
  disp_proc_set_EA_table(&dispatcher1,&fsm_S3_pre_config);
  fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_S3);
}

void  change_fsm_to_S4()
{
  disp_proc_set_EA_table(&dispatcher1,&fsm_S4_data_processing);
  fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_S4);
  send_prompt();
}

void  change_fsm_to_S5()
{
  disp_proc_set_EA_table(&dispatcher1,&fsm_S5_post_config);
  fsm_generate_event(dispatcher1.events_q_Handle,FSM_CHANGE_STATE_TO_S5);
}

void  read_sensor_type()
{
  if(disp_proc_start(&dispatcher1, READ_SENSOR_TYPE, NULL)==F_ERR){/*err processing*/}
}

//----------------- END  ACTION FUNCTIONS------------------------------


void fsm_init()
{
	disp_proc_init_func(&dispatcher1, MICROSD_INIT,&microsd_init_proc,NULL,PROC_CREATE_NEW_Q,NULL,microsd_init_events_num,microsd_init_events);
	disp_proc_init_func(&dispatcher1, REPORT_EVENT,&seaglider_send_evnt_proc,NULL,PROC_CREATE_NEW_Q,&glider1,seaglider_send_evnt_events_num,seaglider_send_evnt_events);
	disp_proc_init_func(&dispatcher1, LOAD_SETTINGS,&load_settings_proc,NULL,PROC_CREATE_NEW_Q,&glider1,load_settings_events_num,load_settings_events);
	disp_proc_init_func(&dispatcher1, READ_SENSOR_TYPE,&read_sensor_type_proc,NULL,PROC_CREATE_NEW_Q,&hydroc_sensor1,read_sensor_type_events_num,read_sensor_type_events);
	disp_proc_init_func(&dispatcher1, SET_CLOCK,&set_clock_proc,NULL,PROC_CREATE_NEW_Q,&hydroc_sensor1,set_clock_events_num,set_clock_events);
	disp_proc_init_func(&dispatcher1, SEND_ERRORS,&send_errors_proc,NULL,PROC_CREATE_NEW_Q,&glider1,send_errors_events_num,send_errors_events);
	disp_proc_init_func(&dispatcher1, SEND_DATA,&send_data_proc,NULL,PROC_CREATE_NEW_Q,NULL,send_data_events_num,send_data_events);
	disp_proc_init_func(&dispatcher1, HYDROC_DATA_PROCESSOR,&hydroc_data_processor_proc,NULL,PROC_CREATE_NEW_Q,NULL,hydroc_data_processor_events_num,hydroc_data_processor_events);
	disp_proc_init_func(&dispatcher1, HYDROC_DATA_SAVER,&hydroc_data_saver_proc,NULL,PROC_CREATE_NEW_Q,NULL,hydroc_data_saver_events_num,hydroc_data_saver_events);
	disp_proc_init_func(&dispatcher1, OPEN_FILES,&open_files_proc,NULL,PROC_CREATE_NEW_Q,NULL,open_files_events_num,open_files_events);
	disp_proc_init_func(&dispatcher1, CLOSE_FILES,&close_files_proc,NULL,PROC_CREATE_NEW_Q,NULL,close_files_events_num,close_files_events);
	disp_proc_init_func(&dispatcher1, SENSOR_POST_CONF,&sensor_post_config_proc,NULL,PROC_CREATE_NEW_Q,NULL,sensor_post_config_events_num,sensor_post_config_events);
	disp_proc_init_func(&dispatcher1, SENSOR_PRE_CONF,&sensor_pre_config_proc,NULL,PROC_CREATE_NEW_Q,NULL,sensor_pre_config_events_num,sensor_pre_config_events);

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

