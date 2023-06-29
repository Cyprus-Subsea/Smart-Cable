/*
 * fsm.h
 *
 *  Created on: Mar 26, 2023
 *      Author: admin
 */

#ifndef INC_FSM_H_
#define INC_FSM_H_

#include "main.h"
#include "cmsis_os.h"

#include "UVP6.h"
#include "seaglider.h"
#include "disp_proc.h"
#include "settings.h"
#include "extra_calc.h"

#include "system_definitions.h"
#include "events.h"

#include "em_sd_storage.h"


//max is DISP_NUM_OF_PROCESSES, update DISP_NUM_OF_PROCESSES!!!!
typedef enum{
	MICROSD_INIT=0,
	REPORT_EVENT,
	LOAD_SETTINGS,
	SEND_DATA,
	OPEN_FILES,
	CLOSE_FILES,
	UVP6_PROC_STOP,
	UVP6_PROC_START_LOW_PROFILE,
	UVP6_PROC_START_HIGH_PROFILE,
	UVP6_PROC_LPM_DATA_AGG,
	SEAGLIDER_PROC_DEPTH_ANALYZER,
	FILE_WRITE
} fsm_proc_id;

void  fsm_init();
void  fsm_generate_event(osMessageQId out_Q,uint32_t event);
F_RES fsm_take_event(osMessageQId Q_handle,uint32_t* event,uint32_t wait_time);

void microsd_init();
void  send_prompt();
void  upload_last_data();

void  clear_lpm_data();
void  save_data();
void  stop_and_report();
void  config_sensor();
void  start_measurement();
void  write_data();
void  uvp6_stop();
void  close_file();



void  change_fsm_to_init();
void  change_fsm_to_idle();
void  change_fsm_from_init_to_idle();
void  change_fsm_from_idle_to_starting();
void  change_fsm_from_starting_to_dive();
void  change_fsm_from_dive_to_stopping();
void  change_fsm_from_stopping_to_idle();


void  init_act();
void  read_settings_from_microsd();


#endif /* INC_FSM_H_ */
