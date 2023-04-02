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
#include "slocum_g3.h"
#include "disp_proc.h"
#include "extra_calc.h"

#include "system_definitions.h"
#include "events.h"

//max is DISP_NUM_OF_PROCESSES, update DISP_NUM_OF_PROCESSES!!!!
typedef enum{
UVP6_PROC_STOP=0,
UVP6_PROC_START_LOW_PROFILE,
UVP6_PROC_START_HIGH_PROFILE,
UVP6_PROC_POWER_ON,
UVP6_PROC_POWER_OFF,
UVP6_PROC_LPM_DATA_AGG,
SLOCUM_PROC_FILE_OPEN,
SLOCUM_PROC_FILE_WRITE,
SLOCUM_PROC_FILE_CLOSE,
SLOCUM_PROC_DEPTH_ANALYZER,
SLOCUM_PROC_EVENTS,
SLOCUM_PROC_FINAL_DEPTH_ANALYZER
} fsm_proc_id;

void  fsm_init();
void  fsm_generate_event(osMessageQId out_Q,uint32_t event);
F_RES fsm_take_event(osMessageQId Q_handle,uint32_t* event,uint32_t wait_time);

void  init_act();
void start_mission();
void set_power_on();
void set_power_off();
void open_file();
void config_sensor();
void clear_lpm_data();
void write_data();
void uvp6_stop();
void stop_measurements();
void save_data();
void write_data();
void close_file();
void start_measurement();
void stop_and_report();
void go_dos();
void stop_and_go_dos();


void S1_powered_on();
void S1_powered_off();
void S1_wait_mission_id();
void S3_start_pre_conf();
void S6_select_state();


#endif /* INC_FSM_H_ */
