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

#include "HydroC.h"
#include "seaglider.h"
#include "disp_proc.h"
#include "settings.h"

#include "system_definitions.h"
#include "events.h"

#include "em_sd_storage.h"


//max is DISP_NUM_OF_PROCESSES, update DISP_NUM_OF_PROCESSES!!!!
typedef enum{
MICROSD_INIT=0,
REPORT_EVENT,
LOAD_SETTINGS,
READ_SENSOR_TYPE,
SET_CLOCK,
SEND_ERRORS,
SEND_DATA,
HYDROC_DATA_PROCESSOR,
HYDROC_DATA_SAVER,
OPEN_FILES,
CLOSE_FILES,
SENSOR_POST_CONF,
SENSOR_PRE_CONF
} fsm_proc_id;

void  fsm_init();
void  fsm_generate_event(osMessageQId out_Q,uint32_t event);
F_RES fsm_take_event(osMessageQId Q_handle,uint32_t* event,uint32_t wait_time);


void  sensor_config();
void  process_TS1();
void  process_DS4();
void  send_prompt();
void  upload_last_data();
void  upload_errors();
void  update_sensor_clock();

void  pre_config();
void  sensor_pre_config();
void  post_config();
void  sensor_post_config();
void  sensor_pre_config();

void  change_fsm_to_S0();
void  change_fsm_to_S1();
void  change_fsm_from_S1_to_S2();
void  change_fsm_from_S5_to_S2();
void  change_fsm_to_S3();
void  change_fsm_to_S4();
void  change_fsm_to_S5();
void  init_act();
void  read_settings_from_microsd();
void  read_sensor_type();

#endif /* INC_FSM_H_ */
