/*
 * time.h
 *
 *  Created on: 11 сент. 2021 г.
 *      Author: admin
 */

#ifndef INC_TIMED_CALLBACKS_H_
#define INC_TIMED_CALLBACKS_H_

#include "main.h"
#include  "stdlib.h"
#include  "stdio.h"

#define NUM_OF_TIMERS  5

enum timer_status{
	TIMER_TIMED_OUT,
	TIMER_ACTIVE,
	TIMER_UPDATE_REQUIRED
};


typedef struct {
	uint32_t target_time;
	uint32_t value;
	uint32_t status;
	void (*callback)(void);
} timed_callback_t;


typedef struct {
	timed_callback_t* callbacks[NUM_OF_TIMERS];
	uint8_t new_callback_index;
	TIM_HandleTypeDef* htim;
}timed_callbacks_list_t;

void timed_callbacks_list_init(TIM_HandleTypeDef* htim);


void timed_callbacks_update();
timed_callback_t* timed_callback_register_new(void (*callback)(void));
void timed_callback_set(timed_callback_t* self_object,uint32_t value);
void timed_callback_off(timed_callback_t* self_object);



#endif /* INC_TIMED_CALLBACKS_H_ */
