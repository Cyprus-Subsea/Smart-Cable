/*
 * timer.c
 *
 *  Created on: 11 сент. 2021 г.
 *      Author: admin
 */
#include <timed_callbacks.h>


timed_callbacks_list_t  timed_callbacks_list;

void timed_callbacks_list_init(TIM_HandleTypeDef* htim)
{
	timed_callbacks_list.new_callback_index=0;
	timed_callbacks_list.htim=htim;
	HAL_TIM_Base_Start_IT(timed_callbacks_list.htim);
}

timed_callback_t* timed_callback_register_new(void (*callback)(void))
{
	if(timed_callbacks_list.new_callback_index<NUM_OF_TIMERS){
		timed_callback_t* new_chronometer = (timed_callback_t*) malloc(sizeof(timed_callback_t));
		timed_callbacks_list.callbacks[timed_callbacks_list.new_callback_index]=new_chronometer;

		new_chronometer->status=TIMER_TIMED_OUT;
		new_chronometer->value=0;
		new_chronometer->target_time=0;
		if(callback!=0) new_chronometer->callback=callback=callback;
		else new_chronometer->callback=callback=0;

		timed_callbacks_list.new_callback_index++;

		return new_chronometer;
	}
	return 0;
}

void timed_callbacks_update()
{

 for(int i=0;i<timed_callbacks_list.new_callback_index;i++){

  if(timed_callbacks_list.callbacks[i]->status==TIMER_UPDATE_REQUIRED)
  {
	  timed_callbacks_list.callbacks[i]->value=timed_callbacks_list.callbacks[i]->target_time;
	  timed_callbacks_list.callbacks[i]->target_time=0;
	  timed_callbacks_list.callbacks[i]->status=TIMER_ACTIVE;
  }
  else if(timed_callbacks_list.callbacks[i]->status!=TIMER_TIMED_OUT)
  {
	  if(timed_callbacks_list.callbacks[i]->status==TIMER_ACTIVE)
	  {
		  if(timed_callbacks_list.callbacks[i]->value==0)
			  {
			    timed_callbacks_list.callbacks[i]->status=TIMER_TIMED_OUT;
			    timed_callbacks_list.callbacks[i]->callback();
			  }
		  else timed_callbacks_list.callbacks[i]->value--;
	  }
  }
 }
}

void timed_callback_set(timed_callback_t* self_object,uint32_t value)
{
  while(self_object->status==TIMER_UPDATE_REQUIRED){}

  self_object->target_time=value;
  self_object->status=TIMER_UPDATE_REQUIRED;
}

void timed_callback_off(timed_callback_t* self_object)
{
	self_object->value=0;
	self_object->status=TIMER_TIMED_OUT;
}

