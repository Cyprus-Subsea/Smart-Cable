/*
 * ADC.c
 *
 *  Created on: 15 сент. 2021 г.
 *      Author: admin
 */

#include "ADC.h"

adc_object adc_main;


void adc_init(I2C_HandleTypeDef* hi2c)
{
	adc_main.hi2c=hi2c;
}


void adc_loop()
{
 while(1)
 {
   adc_read_value(&adc_main);
   osDelay(ADC_UPDATE_INTERVAL);
 }
}


void adc_read_value(adc_object* self_object)
{
	HAL_I2C_Master_Receive(self_object->hi2c,ADC_I2C_ADDR,(uint8_t*)&self_object->adc_ch1_value,sizeof(self_object->adc_ch1_value),1000);
}
