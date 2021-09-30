/*
 * ADC.h
 *
 *  Created on: 15 сент. 2021 г.
 *      Author: admin
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "main.h"
#include "cmsis_os.h"

#define ADC_I2C_ADDR               0x69
#define ADC_UPDATE_INTERVAL        1000     //ms

typedef struct{
  I2C_HandleTypeDef* hi2c;
  uint16_t adc_ch1_value;
  uint16_t adc_ch2_value;

} adc_object;

void adc_init(I2C_HandleTypeDef* hi2c);
void adc_loop();
void adc_read_value(adc_object* self_object);

#endif /* INC_ADC_H_ */
