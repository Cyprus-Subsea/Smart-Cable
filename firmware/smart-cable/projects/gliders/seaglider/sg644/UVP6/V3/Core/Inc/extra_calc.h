/*
 * extra_cacl.h
 *
 *  Created on: Mar 29, 2023
 *      Author: admin
 */

#ifndef INC_EXTRA_CALC_H_
#define INC_EXTRA_CALC_H_

#include "main.h"
#include "UVP6.h"
#include "string.h"
#include "stdio.h"
#include "system_definitions.h"

void lpm_sum_messages(uvp6* uvp6_obj,lpm_data_str* lpm_messages_buffer);
F_RES lpm_aggregate_messages(uvp6* uvp6_obj,lpm_data_str* lpm_messages_buffer,uint32_t* lpm_buffer_num_of_msgs);
F_RES lpm_aggregate_and_close_bloc(uvp6* uvp6_obj,char* res,lpm_data_str* lpm_messages_buffer,uint32_t* lpm_buffer_num_of_msgs);


#endif /* INC_EXTRA_CALC_H_ */
