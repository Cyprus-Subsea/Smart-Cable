/*
 * extra_calc.c
 *
 *  Created on: Mar 29, 2023
 *      Author: admin
 */

#include "extra_calc.h"

extern float bloc_GPS_lat;
extern float bloc_GPS_lon;


void lpm_sum_messages(uvp6* uvp6_obj,lpm_data_str* lpm_messages_buffer)
{
  lpm_messages_buffer->temperature+=uvp6_obj->lpm_data.temperature;
  lpm_messages_buffer->pressure+=uvp6_obj->lpm_data.pressure;
  lpm_messages_buffer->number_of_images+=uvp6_obj->lpm_data.number_of_images;
  for(int y=0;y<UVP6_NUM_OF_CATEGORIES;y++){
	lpm_messages_buffer->data[y]+=uvp6_obj->lpm_data.data[y];
	lpm_messages_buffer->grey_levels[y]+=(uvp6_obj->lpm_data.grey_levels[y]*uvp6_obj->lpm_data.data[y]);
  }
}

F_RES lpm_aggregate_messages(uvp6* uvp6_obj,lpm_data_str* lpm_messages_buffer,uint32_t* lpm_buffer_num_of_msgs)
{
   if(*lpm_buffer_num_of_msgs==0) return F_ERR;
   lpm_messages_buffer->temperature=lpm_messages_buffer->temperature/(float)(*lpm_buffer_num_of_msgs);
   lpm_messages_buffer->pressure=lpm_messages_buffer->pressure/(float)(*lpm_buffer_num_of_msgs);
   for(int y=0;y<UVP6_NUM_OF_CATEGORIES;y++){
	   if(lpm_messages_buffer->data[y]>0){
		   lpm_messages_buffer->grey_levels[y]=(lpm_messages_buffer->grey_levels[y]/lpm_messages_buffer->data[y]);
	   }
   }
   *lpm_buffer_num_of_msgs=0;
   return F_OK;
}

F_RES lpm_aggregate_and_close_bloc(uvp6* uvp6_obj,char* res,lpm_data_str* lpm_messages_buffer,uint32_t* lpm_buffer_num_of_msgs)
{
  	if(lpm_aggregate_messages(uvp6_obj,lpm_messages_buffer,lpm_buffer_num_of_msgs)==F_OK){
	sprintf(res,"LPM_DATA,%s %s,%f,%f,%f,%u,%f,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u;\r\n",
						  lpm_messages_buffer->date,
						  lpm_messages_buffer->time,
						  lpm_messages_buffer->pressure,
						  bloc_GPS_lat,
						  bloc_GPS_lon,
						  lpm_messages_buffer->number_of_images,
						  lpm_messages_buffer->temperature,
						  lpm_messages_buffer->data[0],
						  lpm_messages_buffer->data[1],
						  lpm_messages_buffer->data[2],
						  lpm_messages_buffer->data[3],
						  lpm_messages_buffer->data[4],
						  lpm_messages_buffer->data[5],
						  lpm_messages_buffer->data[6],
						  lpm_messages_buffer->data[7],
						  lpm_messages_buffer->data[8],
						  lpm_messages_buffer->data[9],
						  lpm_messages_buffer->data[10],
						  lpm_messages_buffer->data[11],
						  lpm_messages_buffer->data[12],
						  lpm_messages_buffer->data[13],
						  lpm_messages_buffer->data[14],
						  lpm_messages_buffer->data[15],
						  lpm_messages_buffer->data[16],
						  lpm_messages_buffer->data[17],
						  lpm_messages_buffer->grey_levels[0],
						  lpm_messages_buffer->grey_levels[1],
						  lpm_messages_buffer->grey_levels[2],
						  lpm_messages_buffer->grey_levels[3],
						  lpm_messages_buffer->grey_levels[4],
						  lpm_messages_buffer->grey_levels[5],
						  lpm_messages_buffer->grey_levels[6],
						  lpm_messages_buffer->grey_levels[7],
						  lpm_messages_buffer->grey_levels[8],
						  lpm_messages_buffer->grey_levels[9],
						  lpm_messages_buffer->grey_levels[10],
						  lpm_messages_buffer->grey_levels[11],
						  lpm_messages_buffer->grey_levels[12],
						  lpm_messages_buffer->grey_levels[13],
						  lpm_messages_buffer->grey_levels[14],
						  lpm_messages_buffer->grey_levels[15],
						  lpm_messages_buffer->grey_levels[16],
						  lpm_messages_buffer->grey_levels[17]
    );
	return F_OK;
  }
  return F_ERR;
}
