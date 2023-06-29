/*
 * base64.h
 *
 *  Created on: 26 янв. 2023 г.
 *      Author: admin
 */

#ifndef INC_BASE64_H_
#define INC_BASE64_H_

#include "main.h"

int base64encode( void* data_buf, uint32_t dataLength, char* result, uint32_t resultSize);
uint8_t calc_XOR(char* buffer,uint32_t size);
#endif /* INC_BASE64_H_ */
