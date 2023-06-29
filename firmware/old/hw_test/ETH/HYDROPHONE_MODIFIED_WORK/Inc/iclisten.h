#ifndef __ICLISTEN_H
#define __ICLISTEN_H


#include "stdint.h"


#define ICLISTEN_CMD_TCP_PORT 50000
#define ICLISTEN_EPOCH_TCP_PORT 51680

#define EPOCH_COUNTER_SIZE	5

#pragma pack(push, 1)
struct time_message_type
{
  uint8_t  sync;
  uint8_t  type;
  uint16_t payload_len;
  uint32_t ntime;
  uint16_t crc16;
  
};
#pragma pack(pop)


uint16_t crc_update( char data, uint16_t accum );
uint16_t get_crc16( uint8_t* data,uint16_t length );
void     prepare_timesync_message(struct time_message_type* message,uint32_t unixtime);
uint8_t  check_epoch_message(uint8_t* buffer,uint8_t len);
uint8_t  parse_epoch(uint8_t* buffer,uint8_t len);

#endif /* __ICLISTEN_H */

