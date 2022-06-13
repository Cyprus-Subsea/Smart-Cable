/*
 * icListen.h
 *
 *  Created on: 2 июн. 2022 г.
 *      Author: admin
 */

#ifndef INC_ICLISTEN_H_
#define INC_ICLISTEN_H_




#define MSG_TYPE_SET_TIME              0x41
#define MSG_TYPE_SET_BAUD_RATE         0x42
#define MSG_TYPE_COLLECT_DATA          0x43
#define MSG_TYPE_JOB_SETUP             0x44
#define MSG_TYPE_ENQUIRE_DEVICE        0x45



#pragma pack (push, 1)
typedef struct
{
  uint8_t        sync;
  uint8_t        type;
  uint16_t     length;
} icListen_basic_header;
#pragma pack (pop)

#pragma pack (push, 1)
typedef struct
{
  uint8_t        mask;
} icListen_mask_header;
#pragma pack (pop)


#pragma pack (push, 1)
typedef struct
{
  uint16_t             seq_num;
  uint8_t            bit_depth;
  uint16_t        num_of_bytes;
  uint8_t                 gain;
  uint32_t         sample_rate;
  uint16_t         sensitivity;
  uint16_t       full_scale_mv;
} icListen_waveform_data_header;
#pragma pack (pop)

#pragma pack (push, 1)
typedef struct
{
	icListen_basic_header          basic_hdr;
	icListen_mask_header            mask_hdr;
	icListen_waveform_data_header    wav_hdr;

} icListen_wav_full_header;
#pragma pack (pop)




#endif /* INC_ICLISTEN_H_ */
