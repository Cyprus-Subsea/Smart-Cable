/*
 * icListen.h
 *
 *  Created on: 2 июн. 2022 г.
 *      Author: admin
 */

#ifndef INC_ICLISTEN_H_
#define INC_ICLISTEN_H_

#include "main.h"
#include "system_definitions.h"
#include "UI.h"


#define ICLISTEN_DEFAULT_WAV_SAMPLE_RATE            32000
#define ICLISTEN_DEFAULT_WAV_SAMPLE_BIT_DEPTH          24
#define ICLISTEN_DEFAULT_FILE_DURATION                120

#define MSG_SYNC                       0x2A


#define MSG_TYPE_SET_TIME              0x41
#define MSG_TYPE_SET_BAUD_RATE         0x42
#define MSG_TYPE_COLLECT_DATA          0x43
#define MSG_TYPE_JOB_SETUP             0x44
#define MSG_TYPE_ENQUIRE_DEVICE        0x45



#define MSG_JOB_SETUP_WAVEFORM_HF                        0x0014

#define MSG_JOB_SETUP_TAG_AUX_PIN_FUNC                   0x0000
#define MSG_JOB_SETUP_TAG_LOG_START_TIME                 0x0001
#define MSG_JOB_SETUP_TAG_SPECT_SAMPLE_RATE              0x0002
#define MSG_JOB_SETUP_TAG_SPECT_REF_LEVEL                0x0003
#define MSG_JOB_SETUP_TAG_POINTS_PER_FFT                 0x0004
#define MSG_JOB_SETUP_TAG_SAMPLES_BETWEEN_FFT            0x0005
#define MSG_JOB_SETUP_TAG_FFT_PROCESSING_TYPE            0x0006
#define MSG_JOB_SETUP_TAG_FFT_ACCUMULATED                0x0007
#define MSG_JOB_SETUP_TAG_FFT_WEIGHTING_FACTOR           0x0008
#define MSG_JOB_SETUP_TAG_SPECT_LOGGING_MODE             0x0009
#define MSG_JOB_SETUP_TAG_SPECT_LOG_FILE_LEN             0x000A
#define MSG_JOB_SETUP_TAG_WAVEFORM_DUTY_CYCLE_TIME       0x000B
#define MSG_JOB_SETUP_TAG_WAVEFORM_DUTY_ACTIVE_TIME      0x000C
#define MSG_JOB_SETUP_TAG_WAVEFORM_DUTY_IGNORE_TIME      0x000D
#define MSG_JOB_SETUP_TAG_WAVEFORM_SAMPLE_RATE           0x000E
#define MSG_JOB_SETUP_TAG_WAVEFORM_DATA_BITS_DEPTH       0x000F
#define MSG_JOB_SETUP_TAG_WAVEFORM_DATA_GAIN             0x0010
#define MSG_JOB_SETUP_TAG_WAVEFORM_DATA_ENDIANNESS       0x0011
#define MSG_JOB_SETUP_TAG_WAVEFROM_DATA_LOGGING_MODE     0x0012
#define MSG_JOB_SETUP_TAG_WAVEFORM_LOG_FILE_LENGTH       0x0013
#define MSG_JOB_SETUP_TAG_AUX_PIN_MESSAGE                0x0014




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
  uint16_t        tag;
  uint16_t        tag_value_len;
  uint32_t        tag_value;
} icListen_setup_tag_4byte;
#pragma pack (pop)


#pragma pack (push, 1)
typedef struct
{
	icListen_basic_header          basic_hdr;
	icListen_mask_header            mask_hdr;
	icListen_waveform_data_header    wav_hdr;

} icListen_wav_full_header;
#pragma pack (pop)


//------------------------messages---------------------------


#pragma pack (push, 1)
typedef struct
{
	icListen_basic_header          basic_hdr;
	uint16_t                             crc;
} icListen_enquire_device_msg;
#pragma pack (pop)

#pragma pack (push, 1)
typedef struct
{
	icListen_basic_header          basic_hdr;
	icListen_mask_header            mask_hdr;
	uint16_t                             crc;
} icListen_collect_short_mask_msg;
#pragma pack (pop)

#pragma pack (push, 1)
typedef struct
{
	icListen_basic_header                           basic_hdr;
	uint16_t                                    serial_number;
	uint8_t                               firmware_version[8];
	uint8_t                                    build_date[18];
	uint8_t                                       device_type;
	uint8_t                                            status;
	uint16_t                                         reserved;
	uint8_t                                   enquire_version;
} icListen_status_basic_msg;
#pragma pack (pop)


#pragma pack (push, 1)
typedef struct
{
	icListen_basic_header                           basic_hdr;
	uint16_t                                       setup_type;
	uint16_t                                      num_of_tags;
	icListen_setup_tag_4byte               t01_log_start_time;
	icListen_setup_tag_4byte         t02_spectrum_sample_rate;
	icListen_setup_tag_4byte          t06_fft_processing_type;
	icListen_setup_tag_4byte              t07_fft_accumulates;
	icListen_setup_tag_4byte        t09_spectrum_logging_mode;
	icListen_setup_tag_4byte     t0A_spectrum_log_file_length;
	icListen_setup_tag_4byte         t0E_waveform_sample_rate;
	icListen_setup_tag_4byte      t0F_waveform_data_bit_depth;
	icListen_setup_tag_4byte        t12_waveform_logging_mode;
	icListen_setup_tag_4byte     t13_waveform_log_file_length;
	icListen_setup_tag_4byte             t14_aux_pin_messages;
	uint16_t                                              crc;

} icListen_setup_full_msg;
#pragma pack (pop)

//------------- icListen object -------------------------

typedef enum{
	ICLISTEN_CONNECTED=0,
	ICLISTEN_DISCONNECTED
}icListen_status_typedef;

#pragma pack (push, 1)
typedef struct
{
  uint32_t           wav_sample_rate;
  uint32_t      wav_sample_bit_depth;
  uint32_t             file_duration;
  uint32_t                file_index;
} icListen_settings_typedef;
#pragma pack (pop)


typedef struct
{
	uint16_t                                    serial_number;
	uint8_t                               firmware_version[8];
	uint8_t                                    build_date[18];
	uint8_t                                       device_type;
	icListen_status_typedef                            status;
	icListen_settings_typedef*                       settings;
	uint32_t                             last_collect_msg_num;
	uint32_t                              collect_seq_num_err;
	uint32_t                                wav_misconfig_err;
	uint16_t                                       delay_time;

} icListen_object_typedef;



void icListen_prepare_setup_msg(icListen_setup_full_msg* msg,uint32_t wav_sample_rate,uint32_t wav_sample_bit_depth);
void icListen_prepare_collect_msg(icListen_collect_short_mask_msg* msg,uint8_t mask);
void icListen_prepare_enquire_device_msg(icListen_enquire_device_msg* msg);
F_RES icListen_parse_msg(uint8_t* msg,icListen_object_typedef* self_object,uint8_t* msg_type,memory_region_pointer* parsed_data_ptr);
void icListen_init_sensor_status(icListen_object_typedef* self_object);

#endif /* INC_ICLISTEN_H_ */
