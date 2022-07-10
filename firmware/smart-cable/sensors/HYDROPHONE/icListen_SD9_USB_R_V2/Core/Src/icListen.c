/*
 * icListen.c
 *
 *  Created on: 13 июн. 2022 г.
 *      Author: admin
 */
#include "icListen.h"
#include "crc.h"
#include "string.h"
#include "cmsis_os.h"
#include "UI.h"

icListen_object_typedef icListen;


void icListen_init_sensor_status(icListen_object_typedef* self_object)
{
	self_object->status=ICLISTEN_DISCONNECTED;
	memset(self_object->build_date,0x00,18);
	memset(self_object->firmware_version,0x00,8);
	self_object->serial_number=0;
	self_object->device_type=0;
	self_object->last_collect_msg_num=0;
	self_object->collect_seq_num_err=0;
}

void icListen_prepare_setup_msg(icListen_setup_full_msg* msg,uint32_t wav_sample_rate,uint32_t wav_sample_bit_depth)
{
	msg->basic_hdr.sync=MSG_SYNC;
	msg->basic_hdr.type=MSG_TYPE_JOB_SETUP;
	msg->setup_type=MSG_JOB_SETUP_WAVEFORM_HF;
	msg->num_of_tags=11;

	msg->t01_log_start_time.tag=MSG_JOB_SETUP_TAG_LOG_START_TIME;
	msg->t01_log_start_time.tag_value=4294967295;
	msg->t01_log_start_time.tag_value_len=4;

	msg->t02_spectrum_sample_rate.tag=MSG_JOB_SETUP_TAG_SPECT_SAMPLE_RATE;
	msg->t02_spectrum_sample_rate.tag_value=0;
	msg->t02_spectrum_sample_rate.tag_value_len=4;

	msg->t06_fft_processing_type.tag=MSG_JOB_SETUP_TAG_FFT_PROCESSING_TYPE;
	msg->t06_fft_processing_type.tag_value=0;
	msg->t06_fft_processing_type.tag_value_len=4;

	msg->t07_fft_accumulates.tag=MSG_JOB_SETUP_TAG_FFT_ACCUMULATED;
	msg->t07_fft_accumulates.tag_value=0;
	msg->t07_fft_accumulates.tag_value_len=4;

	msg->t09_spectrum_logging_mode.tag=MSG_JOB_SETUP_TAG_SPECT_LOGGING_MODE;
	msg->t09_spectrum_logging_mode.tag_value=0;
	msg->t09_spectrum_logging_mode.tag_value_len=4;

	msg->t0A_spectrum_log_file_length.tag=MSG_JOB_SETUP_TAG_SPECT_LOG_FILE_LEN;
	msg->t0A_spectrum_log_file_length.tag_value=1;
	msg->t0A_spectrum_log_file_length.tag_value_len=4;

	msg->t0E_waveform_sample_rate.tag=MSG_JOB_SETUP_TAG_WAVEFORM_SAMPLE_RATE;
	msg->t0E_waveform_sample_rate.tag_value=wav_sample_rate;
	msg->t0E_waveform_sample_rate.tag_value_len=4;

	msg->t0F_waveform_data_bit_depth.tag=MSG_JOB_SETUP_TAG_WAVEFORM_DATA_BITS_DEPTH;
	msg->t0F_waveform_data_bit_depth.tag_value=wav_sample_bit_depth;
	msg->t0F_waveform_data_bit_depth.tag_value_len=4;

	msg->t12_waveform_logging_mode.tag=MSG_JOB_SETUP_TAG_WAVEFROM_DATA_LOGGING_MODE;
	msg->t12_waveform_logging_mode.tag_value=0;
	msg->t12_waveform_logging_mode.tag_value_len=4;

	msg->t13_waveform_log_file_length.tag=MSG_JOB_SETUP_TAG_WAVEFORM_LOG_FILE_LENGTH;
	msg->t13_waveform_log_file_length.tag_value=1;
	msg->t13_waveform_log_file_length.tag_value_len=4;

	msg->t14_aux_pin_messages.tag=MSG_JOB_SETUP_TAG_AUX_PIN_MESSAGE;
	msg->t14_aux_pin_messages.tag_value=1;
	msg->t14_aux_pin_messages.tag_value_len=4;


	msg->basic_hdr.length=sizeof(icListen_setup_full_msg)-4-2;
	msg->crc=get_crc16_arc((uint8_t*)msg,sizeof(icListen_setup_full_msg)-2);
}

void icListen_prepare_collect_msg(icListen_collect_short_mask_msg* msg,uint8_t mask)
{
	msg->basic_hdr.sync=MSG_SYNC;
    msg->basic_hdr.type=MSG_TYPE_COLLECT_DATA;
    msg->mask_hdr.mask=mask;
    msg->basic_hdr.length=1;
	msg->crc=get_crc16_arc((uint8_t*)msg,sizeof(icListen_collect_short_mask_msg)-2);
}

void icListen_prepare_enquire_device_msg(icListen_enquire_device_msg* msg)
{
	msg->basic_hdr.sync=MSG_SYNC;
    msg->basic_hdr.type=MSG_TYPE_ENQUIRE_DEVICE;
    msg->basic_hdr.length=0;
	msg->crc=get_crc16_arc((uint8_t*)msg,sizeof(icListen_enquire_device_msg)-2);

}

F_RES icListen_parse_msg(uint8_t* msg,icListen_object_typedef* self_object,uint8_t* msg_type,memory_region_pointer* parsed_data_ptr)
{

	icListen_basic_header* basic_header=(icListen_basic_header*)msg;
	icListen_status_basic_msg* status_msg=(icListen_status_basic_msg*)msg;
	icListen_wav_full_header* collected_wav_header=(icListen_wav_full_header*)msg;

	uint16_t crc_msg=*(uint16_t*)(msg+basic_header->length+4);

	if(basic_header->sync==MSG_SYNC){
	*msg_type=basic_header->type;
	switch(basic_header->type){
	     case MSG_TYPE_ENQUIRE_DEVICE:
	    	 if(crc_msg==get_crc16_arc(msg,(uint16_t)basic_header->length+4)){
		    	  self_object->status=status_msg->status;
		    	  memcpy(self_object->build_date,status_msg->build_date,18);
		    	  memcpy(self_object->firmware_version,status_msg->firmware_version,8);
		    	  self_object->serial_number=status_msg->serial_number;
		    	  self_object->device_type=status_msg->device_type;
		    	  return F_OK;
	    	  }
	    	 else return F_ERR;
	     break;
		 case MSG_TYPE_COLLECT_DATA:
			 if(collected_wav_header->basic_hdr.length>1){
			      if((self_object->last_collect_msg_num+1)!=collected_wav_header->wav_hdr.seq_num && self_object->last_collect_msg_num !=0){
			    	  self_object->collect_seq_num_err++;
			      }
			      self_object->last_collect_msg_num=collected_wav_header->wav_hdr.seq_num;
		    	//parsed_data_ptr->start_addr=msg+sizeof(icListen_basic_header);
			      parsed_data_ptr->start_addr=msg;
				  parsed_data_ptr->size=collected_wav_header->basic_hdr.length+4;
				  return F_OK;
			 }
			 parsed_data_ptr->size=0;
		     return F_OK;
		 break;
		 default:
			 return F_ERR;
	     break;
	 };
	}
	else{
		return F_ERR;
	}
	return F_ERR;
}
