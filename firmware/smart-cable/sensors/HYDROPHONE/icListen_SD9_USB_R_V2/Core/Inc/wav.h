/*
 * wav.h
 *
 *  Created on: 12 мая 2022 г.
 *      Author: admin
 */

#ifndef INC_WAV_H_
#define INC_WAV_H_

#include "main.h"
#include "fatfs.h"
#include "system_definitions.h"

#define DEBUG_SD


typedef struct
{
  FIL      file;
  FILINFO  fileInfo;
  uint32_t byteswritten,bytesread;
  FRESULT  res;
} fatfs_media;


#pragma pack (push, 1)
typedef struct
{
  char     prefix[4];
  uint32_t size;
  char     postfix[4];
} wav_chunk_header;
#pragma pack (pop)

#pragma pack (push, 1)
typedef struct
{
  char     prefix[4];
  uint32_t size;
  uint16_t format;
  uint16_t numchannels;
  uint32_t samplerate;
  uint32_t byterate;
  uint16_t block_align;
  uint16_t bps;
} wav_subchunk_fmt_header;
#pragma pack (pop)

#pragma pack (push, 1)
typedef struct
{
  char     prefix[4];
  uint32_t size;
} wav_subchunk_data_header;
#pragma pack (pop)

#pragma pack (push, 1)
typedef struct
{
	wav_chunk_header          chunk;
	wav_subchunk_fmt_header   subchunk_fmt;
	wav_subchunk_data_header  subchunk_data;
} wav_header_typedef;
#pragma pack (pop)


#pragma pack (push, 1)
typedef struct
{
	wav_header_typedef            header;
	uint32_t                data_counter;
	fatfs_media             media;
} wav_file_typedef;
#pragma pack (pop)

F_RES wav_file_open(wav_file_typedef* self_object,char* filename);
F_RES wav_file_close(wav_file_typedef* self_object);
F_RES wav_file_write(wav_file_typedef* self_object,uint8_t* data,uint32_t length);

void readDir(char* dir_name);

#endif /* INC_WAV_H_ */
