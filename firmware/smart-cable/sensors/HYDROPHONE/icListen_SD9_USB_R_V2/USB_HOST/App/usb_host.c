/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file            : usb_host.c
  * @version         : v2.0_Cube
  * @brief           : This file implements the USB Host
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_cdc.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
CDC_LineCodingTypeDef FrameFormat;
extern osMessageQId AppliEventHandle;
extern osMessageQId USB_rxHandle;

const uint8_t  enquire_device_cmd[]={0x2A,0x45,0x00,0x00,0x19,0xCD};
const uint8_t  collect_data_cmd[]={0x2A,0x43,0x01,0x00,0x20,0x5C,0x5A};
const uint8_t  job_setup_cmd[]={0x2A,0x44,0x5C,0x00,0x14,0x00,0x0B,0x00,0x01,0x00,0x04,0x00,0xFF,0xFF,0xFF,0xFF,0x02,0x00,0x04,0x00,0x80,0xA9,0x03,0x00,0x06,0x00,0x04,0x00,0x04,0x00,0x00,0x00,0x07,0x00,0x04,0x00,0x76,0x00,0x00,0x00,0x09,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x0A,0x00,0x04,0x00,0x01,0x00,0x00,0x00,0x0E,0x00,0x04,0x00,0x80,0xA9,0x03,0x00,0x0F,0x00,0x04,0x00,0x18,0x00,0x00,0x00,0x12,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x13,0x00,0x04,0x00,0x01,0x00,0x00,0x00,0x14,0x00,0x04,0x00,0x01,0x00,0x00,0x00,0x9C,0x49};
const uint8_t  set_time_cmd[]={0x2A,0x41,0x04,0x00,0x9F,0x46,0x99,0x62,0x00,0x00};
uint8_t usb_rx_buff[USB_RX_BUFF_SIZE];

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Host core handle declaration */
USBH_HandleTypeDef hUsbHostFS;
ApplicationTypeDef Appli_state = APPLICATION_IDLE;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

void set_time(){

  USBH_CDC_Stop(&hUsbHostFS);
  USBH_CDC_Transmit(&hUsbHostFS,set_time_cmd,10);
}
void enquire_device(){

  USBH_CDC_Stop(&hUsbHostFS);
  USBH_CDC_Transmit(&hUsbHostFS,enquire_device_cmd,6);
}

void collect_data(){

  USBH_CDC_Stop(&hUsbHostFS);
  USBH_CDC_Transmit(&hUsbHostFS,collect_data_cmd,7);
}

void job_setup(){

  USBH_CDC_Stop(&hUsbHostFS);
  USBH_CDC_Transmit(&hUsbHostFS,job_setup_cmd,98);
}

void stop_function(){
  USBH_CDC_Stop(&hUsbHostFS);
}
void receive_function(){
  USBH_CDC_Stop(&hUsbHostFS);
  USBH_CDC_Receive(&hUsbHostFS,usb_rx_buff,USB_RX_BUFF_SIZE);
}
uint32_t read_bytes(USBH_HandleTypeDef *phost){
	return USBH_CDC_GetLastReceivedDataSize(phost);
}

void set_line_coding(){
	FrameFormat.b.dwDTERate = 1250000;
	FrameFormat.b.bCharFormat = 0;
	FrameFormat.b.bDataBits = 8;
	FrameFormat.b.bParityType = 0;

	USBH_CDC_SetLineCoding(&hUsbHostFS, &FrameFormat);
}

/* USER CODE END 0 */

/*
 * user callback declaration
 */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

void USBH_CDC_TransmitCallback(USBH_HandleTypeDef *phost)
{
  receive_function();
}


void USBH_CDC_ReceiveCallback(USBH_HandleTypeDef *phost)
{
}

/* USER CODE END 1 */

/**
  * Init USB host library, add supported class and start the library
  * @retval None
  */
void MX_USB_HOST_Init(void)
{
  /* USER CODE BEGIN USB_HOST_Init_PreTreatment */


  /* USER CODE END USB_HOST_Init_PreTreatment */

  /* Init host Library, add supported class and start the library. */
  if (USBH_Init(&hUsbHostFS, USBH_UserProcess, HOST_FS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_RegisterClass(&hUsbHostFS, USBH_CDC_CLASS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_Start(&hUsbHostFS) != USBH_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_HOST_Init_PostTreatment */

  /* USER CODE END USB_HOST_Init_PostTreatment */
}

/*
 * user callback definition
 */
static void USBH_UserProcess  (USBH_HandleTypeDef *phost, uint8_t id)
{
  /* USER CODE BEGIN CALL_BACK_1 */
  switch(id)
  {
  case HOST_USER_SELECT_CONFIGURATION:
  break;

  case HOST_USER_DISCONNECTION:
  Appli_state = APPLICATION_DISCONNECT;
  osMessagePut(AppliEventHandle, APPLICATION_DISCONNECT, 0);
  break;

  case HOST_USER_CLASS_ACTIVE:
  Appli_state = APPLICATION_READY;
  osMessagePut(AppliEventHandle, APPLICATION_READY, 0);
  break;

  case HOST_USER_CONNECTION:
  Appli_state = APPLICATION_START;
  osMessagePut(AppliEventHandle, APPLICATION_START, 0);
  break;

  default:
  break;
  }
  /* USER CODE END CALL_BACK_1 */
}

/**
  * @}
  */

/**
  * @}
  */

