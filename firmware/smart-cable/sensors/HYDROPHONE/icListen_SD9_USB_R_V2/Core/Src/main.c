/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "em_sd_storage.h"
#include "wav.h"
#include "icListen.h"
#include "mcu_flash.h"
#include "UI.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define  UI_UART               huart1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId storage_tHandle;
osThreadId main_tHandle;
osThreadId uart_tHandle;
osMessageQId AppliEventHandle;
osMessageQId USB_rxHandle;
osMessageQId USB_txHandle;
osMessageQId storage_wHandle;
/* USER CODE BEGIN PV */
extern uint8_t usb_rx_buff[USB_RX_BUFF_SIZE];
extern ApplicationTypeDef Appli_state;
extern USBH_HandleTypeDef hUsbHostFS;
CDC_HandleTypeDef *CDC_Handle;

extern icListen_object_typedef icListen;
icListen_status_basic_msg   status_msg;
memory_region_pointer      send_mem_ptr;
memory_region_pointer  collect_msg_ptr;


UI_typedef user_interface;
mcu_flash_typedef mcu_flash;

sd_storage_t microsd_storage;
wav_file_typedef wav_file;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void storage_f(void const * argument);
void main_f(void const * argument);
void uart_f(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  UI_init(&user_interface);
  HAL_UART_Receive_IT(&UI_UART,&(user_interface.media_rx_byte),1);

  mcu_flash_init(&mcu_flash,FLASH_SECTOR_11);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of AppliEvent */
  osMessageQDef(AppliEvent, 16, uint16_t);
  AppliEventHandle = osMessageCreate(osMessageQ(AppliEvent), NULL);

  /* definition and creation of USB_rx */
  osMessageQDef(USB_rx, 10, uint32_t);
  USB_rxHandle = osMessageCreate(osMessageQ(USB_rx), NULL);

  /* definition and creation of USB_tx */
  osMessageQDef(USB_tx, 10, uint32_t);
  USB_txHandle = osMessageCreate(osMessageQ(USB_tx), NULL);

  /* definition and creation of storage_w */
  osMessageQDef(storage_w, 5, uint32_t);
  storage_wHandle = osMessageCreate(osMessageQ(storage_w), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of storage_t */
  osThreadDef(storage_t, storage_f, osPriorityNormal, 0, 512);
  storage_tHandle = osThreadCreate(osThread(storage_t), NULL);

  /* definition and creation of main_t */
  osThreadDef(main_t, main_f, osPriorityNormal, 0, 512);
  main_tHandle = osThreadCreate(osThread(main_t), NULL);

  /* definition and creation of uart_t */
  osThreadDef(uart_t, uart_f, osPriorityNormal, 0, 256);
  uart_tHandle = osThreadCreate(osThread(uart_t), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SS_SD3_GPIO_Port, SS_SD3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SS_SD1_Pin|SS_SD2_Pin|SS_SD4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_VBUS_GPIO_Port, USB_VBUS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SS_SD3_Pin */
  GPIO_InitStruct.Pin = SS_SD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SS_SD3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SS_SD1_Pin SS_SD2_Pin SS_SD4_Pin */
  GPIO_InitStruct.Pin = SS_SD1_Pin|SS_SD2_Pin|SS_SD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint8_t tmp1;
uint8_t tmp2;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
 if(huart==&UI_UART)
 {
	 UI_media_process_byte(&user_interface,user_interface.media_rx_byte);
	 HAL_UART_Receive_IT(&UI_UART,&(user_interface.media_rx_byte),1);
 }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

 if(huart==&UI_UART)
 {
	 if(UI_media_get_byte(&user_interface,&tmp2)==UI_F_OK)
	 {
	    HAL_UART_Transmit_IT(&UI_UART,&tmp2,1);
	 }
 }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 5 */
   osEvent event;
  /* Infinite loop */
  for(;;)
  {
	  event = osMessageGet(AppliEventHandle, osWaitForever);

	  if(event.status == osEventMessage)
	  {
		switch(event.value.v)
		{
		 case APPLICATION_DISCONNECT:
			 icListen.status=DISCONNECTED;
		 break;

		 case APPLICATION_READY:
			 osDelay(5000);
			 icListen.status=CONNECTED;
			 osMessagePut(USB_txHandle,&collect_msg_ptr, 0);
		 break;

		 case APPLICATION_START:
		   USB_set_line_coding();
		 break;

		 default:
		 break;
		 }
	   }
  }

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_storage_f */
/**
* @brief Function implementing the storage_t thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_storage_f */
void storage_f(void const * argument)
{
  /* USER CODE BEGIN storage_f */
  osDelay(500);
  char ttr[20];
  memory_region_pointer msg_ptr;
  memory_region_pointer* data_ptr;
  msg_ptr.start_addr=ttr;

  osEvent storage_w_event;

  sd_storage_link_ss(&microsd_storage,0,SS_SD1_Pin,GPIOA);
  sd_storage_link_ss(&microsd_storage,1,SS_SD2_Pin,GPIOA);
  sd_storage_link_ss(&microsd_storage,2,SS_SD3_Pin,SS_SD3_GPIO_Port);
  sd_storage_link_ss(&microsd_storage,3,SS_SD4_Pin,GPIOA);
  sd_storage_init(&microsd_storage);

  if(wav_file_open(&wav_file,"0:test.wav")==F_ERR)
  {
   if(wav_file_open(&wav_file,"1:test.wav")==F_ERR)
   {
	if(wav_file_open(&wav_file,"2:test.wav")==F_ERR)
	{
	 if(wav_file_open(&wav_file,"3:test.wav")==F_ERR)
	 {
		 sprintf(ttr,"SD error\r");
	 }
	 else sprintf(ttr,"SD3 detected\r");
	}
	else sprintf(ttr,"SD2 detected\r");
   }
   else sprintf(ttr,"SD1 detected\r");
  }
  else sprintf(ttr,"SD0 detected\r");
  msg_ptr.size=strlen(msg_ptr.start_addr);
  UI_send_msg(&user_interface,UI_CMD_SEND_DATA,&msg_ptr);

  //readDir("0:/");
  //f_unlink("0:test1.wav");

  /* Infinite loop */
  for(;;)
  {
	  storage_w_event = osMessageGet(storage_wHandle, osWaitForever);
	  if(storage_w_event.status == osEventMessage){
		data_ptr=(memory_region_pointer*)storage_w_event.value.v;
		wav_file_write(&wav_file,data_ptr->start_addr,data_ptr->size);
		wav_file_close(&wav_file);
	  }

  }
  /* USER CODE END storage_f */
}

/* USER CODE BEGIN Header_main_f */
/**
* @brief Function implementing the main_t thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_main_f */
void main_f(void const * argument)
{
  /* USER CODE BEGIN main_f */
  icListen.settings=(icListen_settings_typedef*)mcu_flash.data.raw_data;
  memory_region_pointer* mem_ptr;


  icListen_collect_short_mask_msg collect_msg;
  icListen_prepare_collect_msg(&collect_msg,0x20);

  collect_msg_ptr.start_addr=(uint8_t*)&collect_msg;
  collect_msg_ptr.size=sizeof(icListen_collect_short_mask_msg);

  osEvent event;

  if(mcu_flash_read(&mcu_flash)!=F_OK){
		icListen.settings->wav_sample_rate=ICLISTEN_DEFAULT_WAV_SAMPLE_RATE;
		icListen.settings->wav_sample_bit_depth=ICLISTEN_DEFAULT_WAV_SAMPLE_BIT_DEPTH;
		icListen.settings->file_duration=ICLISTEN_DEFAULT_FILE_DURATION;
		mcu_flash_save(&mcu_flash);
  }
  icListen.status=DISCONNECTED;

  while(icListen.status==DISCONNECTED) osDelay(1);

  /* Infinite loop */
  for(;;)
  {
	  if(icListen.status==CONNECTED){
	   event = osMessageGet(USB_txHandle, osWaitForever);
	   if(event.status == osEventMessage){
		   mem_ptr=(memory_region_pointer*)event.value.v;
		   USB_transmit_msg(mem_ptr->start_addr,mem_ptr->size);
		   //osMessagePut(USB_txHandle,(uint8_t*)&collect_msg_ptr, 0);
	   }
	   event = osMessageGet(USB_rxHandle, 10);
	   if(event.status == osEventMessage){
		   icListen_parse_msg((uint8_t*)event.value.v,&icListen);
	   }
	  }
   	  osDelay(2);
  }
  /* USER CODE END main_f */
}

/* USER CODE BEGIN Header_uart_f */
/**
* @brief Function implementing the uart_t thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uart_f */
void uart_f(void const * argument)
{
  /* USER CODE BEGIN uart_f */
  uint8_t tmp;
  /* Infinite loop */
  user_interface.media_status=UI_MEDIA_READY;
  for(;;)
  {
	   if(UI_UART.gState!=HAL_UART_STATE_BUSY_TX)
	   {
		if(UI_media_get_byte(&user_interface,&tmp)==UI_F_OK)
		{
		   HAL_UART_Transmit_IT(&UI_UART,&tmp,1);
		}
	   }
	  osDelay(1);
  }
  /* USER CODE END uart_f */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
