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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
osThreadId icListen_tHandle;
osThreadId UI_tHandle;
osMessageQId AppliEventHandle;
osMessageQId USB_rxHandle;
/* USER CODE BEGIN PV */
extern uint8_t usb_rx_buff[USB_RX_BUFF_SIZE];
extern ApplicationTypeDef Appli_state;
extern USBH_HandleTypeDef hUsbHostFS;
CDC_HandleTypeDef *CDC_Handle;

icListen_wav_full_header* wav_full_hdr_p;


wav_file wav_file1;
wav_file wav_file2;
wav_file wav_file3;
wav_file wav_file4;

sd_storage_t storage1;


char t5[50];

char temp[100];
uint8_t usb_status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void storage_f(void const * argument);
void icListen_f(void const * argument);
void UI_f(void const * argument);

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

  /* definition and creation of icListen_t */
  osThreadDef(icListen_t, icListen_f, osPriorityNormal, 0, 512);
  icListen_tHandle = osThreadCreate(osThread(icListen_t), NULL);

  /* definition and creation of UI_t */
  osThreadDef(UI_t, UI_f, osPriorityIdle, 0, 256);
  UI_tHandle = osThreadCreate(osThread(UI_t), NULL);

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//HAL_UART_Receive_IT(&huart1,uart_tx_msg,1);
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
   //HAL_UART_Transmit(&huart1,"USB start\n",10,100);
  /* Infinite loop */
  for(;;)
  {
	  event = osMessageGet(AppliEventHandle, osWaitForever);

	  if(event.status == osEventMessage)
	  {
		switch(event.value.v)
		{
		 case APPLICATION_DISCONNECT:
			 usb_status=0;
		 break;

		 case APPLICATION_READY:
			 osDelay(5000);
			 usb_status=1;

		 break;

		 case APPLICATION_START:
		   set_line_coding();
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

  /*
  HAL_UART_Transmit(&huart1,"FATFS start\n",12,100);

  sd_storage_link_ss(&storage1,0,SS_SD1_Pin,GPIOA);
  sd_storage_link_ss(&storage1,1,SS_SD2_Pin,GPIOA);
  sd_storage_link_ss(&storage1,2,SS_SD3_Pin,SS_SD3_GPIO_Port);
  sd_storage_link_ss(&storage1,3,SS_SD4_Pin,GPIOA);
  sd_storage_init(&storage1);

  for(int i=0;i<SD_STORAGE_NUM_DISKS;i++)
  {
	sprintf(t5,"I:%d S:%d T:%d F:%d\n",i,storage1.disks[i].status,storage1.disks[i].size,storage1.disks[i].free_space);
	HAL_UART_Transmit(&huart1,t5,strlen(t5),100);
  }


  wav_file_open(&wav_file1,"0:test1.wav");
  wav_file_write(&wav_file1,"Test1.wav",8);
  wav_file_close(&wav_file1);


  wav_file_open(&wav_file2,"1:test2.wav");
  wav_file_write(&wav_file2,"Test2.wav",8);
  wav_file_close(&wav_file2);

  wav_file_open(&wav_file3,"2:test3.wav");
  wav_file_write(&wav_file3,"Test3.wav",8);
  wav_file_close(&wav_file3);

  wav_file_open(&wav_file4,"3:test4.wav");
  wav_file_write(&wav_file4,"Test4.wav",8);
  wav_file_close(&wav_file4);


  HAL_UART_Transmit(&huart1,"\nread SD1\n",10,100);
  readDir("0:/");
  HAL_UART_Transmit(&huart1,"read SD2\n",9,100);
  readDir("1:/");
  HAL_UART_Transmit(&huart1,"read SD3\n",9,100);
  readDir("2:/");
  HAL_UART_Transmit(&huart1,"read SD4\n",9,100);
  readDir("3:/");

  //f_unlink("0:test1.wav");
  //f_unlink("1:test2.wav");
  //f_unlink("2:test3.wav");
  //f_unlink("3:test4.wav");

  HAL_UART_Transmit(&huart1,"\nread SD1\n",10,100);
  readDir("0:/");
  HAL_UART_Transmit(&huart1,"read SD2\n",9,100);
  readDir("1:/");
  HAL_UART_Transmit(&huart1,"read SD3\n",9,100);
  readDir("2:/");
  HAL_UART_Transmit(&huart1,"read SD4\n",9,100);
  readDir("3:/");


  HAL_UART_Transmit(&huart1,"FATFS finished\n",15,100);

  */
  /* Infinite loop */
  for(;;)
  {
    osDelay(200);
  }
  /* USER CODE END storage_f */
}

/* USER CODE BEGIN Header_icListen_f */
/**
* @brief Function implementing the icListen_t thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_icListen_f */
void icListen_f(void const * argument)
{
  /* USER CODE BEGIN icListen_f */
	osEvent event;
  /* Infinite loop */
	/*
  usb_status=0;
  uint16_t prev_seq;

  while(usb_status==0)
  {
	  osDelay(1);
  }

  set_time();
  osDelay(100);
  enquire_device();
  osDelay(100);
  job_setup();
  osDelay(2000);
  collect_data();
  */
  for(;;)
  {   /*
	  if(usb_status==1){
	   event = osMessageGet(USB_rxHandle, osWaitForever);
	   if(event.status == osEventMessage)
	   {
		   if(event.value.v!=(prev_seq+1) || event.value.v==0){
	         //sprintf(temp,"Type:%d Len:%d\n",wav_full_hdr_p->basic_hdr.type,wav_full_hdr_p->basic_hdr.length);
	         //HAL_UART_Transmit(&huart1,temp,strlen(temp),100);
	         //sprintf(temp,"Mask:%d\n",wav_full_hdr_p->mask_hdr.mask);
	         //HAL_UART_Transmit(&huart1,temp,strlen(temp),100);
	         //sprintf(temp,"S:%d D:%d L:%d G:%d S:%d HS:%d MF:%d\n",wav_full_hdr_p->wav_hdr.seq_num,wav_full_hdr_p->wav_hdr.bit_depth,wav_full_hdr_p->wav_hdr.num_of_bytes,wav_full_hdr_p->wav_hdr.gain,wav_full_hdr_p->wav_hdr.sample_rate,wav_full_hdr_p->wav_hdr.sensitivity,wav_full_hdr_p->wav_hdr.full_scale_mv);
	         //HAL_UART_Transmit(&huart1,temp,strlen(temp),100);
		     sprintf(temp,"Seq:%d %d\n",event.value.v,prev_seq);
		     HAL_UART_Transmit_IT(&huart1,temp,strlen(temp));
		   }
		   prev_seq=event.value.v;
		 }
	   }
	   */
	  osDelay(2);

  }
  /* USER CODE END icListen_f */
}

/* USER CODE BEGIN Header_UI_f */
/**
* @brief Function implementing the UI_t thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UI_f */
void UI_f(void const * argument)
{
  /* USER CODE BEGIN UI_f */
  mcu_flash_typedef flash1;

  icListen_setup_full_msg setup_msg;
  icListen_collect_short_mask_msg collect_msg;

  mcu_flash_init(&flash1,7);
  //HAL_UART_Transmit(&huart1,"Flash init\n",11,100);
  if(mcu_flash_read(&flash1)==F_OK){
	  //HAL_UART_Transmit(&huart1,"CRC OK\n",7,100);

  }
  else{
	  //HAL_UART_Transmit(&huart1,"CRC BAD\n",8,100);
  }

  //HAL_UART_Transmit(&huart1,(uint8_t*)&flash1.data,18,100);
  sprintf(flash1.data.raw_data,"Hello world!!!!\n");
  mcu_flash_save(&flash1);

  icListen_prepare_setup_msg(&setup_msg,128000,24);
  icListen_prepare_collect_msg(&collect_msg,0x20);

  HAL_UART_Transmit(&huart1,(uint8_t*)&setup_msg,sizeof(icListen_setup_full_msg),100);
  HAL_UART_Transmit(&huart1,(uint8_t*)&collect_msg,sizeof(icListen_collect_short_mask_msg),100);
  /* Infinite loop */
  for(;;)
  {

    osDelay(1);
  }
  /* USER CODE END UI_f */
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
