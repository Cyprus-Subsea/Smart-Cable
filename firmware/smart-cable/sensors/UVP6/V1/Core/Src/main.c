/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "seaglider.h"
#include "UVP6.h"
#include "mcu_flash.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define  UVP6_UART               huart1
#define  GLIDER_UART             huart5

#define  UVP_DEPTH_HL_LEVEL      100.0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId uart_tx_tHandle;
/* USER CODE BEGIN PV */

uvp6 uvp6_sensor1;
seaglider glider1;
mcu_flash data_flash;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void uart_tx_f(void const * argument);

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
  MX_UART5_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  uvp6_init(&uvp6_sensor1);
  HAL_UART_Receive_IT(&UVP6_UART,&(uvp6_sensor1.media_rx_byte),1);

  seaglider_init(&glider1);
  HAL_UART_Receive_IT(&GLIDER_UART,&(glider1.media_rx_byte),1);

  mcu_flash_init(&data_flash,82);

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of uart_tx_t */
  osThreadDef(uart_tx_t, uart_tx_f, osPriorityNormal, 0, 128);
  uart_tx_tHandle = osThreadCreate(osThread(uart_tx_t), NULL);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL5;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart1.Init.BaudRate = 38400;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
uint8_t tmp1;
uint8_t tmp2;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
 if(huart==&UVP6_UART)
 {   //send new received byte to uvp6 object
	 uvp6_media_process_byte(&uvp6_sensor1,uvp6_sensor1.media_rx_byte);
	 HAL_UART_Receive_IT(&UVP6_UART,&(uvp6_sensor1.media_rx_byte),1);
 }
 else if(huart==&GLIDER_UART)
 {
	 seaglider_media_process_byte(&glider1,glider1.media_rx_byte);
	 HAL_UART_Receive_IT(&GLIDER_UART,&(glider1.media_rx_byte),1);
 }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

 if(huart==&UVP6_UART)
 {
	 if(uvp6_media_get_byte(&uvp6_sensor1,&tmp2)==UVP6_F_OK)
	 {
	    HAL_UART_Transmit_IT(&UVP6_UART,&tmp2,1);
	 }
 }
 else if(huart==&GLIDER_UART)
 {
	 if(seaglider_media_get_byte(&glider1,&tmp1)==SEAGLIDER_F_OK)
	 {
	    HAL_UART_Transmit_IT(&GLIDER_UART,&tmp1,1);
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
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  mcu_flash_open(&data_flash);
  char tmp_str[20];
  uint8_t event_id;
  char avg_str[40];
  uint16_t avg_data[5];
  uint8_t y=0;
  memory_region_pointer ptr1;

  uint8_t lpm_messages_couter=0;

  for(;;)
  {
	 //glider task
	 if(seaglider_get_event(&glider1,&event_id)==SEAGLIDER_F_OK)
	 {
		switch(event_id)
		{
		 case SEAGLIDER_EVNT_DEPTH_RCVD:

			 memcpy(tmp_str,glider1.date,8);
			 memcpy(tmp_str+8,",",1);
			 memcpy(tmp_str+9,glider1.time,6);
			 tmp_str[15]=0x00;

			 if(glider1.last_depth>UVP_DEPTH_HL_LEVEL&&glider1.prev_depth<=UVP_DEPTH_HL_LEVEL)
		     {
				uvp6_send_cmd(&uvp6_sensor1,UVP6_CMD_STOP_ACQ,NULL);
				osDelay(1000);
				uvp6_send_cmd(&uvp6_sensor1,UVP6_CMD_START_L_ACQ,tmp_str);
			 }
			 else if(glider1.last_depth<=UVP_DEPTH_HL_LEVEL&&glider1.prev_depth>UVP_DEPTH_HL_LEVEL)
			 {
				uvp6_send_cmd(&uvp6_sensor1,UVP6_CMD_STOP_ACQ,NULL);
			    osDelay(1000);
				uvp6_send_cmd(&uvp6_sensor1,UVP6_CMD_START_H_ACQ,tmp_str);
			 }
			 seaglider_send_cmd(&glider1,SEAGLIDER_CMD_PROMPT,NULL);
         break;
		 case SEAGLIDER_EVNT_START_RCVD:
		   if(uvp6_sensor1.status!=UVP6_READY) osDelay(2000);
		   if(uvp6_sensor1.status!=UVP6_READY) break;
			 memcpy(tmp_str,glider1.date,8);
			 memcpy(tmp_str+8,",",1);
			 memcpy(tmp_str+9,glider1.time,6);
			 tmp_str[15]=0x00;
			 mcu_flash_flush(&data_flash);
			 if(glider1.last_depth<=UVP_DEPTH_HL_LEVEL)
			  {
  		        uvp6_send_cmd(&uvp6_sensor1,UVP6_CMD_START_H_ACQ,tmp_str);
			  }
			 else if(glider1.last_depth>UVP_DEPTH_HL_LEVEL)
  			  {
  		    	uvp6_send_cmd(&uvp6_sensor1,UVP6_CMD_START_L_ACQ,tmp_str);
  			  }
			 seaglider_send_cmd(&glider1,SEAGLIDER_CMD_PROMPT,NULL);
		 break;
		 case SEAGLIDER_EVNT_CLOCK_RCVD:
			 seaglider_send_cmd(&glider1,SEAGLIDER_CMD_PROMPT,NULL);
		 break;
		 case SEAGLIDER_EVNT_WAKEUP_RCVD:
			 seaglider_send_cmd(&glider1,SEAGLIDER_CMD_PROMPT,NULL);
		 break;
		 case SEAGLIDER_EVNT_STOP_RCVD:
			  uvp6_send_cmd(&uvp6_sensor1,UVP6_CMD_STOP_ACQ,NULL);
			  osDelay(1000);
			  mcu_flash_close(&data_flash,MCU_FLASH_CLEAN_FLAG);
			  seaglider_send_cmd(&glider1,SEAGLIDER_CMD_PROMPT,NULL);
		 break;
		 case SEAGLIDER_EVNT_TEST_RCVD:
			  ptr1.start_addr=data_flash.data_pages_addr;
			  ptr1.size=data_flash.flash_state.write_indx;
			  seaglider_send_cmd(&glider1,SEAGLIDER_CMD_SEND_DATA,&ptr1);
			  seaglider_send_cmd(&glider1,SEAGLIDER_CMD_PROMPT,NULL);
		 break;
		 case SEAGLIDER_EVNT_SEND_TXT_FILE_RCVD:
			  mcu_flash_close(&data_flash,MCU_FLASH_DIRTY_FLAG);
			  ptr1.start_addr=data_flash.data_pages_addr;
			  ptr1.size=data_flash.flash_state.write_indx;
			  seaglider_send_cmd(&glider1,SEAGLIDER_CMD_SEND_DATA,&ptr1);
			  seaglider_send_cmd(&glider1,SEAGLIDER_CMD_PROMPT,NULL);
		 break;

		}
	 }

	 //UVP6 tasks

	 if(uvp6_get_event(&uvp6_sensor1,&event_id)==UVP6_F_OK)
	 {
        switch(event_id)
        {
         case UVP6_EVNT_BOOTED:
        	 //osDelay(1000);
        	 seaglider_send_cmd(&glider1,SEAGLIDER_CMD_PROMPT,NULL);
         break;
         case UVP6_EVNT_LPM_DATA_RCVD:
			//save only every 10 message
			lpm_messages_couter++;
			lpm_messages_couter=lpm_messages_couter%10;
        	//read LPM values
          if(uvp6_sensor1.status==UVP6_ACQ_STARTED && lpm_messages_couter==1)
        	{

        	  y=0;
			  for(int i=0;i<13;i+=4)
			  {
				avg_data[y]=uvp6_sensor1.lpm_data.data[i]+uvp6_sensor1.lpm_data.data[i+1]+uvp6_sensor1.lpm_data.data[i+2]+uvp6_sensor1.lpm_data.data[i+3];
				y++;
			  }
			avg_data[4]=uvp6_sensor1.lpm_data.data[16]+uvp6_sensor1.lpm_data.data[17];
			sprintf(avg_str,"%d,%d,%d,%d,%d\n",avg_data[0],avg_data[1],avg_data[2],avg_data[3],avg_data[4]);
			mcu_flash_write(&data_flash,avg_str,strlen(avg_str));
			//seaglider_send_cmd(&glider1,SEAGLIDER_CMD_PROMPT,NULL);
          }
		 break;
        }
	 }
	 osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_uart_tx_f */
/**
* @brief Function implementing the uart_tx_t thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uart_tx_f */
void uart_tx_f(void const * argument)
{
  /* USER CODE BEGIN uart_tx_f */
  /* Infinite loop */
  uint8_t tmp1;
  uint8_t tmp2;
  uvp6_sensor1.media_status=UVP6_MEDIA_READY;
  glider1.media_status=SEAGLIDER_MEDIA_READY;

  for(;;)
  {
       if(GLIDER_UART.gState!=HAL_UART_STATE_BUSY_TX)
       {
		if(seaglider_media_get_byte(&glider1,&tmp1)==SEAGLIDER_F_OK)
		{
		   HAL_UART_Transmit_IT(&GLIDER_UART,&tmp1,1);
		}
       }
       if(UVP6_UART.gState!=HAL_UART_STATE_BUSY_TX)
       {
		if(uvp6_media_get_byte(&uvp6_sensor1,&tmp2)==UVP6_F_OK)
		{
		   HAL_UART_Transmit_IT(&UVP6_UART,&tmp2,1);
		}
       }
    osDelay(1);
  }
  /* USER CODE END uart_tx_f */
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
