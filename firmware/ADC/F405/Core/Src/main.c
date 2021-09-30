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
#include "interrupts.h"
#include "PUCK.h"
#include "SSP.h"
#include "smac.h"
#include "ADC.h"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId defaultTaskHandle;
osThreadId uart1_smac_tHandle;
osThreadId puck_tHandle;
osThreadId ssp_tHandle;
osThreadId adc_tHandle;
osMessageQId uart1_smac_rx_inQHandle;
osMessageQId uart1_smac_tx_outQHandle;
osMessageQId puck_rxQHandle;
osMessageQId ssp_rxQHandle;
/* USER CODE BEGIN PV */
smac_controller*  uart1_smac;
extern adc_object adc_main;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void uart1_smac_f(void const * argument);
void puck_f(void const * argument);
void ssp_f(void const * argument);
void adc_f(void const * argument);

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
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
  /* definition and creation of uart1_smac_rx_inQ */
  osMessageQDef(uart1_smac_rx_inQ, 100, uint8_t);
  uart1_smac_rx_inQHandle = osMessageCreate(osMessageQ(uart1_smac_rx_inQ), NULL);

  /* definition and creation of uart1_smac_tx_outQ */
  osMessageQDef(uart1_smac_tx_outQ, 10, uint8_t*);
  uart1_smac_tx_outQHandle = osMessageCreate(osMessageQ(uart1_smac_tx_outQ), NULL);

  /* definition and creation of puck_rxQ */
  osMessageQDef(puck_rxQ, 50, uint8_t);
  puck_rxQHandle = osMessageCreate(osMessageQ(puck_rxQ), NULL);

  /* definition and creation of ssp_rxQ */
  osMessageQDef(ssp_rxQ, 50, uint8_t);
  ssp_rxQHandle = osMessageCreate(osMessageQ(ssp_rxQ), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of uart1_smac_t */
  osThreadDef(uart1_smac_t, uart1_smac_f, osPriorityIdle, 0, 128);
  uart1_smac_tHandle = osThreadCreate(osThread(uart1_smac_t), NULL);

  /* definition and creation of puck_t */
  osThreadDef(puck_t, puck_f, osPriorityIdle, 0, 128);
  puck_tHandle = osThreadCreate(osThread(puck_t), NULL);

  /* definition and creation of ssp_t */
  osThreadDef(ssp_t, ssp_f, osPriorityIdle, 0, 256);
  ssp_tHandle = osThreadCreate(osThread(ssp_t), NULL);

  /* definition and creation of adc_t */
  osThreadDef(adc_t, adc_f, osPriorityIdle, 0, 128);
  adc_tHandle = osThreadCreate(osThread(adc_t), NULL);

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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim2.Init.Period = 800000;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uart_rx_it(huart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uart_tx_it(huart);
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

  timed_callbacks_list_init(&htim2);
  smac_list_init();
  uart1_smac=smac_create_new(&huart1,uart1_smac_rx_inQHandle,uart1_smac_tx_outQHandle);
  puck_init(uart1_smac,puck_rxQHandle,ssp_rxQHandle);
  ssp_init(uart1_smac,ssp_rxQHandle);
  adc_init(&hi2c1);

  uint8_t uart_buf[20];

  for(;;)
  {
	  /*
	  p=(uint8_t*) malloc(sizeof(uint8_t));
	  *p=*p+1;
	 if(smac_send(uart1_smac,p,1,FREE_MEMORY_FLAG)!=RES_OK)
	 {
		 free(p);
	 }
	 */
	 //smac_send(uart1_smac,tt,10,KEEP_MEMORY_FLAG);

	 //sprintf(uart_buf,"adc:%d\r",adc_main.adc_ch1_value);
	 //HAL_UART_Transmit(&huart1,uart_buf,strlen(uart_buf),100);
	 osDelay(1000);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_uart1_smac_f */
/**
* @brief Function implementing the uart1_smac_t thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uart1_smac_f */
void uart1_smac_f(void const * argument)
{
  /* USER CODE BEGIN uart1_smac_f */
  /* Infinite loop */
  osDelay(500);
  smac_loop(uart1_smac);
  /* USER CODE END uart1_smac_f */
}

/* USER CODE BEGIN Header_puck_f */
/**
* @brief Function implementing the puck_t thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_puck_f */
void puck_f(void const * argument)
{
  /* USER CODE BEGIN puck_f */
  /* Infinite loop */
  osDelay(500);
  puck_loop();
  /* USER CODE END puck_f */
}

/* USER CODE BEGIN Header_ssp_f */
/**
* @brief Function implementing the ssp_t thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ssp_f */
void ssp_f(void const * argument)
{
  /* USER CODE BEGIN ssp_f */
  /* Infinite loop */
  osDelay(500);
  ssp_loop();
  /* USER CODE END ssp_f */
}

/* USER CODE BEGIN Header_adc_f */
/**
* @brief Function implementing the adc_t thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_adc_f */
void adc_f(void const * argument)
{
  /* USER CODE BEGIN adc_f */
  /* Infinite loop */
  osDelay(500);
  adc_loop();
  /* USER CODE END adc_f */
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

  else if (htim->Instance == TIM2) { // should be every 10 ms
	  timed_callbacks_update();
   }

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
