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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "seaglider.h"
#include "HYDROC.h"
#include "string.h"
#include "em_sd_storage.h"
#include "stdio.h"
#include "settings.h"
#include "disp_proc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define  SENSOR_UART             huart1
#define  GLIDER_UART             huart5

#define TX_BUFF_SIZE             150

//#define DEBUG_MSG

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId glider_uart_tx_Handle;
osThreadId worker1_tHandle;
osThreadId worker2_tHandle;
osThreadId worker3_tHandle;
osThreadId worker4_tHandle;
osThreadId hydroc_uart_tx_Handle;
osThreadId worker5_tHandle;
osThreadId worker6_tHandle;
osMessageQId worker_cmd_qHandle;
osMessageQId events_qHandle;
osSemaphoreId seaglider_uart_q_semHandle;
osSemaphoreId hydroc_uart_q_semHandle;
osSemaphoreId seaglider_uart_media_semHandle;
osSemaphoreId hydroc_uart_media_semHandle;
osSemaphoreId microSD_semHandle;
/* USER CODE BEGIN PV */

seaglider glider1;
hydroc hydroc_sensor1;
sd_storage_t microsd_storage;
extern settings_str run_cfg;
proc_dispatcher dispatcher1;


uint32_t tick1,tick2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void glider_uart_tx_f(void const * argument);
void worker_f(void const * argument);
void hydroc_uart_tx_f(void const * argument);

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
  MX_SPI1_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of seaglider_uart_q_sem */
  osSemaphoreDef(seaglider_uart_q_sem);
  seaglider_uart_q_semHandle = osSemaphoreCreate(osSemaphore(seaglider_uart_q_sem), 1);

  /* definition and creation of hydroc_uart_q_sem */
  osSemaphoreDef(hydroc_uart_q_sem);
  hydroc_uart_q_semHandle = osSemaphoreCreate(osSemaphore(hydroc_uart_q_sem), 1);

  /* definition and creation of seaglider_uart_media_sem */
  osSemaphoreDef(seaglider_uart_media_sem);
  seaglider_uart_media_semHandle = osSemaphoreCreate(osSemaphore(seaglider_uart_media_sem), 1);

  /* definition and creation of hydroc_uart_media_sem */
  osSemaphoreDef(hydroc_uart_media_sem);
  hydroc_uart_media_semHandle = osSemaphoreCreate(osSemaphore(hydroc_uart_media_sem), 1);

  /* definition and creation of microSD_sem */
  osSemaphoreDef(microSD_sem);
  microSD_semHandle = osSemaphoreCreate(osSemaphore(microSD_sem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of worker_cmd_q */
  osMessageQDef(worker_cmd_q, 16, uint32_t);
  worker_cmd_qHandle = osMessageCreate(osMessageQ(worker_cmd_q), NULL);

  /* definition and creation of events_q */
  osMessageQDef(events_q, 16, uint32_t);
  events_qHandle = osMessageCreate(osMessageQ(events_q), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of glider_uart_tx_ */
  osThreadDef(glider_uart_tx_, glider_uart_tx_f, osPriorityNormal, 0, 256);
  glider_uart_tx_Handle = osThreadCreate(osThread(glider_uart_tx_), NULL);

  /* definition and creation of worker1_t */
  osThreadDef(worker1_t, worker_f, osPriorityNormal, 0, 512);
  worker1_tHandle = osThreadCreate(osThread(worker1_t), NULL);

  /* definition and creation of worker2_t */
  osThreadDef(worker2_t, worker_f, osPriorityNormal, 0, 512);
  worker2_tHandle = osThreadCreate(osThread(worker2_t), NULL);

  /* definition and creation of worker3_t */
  osThreadDef(worker3_t, worker_f, osPriorityNormal, 0, 512);
  worker3_tHandle = osThreadCreate(osThread(worker3_t), NULL);

  /* definition and creation of worker4_t */
  osThreadDef(worker4_t, worker_f, osPriorityNormal, 0, 512);
  worker4_tHandle = osThreadCreate(osThread(worker4_t), NULL);

  /* definition and creation of hydroc_uart_tx_ */
  osThreadDef(hydroc_uart_tx_, hydroc_uart_tx_f, osPriorityNormal, 0, 256);
  hydroc_uart_tx_Handle = osThreadCreate(osThread(hydroc_uart_tx_), NULL);

  /* definition and creation of worker5_t */
  osThreadDef(worker5_t, worker_f, osPriorityNormal, 0, 512);
  worker5_tHandle = osThreadCreate(osThread(worker5_t), NULL);

  /* definition and creation of worker6_t */
  osThreadDef(worker6_t, worker_f, osPriorityNormal, 0, 512);
  worker6_tHandle = osThreadCreate(osThread(worker6_t), NULL);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  huart5.Init.BaudRate = 9600;
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
  huart1.Init.BaudRate = 9600;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SS_SD1_GPIO_Port, SS_SD1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SS_SD1_Pin */
  GPIO_InitStruct.Pin = SS_SD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SS_SD1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t tmp1;
uint8_t tmp2;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
 if(huart==&SENSOR_UART)
 {
	 hydroc_media_process_byte(&hydroc_sensor1,hydroc_sensor1.media_rx_byte);
	 HAL_UART_Receive_IT(&SENSOR_UART,&(hydroc_sensor1.media_rx_byte),1);
 }
 else if(huart==&GLIDER_UART)
 {
	 seaglider_media_process_byte(&glider1,glider1.media_rx_byte);
	 HAL_UART_Receive_IT(&GLIDER_UART,&(glider1.media_rx_byte),1);
 }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

 if(huart==&SENSOR_UART)
 {
	 if(hydroc_media_get_byte(&hydroc_sensor1,&tmp2)==HYDROC_F_OK)
	 {
	    HAL_UART_Transmit_IT(&SENSOR_UART,&tmp2,1);
	 }
	 else osSemaphoreRelease(hydroc_uart_media_semHandle);
 }
 else if(huart==&GLIDER_UART)
 {
	 if(seaglider_media_get_byte(&glider1,&tmp1)==SEAGLIDER_F_OK)
	 {
	    HAL_UART_Transmit_IT(&GLIDER_UART,&tmp1,1);
	 }
	 else osSemaphoreRelease(seaglider_uart_media_semHandle);
 }
}
#ifdef DEBUG_MSG
void vApplicationStackOverflowHook( TaskHandle_t xTask,signed char *pcTaskName )
{
	char tt[40];
    sprintf(tt,"Stack ovrfl:%s\n",pcTaskName);
	HAL_UART_Transmit(&GLIDER_UART,tt,strlen(tt),100);
}
#endif

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

  seaglider_init(&glider1,events_qHandle,seaglider_uart_q_semHandle);
  HAL_UART_Receive_IT(&GLIDER_UART,&(glider1.media_rx_byte),1);

  hydroc_init(&hydroc_sensor1,events_qHandle,hydroc_uart_q_semHandle);
  HAL_UART_Receive_IT(&SENSOR_UART,&(hydroc_sensor1.media_rx_byte),1);

  disp_proc_init(&dispatcher1,worker_cmd_qHandle,events_qHandle);
  disp_proc_loop(&dispatcher1);

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_glider_uart_tx_f */
/**
* @brief Function implementing the glider_uart_tx_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_glider_uart_tx_f */
void glider_uart_tx_f(void const * argument)
{
  /* USER CODE BEGIN glider_uart_tx_f */
  /* Infinite loop */
  uint8_t tmp1;
  glider1.media_status=SEAGLIDER_MEDIA_READY;
#ifdef DEBUG_MSG
  HAL_UART_Transmit_IT(&GLIDER_UART,"G",1);
#endif
  for(;;)
  {
	osSemaphoreWait(seaglider_uart_media_semHandle,osWaitForever);
	if(seaglider_media_get_byte(&glider1,&tmp1)==SEAGLIDER_F_OK) HAL_UART_Transmit_IT(&GLIDER_UART,&tmp1,1);
	else osSemaphoreRelease(seaglider_uart_media_semHandle);
	osDelay(1);
  }
  /* USER CODE END glider_uart_tx_f */
}

/* USER CODE BEGIN Header_worker_f */
/**
* @brief Function implementing the worker1_t thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_worker_f */
void worker_f(void const * argument)
{
  /* USER CODE BEGIN worker_f */
  /* Infinite loop */
  for(;;)
  {
		osEvent res=osMessageGet(worker_cmd_qHandle,osWaitForever);
		if(res.status==osEventMessage){
			disp_proc_execute(&dispatcher1,res.value.v);
		}
  }
  /* USER CODE END worker_f */
}

/* USER CODE BEGIN Header_hydroc_uart_tx_f */
/**
* @brief Function implementing the hydroc_uart_tx_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_hydroc_uart_tx_f */
void hydroc_uart_tx_f(void const * argument)
{
  /* USER CODE BEGIN hydroc_uart_tx_f */
  /* Infinite loop */
  uint8_t tmp1;
  hydroc_sensor1.media_status=HYDROC_MEDIA_READY;
#ifdef DEBUG_MSG
  HAL_UART_Transmit_IT(&SENSOR_UART,"S",1);
#endif
  for(;;)
  {
	osSemaphoreWait(hydroc_uart_media_semHandle,osWaitForever);
	if(hydroc_media_get_byte(&hydroc_sensor1,&tmp1)==HYDROC_F_OK) HAL_UART_Transmit_IT(&SENSOR_UART,&tmp1,1);
	else osSemaphoreRelease(hydroc_uart_media_semHandle);
	osDelay(1);
  }
  /* USER CODE END hydroc_uart_tx_f */
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
