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
osThreadId uart_tx_tHandle;
osTimerId timer1Handle;
osSemaphoreId glider_uart_semHandle;
osSemaphoreId sensor_uart_semHandle;
/* USER CODE BEGIN PV */

typedef enum{
	FSM_IDLE,
	FSM_INIT,
	FSM_MAIN_ALG
}fsm_t;

typedef enum{
	CLK_UNSYNC,
	CLK_SYNCED
}clk_sync_t;

seaglider glider1;
hydroc hydroc_sensor1;
sd_storage_t microsd_storage;
extern settings_str run_cfg;

uint32_t tick1,tick2;

fsm_t fsm_status;
clk_sync_t clock_sync_status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void uart_tx_f(void const * argument);
void timer1_cb(void const * argument);

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
  fsm_status=FSM_IDLE;
  clock_sync_status=CLK_UNSYNC;
  seaglider_init(&glider1);
  hydroc_init(&hydroc_sensor1);


  HAL_UART_Receive_IT(&SENSOR_UART,&(hydroc_sensor1.media_rx_byte),1);
  HAL_UART_Receive_IT(&GLIDER_UART,&(glider1.media_rx_byte),1);


  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of glider_uart_sem */
  osSemaphoreDef(glider_uart_sem);
  glider_uart_semHandle = osSemaphoreCreate(osSemaphore(glider_uart_sem), 1);

  /* definition and creation of sensor_uart_sem */
  osSemaphoreDef(sensor_uart_sem);
  sensor_uart_semHandle = osSemaphoreCreate(osSemaphore(sensor_uart_sem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of timer1 */
  osTimerDef(timer1, timer1_cb);
  timer1Handle = osTimerCreate(osTimer(timer1), osTimerOnce, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of uart_tx_t */
  osThreadDef(uart_tx_t, uart_tx_f, osPriorityNormal, 0, 256);
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
  uint8_t event_id;
  char tmp_str[30];

  memory_region_pointer ptr1;
  uint8_t tx_buff[TX_BUFF_SIZE];
  uint32_t byteswritten;
  uint32_t bytesreaded;
  FIL  data_file;
  FIL  err_file;
  char tmp_filename[FILENAME_LEN+2];
  char log_msg[512];

  osDelay(200);
  #ifdef DEBUG_MSG
   HAL_UART_Transmit(&GLIDER_UART,"Glider port start\r",18,100);
   HAL_UART_Transmit(&SENSOR_UART,"Sensor port start\r",18,100);
  #endif


  MX_FATFS_Init();
  sd_storage_link_ss(&microsd_storage,0,SS_SD1_Pin,GPIOA);


  while(sd_storage_init(&microsd_storage)!=F_OK){
		osDelay(500);
        #ifdef DEBUG_MSG
			HAL_UART_Transmit(&SENSOR_UART,"SD init error\r",14,100);
        #endif
  }
  run_cfg.disk_id=microsd_storage.active_disk_indx;
  if(read_settings()==F_ERR){
	  set_default_settings();
	  save_settings();
  }
  #ifdef DEBUG_MSG
	HAL_UART_Transmit(&SENSOR_UART,"Init finished\r",14,100);
  #endif

  hydroc_sensor1.errors.P_in=run_cfg.sensor_errors.P_in;
  hydroc_sensor1.errors.rH_gas=run_cfg.sensor_errors.rH_gas;
  hydroc_sensor1.errors.T_control=run_cfg.sensor_errors.T_control;
  hydroc_sensor1.errors.P_pump=run_cfg.sensor_errors.P_pump;

  fsm_status=FSM_MAIN_ALG;

  seaglider_send_cmd(&glider1,SEAGLIDER_CMD_PROMPT,NULL);
  for(;;)
  {
		 //glider task
		 if(seaglider_get_event(&glider1,&event_id)==SEAGLIDER_F_OK)
		 {
			switch(event_id)
			{
			 case SEAGLIDER_EVNT_CLEAR_RCVD:
				  hydroc_send_cmd(&hydroc_sensor1,HYDROC_CMD_ENTER_CFG,NULL);
				  osDelay(1000);
				  hydroc_send_cmd(&hydroc_sensor1,HYDROC_CMD_CLEAR,NULL);
				  osDelay(1000);
				  hydroc_send_cmd(&hydroc_sensor1,HYDROC_CMD_EXIT_CFG,NULL);
				  seaglider_send_cmd(&glider1,SEAGLIDER_CMD_PROMPT,NULL);
			 break;
			 case SEAGLIDER_EVNT_POFF_RCVD:
				  hydroc_send_cmd(&hydroc_sensor1,HYDROC_CMD_ENTER_CFG,NULL);
				  osDelay(1000);
				  hydroc_send_cmd(&hydroc_sensor1,HYDROC_CMD_DISABLE_PUMP,NULL);
				  osDelay(1000);
				  hydroc_send_cmd(&hydroc_sensor1,HYDROC_CMD_EXIT_CFG,NULL);
				  seaglider_send_cmd(&glider1,SEAGLIDER_CMD_PROMPT,NULL);
			 break;
			 case SEAGLIDER_EVNT_DEPTH_RCVD:
				  seaglider_send_cmd(&glider1,SEAGLIDER_CMD_PROMPT,NULL);
	         break;
			 case SEAGLIDER_EVNT_STOP_RCVD:

				  if(glider1.dive_status==glider1.param_y){
					hydroc_send_cmd(&hydroc_sensor1,HYDROC_CMD_ENTER_CFG,NULL);
					osDelay(1000);
					hydroc_send_cmd(&hydroc_sensor1,HYDROC_CMD_SET_ZERO_MODE,NULL);
					osDelay(1000);
				    if(glider1.stop_trigger==SEAGLIDER_STOP_WAIT){
					  hydroc_send_cmd(&hydroc_sensor1,HYDROC_CMD_DISABLE_PUMP,NULL);
					  osDelay(1000);
				    }
					hydroc_send_cmd(&hydroc_sensor1,HYDROC_CMD_EXIT_CFG,NULL);
					osDelay(1000);
					glider1.stop_trigger=SEAGLIDER_STOP_TRIGGERED;
				  }

				  f_close(&data_file);
				  run_cfg.sensor_errors.P_in=hydroc_sensor1.errors.P_in;
				  run_cfg.sensor_errors.rH_gas=hydroc_sensor1.errors.rH_gas;
				  run_cfg.sensor_errors.T_control=hydroc_sensor1.errors.T_control;
				  run_cfg.sensor_errors.P_pump=hydroc_sensor1.errors.P_pump;
				  //f_close(&err_file);
				  save_settings();

				  //hydroc_send_cmd(&hydroc_sensor1,HYDROC_CMD_TEST,NULL);
				  hydroc_sensor1.status=HYDROC_IDLE;
				  seaglider_send_cmd(&glider1,SEAGLIDER_CMD_PROMPT,NULL);
			 break;
			 case SEAGLIDER_EVNT_START_RCVD:
				  hydroc_sensor1.data_profile_id=glider1.param_z;

				  if(glider1.dive_status==SEAGLIDER_STATUS_DIVE){
					  run_cfg.last_file_index++;
					  sprintf(tmp_filename,"%u:%s%u%s",run_cfg.disk_id,DATA_FILE_PREFIX,run_cfg.last_file_index,DATA_FILE_EXTENSION);
					  f_open(&data_file,tmp_filename,FA_CREATE_ALWAYS|FA_WRITE);
					  //sprintf(tmp_filename,"%u:%s%u%s",run_cfg.disk_id,ERR_FILE_PREFIX,run_cfg.last_file_index,ERR_FILE_EXTENSION);
					  //f_open(&err_file,tmp_filename,FA_CREATE_ALWAYS|FA_WRITE);

					  hydroc_sensor1.errors.P_in=0;
					  hydroc_sensor1.errors.rH_gas=0;
					  hydroc_sensor1.errors.T_control=0;
					  hydroc_sensor1.errors.P_pump=0;
				  }
				  else if(glider1.dive_status==SEAGLIDER_STATUS_CLIMB){

					  sprintf(tmp_filename,"%u:%s%u%s",run_cfg.disk_id,DATA_FILE_PREFIX,run_cfg.last_file_index,DATA_FILE_EXTENSION);
					  f_open(&data_file,tmp_filename,FA_OPEN_APPEND|FA_WRITE);
					  //sprintf(tmp_filename,"%u:%s%u%s",run_cfg.disk_id,ERR_FILE_PREFIX,run_cfg.last_file_index,ERR_FILE_EXTENSION);
					  //f_open(&err_file,tmp_filename,FA_OPEN_APPEND|FA_WRITE);
				  }

				  hydroc_sensor1.status=HYDROC_WAIT_DATA;

				  if(glider1.start_trigger==SEAGLIDER_START_WAIT){
					hydroc_send_cmd(&hydroc_sensor1,HYDROC_CMD_ENTER_CFG,NULL);
					osDelay(1000);
					hydroc_send_cmd(&hydroc_sensor1,HYDROC_CMD_ENABLE_PUMP,NULL);
					osDelay(1000);
					hydroc_send_cmd(&hydroc_sensor1,HYDROC_CMD_EXIT_CFG,NULL);
					glider1.start_trigger=SEAGLIDER_START_TRIGGERED;
				  }
				  //hydroc_send_cmd(&hydroc_sensor1,HYDROC_CMD_TEST,NULL);
				  seaglider_send_cmd(&glider1,SEAGLIDER_CMD_PROMPT,NULL);
			 break;
			 case SEAGLIDER_EVNT_CLOCK_RCVD:
				  seaglider_send_cmd(&glider1,SEAGLIDER_CMD_PROMPT,NULL);
                  if(clock_sync_status==CLK_UNSYNC){
					  xSemaphoreTake(sensor_uart_semHandle,portMAX_DELAY);
					  clock_sync_status=CLK_SYNCED;
					  hydroc_send_cmd(&hydroc_sensor1,HYDROC_CMD_ENTER_CFG,tmp_str);
					  osDelay(1000);
					  memcpy(tmp_str,glider1.date,8);
					  memcpy(tmp_str+8,",0,",3);
					  memcpy(tmp_str+11,glider1.time,8);
					  memcpy(tmp_str+19,",0",2);
					  tmp_str[21]=0x00;
					  hydroc_send_cmd(&hydroc_sensor1,HYDROC_CMD_SET_REAL_TIME,tmp_str);
					  osDelay(1000);
					  hydroc_send_cmd(&hydroc_sensor1,HYDROC_CMD_EXIT_CFG,tmp_str);
					  xSemaphoreGive(sensor_uart_semHandle);
                  }

			 break;
			 case SEAGLIDER_EVNT_WAKEUP_RCVD:
				  seaglider_send_cmd(&glider1,SEAGLIDER_CMD_PROMPT,NULL);
			 break;

			 case SEAGLIDER_EVNT_TEST_RCVD:
				  seaglider_send_cmd(&glider1,SEAGLIDER_CMD_PROMPT,NULL);
			 break;
			 case SEAGLIDER_EVNT_SEND_TXT_FILE_RCVD:
				  sprintf(tmp_filename,"%s%u%s",DATA_FILE_PREFIX,run_cfg.last_file_index,DATA_FILE_EXTENSION);
				  if(f_open(&data_file,tmp_filename,FA_READ)==FR_OK){
					ptr1.start_addr=tmp_filename;
					ptr1.size=strlen(tmp_filename);
					seaglider_send_cmd(&glider1,SEAGLIDER_CMD_SEND_DATA,&ptr1);
					do{
					   if(f_read(&data_file,tx_buff,50,&bytesreaded)==FR_OK){
					    ptr1.start_addr=tx_buff;
					    ptr1.size=bytesreaded;
					    seaglider_send_cmd(&glider1,SEAGLIDER_CMD_SEND_DATA,&ptr1);
					   }
					}while(bytesreaded==50);
				  }
				  /* send error file
				  sprintf(tmp_filename,"%s%u%s",ERR_FILE_PREFIX,run_cfg.last_file_index,ERR_FILE_EXTENSION);
				  if(f_open(&data_file,tmp_filename,FA_READ)==FR_OK){
					ptr1.start_addr=tmp_filename;
					ptr1.size=strlen(tmp_filename);
					seaglider_send_cmd(&glider1,SEAGLIDER_CMD_SEND_DATA,&ptr1);
					do{
					   if(f_read(&data_file,tx_buff,50,&bytesreaded)==FR_OK){
					    ptr1.start_addr=tx_buff;
					    ptr1.size=bytesreaded;
					    seaglider_send_cmd(&glider1,SEAGLIDER_CMD_SEND_DATA,&ptr1);
					   }
					}while(bytesreaded==TX_BUFF_SIZE);
				  }
				  */
				  seaglider_send_cmd(&glider1,SEAGLIDER_CMD_PROMPT,NULL);
			 break;
			 case SEAGLIDER_EVNT_ERRORS_RCVD:
				  sprintf(tx_buff,"P_in:%u,rH_gas:%u,T_control:%u,P_pump:%u\r",
						   hydroc_sensor1.errors.P_in,
						   hydroc_sensor1.errors.rH_gas,
						   hydroc_sensor1.errors.T_control,
						   hydroc_sensor1.errors.P_pump
						   );
  			      ptr1.start_addr=tx_buff;
				  ptr1.size=strlen(tx_buff);
				  seaglider_send_cmd(&glider1,SEAGLIDER_CMD_SEND_DATA,&ptr1);
				  seaglider_send_cmd(&glider1,SEAGLIDER_CMD_PROMPT,NULL);
			 break;

			}
		 }

		 //sensor tasks

		 if(hydroc_get_event(&hydroc_sensor1,&event_id)==HYDROC_F_OK)
		 {
	        switch(event_id)
	        {
	         case HYDROC_EVNT_CODS4:
	             log_msg[0]=0x00;
	             switch(hydroc_sensor1.data_profile_id){
	              case HYDROC_PROFILE_0:

                  break;
	              case HYDROC_PROFILE_1:
                   sprintf(log_msg,"DS4:%s,%s,%s\r"
						 ,hydroc_sensor1.ds4.pCO2_corr
						 ,hydroc_sensor1.ds4.P_IN
						 ,hydroc_sensor1.ds4.runtime);
                  break;
	              case HYDROC_PROFILE_2:
                   sprintf(log_msg,"DS4:%s,%s,%s,%s,%s\r"
						 ,hydroc_sensor1.ds4.pCO2_corr
						 ,hydroc_sensor1.ds4.P_IN
						 ,hydroc_sensor1.ds4.runtime
						 ,hydroc_sensor1.ds4.signal_raw
						 ,hydroc_sensor1.ds4.signal_ref);
                  break;
	              case HYDROC_PROFILE_3:
                   sprintf(log_msg,"DS4:%s,%s,%s,%s,%s,%s,%s,%s,%s\r"
						 ,hydroc_sensor1.ds4.P_IN
						 ,hydroc_sensor1.ds4.pCO2_corr
						 ,hydroc_sensor1.ds4.T_sensor
						 ,hydroc_sensor1.ds4.xCO2_corr
						 ,hydroc_sensor1.ds4.runtime
                		 ,hydroc_sensor1.ds4.date
                		 ,hydroc_sensor1.ds4.time
						 ,hydroc_sensor1.ds4.signal_raw
						 ,hydroc_sensor1.ds4.signal_ref);
                  break;
	        	 };
	        	 f_write(&data_file,log_msg,strlen(log_msg),(UINT*)&byteswritten);

	        	 uint32_t P_IN=strtol(hydroc_sensor1.ds4.P_IN,NULL,10);
	        	 uint32_t pump_pwr=strtol(hydroc_sensor1.ds4.pump_pwr,NULL,10);

	        	 if(P_IN>110000) hydroc_sensor1.errors.P_in++;
	       		 if(pump_pwr>1600 || pump_pwr<300)hydroc_sensor1.errors.P_pump++;

	        	 /* save errors
	        	 if(P_IN>110000 || pump_pwr>1600 || pump_pwr<300){
	              sprintf(log_msg,"DS4:%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\r"
							 ,hydroc_sensor1.ds4.date
							 ,hydroc_sensor1.ds4.time
							 ,hydroc_sensor1.ds4.P_IN
							 ,hydroc_sensor1.ds4.pCO2_corr
							 ,hydroc_sensor1.ds4.T_sensor
							 ,hydroc_sensor1.ds4.xCO2_corr
							 ,hydroc_sensor1.ds4.runtime
	                		 ,hydroc_sensor1.ds4.date
	                		 ,hydroc_sensor1.ds4.time
							 ,hydroc_sensor1.ds4.signal_raw
							 ,hydroc_sensor1.ds4.signal_ref);
	        	  f_write(&err_file,log_msg,strlen(log_msg),&byteswritten);
	        	 }
	        	 */

	        	 //seaglider_send_cmd(&glider1,SEAGLIDER_CMD_PROMPT,NULL);
	         break;
	         case HYDROC_EVNT_COTS1:
	        	 log_msg[0]=0x00;
	             switch(hydroc_sensor1.data_profile_id){
	              case HYDROC_PROFILE_0:

                 break;
	              case HYDROC_PROFILE_1:
                  sprintf(log_msg,"TS1:%s,%s\r"
					 ,hydroc_sensor1.ts1.T_gas
					 ,hydroc_sensor1.ts1.rH_gas
                      );
                 break;
	              case HYDROC_PROFILE_2:
                  sprintf(log_msg,"TS1:%s,%s\r"
					 ,hydroc_sensor1.ts1.T_gas
					 ,hydroc_sensor1.ts1.rH_gas
                      );
                 break;
	              case HYDROC_PROFILE_3:
                  sprintf(log_msg,"TS1:%s,%s,%s\r"
					 ,hydroc_sensor1.ts1.rH_gas
					 ,hydroc_sensor1.ts1.T_gas
					 ,hydroc_sensor1.ts1.T_control
                     );
                 break;
	             };
	        	 f_write(&data_file,log_msg,strlen(log_msg),(UINT*)&byteswritten);

	        	 uint32_t rH_gas=strtol(hydroc_sensor1.ts1.rH_gas,NULL,10);
	        	 uint32_t T_control=strtol(hydroc_sensor1.ts1.T_control,NULL,10);

	        	 if(rH_gas>85000) hydroc_sensor1.errors.rH_gas++;
	       		 if(T_control>29000 || T_control<27000)hydroc_sensor1.errors.T_control++;

	        	 /* save errors
	        	 if(rH_gas>85000 || T_control>36500 || T_control<34500){
	              sprintf(log_msg,"TS1:%s,%s,%s,%s,%s\r"
		   				 ,hydroc_sensor1.ts1.date
		   				 ,hydroc_sensor1.ts1.time
	   					 ,hydroc_sensor1.ts1.rH_gas
	   					 ,hydroc_sensor1.ts1.T_gas
	   					 ,hydroc_sensor1.ts1.T_control
	                     );
	        	  f_write(&err_file,log_msg,strlen(log_msg),&byteswritten);
	        	 }
	        	 */

	        	 //seaglider_send_cmd(&glider1,SEAGLIDER_CMD_PROMPT,NULL);
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
  glider1.media_status=SEAGLIDER_MEDIA_READY;
  hydroc_sensor1.media_status=HYDROC_MEDIA_READY;

  for(;;)
  {
   if(fsm_status==FSM_MAIN_ALG){
	   if(GLIDER_UART.gState!=HAL_UART_STATE_BUSY_TX)
	   {
		if(seaglider_media_get_byte(&glider1,&tmp1)==SEAGLIDER_F_OK)
		{
		   HAL_UART_Transmit_IT(&GLIDER_UART,&tmp1,1);
		}
	   }
	   if(SENSOR_UART.gState!=HAL_UART_STATE_BUSY_TX)
	   {

		if(hydroc_media_get_byte(&hydroc_sensor1,&tmp2)==HYDROC_F_OK)
		{
		   HAL_UART_Transmit_IT(&SENSOR_UART,&tmp2,1);
		}

	   }
   }
   osDelay(1);
  }
  /* USER CODE END uart_tx_f */
}

/* timer1_cb function */
void timer1_cb(void const * argument)
{
  /* USER CODE BEGIN timer1_cb */

  /* USER CODE END timer1_cb */
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
