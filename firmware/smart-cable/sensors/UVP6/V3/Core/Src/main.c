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
#include "slocum_g3.h"
#include "UVP6.h"
#include "string.h"
#include "mcu_flash.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define  UVP6_UART               huart1
#define  GLIDER_UART             huart5
//#define  DEBUG

#define  UVP_DEPTH_HL_LEVEL      15.0

#define  BLOC_STARTED                      0
#define  BLOC_FINISHED                     1
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
slocum glider1;
mcu_flash data_flash;

uint32_t     dive_counter;
uint32_t     lpm_buffer_num_of_msgs=0;
lpm_data_str lpm_messages_buffer;

float   lpm_bloc_depth_start=0.0;
float   lpm_bloc_depth_size=0.0;

char filename[30];
char file_data[500];

memory_region_pointer ptr1;
memory_region_pointer ptr2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void uart_tx_f(void const * argument);

/* USER CODE BEGIN PFP */
void lpm_sum_messages();
void lpm_aggregate_messages();
void lpm_aggregate_and_send();

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

  slocum_init(&glider1);
  HAL_UART_Receive_IT(&GLIDER_UART,&(glider1.media_rx_byte),1);

  //mcu_flash_init(&data_flash,82);

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of uart_tx_t */
  osThreadDef(uart_tx_t, uart_tx_f, osPriorityNormal, 0, 512);
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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV4;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SENSOR_PWR_CTRL_GPIO_Port, SENSOR_PWR_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SENSOR_PWR_CTRL_Pin */
  GPIO_InitStruct.Pin = SENSOR_PWR_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SENSOR_PWR_CTRL_GPIO_Port, &GPIO_InitStruct);

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
	 slocum_media_process_byte(&glider1,glider1.media_rx_byte);
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
	 if(slocum_media_get_byte(&glider1,&tmp1)==SLOCUM_F_OK)
	 {
	    HAL_UART_Transmit_IT(&GLIDER_UART,&tmp1,1);
	 }
 }
}

void lpm_sum_messages()
{
  lpm_messages_buffer.temperature+=uvp6_sensor1.lpm_data.temperature;
  lpm_messages_buffer.pressure+=uvp6_sensor1.lpm_data.pressure;
  lpm_messages_buffer.number_of_images+=uvp6_sensor1.lpm_data.number_of_images;
  for(int y=0;y<UVP6_NUM_OF_CATEGORIES;y++){
	lpm_messages_buffer.data[y]+=uvp6_sensor1.lpm_data.data[y];
	lpm_messages_buffer.grey_levels[y]+=(uvp6_sensor1.lpm_data.grey_levels[y]*uvp6_sensor1.lpm_data.data[y]);
  }
}

void lpm_aggregate_messages()
{
   lpm_messages_buffer.temperature=lpm_messages_buffer.temperature/(float)(lpm_buffer_num_of_msgs);
   lpm_messages_buffer.pressure=lpm_messages_buffer.pressure/(float)(lpm_buffer_num_of_msgs);
   for(int y=0;y<UVP6_NUM_OF_CATEGORIES;y++){
	   if(lpm_messages_buffer.data[y]>0){
		   lpm_messages_buffer.grey_levels[y]=(lpm_messages_buffer.grey_levels[y]/lpm_messages_buffer.data[y]);
	   }
   }
   lpm_buffer_num_of_msgs=0;
}


void lpm_aggregate_and_send()
{
  	lpm_aggregate_messages();
	sprintf(file_data,"LPM_DATA,%f,%s,%s,%u,%f,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u;\r\n",
						  lpm_messages_buffer.pressure,
						  lpm_messages_buffer.date,
						  lpm_messages_buffer.time,
						  lpm_messages_buffer.number_of_images,
						  lpm_messages_buffer.temperature,
						  lpm_messages_buffer.data[0],
						  lpm_messages_buffer.data[1],
						  lpm_messages_buffer.data[2],
						  lpm_messages_buffer.data[3],
						  lpm_messages_buffer.data[4],
						  lpm_messages_buffer.data[5],
						  lpm_messages_buffer.data[6],
						  lpm_messages_buffer.data[7],
						  lpm_messages_buffer.data[8],
						  lpm_messages_buffer.data[9],
						  lpm_messages_buffer.data[10],
						  lpm_messages_buffer.data[11],
						  lpm_messages_buffer.data[12],
						  lpm_messages_buffer.data[13],
						  lpm_messages_buffer.data[14],
						  lpm_messages_buffer.data[15],
						  lpm_messages_buffer.data[16],
						  lpm_messages_buffer.data[17],
						  lpm_messages_buffer.grey_levels[0],
						  lpm_messages_buffer.grey_levels[1],
						  lpm_messages_buffer.grey_levels[2],
						  lpm_messages_buffer.grey_levels[3],
						  lpm_messages_buffer.grey_levels[4],
						  lpm_messages_buffer.grey_levels[5],
						  lpm_messages_buffer.grey_levels[6],
						  lpm_messages_buffer.grey_levels[7],
						  lpm_messages_buffer.grey_levels[8],
						  lpm_messages_buffer.grey_levels[9],
						  lpm_messages_buffer.grey_levels[10],
						  lpm_messages_buffer.grey_levels[11],
						  lpm_messages_buffer.grey_levels[12],
						  lpm_messages_buffer.grey_levels[13],
						  lpm_messages_buffer.grey_levels[14],
						  lpm_messages_buffer.grey_levels[15],
						  lpm_messages_buffer.grey_levels[16],
						  lpm_messages_buffer.grey_levels[17]
    );
	ptr2.start_addr=file_data;
	ptr2.size=strlen(file_data);
	osDelay(500);
	slocum_send_cmd(&glider1,SLOCUM_CMD_WRITE_FILE_DATA,&ptr2);


	if(lpm_bloc_depth_start>1000.0) lpm_bloc_depth_size=20.0;
	else if(lpm_bloc_depth_start>500.0) lpm_bloc_depth_size=20.0;
	else if(lpm_bloc_depth_start>100.0) lpm_bloc_depth_size=10.0;
	else if(lpm_bloc_depth_start>2.0) lpm_bloc_depth_size=5.0;
	else lpm_bloc_depth_size=0.0;

	lpm_bloc_depth_start=glider1.last_depth;
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
  //mcu_flash_open(&data_flash);

  uint8_t event_id;

  dive_counter=0;
  uvp6_sensor1.profile=UVP_PROFILE_L;

  glider1.fsm_status=GLIDER_FSM_CLK_UNSYNCED;
  uvp6_sensor1.status=UVP6_IDLE;

  uvp6_power_off();
  osDelay(200);
  uvp6_power_on();

  uvp6_send_cmd(&uvp6_sensor1,UVP6_CMD_STOP_ACQ,NULL);

  uint8_t bloc_status=BLOC_STARTED;


  for(;;)
  {
	 //glider task
	 if(slocum_get_event(&glider1,&event_id)==SLOCUM_F_OK)
	 {
		switch(event_id)
		{
		 case SLOCUM_EVNT_DEPTH_RCVD:
			 if(glider1.fsm_status==GLIDER_FSM_ACTIVE){
				 switch(uvp6_sensor1.status){
				  case UVP6_ACQ_L_STARTED:
					  if(glider1.last_depth>UVP_DEPTH_HL_LEVEL&&glider1.prev_depth>UVP_DEPTH_HL_LEVEL){
						uvp6_sensor1.profile=UVP_PROFILE_H;
						uvp6_send_cmd(&uvp6_sensor1,UVP6_CMD_STOP_ACQ,NULL);
						osDelay(1000);
						uvp6_send_cmd(&uvp6_sensor1,UVP6_CMD_START_H_ACQ,NULL);
						uvp6_sensor1.status=UVP6_ACQ_H_STARTED;
					  }
				  break;
				  case UVP6_ACQ_H_STARTED:
					  if(glider1.last_depth<UVP_DEPTH_HL_LEVEL&&glider1.prev_depth<UVP_DEPTH_HL_LEVEL){
						uvp6_sensor1.profile=UVP_PROFILE_L;
						uvp6_send_cmd(&uvp6_sensor1,UVP6_CMD_STOP_ACQ,NULL);
					    osDelay(1000);
						uvp6_send_cmd(&uvp6_sensor1,UVP6_CMD_START_L_ACQ,NULL);
						uvp6_sensor1.status=UVP6_ACQ_L_STARTED;
					  }
				  break;
				 };
			 }
			 else if(glider1.fsm_status==GLIDER_FSM_START_PROFILE){
				 if(glider1.last_depth>UVP_DEPTH_HL_LEVEL){
				   uvp6_send_cmd(&uvp6_sensor1,UVP6_CMD_STOP_ACQ,NULL);
				   osDelay(1000);
				   uvp6_send_cmd(&uvp6_sensor1,UVP6_CMD_START_H_ACQ,NULL);
				   uvp6_sensor1.status=UVP6_ACQ_H_STARTED;
				   glider1.fsm_status=GLIDER_FSM_ACTIVE;
				 }
				 else if(glider1.last_depth<=UVP_DEPTH_HL_LEVEL){
				   uvp6_send_cmd(&uvp6_sensor1,UVP6_CMD_STOP_ACQ,NULL);
				   osDelay(1000);
				   uvp6_send_cmd(&uvp6_sensor1,UVP6_CMD_START_L_ACQ,NULL);
				   uvp6_sensor1.status=UVP6_ACQ_L_STARTED;
				   glider1.fsm_status=GLIDER_FSM_ACTIVE;
				 }
			 }
			 //slocum_send_cmd(&glider1,SLOCUM_CMD_PROMPT,NULL);
         break;
		 case SLOCUM_EVNT_STATUS_RCVD:
			 if(glider1.behavior_state==GLIDER_BEHAVIOR_DV
					 ||glider1.behavior_state==GLIDER_BEHAVIOR_SAC
					 ||glider1.behavior_state==GLIDER_BEHAVIOR_NONE){
			   if(glider1.fsm_status==GLIDER_FSM_ACTIVE) {
				   uvp6_send_cmd(&uvp6_sensor1,UVP6_CMD_STOP_ACQ,NULL);
				   sprintf(filename,"ef.tmp");
				   ptr1.start_addr=filename;
				   ptr1.size=strlen(filename);
				   slocum_send_cmd(&glider1,SLOCUM_CMD_OPEN_FILE_W,&ptr1);
				   bloc_status=BLOC_FINISHED;
				   lpm_aggregate_and_send();
			   }
			   glider1.fsm_status=GLIDER_FSM_WAIT_DIVE;
			 }
			 else if(glider1.behavior_state==GLIDER_BEHAVIOR_CL){
			   if(glider1.fsm_status==GLIDER_FSM_ACTIVE) {
				   uvp6_send_cmd(&uvp6_sensor1,UVP6_CMD_STOP_ACQ,NULL);
				   sprintf(filename,"ef.tmp");
				   ptr1.start_addr=filename;
				   ptr1.size=strlen(filename);
				   slocum_send_cmd(&glider1,SLOCUM_CMD_OPEN_FILE_W,&ptr1);
				   bloc_status=BLOC_FINISHED;
				   lpm_aggregate_and_send();
			   }
			   glider1.fsm_status=GLIDER_FSM_WAIT_CLIMB;
			 }
			 else if(glider1.behavior_state==GLIDER_BEHAVIOR_NTR){
			   if(glider1.fsm_status==GLIDER_FSM_WAIT_DIVE){
				   glider1.fsm_status=GLIDER_FSM_START_DIVE;
			   }
			   else if(glider1.fsm_status==GLIDER_FSM_WAIT_CLIMB){
				   glider1.fsm_status=GLIDER_FSM_START_CLIMB;
			   }
			 }
	     break;
		 case SLOCUM_EVNT_CLK_RCVD:

			   if(glider1.fsm_status==GLIDER_FSM_START_DIVE){
				   dive_counter++;
				   dive_counter%=999;
				   if(uvp6_sensor1.profile==UVP_PROFILE_H) sprintf(filename,"%s%u.uv6",glider1.mission_id+2,dive_counter);
				   else if(uvp6_sensor1.profile==UVP_PROFILE_L) sprintf(filename,"%s%u.uv6",glider1.mission_id+2,dive_counter);
				   ptr1.start_addr=filename;
				   ptr1.size=strlen(filename);
				   osDelay(500);
				   slocum_send_cmd(&glider1,SLOCUM_CMD_OPEN_FILE_W,&ptr1);
				   glider1.fsm_status=GLIDER_FSM_START_PROFILE;
				   glider1.dicl_status=GLIDER_DICL_DIVE;
				   lpm_bloc_depth_start=glider1.last_depth;
			   }
			   else if(glider1.fsm_status==GLIDER_FSM_START_CLIMB){
				   dive_counter++;
				   dive_counter%=999;
				   if(uvp6_sensor1.profile==UVP_PROFILE_H) sprintf(filename,"%s%u.uv6",glider1.mission_id+2,dive_counter);
				   else if(uvp6_sensor1.profile==UVP_PROFILE_L) sprintf(filename,"%s%u.uv6",glider1.mission_id+2,dive_counter);
				   ptr1.start_addr=filename;
				   ptr1.size=strlen(filename);
				   osDelay(500);
				   slocum_send_cmd(&glider1,SLOCUM_CMD_OPEN_FILE_W,&ptr1);
				   glider1.fsm_status=GLIDER_FSM_START_PROFILE;
				   glider1.dicl_status=GLIDER_DICL_CLIMB;
				   lpm_bloc_depth_start=glider1.last_depth;
			   }
	     break;
		}
	 }

	 //UVP6 tasks

	 if(uvp6_get_event(&uvp6_sensor1,&event_id)==UVP6_F_OK)
	 {
        switch(event_id)
        {
         case UVP6_EVNT_BOOTED:
        	if(uvp6_sensor1.status==UVP6_IDLE){
        		uvp6_send_cmd(&uvp6_sensor1,UVP6_CMD_STOP_ACQ,NULL);
        		uvp6_sensor1.status=UVP6_STARTED;
        	}
         break;
         case UVP6_EVNT_LPM_DATA_RCVD:
          if(glider1.fsm_status==GLIDER_FSM_ACTIVE){
        	lpm_buffer_num_of_msgs++;
        	if(lpm_buffer_num_of_msgs==1){
               memcpy(&lpm_messages_buffer,&uvp6_sensor1.lpm_data,sizeof(uvp6_sensor1.lpm_data));
               for(int y=0;y<UVP6_NUM_OF_CATEGORIES;y++){
             	lpm_messages_buffer.grey_levels[y]=(lpm_messages_buffer.grey_levels[y]*lpm_messages_buffer.data[y]);
               }
        	}
        	else lpm_sum_messages();

            switch(glider1.dicl_status){
              case GLIDER_DICL_CLIMB:
            	  if(glider1.last_depth<(lpm_bloc_depth_start-lpm_bloc_depth_size)) bloc_status=BLOC_FINISHED;
              break;
              case GLIDER_DICL_DIVE:
            	  if(glider1.last_depth>(lpm_bloc_depth_start+lpm_bloc_depth_size)) bloc_status=BLOC_FINISHED;
              break;
            };

            if(bloc_status==BLOC_FINISHED){
                lpm_aggregate_and_send();
            	bloc_status=BLOC_STARTED;
            }
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
  glider1.media_status=SLOCUM_MEDIA_READY;
#ifdef DEBUG
  HAL_UART_Transmit(&UVP6_UART,"U",1,100);
  HAL_UART_Transmit(&GLIDER_UART,"G",1,100);
#endif

  for(;;)
  {
       if(GLIDER_UART.gState!=HAL_UART_STATE_BUSY_TX)
       {
		if(slocum_media_get_byte(&glider1,&tmp1)==SLOCUM_F_OK)
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
