/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "w25qxx.h"
#include "MS5837.h"
#include "string.h"
#include "stdlib.h"
#include "mbcrc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define  DEPTH_THR_LEVEL         100.0
#define  H_DEPTH                 0
#define  L_DEPTH                 1
#define  UNKNOWN_DEPTH           2

#define  SENSOR_UART             huart2
#define  GLIDER_UART             huart1


#define  READY									 1
#define  STOPPED                 2

#define  DEBUG_PROMPT_ON
//#define  DEBUG_ON
//#define  DEBUG_EXT__ON

#define  START_MESSAGE           "g_start:v1.23 UV>"
#define  PROMPT                  "UV>"

#define  SENSOR_MESSAGE_LEN      256
#define  SENSOR_RX_BUFFER_SIZE   1024
#define  SENSOR_RX_BUFFER_THR    768

#define  GLIDER_RX_BUFFER_SIZE   100
#define  GLIDER_RX_BUFFER_THR    70

#define  MAX_LPM_DATA_STRING     250
#define  LPM_DATA_AVG_NUMBER     5

#define  RESP_STRINGS_NUM        2

#define  SENSORDATA_BLOCK            0
#define  SENSORDATA_START_ADDR       4096
#define  SENSORDATA_END_ADDR         12000000

#define  MS5837_BLOCK								 1 
#define  MS5837_START_ADDR           12000001
#define  MS5837_END_ADDR             16700000

#define  NUM_OF_DATABLOCKS       2
#define  NO_FREE_SPACE           0
#define  FREE_SPACE              1

#define  SYS_INFO_ADDR           1

#define  NUM_OF_FUNCTIONS           7
#define  GLIDER_CMD_DEPTH           0
#define  GLIDER_CMD_STOP            1
#define  GLIDER_CMD_SEND_TXT_FILE   2
#define  SEND_TXT_FILE              3
#define  GLIDER_CMD_START           4
#define  SEND_INFO                  5
#define  RESET                      6
  

#define  FLASH_ERASE_FLAG        0x01
#define  FLASH_NOERASE_FLAG      0x00

#define  F_ERROR										0
#define  F_NO_ERROR									1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId sensor_tHandle;
osThreadId glider_tHandle;
osMessageQId sensor_uart_rxHandle;
osMessageQId glider_uart_rxHandle;
osMessageQId glider_ptHandle;
/* USER CODE BEGIN PV */
uint8_t avg_lpm_data_frame[MAX_LPM_DATA_STRING];
struct LPM_DATA_values LPM_DATA_avg;
uint8_t number_of_frame=0;

uint8_t system_status=STOPPED;
uint8_t sensor_rx_byte;
uint8_t glider_rx_byte;
uint8_t sensor_rx_buffer[SENSOR_RX_BUFFER_SIZE];
uint8_t glider_rx_buffer[GLIDER_RX_BUFFER_SIZE];
uint16_t sensor_rx_buffer_indx=0;
uint16_t glider_rx_buffer_indx=0;
uint16_t sensor_rx_buffer_new_string_indx=0;
uint16_t glider_rx_buffer_new_string_indx=0;
char* glider_commands_strings[]={"DEPTH","STOP","SEND_TXT_FILE","READ_FLASH","START","SEND_INFO","RESET"};
char* data_response_strings[]={"LPM_DATA","BLACK_DATA"};
char  START_ACQ_H[]="$start:ACQ_SG_2H;\n";
char  START_ACQ_L[]="$start:ACQ_SG_2L;\n";
char  STOP_ACQ[]="$stop;\n";
char  PT_MSG_PRE[]="$pt:";
char  PT_MSG_POST[]=";\n";

int ((*glider_functions[NUM_OF_FUNCTIONS]))(uint8_t* msg);

int GLIDER_CMD_DEPTH_f(uint8_t* msg);
int GLIDER_CMD_STOP_f(uint8_t* msg);
int GLIDER_CMD_SEND_TXT_FILE_f(uint8_t* msg);
int SEND_TXT_FILE_f(uint8_t* msg);
int GLIDER_CMD_START_f(uint8_t* msg);
int SEND_INFO_f(uint8_t* msg);
int GLIDER_CMD_RESET_f(uint8_t* msg);


void messages_init(void);
int  parse_message(uint8_t* msg);

void flash_save_sysinfo(uint8_t dirty_flag);

char pressure_string[50];
uint8_t  msg_buffer[SENSOR_MESSAGE_LEN];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);
void sensor_f(void const * argument);
void glider_f(void const * argument);

/* USER CODE BEGIN PFP */
uint8_t wait_sensor_response_string( uint32_t timeout,uint16_t* msg_indx);
void    save_message_to_flash(uint8_t* data);
uint8_t read_message_from_flash(uint8_t* data);
void    stop_flash_recording(void);
uint8_t check_flash_dirty_flag(void);
double  get_pressure(char* pressure);
void    clearSensorQueue(void);
void    set_L_profile(void);
void    set_H_profile(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern struct MS5837_t MS5837;


#pragma pack (push, 1)
struct LPM_DATA_values
{
 double temperature;
 uint64_t arr1[2];
 uint32_t arr2[16];
 uint16_t arr3[18]
};
#pragma pack (pop)


#pragma pack (push, 1)
struct data_block_str
{
 	uint32_t start_addr;
	uint32_t end_addr;
	uint32_t write_pointer;
	uint32_t read_pointer;
	uint8_t  nofree_space_flag;
};
#pragma pack (pop)

#pragma pack (push, 1)
struct system_info_str
{
  uint8_t  dirty_flag;
  struct   data_block_str data_blocks[NUM_OF_DATABLOCKS];
	int16_t  crc16;
};
#pragma pack (pop)
struct system_info_str running_sys_info;

struct data_block_str* sensor_data_hdr;


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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
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
  /* definition and creation of sensor_uart_rx */
  osMessageQDef(sensor_uart_rx, 20, uint16_t);
  sensor_uart_rxHandle = osMessageCreate(osMessageQ(sensor_uart_rx), NULL);

  /* definition and creation of glider_uart_rx */
  osMessageQDef(glider_uart_rx, 5, uint16_t);
  glider_uart_rxHandle = osMessageCreate(osMessageQ(glider_uart_rx), NULL);

  /* definition and creation of glider_pt */
  osMessageQDef(glider_pt, 2, char*);
  glider_ptHandle = osMessageCreate(osMessageQ(glider_pt), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of sensor_t */
  osThreadDef(sensor_t, sensor_f, osPriorityIdle, 0, 256);
  sensor_tHandle = osThreadCreate(osThread(sensor_t), NULL);

  /* definition and creation of glider_t */
  osThreadDef(glider_t, glider_f, osPriorityIdle, 0, 128);
  glider_tHandle = osThreadCreate(osThread(glider_t), NULL);

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI1_SS_Pin */
  GPIO_InitStruct.Pin = SPI1_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI1_SS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
 if(huart==&SENSOR_UART)
 {  
	  if(system_status==READY)
	  {
			sensor_rx_buffer[sensor_rx_buffer_indx]=sensor_rx_byte;
			if(sensor_rx_byte=='\n')
			{
				sensor_rx_buffer[sensor_rx_buffer_indx]=0x00;
				osMessagePut(sensor_uart_rxHandle,sensor_rx_buffer_new_string_indx,1);
				if(sensor_rx_buffer_indx>SENSOR_RX_BUFFER_THR) sensor_rx_buffer_indx=0;
				else 	sensor_rx_buffer_indx++; 
				sensor_rx_buffer_new_string_indx=sensor_rx_buffer_indx;
			}
			else sensor_rx_buffer_indx++;  
			if(sensor_rx_buffer_indx==SENSOR_RX_BUFFER_SIZE) 
			{
				sensor_rx_buffer_indx=0;
				sensor_rx_buffer_new_string_indx=sensor_rx_buffer_indx;
			}
	  }
		else
		{
		  sensor_rx_buffer_indx=0;
			sensor_rx_buffer_new_string_indx=sensor_rx_buffer_indx;
		}
		
		HAL_UART_Receive_IT(&SENSOR_UART,&sensor_rx_byte,1);
 }
 else if(huart==&GLIDER_UART)
 {  
		glider_rx_buffer[glider_rx_buffer_indx]=glider_rx_byte;
	  if(glider_rx_byte=='\r')
		{
			glider_rx_buffer[glider_rx_buffer_indx]=0x00;
			osMessagePut(glider_uart_rxHandle,glider_rx_buffer_new_string_indx,1);
			if(glider_rx_buffer_indx>GLIDER_RX_BUFFER_THR) glider_rx_buffer_indx=0;
			else 	glider_rx_buffer_indx++; 
			glider_rx_buffer_new_string_indx=glider_rx_buffer_indx;
		}
    else glider_rx_buffer_indx++;  
		if(glider_rx_buffer_indx==GLIDER_RX_BUFFER_SIZE) 
		{
			glider_rx_buffer_indx=0;
			glider_rx_buffer_new_string_indx=glider_rx_buffer_indx;
		}
		HAL_UART_Receive_IT(&GLIDER_UART,&glider_rx_byte,1);
 }
}



void clearSensorQueue(void)
{
	uint16_t new_message_indx;
   while(xQueueReceive(sensor_uart_rxHandle,&new_message_indx,1)){}
}



uint8_t wait_sensor_response_string( uint32_t timeout,uint16_t* msg_indx)
{
	uint16_t new_message_indx;
  uint32_t start_time=xTaskGetTickCount();
  char* tmp_char;
	
	while(xTaskGetTickCount()-start_time<timeout)
	{
		if(xQueueReceive(sensor_uart_rxHandle,&new_message_indx,1))
		{ 
			for(int i=0;i<RESP_STRINGS_NUM;i++) 
			{
			 tmp_char=strstr(sensor_rx_buffer+new_message_indx,data_response_strings[i]);
			 if(tmp_char) 
			 { 
				 *msg_indx=new_message_indx;
			   return i+1;
			 }
		  }
			osDelay(50);
			return 0;
		}
		osDelay(10);
  }
	return 0;
}

uint8_t flash_check_CRC(struct system_info_str* sys_info)
{
	int16_t CRC16_calc=usMBCRC16((uint8_t*)sys_info,sizeof(sys_info)-2);
	if(CRC16_calc==sys_info->crc16) return 1;
	else return 0;
}
void flash_generate_CRC(struct system_info_str* sys_info)
{

	int16_t new_crc=usMBCRC16((uint8_t*)sys_info,sizeof(sys_info)-2);
	sys_info->crc16=new_crc;
	
	#ifdef DEBUG2_ON
	
	uint8_t ttt[20];
	sprintf(ttt,"CRC_CALC:%02x,%02x\r\n",(uint8_t)(new_crc),(uint8_t)(new_crc>>8));
	HAL_UART_Transmit(&GLIDER_UART,ttt,strlen(ttt),1000);
	
	#endif
}

void flash_erase()
{

 	W25qxx_EraseChip();	
	
	running_sys_info.dirty_flag=FLASH_NOERASE_FLAG;
	running_sys_info.data_blocks[0].start_addr=SENSORDATA_START_ADDR;
	running_sys_info.data_blocks[0].end_addr=SENSORDATA_END_ADDR;
	running_sys_info.data_blocks[0].write_pointer=0;
	running_sys_info.data_blocks[0].read_pointer=0;
	running_sys_info.data_blocks[0].nofree_space_flag=FREE_SPACE;
  
	running_sys_info.data_blocks[1].start_addr=MS5837_START_ADDR;
	running_sys_info.data_blocks[1].end_addr=MS5837_END_ADDR;
	running_sys_info.data_blocks[1].write_pointer=0;
	running_sys_info.data_blocks[1].read_pointer=0;
	running_sys_info.data_blocks[1].nofree_space_flag=FREE_SPACE;
	
	#ifdef DEBUG_EXT_ON
  uint8_t ttt[30];
	sprintf(ttt,"CFG_CRC:%02x,%02x\r\n",(uint8_t)(running_sys_info.crc16),(uint8_t)(running_sys_info.crc16>>8));
	HAL_UART_Transmit(&GLIDER_UART,ttt,strlen(ttt),1000);
	sprintf(ttt,"CFG_1:%d\r\n",running_sys_info.data_blocks[0].start_addr);
	HAL_UART_Transmit(&GLIDER_UART,ttt,strlen(ttt),1000);
	sprintf(ttt,"CFG_2:%d\r\n",running_sys_info.data_blocks[0].end_addr);
	HAL_UART_Transmit(&GLIDER_UART,ttt,strlen(ttt),1000);
	sprintf(ttt,"CFG_3:%d\r\n",running_sys_info.data_blocks[0].read_pointer);
	HAL_UART_Transmit(&GLIDER_UART,ttt,strlen(ttt),1000);
  #endif
	
	flash_save_sysinfo(FLASH_NOERASE_FLAG);
	

  
}

int flash_open() 
{

	W25qxx_Init();
	uint8_t ttt[20];
	uint8_t NB;
	
	uint32_t ttID=W25qxx_ReadID();
	#ifdef DEBUG_ON
	sprintf(ttt,"MEM_ID:%02X%02X%02X%02X\r\n",(uint8_t)ttID,(uint8_t)(ttID>>8),(uint8_t)(ttID>>16),(uint8_t)(ttID>>24));
	HAL_UART_Transmit(&GLIDER_UART,ttt,strlen(ttt),1000);
	#endif
	
	for(int i=0;i<sizeof(running_sys_info);i++)
	{
		W25qxx_ReadByte(&NB,i+SYS_INFO_ADDR);
		*((uint8_t*)&running_sys_info+i)=NB;
		
		#ifdef DEBUG_EXT_ON
		sprintf(ttt,"%02x",(uint8_t)NB);
	  HAL_UART_Transmit(&GLIDER_UART,ttt,strlen(ttt),1000);
		#endif
	}
	

	#ifdef DEBUG_EXT_ON
	sprintf(ttt,"CRC_FLASH:%02x,%02x\r\n",(uint8_t)(running_sys_info.crc16),(uint8_t)(running_sys_info.crc16>>8));
	HAL_UART_Transmit(&GLIDER_UART,ttt,strlen(ttt),1000);
	sprintf(ttt,"CFG_1:%d\r\n",running_sys_info.data_blocks[0].start_addr);
	HAL_UART_Transmit(&GLIDER_UART,ttt,strlen(ttt),1000);
	sprintf(ttt,"CFG_2:%d\r\n",running_sys_info.data_blocks[0].end_addr);
	HAL_UART_Transmit(&GLIDER_UART,ttt,strlen(ttt),1000);
	sprintf(ttt,"CFG_3:%d\r\n",running_sys_info.data_blocks[0].read_pointer);
	HAL_UART_Transmit(&GLIDER_UART,ttt,strlen(ttt),1000);
	#endif
	
	if(flash_check_CRC(&running_sys_info))
	{ 
		#ifdef DEBUG_ON
		HAL_UART_Transmit(&GLIDER_UART,"CRC_OK\r\n",8,1000);
		#endif
	}
	else
	{
		#ifdef DEBUG_ON
		HAL_UART_Transmit(&GLIDER_UART,"CRC_BAD\r\n",9,1000);
		#endif
		flash_erase();
	}
	
	sensor_data_hdr=&running_sys_info.data_blocks[SENSORDATA_BLOCK];
	return F_NO_ERROR;	
}


void flash_save_sysinfo(uint8_t dirty_flag)
{   
	  running_sys_info.dirty_flag=dirty_flag;
	  flash_generate_CRC(&running_sys_info);
	  W25qxx_EraseSector(0);
	  for(int i=0;i<sizeof(running_sys_info);i++)
	  {
 		 W25qxx_WriteByte(*((uint8_t*)&running_sys_info+i),i+SYS_INFO_ADDR);
	  }
}


uint8_t flash_write_byte(struct data_block_str* flash_hdr,uint8_t* data)
{
	if(flash_hdr->nofree_space_flag==NO_FREE_SPACE) return 0;
	W25qxx_WriteByte(*data,flash_hdr->start_addr+flash_hdr->write_pointer);
	flash_hdr->write_pointer++;
	if(flash_hdr->write_pointer==flash_hdr->end_addr-flash_hdr->start_addr) {
		flash_hdr->nofree_space_flag=NO_FREE_SPACE;
	}
  return 1;
}

uint8_t flash_read_byte(struct data_block_str* flash_hdr,uint8_t* data)
{  
	
   if(flash_hdr->read_pointer==flash_hdr->write_pointer) return 0;
	 if(flash_hdr->read_pointer>=flash_hdr->end_addr) return 0;
   W25qxx_ReadByte(data,flash_hdr->start_addr+flash_hdr->read_pointer);
	 flash_hdr->read_pointer++;
	 return 1;

}
uint8_t flash_set_read_addr(struct data_block_str* flash_hdr,uint32_t new_read_addr)
{ 
	if(new_read_addr>=flash_hdr->end_addr) return 0;
  flash_hdr->read_pointer=new_read_addr;
	return 1;
}

void save_ascii_message_to_flash(struct data_block_str* flash_hdr,uint8_t* data)
{
	  if(running_sys_info.dirty_flag==FLASH_NOERASE_FLAG)
		{
 	   for(int i=0;i<strlen(data);i++)
		 {
		  if(!flash_write_byte(flash_hdr,data+i)) return ;
		 }
	  }
}


void processing_sensor_message(uint8_t* data)
{
	char* msg_hdr=strstr(data,"BLACK_DATA");
	if(msg_hdr) save_ascii_message_to_flash(sensor_data_hdr,data);
	else
	{
		msg_hdr=strstr(data,"LPM_DATA");
	  if(msg_hdr) 
		 {
			 char * pch;
			 if(number_of_frame==0) 
			 {
				 avg_lpm_data_frame[0]=0;
				 LPM_DATA_avg.temperature=0.0;
				 for(int i=0;i<2;i++)LPM_DATA_avg.arr1[i]=0;
  			 for(int i=0;i<16;i++)LPM_DATA_avg.arr2[i]=0;
  			 for(int i=0;i<18;i++)LPM_DATA_avg.arr3[i]=0;
				 
				 pch = strtok (data,",");//header
  			 strcat(avg_lpm_data_frame,pch);
				 strcat(avg_lpm_data_frame,",");
				 pch = strtok (NULL,",");//depth
				 strcat(avg_lpm_data_frame,pch);
				 strcat(avg_lpm_data_frame,",");
			 	 pch = strtok (NULL,",");//date
				 strcat(avg_lpm_data_frame,pch);
				 strcat(avg_lpm_data_frame,",");
			 	 pch = strtok (NULL,",");//time
				 strcat(avg_lpm_data_frame,pch);
				 strcat(avg_lpm_data_frame,",");
				 pch = strtok (NULL,",");//avg_images
				 strcat(avg_lpm_data_frame,pch);
				 strcat(avg_lpm_data_frame,",");
			 }
			 else
       {
				 pch = strtok (data,",");//header
				 pch = strtok (NULL,",");//depth
			 	 pch = strtok (NULL,",");//date
			 	 pch = strtok (NULL,",");//time
				 pch = strtok (NULL,",");//avg_images		 
			 }
	
			 
			 //here start connvertion from ascii
			 
			 pch = strtok (NULL,",");//temperature
			 LPM_DATA_avg.temperature+=strtod(pch, NULL);
				 
			 for(int i=0;i<2;i++){
        pch = strtok (NULL, ",");
				LPM_DATA_avg.arr1[i]+=strtoul (pch, NULL, 0);
       }
			 for(int i=0;i<16;i++){
        pch = strtok (NULL, ",");
				 LPM_DATA_avg.arr2[i]+=strtoul (pch, NULL, 0);
       }
			 for(int i=0;i<18;i++){
        pch = strtok (NULL, ",");
				 LPM_DATA_avg.arr3[i]+=strtoul (pch, NULL, 0);
       }
			 
			 number_of_frame++;
			 
			 if(number_of_frame==LPM_DATA_AVG_NUMBER)
			 {
				 double d_num=(double) LPM_DATA_AVG_NUMBER;
				 double uint64_num=(uint64_t) LPM_DATA_AVG_NUMBER;
				 double uint32_num=(uint32_t) LPM_DATA_AVG_NUMBER;
				 double uint16_num=(uint16_t) LPM_DATA_AVG_NUMBER;
				 
				 LPM_DATA_avg.temperature/=d_num;
				 for(int i=0;i<2;i++)	LPM_DATA_avg.arr1[i]/=uint64_num;
				 for(int i=0;i<16;i++) LPM_DATA_avg.arr2[i]/=uint32_num;
				 for(int i=0;i<18;i++) LPM_DATA_avg.arr3[i]/=uint16_num;
				 
				 uint8_t tmp[11];			 
				 sprintf(tmp,"%lf,",LPM_DATA_avg.temperature);
				 strcat(avg_lpm_data_frame,tmp);
				 
				 for(int i=0;i<2;i++)	
				 {	
					 sprintf(tmp,"%d,",LPM_DATA_avg.arr1[i]);
				   strcat(avg_lpm_data_frame,tmp);
				 }
				 
				 for(int i=0;i<16;i++) 
				 {
					 sprintf(tmp,"%d,",LPM_DATA_avg.arr2[i]);
				   strcat(avg_lpm_data_frame,tmp);
				 }
				 for(int i=0;i<18;i++)
				 {
					 sprintf(tmp,"%d,",LPM_DATA_avg.arr3[i]);
				   strcat(avg_lpm_data_frame,tmp);
				 }
				 
				 MS5837_read(&hi2c1);
				 sprintf(tmp,"P,%f;",MS5837.pressure);
				 strcat(avg_lpm_data_frame,tmp);
				 
		 
				 save_ascii_message_to_flash(sensor_data_hdr,avg_lpm_data_frame);
				 number_of_frame=0;
			 }		 
		 }
	}

}

uint8_t read_ascii_message_from_flash(struct data_block_str* flash_hdr,uint8_t* data,uint32_t data_size)
 {   
  for(int i=0;;i++)
  { 
		if(i>=data_size) return 0;
		if(flash_read_byte(flash_hdr,data+i))
		{
			if(data[i]==';')  
			{	
				data[i+1]=0x00;
				return 1;
			}	
	  }
		else return 0;
	}
}

double get_pressure(char* pressure)
{ 
	char * ptrEnd;
  return strtod(pressure,&ptrEnd);
}


void set_L_profile(void)
{
	HAL_UART_Transmit(&SENSOR_UART,(uint8_t*)STOP_ACQ,strlen(STOP_ACQ),1000);
	osDelay(200);
	HAL_UART_Transmit(&SENSOR_UART,(uint8_t*)STOP_ACQ,strlen(STOP_ACQ),1000);
	osDelay(200);
	HAL_UART_Transmit(&SENSOR_UART,(uint8_t*)START_ACQ_L,strlen(START_ACQ_L),1000);
	osDelay(2000);	
}

void set_H_profile(void)
{
	HAL_UART_Transmit(&SENSOR_UART,(uint8_t*)STOP_ACQ,strlen(STOP_ACQ),1000);
	osDelay(200);
	HAL_UART_Transmit(&SENSOR_UART,(uint8_t*)STOP_ACQ,strlen(STOP_ACQ),1000);
	osDelay(200);
	HAL_UART_Transmit(&SENSOR_UART,(uint8_t*)START_ACQ_H,strlen(START_ACQ_H),1000);
	osDelay(2000);	
}

void messages_init()
{
	glider_functions[GLIDER_CMD_DEPTH] = GLIDER_CMD_DEPTH_f;
	glider_functions[GLIDER_CMD_STOP] = GLIDER_CMD_STOP_f;
	glider_functions[GLIDER_CMD_SEND_TXT_FILE] = GLIDER_CMD_SEND_TXT_FILE_f;
	glider_functions[SEND_TXT_FILE] = SEND_TXT_FILE_f;
	glider_functions[GLIDER_CMD_START] =GLIDER_CMD_START_f;
	glider_functions[SEND_INFO] =SEND_INFO_f;
	glider_functions[RESET] =GLIDER_CMD_RESET_f;
}
int parse_message(uint8_t* msg)
{
 uint8_t* tmp_ptr;
 for(int i=0;i<NUM_OF_FUNCTIONS;i++)
 {  
	  tmp_ptr=0;
	  tmp_ptr=strstr(msg,glider_commands_strings[i]);
	  if(tmp_ptr) 
		{ 
			if(glider_functions[i](tmp_ptr)==F_NO_ERROR) return F_NO_ERROR;
      else return F_ERROR;		
		}
 }
 return F_ERROR;
}

int GLIDER_CMD_DEPTH_f(uint8_t* msg)
{
	#ifdef DEBUG_ON
	HAL_UART_Transmit(&GLIDER_UART,"!D",2,1000);
	#endif
	pressure_string[0]=0x00;
	strcat(pressure_string,msg+5);
	osMessagePut(glider_ptHandle,(uint32_t)pressure_string,1);
	return F_NO_ERROR;
}
int GLIDER_CMD_STOP_f(uint8_t* msg)
{
	#ifdef DEBUG_ON
	HAL_UART_Transmit(&GLIDER_UART,"!S",2,1000);
	#endif
	
	uint8_t* stub;
	uint8_t new_message_indx;
 	flash_save_sysinfo(FLASH_NOERASE_FLAG);
	system_status=STOPPED;
	
	number_of_frame=0;
  avg_lpm_data_frame[0]=0;
	LPM_DATA_avg.temperature=0.0;
	for(int i=0;i<2;i++)LPM_DATA_avg.arr1[i]=0;
  for(int i=0;i<16;i++)LPM_DATA_avg.arr2[i]=0;
  for(int i=0;i<18;i++)LPM_DATA_avg.arr3[i]=0;
	
	while(xQueueReceive(sensor_uart_rxHandle,&new_message_indx,1)){} 
		
	SEND_INFO_f(stub);
		
	HAL_UART_Transmit(&GLIDER_UART,"STOPPED",7,1000);
	HAL_UART_Transmit(&GLIDER_UART,PROMPT,strlen(PROMPT),1000);
	return F_NO_ERROR;
}

int GLIDER_CMD_START_f(uint8_t* msg)
{
	#ifdef DEBUG_ON
	HAL_UART_Transmit(&GLIDER_UART,"!G",2,1000);
	#endif
	


	uint8_t* stub;
	flash_open();	
	if(running_sys_info.dirty_flag==FLASH_ERASE_FLAG) 
	{
			HAL_UART_Transmit(&GLIDER_UART,"ERASE_FLAG\r\n",12,1000);
			flash_erase();	
      HAL_UART_Transmit(&GLIDER_UART,"ERASED\r",7,1000);
	}
	
	system_status=READY;
	SEND_INFO_f(stub);
	HAL_UART_Transmit(&GLIDER_UART,"READY",5,1000);
	HAL_UART_Transmit(&GLIDER_UART,PROMPT,strlen(PROMPT),1000);
	return F_NO_ERROR;
}
 
int GLIDER_CMD_SEND_TXT_FILE_f(uint8_t* msg)
{
	#ifdef DEBUG_ON
	HAL_UART_Transmit(&GLIDER_UART,"!T",2,1000);
	#endif
	
	//HAL_UART_Transmit(&GLIDER_UART,PROMPT,strlen(PROMPT),1000);
	flash_set_read_addr(sensor_data_hdr,0);
	while(read_ascii_message_from_flash(sensor_data_hdr,msg_buffer,SENSOR_MESSAGE_LEN))
	{
	 HAL_UART_Transmit(&GLIDER_UART,msg_buffer,strlen(msg_buffer),1000);							
	}
	if(running_sys_info.dirty_flag==FLASH_NOERASE_FLAG)flash_save_sysinfo(FLASH_ERASE_FLAG);
	HAL_UART_Transmit(&GLIDER_UART,PROMPT,strlen(PROMPT),1000);
  return F_NO_ERROR;
}

int GLIDER_CMD_RESET_f(uint8_t* msg)
{
	#ifdef DEBUG_ON
	HAL_UART_Transmit(&GLIDER_UART,"!R",2,1000);
	#endif
	

	
	HAL_UART_Transmit(&GLIDER_UART,PROMPT,strlen(PROMPT),1000);
  return F_NO_ERROR;
}

int SEND_TXT_FILE_f(uint8_t* msg)
{
	HAL_UART_Transmit(&GLIDER_UART,"FLASH READING\r\n",15,1000);
	flash_set_read_addr(sensor_data_hdr,0);
	while(flash_read_byte(sensor_data_hdr,msg_buffer))
	{
	 HAL_UART_Transmit(&GLIDER_UART,msg_buffer,1,1000);							
	}	
	HAL_UART_Transmit(&GLIDER_UART,"FLASH READED\r\n",14,1000);
  return F_NO_ERROR;
}

int SEND_INFO_f(uint8_t* msg)
{
	uint8_t ttt[30];
	
	sprintf(ttt,"START_PTR:%d\r\n",sensor_data_hdr->start_addr);
	HAL_UART_Transmit(&GLIDER_UART,ttt,strlen(ttt),1000);
	sprintf(ttt,"END_PTR:%d\r\n",sensor_data_hdr->end_addr);
	HAL_UART_Transmit(&GLIDER_UART,ttt,strlen(ttt),1000);
	sprintf(ttt,"READ_PTR:%d\r\n",sensor_data_hdr->read_pointer);
	HAL_UART_Transmit(&GLIDER_UART,ttt,strlen(ttt),1000);
	sprintf(ttt,"WRITE_PTR:%d\r\n",sensor_data_hdr->write_pointer);
	HAL_UART_Transmit(&GLIDER_UART,ttt,strlen(ttt),1000);
	sprintf(ttt,"FREE_FLAG:%d\r\n",sensor_data_hdr->nofree_space_flag);
	HAL_UART_Transmit(&GLIDER_UART,ttt,strlen(ttt),1000);
  
	return F_NO_ERROR;
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
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_sensor_f */
/**
* @brief Function implementing the sensor_t thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sensor_f */
void sensor_f(void const * argument)
 {
  /* USER CODE BEGIN sensor_f */
  /* Infinite loop */
	
	char* pressure;
	char PT_MSG[100];
	
  uint8_t resp_OK;
	uint8_t tries;
	uint16_t message_indx;
  
	uint8_t prev_depth=UNKNOWN_DEPTH;
	uint8_t cur_depth=UNKNOWN_DEPTH;
	 
 
  HAL_UART_Transmit(&SENSOR_UART,"s_start\r\n",9,1000);
	HAL_UART_Receive_IT(&SENSOR_UART,&sensor_rx_byte,1); 
		
	for(;;)
  { 
		if(system_status==READY)
		{
		 if(xQueueReceive(glider_ptHandle,&pressure,1))
	   { 
			
			PT_MSG[0]=0x00;
		  strcat(PT_MSG,PT_MSG_PRE);
			strcat(PT_MSG,pressure);
			strcat(PT_MSG,PT_MSG_POST);
		  resp_OK=0;	
			tries=0;
			
			//define profile
			if(get_pressure(pressure)>DEPTH_THR_LEVEL)
			{
				cur_depth=L_DEPTH;
				if(prev_depth==UNKNOWN_DEPTH)prev_depth=H_DEPTH;
			}
			else 
			{
				cur_depth=H_DEPTH;
				if(prev_depth==UNKNOWN_DEPTH)prev_depth=L_DEPTH;
			}
        
			if(cur_depth==H_DEPTH&&prev_depth==L_DEPTH) 
			{
				set_H_profile();
				prev_depth=cur_depth;
			}
			
			if(cur_depth==L_DEPTH&&prev_depth==H_DEPTH) 
			{
				set_L_profile();
				prev_depth=cur_depth;
			}
      //end of deine profile
			
			clearSensorQueue();
      do
			{ 
				#ifdef DEBUG_ON
				HAL_UART_Transmit(&GLIDER_UART,"?D",2,1000);	
				#endif
				
        HAL_UART_Transmit(&SENSOR_UART,PT_MSG,strlen(PT_MSG),1000);		
				
				if(wait_sensor_response_string(2500,&message_indx)>0)//check for LPM_DATA
				{
					processing_sensor_message(sensor_rx_buffer+message_indx);
					if(wait_sensor_response_string(500,&message_indx)>0)//check for BLACK_DATA
				  {
						processing_sensor_message(sensor_rx_buffer+message_indx);
				  }
					resp_OK=1;
				}
				
				tries++;
				
			}while(!resp_OK&&tries<2);
   	 }
	  }
		osDelay(1);
	}
   
  /* USER CODE END sensor_f */
}

/* USER CODE BEGIN Header_glider_f */
/**
* @brief Function implementing the glider_t thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_glider_f */
void glider_f(void const * argument)
{
  /* USER CODE BEGIN glider_f */

  /* Infinite loop */
	uint16_t message_indx;
	messages_init();
  

  HAL_UART_Receive_IT(&GLIDER_UART,&glider_rx_byte,1);
	HAL_UART_Transmit_IT(&GLIDER_UART,START_MESSAGE,strlen(START_MESSAGE));



	
	MS5837_init(&hi2c1);
	flash_open();
  system_status=READY;
	

	
  for(;;)
  {
  	  if(xQueueReceive(glider_uart_rxHandle,&message_indx,1))
		  {
			 parse_message(glider_rx_buffer+message_indx);
		  }
      osDelay(1);

  }
  /* USER CODE END glider_f */
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
