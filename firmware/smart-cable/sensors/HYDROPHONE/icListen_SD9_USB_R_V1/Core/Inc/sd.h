#ifndef SD_H_
#define SD_H_
//--------------------------------------------------
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "main.h"
#include "fatfs.h"

//--------------------------------------------------
//--------------------------------------------------
#endif /* SD_H_ */


void    SD_PowerOn(void);
uint8_t sd_ini(void);

uint8_t SPIx_WriteRead(uint8_t Byte);
void    SPI_SendByte(uint8_t bt);
uint8_t SPI_ReceiveByte(void);
void    SPI_Release(void);
uint8_t SD_Read_Block (uint8_t *buff, uint32_t lba);
uint8_t SD_Write_Block (uint8_t *buff, uint32_t lba);
uint8_t SPI_wait_ready(void);

void    sd_ss_set_active(uint8_t drv);
void    sd_ss_active_pin_down();
void    sd_ss_active_pin_up();
void    sd_init_disk(FATFS* fs,char* path);
void    sd_init_lib();
void    sd_read_free_space(FATFS* fs,char* path);


#define SS_SD_SELECT()   sd_ss_active_pin_down()
#define SS_SD_DESELECT() sd_ss_active_pin_up()
//#define LD_ON HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); //RED
//#define LD_OFF HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); //RED



//--------------------------------------------------
/* Card type flags (CardType) */
#define CT_MMC 0x01 /* MMC ver 3 */
#define CT_SD1 0x02 /* SD ver 1 */
#define CT_SD2 0x04 /* SD ver 2 */
#define CT_SDC (CT_SD1|CT_SD2) /* SD */
#define CT_BLOCK 0x08 /* Block addressing */
//--------------------------------------------------
typedef struct sd_info {
  volatile uint8_t type;
} sd_info_ptr;

typedef struct{
	uint16_t sd_ss_pin;
	GPIO_TypeDef* sd_ss_port;
}ss_pp;




//--------------------------------------------------
