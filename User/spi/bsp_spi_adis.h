#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

#include "stm32f10x.h"

#define SPI_FLASH_SPI                           SPI1
#define SPI_FLASH_SPI_CLK                       RCC_APB2Periph_SPI1
#define SPI_FLASH_SPI_SCK_PIN                   GPIO_Pin_5                  /* PA.05 */
#define SPI_FLASH_SPI_SCK_GPIO_PORT             GPIOA                       /* GPIOA */
#define SPI_FLASH_SPI_SCK_GPIO_CLK              RCC_APB2Periph_GPIOA
#define SPI_FLASH_SPI_MISO_PIN                  GPIO_Pin_6                  /* PA.06 */
#define SPI_FLASH_SPI_MISO_GPIO_PORT            GPIOA                       /* GPIOA */
#define SPI_FLASH_SPI_MISO_GPIO_CLK             RCC_APB2Periph_GPIOA
#define SPI_FLASH_SPI_MOSI_PIN                  GPIO_Pin_7                  /* PA.07 */
#define SPI_FLASH_SPI_MOSI_GPIO_PORT            GPIOA                       /* GPIOA */
#define SPI_FLASH_SPI_MOSI_GPIO_CLK             RCC_APB2Periph_GPIOA
#define SPI_FLASH_CS_PIN                        GPIO_Pin_4                  /* PA.04 */
#define SPI_FLASH_CS_GPIO_PORT                  GPIOA                       /* GPIOA */
#define SPI_FLASH_CS_GPIO_CLK                   RCC_APB2Periph_GPIOA


#define SPI_FLASH_CS_LOW()       GPIO_ResetBits(GPIOA, GPIO_Pin_4)
#define SPI_FLASH_CS_HIGH()      GPIO_SetBits(GPIOA, GPIO_Pin_4)


void SPI_FLASH_Init(void);
uint16_t SPI_FLASH_ReadDeviceID(void);
uint16_t SPI_READ_DIAG_STS(void);
int16_t SPI_READ_TEMP(void);
uint16_t * SPI_READ_DELTANG(void);
uint16_t * SPI_READ_GYRO(void);
uint16_t * SPI_READ_ACCL(void);
uint16_t * SPI_READ_MAGN(void);

float * CONVERT_DELTANG(__IO uint16_t * DeltAng);  //转换后单位为角度值°
float * CONVERT_GYRO(__IO uint16_t * Gyro);  //转换后单位为角度/秒  °/s
float * CONVERT_ACCL(__IO uint16_t * Accl);  //转换后单位为一个重力加速度 g
float * CONVERT_MAGN(__IO uint16_t * Magn);  //转换后单位为高斯  gauss



u16 SPI_FLASH_ReadTwoByte(void);
u16 SPI_FLASH_SendTwoByte(u16 byte);

void EXTI_PB0_Config(void);
#endif /* __SPI_FLASH_H */

