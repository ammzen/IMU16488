
#include "bsp_spi_adis.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define SEQ_CNT                   0X0600
#define SYS_E_FLAG                0X0800
#define DIAG_STS                  0X0A00
#define ALM_STS                   0X0C00
#define TEMP_OUT                  0X0E00
#define X_GYRO_LOW                0X1000
#define X_GYRO_OUT                0X1200
#define Y_GYRO_LOW                0X1400
#define Y_GYRO_OUT                0X1600
#define Z_GYRO_LOW                0X1800
#define Z_GYRO_OUT                0X1A00
#define X_ACCL_LOW                0X1C00
#define X_ACCL_OUT                0X1E00
#define Y_ACCL_LOW                0X2000
#define Y_ACCL_OUT                0X2200
#define Z_ACCL_LOW                0X2400
#define Z_ACCL_OUT                0X2600
#define X_MAGN_OUT                0X2800
#define Y_MAGN_OUT                0X2A00
#define Z_MAGN_OUT                0X2C00
#define BAROM_LOW                 0X2E00
#define BAROM_OUT                 0X3000
#define X_DELTANG_LOW             0X4000
#define X_DELTANG_OUT             0X4200
#define Y_DELTANG_LOW             0X4400
#define Y_DELTANG_OUT             0X4600
#define Z_DELTANG_LOW             0X4800
#define Z_DELTANG_OUT             0X4A00
#define TIME_MS_OUT               0X7800
#define TIME_DH_OUT               0X7A00
#define TIME_YM_OUT               0X7C00
#define ADIS_DEVICEID             0X7E00  //PROD_ID 产品标识(16,488)输出，值0X4068

#define GYRO_LSB                  0.02/65536.0  // 1 LSB = 0.02/2^16 °/sec
#define ACCL_LSB                  0.0008/65536.0  // 1 LSB = 0.8/2^16 mg
#define MAGN_LSB                  0.1  // 1 LSB = 0.1 mgauss
#define DELTANG_LSB               720/2147483648.0  // 1 LSB = 720°/2^15/2^16

#define Dummy_Byte                0xFFFF

/*******************************************************************************
* Function Name  : SPI_FLASH_Init
* Description    : Initializes the peripherals used by the SPI FLASH driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_Init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable SPI1 and GPIO clocks */
  /*!< SPI_FLASH_SPI_CS_GPIO, SPI_FLASH_SPI_MOSI_GPIO, 
       SPI_FLASH_SPI_MISO_GPIO, SPI_FLASH_SPI_DETECT_GPIO 
       and SPI_FLASH_SPI_SCK_GPIO Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

  /*!< SPI_FLASH_SPI Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); 

    
  /*!< Configure SPI_FLASH_SPI pins: SCK MOSI*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /*!< Configure SPI_FLASH_SPI pins: MISO*/  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /*!< Configure SPI_FLASH_SPI_CS_PIN pin: SPI_FLASH Card CS pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  /* SPI1 configuration */
  // W25X16: data input on the DIO pin is sampled on the rising edge of the CLK. 
  // Data on the DO and DIO pins are clocked out on the falling edge of CLK.
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);

  /* Enable SPI1  */
  SPI_Cmd(SPI1, ENABLE);
}



uint16_t SPI_FLASH_ReadDeviceID(void)
{
  u16 Temp = 0;
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

    /* Send "PROD_ID " instruction */
  SPI_FLASH_SendTwoByte(ADIS_DEVICEID);
  
  /* Read two bytes from the FLASH */
  Temp = SPI_FLASH_SendTwoByte(Dummy_Byte);

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  return Temp;
}



uint16_t * SPI_READ_DELTANG(void){
  static uint16_t DeltAng[6];
 
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendTwoByte(X_DELTANG_OUT);
  DeltAng[0] = SPI_FLASH_SendTwoByte(Dummy_Byte);
  SPI_FLASH_CS_HIGH();

  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendTwoByte(X_DELTANG_LOW);
  DeltAng[1] = SPI_FLASH_SendTwoByte(Dummy_Byte);
  SPI_FLASH_CS_HIGH();

  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendTwoByte(Y_DELTANG_OUT);
  DeltAng[2] = SPI_FLASH_SendTwoByte(Dummy_Byte);
  SPI_FLASH_CS_HIGH();

  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendTwoByte(Y_DELTANG_LOW);
  DeltAng[3] = SPI_FLASH_SendTwoByte(Dummy_Byte);
  SPI_FLASH_CS_HIGH();

  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendTwoByte(Z_DELTANG_OUT);
  DeltAng[4] = SPI_FLASH_SendTwoByte(Dummy_Byte);
  SPI_FLASH_CS_HIGH();
  
  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendTwoByte(Z_DELTANG_LOW);
  DeltAng[5] = SPI_FLASH_SendTwoByte(Dummy_Byte);
  SPI_FLASH_CS_HIGH();

  return DeltAng;
}
float * CONVERT_DELTANG(__IO uint16_t * DeltAng)  //转换后单位为角度值°
{
  static float DELTANG[3];

  DELTANG[0] = ((int32_t)(*(DeltAng + 0) << 16 | *(DeltAng + 1))) * DELTANG_LSB;
  DELTANG[1] = ((int32_t)(*(DeltAng + 2) << 16 | *(DeltAng + 3))) * DELTANG_LSB;
  DELTANG[2] = ((int32_t)(*(DeltAng + 4) << 16 | *(DeltAng + 5))) * DELTANG_LSB;

  return DELTANG;
}

uint16_t * SPI_READ_GYRO(void){
  static uint16_t Gyro[6];
 
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendTwoByte(X_GYRO_OUT);
  Gyro[0] = SPI_FLASH_SendTwoByte(Dummy_Byte);
  SPI_FLASH_CS_HIGH();

  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendTwoByte(X_GYRO_LOW);
  Gyro[1] = SPI_FLASH_SendTwoByte(Dummy_Byte);
  SPI_FLASH_CS_HIGH();

  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendTwoByte(Y_GYRO_OUT);
  Gyro[2] = SPI_FLASH_SendTwoByte(Dummy_Byte);
  SPI_FLASH_CS_HIGH();

  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendTwoByte(Y_GYRO_LOW);
  Gyro[3] = SPI_FLASH_SendTwoByte(Dummy_Byte);
  SPI_FLASH_CS_HIGH();

  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendTwoByte(Z_GYRO_OUT);
  Gyro[4] = SPI_FLASH_SendTwoByte(Dummy_Byte);
  SPI_FLASH_CS_HIGH();
  
  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendTwoByte(Z_GYRO_LOW);
  Gyro[5] = SPI_FLASH_SendTwoByte(Dummy_Byte);
  SPI_FLASH_CS_HIGH();

  return Gyro;
}
float * CONVERT_GYRO(__IO uint16_t * Gyro)  //转换后单位为角度/秒  °/s
{
  static float GYRO[3];

  GYRO[0] = ((int32_t)(*(Gyro + 0) << 16 | *(Gyro + 1))) * GYRO_LSB;
  GYRO[1] = ((int32_t)(*(Gyro + 2) << 16 | *(Gyro + 3))) * GYRO_LSB;
  GYRO[2] = ((int32_t)(*(Gyro + 4) << 16 | *(Gyro + 5))) * GYRO_LSB;

  return GYRO;
}

uint16_t * SPI_READ_ACCL(void){
  static uint16_t Accl[6];
 
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendTwoByte(X_ACCL_OUT);
  Accl[0] = SPI_FLASH_SendTwoByte(Dummy_Byte);
  SPI_FLASH_CS_HIGH();

  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendTwoByte(X_ACCL_LOW);
  Accl[1] = SPI_FLASH_SendTwoByte(Dummy_Byte);
  SPI_FLASH_CS_HIGH();

  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendTwoByte(Y_ACCL_OUT);
  Accl[2] = SPI_FLASH_SendTwoByte(Dummy_Byte);
  SPI_FLASH_CS_HIGH();

  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendTwoByte(Y_ACCL_LOW);
  Accl[3] = SPI_FLASH_SendTwoByte(Dummy_Byte);
  SPI_FLASH_CS_HIGH();

  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendTwoByte(Z_ACCL_OUT);
  Accl[4] = SPI_FLASH_SendTwoByte(Dummy_Byte);
  SPI_FLASH_CS_HIGH();
  
  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendTwoByte(Z_ACCL_LOW);
  Accl[5] = SPI_FLASH_SendTwoByte(Dummy_Byte);
  SPI_FLASH_CS_HIGH();

  return Accl;
}
float * CONVERT_ACCL(__IO uint16_t * Accl)  //转换后单位为一个重力加速度 g
{
  static float ACCL[3];

  ACCL[0] = ((int32_t)(*(Accl + 0) << 16 | *(Accl + 1))) * ACCL_LSB;
  ACCL[1] = ((int32_t)(*(Accl + 2) << 16 | *(Accl + 3))) * ACCL_LSB;
  ACCL[2] = ((int32_t)(*(Accl + 4) << 16 | *(Accl + 5))) * ACCL_LSB;

  return ACCL;
}


uint16_t * SPI_READ_MAGN(void){
  static uint16_t Magn[3];
 
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendTwoByte(X_MAGN_OUT);
  Magn[0] = SPI_FLASH_SendTwoByte(Dummy_Byte);
  SPI_FLASH_CS_HIGH();

  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendTwoByte(Y_MAGN_OUT);
  Magn[1] = SPI_FLASH_SendTwoByte(Dummy_Byte);
  SPI_FLASH_CS_HIGH();

  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendTwoByte(Z_MAGN_OUT);
  Magn[2] = SPI_FLASH_SendTwoByte(Dummy_Byte);
  SPI_FLASH_CS_HIGH();

  return Magn;
}
float * CONVERT_MAGN(__IO uint16_t * Magn)  //转换后单位为高斯  gauss
{
  static float MAGN[3];

  MAGN[0] = ((int16_t)(*(Magn + 0))) * MAGN_LSB;
  MAGN[1] = ((int16_t)(*(Magn + 1))) * MAGN_LSB;
  MAGN[2] = ((int16_t)(*(Magn + 2))) * MAGN_LSB;

  return MAGN;
}

u16 SPI_FLASH_ReadTwoByte(void)
{
  return (SPI_FLASH_SendTwoByte(Dummy_Byte));
}



u16 SPI_FLASH_SendTwoByte(u16 byte)
{
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(SPI1, byte);

  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI1);
}


/*********************************************END OF FILE**********************/
