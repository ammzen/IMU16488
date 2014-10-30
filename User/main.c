#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "bsp_usart1.h"
#include "bsp_spi_adis.h"
#include "bsp_exti.h"
#include "bsp_led.h"
#include "bsp_TiMbase.h"
#include "AHRS_Attitude.h"

#define CLI()       __set_PRIMASK(1)		/* 关闭总中断 */  
#define SEI()       __set_PRIMASK(0)				/* 开放总中断 */ 

__IO uint32_t DeviceID = 0;
__IO int KEYDOWN = 0;
extern float init_ax, init_ay, init_az, init_gx, init_gy, init_gz, init_mx, init_my, init_mz;
extern float q0, q1, q2, q3;
int i;
__IO float Roll=0, Pitch=0, Yaw=0;
extern uint8_t SendBuff[SENDBUFF_SIZE];  //USART1 DMA 发送缓存数据包 
// 函数原型声明
void Delay(__IO uint32_t nCount);

/*
 * 函数名：main
 * 描述  ：主函数
 * 输入  ：无
 * 输出  ：无
 */
int main(void)
{ 	
	/* 配置串口1为：115200 8-N-1 */
//	CLI(); //关闭全局中断
//    SEI();  //开放全局中断
	USART1_Config();
	USART1_DMA_Config();
	LED_GPIO_Config();
    TIM2_Configuration();
    TIM2_NVIC_Configuration();
    SysTick_Init();
    printf("\r\n串口初始化成功");
    /* 配置ADIS采集数据就绪外部中断*/
	EXTI_PA0_Config();
	/* ADIS spi初始化 */
	SPI_FLASH_Init();
	Delay(0xFFFFF);
	/* Get SPI Flash Device ID */
	DeviceID = SPI_FLASH_ReadDeviceID();
	
    if (DeviceID != 0x4068)
        {
            printf("\r\n     PROD_ID = 0x%X ", DeviceID);
            printf("\r\n     ADIS16488初始化失败");
            while(DeviceID != 0x4068){
               // Delay(0xFFF);
                DeviceID = SPI_FLASH_ReadDeviceID();
                printf("\r\n     PROD_ID = 0x%X ", DeviceID);
            }  
        }
    printf("\r\n     PROD_ID = 0x%X ", DeviceID);
    printf("\r\n     ADIS16488初始化成功\n\r");
    init_calibparams();
    gyro_calibration();
//    get_acc_bias();
//    printf("\r\n     进行磁力计校准\n\r     LED2闪烁过程中将传感器绕空间8字转动，按下按键2则校准完毕");
//    while(!KEYDOWN)  //按键2按下则停止校准
//    {
//        LED2_TOGGLE;
//        get_compass_bias();
//    }  
//    compass_calibration();
//    printf("\r\n     磁力计校准完毕\n\r     将传感器静止放于桌面，按下按键2开始初始化四元数\n\r");
//    while(!KEYDOWN)  
//    {
//        LED2_TOGGLE;
//        Delay(0xFFFFF);
//    }
    Delay(0xFFFF);
    init_quaternion();
    //Init_quaternion();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);    
    SystemInitReady = 1;
    LED3(ON);
    //TimeSpendStart();
    //GetRawData();
    //printf("\r\ntime spend: %dus\r\n", TimeSpendEnd());
    TIM3_HalfT_INIT();
    
    while(1){
        
        if (Attitude_500Hz)
        {
            Attitude_500Hz = 0;
       //     Motor_200Hz = 0;
            TIM_Cmd(TIM3, ENABLE);
            GetData();
            Get_Attitude();
        }
        if (Com_10Hz)
        {
            Com_10Hz = 0;
           // printf("%6.3f,  %6.3f,  %6.3f\r", Yaw, Pitch, Roll);
            //printf("status:%8.5f,%8.5f,%8.5f,%8.5f,yaw,pitch,roll:%8.3f,%8.3f,%8.3f\r",q0,q1,q2,q3,Yaw, Pitch, Roll); 
            //printf("init_ax=%6.3f, init_ay=%6.3f, init_az=%6.3f\r",init_ax, init_ay, init_az);
            printf("status:%9.6f,%9.6f,%9.6f,%9.6f,yaw,pitch,roll:%8.3f,%8.3f,%8.3f\r",q0,q1,q2,q3,Yaw, Pitch, Roll); 
        }
	}

}

void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}
/*********************************************END OF FILE**********************/
