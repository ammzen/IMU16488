/**
  ******************************************************************************
  * @file    bsp_led.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   led应用函数接口
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 iSO STM32 开发板 
  * 论坛    :http://www.chuxue123.com
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "bsp_led.h"   

 /**
  * @brief  初始化控制LED的IO
  * @param  无
  * @retval 无
  */
void LED_GPIO_Config(void)
{   
    /*定义一个GPIO_InitTypeDef类型的结构体*/
    GPIO_InitTypeDef GPIO_InitStructure;

    /*开启GPIOF的外设时钟*/
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOF, ENABLE); 

    /*选择要控制的GPIOF引脚*/                                
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /*调用库函数，初始化GPIOF7*/
    GPIO_Init(GPIOF, &GPIO_InitStructure);
    
    /*选择要控制的GPIOF引脚*/                                
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;

    /*调用库函数，初始化GPIOF7*/
    GPIO_Init(GPIOF, &GPIO_InitStructure);        

    /* 关闭所有led灯 */
    GPIO_SetBits(GPIOF, GPIO_Pin_7|GPIO_Pin_8);  
}
/*********************************************END OF FILE**********************/
