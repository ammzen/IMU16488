/**
  ******************************************************************************
  * @file    bsp_TimBase.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   TIM2 1ms 定时应用bsp
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 iSO STM32 开发板 
  * 论坛    :http://www.chuxue123.com
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 

#include "bsp_TiMbase.h"
#include "stm32f10x_it.h"

#define T_1ms   SystemCoreClock / 1000      //1ms中断一次
#define T_10us  SystemCoreClock / 100000	//10us中断一次
#define T_1us   SystemCoreClock / 1000000   //1us中断一次

volatile uint32_t TimeDelay;

// SysTick设置，用于定时及延时
void SysTick_Init(void)
{
    // 配置时钟源，选择AHB时钟
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
    // 初始化失败，则进入死循环
    if( SysTick_Config(T_1us) )
    {
        while(1);
    }
    // 关闭滴答定时器，使用时再进行开启
    SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;
}
// us延时程序， 1us为一个单位
void Delay_us(volatile uint32_t nTime)
{
    TimeDelay = nTime;
    SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk; // 使能滴答定时器
    while(TimeDelay != 0);
    // 关闭滴答定时器
    SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;
}
void Delay_ms(volatile uint32_t nTime)
{
    Delay_us(1000 * nTime);
}
/// TIM2中断优先级配置
void TIM2_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*
 * TIM_Period / Auto Reload Register(ARR) = 1000   TIM_Prescaler--71 
 * 中断周期为 = 1/(72MHZ /72) * 1000 = 1ms
 *
 * TIMxCLK/CK_PSC --> TIMxCNT --> TIM_Period(ARR) --> 中断 且TIMxCNT重置为0重新计数 
 */
void TIM2_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		
		/* 设置TIM2CLK 为 72MHZ */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
    //TIM_DeInit(TIM2);
	
	/* 自动重装载寄存器周期的值(计数值) */
    TIM_TimeBaseStructure.TIM_Period=1000;
	
    /* 累计 TIM_Period个频率后产生一个更新或者中断 */
	/* 时钟预分频数为72 */
    TIM_TimeBaseStructure.TIM_Prescaler= 71;
	
	/* 对外部时钟进行采样的时钟分频,这里没有用到 */
    //TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
		
    TIM_Cmd(TIM2, ENABLE);																		
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE);		/*先关闭等待使用*/    
}

//功　　能: 用于确定姿态解算函数中的HalfT		
void TIM3_HalfT_INIT(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能

	TIM_TimeBaseStructure.TIM_Period = 0xFFFF; //1us计一次数，计到n则为n us
	TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; //设置用来作为TIMx时钟频率除数的预分频值，1us计一个数
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

}
// 记录程序运行开始时间
void TimeSpendStart(void)
{
    SysTickCounter = 0;
    SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk; // 使能滴答定时器
}
// 计算程序运行结束时间
uint32_t TimeSpendEnd(void)
{
    SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;    // 关闭滴答定时器
    return SysTickCounter;
}
/*********************************************END OF FILE**********************/
