#ifndef TIME_TEST_H
#define TIME_TEST_H

#include "stm32f10x.h"

void SysTick_Init(void);
void TIM2_NVIC_Configuration(void);
void TIM2_Configuration(void);
void Delay_us(volatile uint32_t nTime);  //1us 为一个单位
void Delay_ms(volatile uint32_t nTime);  //1ms 为一个单位
void TIM3_HalfT_INIT(void);
void TimeSpendStart(void);  //计算一段程序花费的时间开始点
uint32_t TimeSpendEnd(void);//计算一段程序花费的时间结束点


#endif	/* TIME_TEST_H */
