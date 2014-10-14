/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTI
  
  AL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "bsp_spi_adis.h"

#define SYSTEM_COUNT   100         
#define COUNT_500HZ   2
#define COUNT_200HZ   5
#define COUNT_10HZ    100

uint32_t SystemCounter = 0;
uint32_t SysTickCounter = 0;
uint8_t Attitude_500Hz = 0;
uint8_t Motor_200Hz = 0;
uint8_t Com_10Hz = 0;
uint8_t SystemInitReady = 0;
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
extern volatile uint32_t TimeDelay;
void SysTick_Handler(void)
{
    SysTickCounter++;
  if (TimeDelay != 0x00)
    { 
        TimeDelay--;
    }
}

extern __IO float DELTANG[3];
extern float Roll, Pitch, Yaw;
extern __IO uint16_t *p;
extern __IO float * TRUEVALUE;
extern __IO int KEYDOWN;

void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
//    p = SPI_READ_DELTANG();
//    TRUEVALUE = CONVERT_DELTANG(p);
//    DELTANG[0] = *(TRUEVALUE+0);
//    DELTANG[1] = *(TRUEVALUE+1);
//    DELTANG[2] = *(TRUEVALUE+2);
//		Roll += DELTANG[0];
//		Pitch += DELTANG[1];
//		Yaw += DELTANG[2];
		//printf("\r\n 中断发生\r\n");
        KEYDOWN = 1;
		EXTI_ClearITPendingBit(EXTI_Line0);
	}  
}
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/
//extern volatile uint32_t EVENT;
void TIM2_IRQHandler(void)
{
	if ( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET ) 
	{	
	    if(SystemInitReady == 1)
      {
        SystemCounter++;
        if(SystemCounter > SYSTEM_COUNT)
        {SystemCounter = 1;}

        if((SystemCounter % COUNT_500HZ) == 0)	   
        {Attitude_500Hz = 1;}

        if((SystemCounter % COUNT_200HZ) == 0)
        {Motor_200Hz = 1;}

        if((SystemCounter % COUNT_10HZ) == 0)
        {Com_10Hz = 1;}
      }
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);  		 
	}	    
}
/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
