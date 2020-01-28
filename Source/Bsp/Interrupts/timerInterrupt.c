/**
  ******************************************************************************
  * @file      	timerInterrupt.c
  * @author		Hardware Team
  * @version	v2.2.0
  * @date		04/08/16
  * @brief		Contains all init/configuration and operation related to timers.
  ******************************************************************************
  * @attention
  *
  * COPYRIGHT(c) 2016 AgTech Labs, Inc.
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of AgTech Labs, Inc. nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ----------------------------------------------------------------*/
#include <deployAssert.h>

#include "productCfg.h"
#include "agboardInit.h"
#include "dmMain.h"
#include "stm32f779xx.h"
#include "delay.h"
#include "cloud.h"
#include "isrMain.h"
/* Constants ================================================================*/

/* Global Declarations ======================================================*/
TIM_HandleTypeDef 	timer1_handle, timer2_handle, timer3_handle,
					timer4_handle, timer5_handle, timer6_handle,
					timer7_handle, timer9_handle, timer10_handle;

#define MIN_INTERVAL   			((uint32_t)2)		//(1 minute)
#define PORT1_DAQ_INTERVAL		(uint8_t)(PORT1_INTERVAL * MIN_INTERVAL) 
#define PORT2_DAQ_INTERVAL	 	(uint8_t)(PORT2_INTERVAL * MIN_INTERVAL)
#define PORT3_DAQ_INTERVAL	 	(uint8_t)(PORT3_INTERVAL * MIN_INTERVAL)
#define PORT4_DAQ_INTERVAL	 	(uint8_t)(PORT4_INTERVAL * MIN_INTERVAL)
#define SYSSTAT_INTERVAL		(uint8_t)(SYS_ACQUIRE_INTERVAL * MIN_INTERVAL) 
#define POLLING_INTERVAL  		(uint8_t)(RADIO_POLL_INTERVAL * MIN_INTERVAL)
#define POSTING_INTERVAL 		(uint8_t)(CLOUD_POST_INTERVAL * MIN_INTERVAL)
#define	LPTIM_PERIOD			(65535)
#define	LPTIM_TIMEOUT			(32767)

/* Global Declarations  =====================================================*/
LPTIM_HandleTypeDef LptimHandle;

/* External Declarations ====================================================*/
#if !RTC_BASED_CYCLE
	extern bool exec_daq1, exec_daq2, exec_daq3, exec_daq4, exec_sysstat, exec_poll_post;
#endif

/* Function prototypes ------------------------------------------------------*/
void timerIrqCfg(TIM_HandleTypeDef *TmrHndlPtr);

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{

	switch((uint32_t)htim->Instance)
	{
	case (uint32_t)TIM1:
		__HAL_RCC_TIM1_CLK_ENABLE();
		break;
	case (uint32_t)TIM2:
		__HAL_RCC_TIM2_CLK_ENABLE();
		break;
	case (uint32_t)TIM3:
		__HAL_RCC_TIM3_CLK_ENABLE();
		break;
	case (uint32_t)TIM4:
		__HAL_RCC_TIM4_CLK_ENABLE();
		break;
	case (uint32_t)TIM5:
		__HAL_RCC_TIM5_CLK_ENABLE();
		break;
	case (uint32_t)TIM6:
		__HAL_RCC_TIM6_CLK_ENABLE();
		break;
	case (uint32_t)TIM7:
		__HAL_RCC_TIM7_CLK_ENABLE();
		break;
	case (uint32_t)TIM9:
		__HAL_RCC_TIM9_CLK_ENABLE();
		break;
	case (uint32_t)TIM10:
		__HAL_RCC_TIM10_CLK_ENABLE();
		break;
	default:
		break;
	}
}

void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&timer2_handle);
}

void TIM4_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&timer4_handle);
  	gpioDigitalWrite(io_dth_sck, davisdth_sck_state);
	davisdth_sck_state = !davisdth_sck_state;

}

void TIM5_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&timer5_handle);
}
/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
}

/* Functions --------------------------------------------------------------*/

void LPTIM1_IRQHandler(void)
{
  HAL_LPTIM_IRQHandler(&LptimHandle);
}
	
void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
  /* Timeout was reached, turn on LED1 */
  	static bool test = false;
	if(led_pattern != LED_SLEEPMODE)
		HAL_IWDG_Refresh(&IwdgHandle);
	switch(led_pattern){
		case LED_RUNMODE:
		  	gpioDigitalWrite(&LD_USER1_R, test);
			gpioDigitalWrite(&LD_USER2_G, !test);
			break;
		case LED_RUNMODE_GPS:
		  	gpioDigitalWrite(&LD_USER1_R, test);
			gpioDigitalWrite(&LD_USER2_G, test);
			break;
		case LED_RUNMODE_ACCNTID_SET:
		  	gpioDigitalWrite(&LD_USER1_R, 0);
			gpioDigitalWrite(&LD_USER2_G, test);
			break;
		case LED_SLEEPMODE:
		  	gpioDigitalWrite(&LD_USER2_G, 0);
			gpioDigitalWrite(&LD_USER1_R, 1);
			delayMillis(100);
			gpioDigitalWrite(&LD_USER1_R, 0);
			break;
		case LED_BLEMODE:
		  	gpioDigitalWrite(&LD_USER1_R, 1);
			gpioDigitalWrite(&LD_USER2_G, test);
		  	break;
		default:
		  	const char *led_show = "\nUnknown LED Pattern\n";
		  	uartSHOW((uint8_t*)led_show, strlen(led_show));
			assert(0);
	}
	test = !test;
}

void HAL_LPTIM_MspInit(LPTIM_HandleTypeDef *hlptim)
{
  __HAL_RCC_LPTIM1_CLK_ENABLE();
  __HAL_RCC_LPTIM1_FORCE_RESET();
  __HAL_RCC_LPTIM1_RELEASE_RESET();
  

  HAL_NVIC_SetPriority(LPTIM1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(LPTIM1_IRQn); 

}

void setLPTIMLowFreq(bool status)
{
  	HAL_LPTIM_TimeOut_Stop_IT(&LptimHandle);
	HAL_LPTIM_DeInit(&LptimHandle);
	
	if(status)
	  	LptimHandle.Init.Clock.Prescaler    = LPTIM_PRESCALER_DIV4;
	else
		LptimHandle.Init.Clock.Prescaler    = LPTIM_PRESCALER_DIV1;  
	
	if (HAL_LPTIM_Init(&LptimHandle) != HAL_OK)
	{
		assert(0);
	}

	if (HAL_LPTIM_TimeOut_Start_IT(&LptimHandle, LPTIM_PERIOD, LPTIM_TIMEOUT) != HAL_OK)
	{
		assert(0);
	}
}

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void timerIrqEnable(void)
{
  	timer4_handle.Instance = TIM4;
	timer4_handle.Init.Prescaler = TIM_PRESCALER_VALUE;
	timer4_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	timer4_handle.Init.Period = TIMER_2_RELOAD;
	timer4_handle.Init.ClockDivision = 0;
	timerIrqCfg(&timer4_handle);

}

void timerIrqDisable(void)
{
	timerIrqStop(&timer4_handle);
	timerIrqStop(&timer5_handle);
	__HAL_RCC_TIM4_CLK_DISABLE();
	__HAL_RCC_TIM5_CLK_DISABLE();
}

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void timerIrqCfg(TIM_HandleTypeDef *TmrHndlPtr)
{
	if(HAL_TIM_Base_Init(TmrHndlPtr) != HAL_OK)
	{
		assert(0);
	}

	__HAL_TIM_CLEAR_FLAG(TmrHndlPtr, TIM_FLAG_UPDATE);
    __HAL_TIM_CLEAR_IT(TmrHndlPtr, TIM_IT_UPDATE);

	/* Start the TIM Base generation in interrupt mode*/
	if(HAL_TIM_Base_Start_IT(TmrHndlPtr) != HAL_OK)
	{
		assert(0);
	}
	return;
}

void timerIrqStop(TIM_HandleTypeDef *TmrHndlPtr)
{
	if(HAL_TIM_Base_Stop_IT(TmrHndlPtr) != HAL_OK)
	{
		assert(0);
	}
	
	if(HAL_TIM_Base_DeInit(TmrHndlPtr) != HAL_OK)
	{
		assert(0);
	}
	
}
#if !RTC_BASED_CYCLE

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void timerTelemetryDataLogPort1(void)
{
	static uint8_t ctr = 0;
	ctr++;
	if(ctr != PORT1_DAQ_INTERVAL)
		return;
	exec_daq1 = true;
	ctr = 0;
	return;
}

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
static void timerTelemetryDataLogPort2(void)
{
	static uint8_t ctr = 0;
	ctr++;
	if(ctr != PORT2_DAQ_INTERVAL)
		return;
	ctr = 0;
	exec_daq2 = true;
	return;
}

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void timerTelemetryDataLogPort3(void)
{

	static uint8_t ctr = 0;
	ctr++;
	if(ctr != PORT3_DAQ_INTERVAL)
		return;
	ctr = 0;
	exec_daq3 = true;
	return;
}

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void timerTelemetryDataLogPort4(void)
{
	static uint8_t ctr = 0;
	ctr++;
	if(ctr != PORT4_DAQ_INTERVAL)
		return;
	ctr = 0;
	exec_daq4 = true;
	return;
}

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
static void timerSysAcquire(void)
{
	static uint8_t ctr = 0;
	ctr++;
	if(ctr != SYSSTAT_INTERVAL)
		return;
	ctr = 0;
	exec_sysstat = true;
	return;
}
#endif

#if !RTC_BASED_CYCLE
/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void timerPollPost(void)
{
	static uint8_t ctr = 0;
	ctr++;
//		ctr = RADIO_POLL_INTERVAL;

//
//	if(ctr != RADIO_POLL_INTERVAL)	//2 minutes
//		return;
//	ctr = 0;
//	exec_poll_post = true;
	return;
}
#endif

#if !RTC_BASED_CYCLE
/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void timerInterruptElapsedCallback(TIM_HandleTypeDef *htim)
{
	char printthis[30];
	switch((uint32_t)htim->Instance)
	{
		case (uint32_t)TIM2:
			timerTelemetryDataLogPort1();
			break;
		case (uint32_t)TIM3:
			timerTelemetryDataLogPort2();
			break;
		case (uint32_t)TIM4:
			timerTelemetryDataLogPort3();
			break;
		case (uint32_t)TIM5:
			timerTelemetryDataLogPort4();
			break;
		case (uint32_t)TIM6:
			timerSysAcquire();
			//
			break;
		case (uint32_t)TIM7:
			timerPollPost();;
			break;
		case (uint32_t)TIM9:
			break;
		case (uint32_t)TIM10:
			break;
		default:
			snprintf(printthis,30,"TIMER %x\n",(unsigned int)htim->Instance);
            uartSHOW((uint8_t*)printthis,strlen(printthis));
	}
}

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&timer2_handle);
	
	#if PRINT_PROCESSES_IN_UART
		//const char *msg = "=====DAQ 1 TIMER END=====\n\n";
		//uartSHOW((uint8_t*)msg, strlen(msg));
	#endif
}

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void TIM3_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&timer3_handle);
	#if PRINT_PROCESSES_IN_UART
		const char *msg = "=====DAQ 2 TIMER END=====\n\n";
		//uartSHOW((uint8_t*)msg, strlen(msg));
	#endif
}

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void TIM4_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&timer4_handle);
	#if PRINT_PROCESSES_IN_UART
		const char *msg = "=====DAQ 3 TIMER END=====\n\n";
		//uartSHOW((uint8_t*)msg, strlen(msg));
	#endif
}

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void TIM5_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&timer5_handle);
	#if PRINT_PROCESSES_IN_UART
		const char *msg = "=====DAQ 4 TIMER END=====\n\n";
		//uartSHOW((uint8_t*)msg, strlen(msg));
	#endif
}

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void TIM6_DAC_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&timer6_handle);
}

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void TIM7_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&timer7_handle);

	#if PRINT_PROCESSES_IN_UART
		const char *msg = "=====SYS STAT TIMER END=====\n\n";
		//uartSHOW((uint8_t*)msg, strlen(msg));
	#endif
}

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void TIM1_BRK_TIM9_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&timer9_handle);
}

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void TIM1_UP_TIM10_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&timer10_handle);
}
#endif

/******************************************************************************
  * Revision History
  *	@file      	timerInterrupt.c
  *****************************************************************************
  * @version    v2.2.0
  * @date       04/08/16
  * @author     D.Lasdoce
  * @changes
  *****************************************************************************
  * @version    v2.1.0
  * @date       12/23/2015
  * @author     D.Lasdoce
  * @changes
  *****************************************************************************
  * @version    v2.1.0
  * @date       12/23/2015
  * @author     M.I.Lorica
  * @changes	radio, cloud, sysstat callback function
  *****************************************************************************
  * @version
  * @date
  * @author		Ma. Irene Lorica
  * @changes	- created file
  *****************************************************************************
  */
