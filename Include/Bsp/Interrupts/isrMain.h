/********************************************************************************
  * @file:			isrMain.h
  * @author:		Hardware Team
  * @version:
  * @date
  * @brief:
  ******************************************************************************
  * @attention
  *
  * COPYRIGHT(c) 2015 AgTech Labs, Inc.
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
 *
 *  Created on: Aug 26, 2015
 *      Author: admin
 */

#ifndef __ISRMAIN_H_
#define __ISRMAIN_H_

/* Includes =================================================================*/
#include <stdbool.h>
#include <stdint.h>

#include "stm32f7xx_hal.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Definition for TIMx clock resources */

#define TIM_PRESCALER_VALUE				((uint32_t) (((SystemCoreClock/2)/200000) - 1))
#define TIM_PRESCALER_VALUE_DELAY		((uint32_t) (((SystemCoreClock/2)/10000) - 1))
#define RELOAD_MAX_30SEC				((uint32_t)60000)	
#define TIMER_2_RELOAD					(10)
#define TIMER_3_RELOAD					RELOAD_MAX_30SEC
#define TIMER_4_RELOAD					RELOAD_MAX_30SEC
#define TIMER_5_RELOAD					RELOAD_MAX_30SEC
#define TIMER_6_RELOAD					RELOAD_MAX_30SEC	/*RADIO POLLING*/
#define TIMER_7_RELOAD					((uint32_t)6000)	/*CLOUD POSTING*/
#define TIMER_9_RELOAD					RELOAD_MAX_30SEC
#define TIMERTEST_RELOAD				((uint32_t)10000)	/*1 sec*/
/* Exported macro ------------------------------------------------------------*/

typedef enum
{
	DAQTIM_P1_IRQn 		= TIM2_IRQn,
	DAQTIM_P2_IRQn 		= TIM3_IRQn,
	DAQTIM_P3_IRQn 		= TIM4_IRQn,
	DAQTIM_P4_IRQn 		= TIM5_IRQn,
	DAQCTR_P1_IRQn		= EXTI3_IRQn,
	DAQCTR_P2_IRQn 		= EXTI4_IRQn,
	DAQCTR_P3_IRQn 		= EXTI0_IRQn,
	DAVIS_DTH_SCK_IRQn	= EXTI1_IRQn,
	WIFI_IRQn			= USART6_IRQn,
	XBEE_IRQn			= UART4_IRQn,
	SYSSTATUS_IRQn		= TIM6_DAC_IRQn,
	RADIO_CLOUD_IRQn	= TIM7_IRQn,
	//SYSSTATUS_IRQn	= TIM1_BRK_TIM9_IRQn,
	RS485_IRQn			= UART8_IRQn
}AgBoardIRQnVectorEnum;

/* User can use this section to tailor TIMx instance used and associated
   resources */
/* External Declarations ====================================================*/
//bool is_timer_running;
//bool is_uartit_set;

/* Function Prototypes ======================================================*/
void extiResetPortTicks(uint8_t port);
uint32_t extiGetPortTicks(uint8_t port);
void timerIrqEnable(void);
void timerIrqCfg(TIM_HandleTypeDef *TmrHndlPtr);
void timerPollPost(void);
void radioPollPost(uint8_t record_typ);
void extiCfgGPIO(uint8_t port, bool exti_en);
void timerIrqDisable(void);
void timerIrqStop(TIM_HandleTypeDef *TmrHndlPtr);
void setLPTIMLowFreq(bool status);

extern TIM_HandleTypeDef 	timer1_handle, timer2_handle, timer3_handle,
					timer4_handle, timer5_handle, timer6_handle,
					timer7_handle, timer9_handle, timer10_handle;

extern LPTIM_HandleTypeDef             LptimHandle;

#endif /* __ISRMAIN_H_ */
/*********************************************************************************
  * Revision History
  * @file         isrMain.h
  ********************************************************************************
  * @version
  * @date
  * @author         
  * @changes     - renamed file
  ********************************************************************************
  * @version
  * @date
  * @author         
  * @changes     - created file (interruptCfg.h)
  ********************************************************************************
  */

