/**
  ******************************************************************************
  * @file      	gpio.h
  * @author     Hardware Team
  * @version    v2.2.0
  * @date       04/08/162
  * @brief
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

#ifndef __GPIO_H_
#define __GPIO_H_

/* Includes =================================================================*/
#include <stdbool.h>
#include "stm32f7xx_hal.h"


/* Defines ==================================================================*/
#define BITS_TO_EDIT_2			((uint32_t)0x00000002)

#define HIGH 					((uint8_t)1)
#define LOW 					((uint8_t)0)

#define MODE_INPUT 				((uint32_t)0x00000000)
#define MODE_OUTPUT 			((uint32_t)0x00000001)
#define MODE_ALTERNATE 			((uint32_t)0x00000002)
#define MODE_ANALOG 			((uint32_t)0x00000003)

#define CFG_PUSHPULL 			((uint32_t)0x00000000)
#define CFG_OPENDRAIN 			((uint32_t)0x00000001)

#define CFG_PULLNO 				((uint32_t)0x00000000)
#define CFG_PULLUP 				((uint32_t)0x00000001)
#define CFG_PULLDOWN 			((uint32_t)0x00000002)

#define CFG_SPEEDLOW 			((uint32_t)0x00000000)
#define CFG_SPEEDMID 			((uint32_t)0x00000001)
#define CFG_SPEEDFAST 			((uint32_t)0x00000002)
#define CFG_SPEEDHIGH 			((uint32_t)0x00000003)

#define EXTI_MODE            	((uint32_t)0x10000000)
#define GPIO_MODE_IT         	((uint32_t)0x00010000)
#define GPIO_MODE_EVT        	((uint32_t)0x00020000)
#define RISING_EDGE          	((uint32_t)0x00100000)
#define FALLING_EDGE         	((uint32_t)0x00200000)
#define IT_MODE_RISING          ((uint32_t)0x10110000)   /*!< External Interrupt Mode with Rising edge trigger detection          */
#define IT_MODE_FALLING         ((uint32_t)0x10210000)   /*!< External Interrupt Mode with Falling edge trigger detection         */
#define IT_MODE_RISING_FALLING  ((uint32_t)0x10310000)   /*!< External Interrupt Mode with Rising/Falling edge trigger detection  */

#define BM_BSSR 				((uint32_t)0x00000001)
#define BM_OTYPER				((uint32_t)0x00000001)

#define BM_AFR_I2C				((uint32_t)0x00000004)
#define BM_AFR_SYS				((uint_32_t)0x0000000F)

#define BM_MODER 				((uint32_t)0x00000003)
#define BM_OSPEEDR 				((uint32_t)0x00000003)
#define BM_PUPDR				((uint32_t)0x00000003)
#define BM_AFR					((uint32_t)0x00000007)

#define IS_OTYPE(OTYPE)			((OTYPE == CFG_PUSHPULL) \
								|| (OTYPE == CFG_OPENDRAIN))
#define IS_PUPD(PUPD)			((PUPD == CFG_PULLNO) || (PUPD == CFG_PULLUP) \
									|| (PUPD == CFG_PULLDOWN))
#define IS_OSPEED(OSPEED)		((OSPEED == CFG_SPEEDLOW) \
								|| (OSPEED == CFG_SPEEDMID) \
								|| (OSPEED == CFG_SPEEDFAST) \
								|| (OSPEED == CFG_SPEEDHIGH))
#define IS_PIN(PIN)				( (PIN & (uint32_t)~0xF) == (uint32_t)0x00)


/* Typedefs =================================================================*/

typedef struct
{
  GPIO_TypeDef *group;
  uint32_t pin;    /*!< GPIO port mode register,               Address offset: 0x00      */
}AgTechIOStruct;

typedef struct
{
  ADC_HandleTypeDef *AdcHandle;
  uint32_t channel;    /*!< GPIO port mode register,               Address offset: 0x00      */
}AgTechAdcStruct;
/* Function Prototypes ======================================================*/
void gpioReset(GPIO_TypeDef *gpio);
void gpioConfig(AgTechIOStruct *gpio, uint32_t otype, uint32_t pupd, uint32_t ospeed);
void gpioAltFuncSelect(AgTechIOStruct *gpio, uint32_t function);
void gpioMode(AgTechIOStruct *gpio, uint32_t mode);
void gpioDigitalWrite(AgTechIOStruct *gpio, bool state);
bool gpioDigitalRead(AgTechIOStruct *gpio);
void readWriteTest(AgTechIOStruct *gpio_in,AgTechIOStruct *gpio_out, uint32_t delay);
void gpioToggle(AgTechIOStruct *gpio,uint32_t udelay);
float gpioAnalogRead(AgTechAdcStruct *ADCHandlePtr);
void gpioAttachInt(AgTechIOStruct *gpio, uint32_t interrupt_type);
void gpioDettachInt(AgTechIOStruct *gpio);
void gpioDigitalSet(GPIO_TypeDef *gpio, uint32_t pin);
void gpioDigitalReset(GPIO_TypeDef *gpio, uint32_t pin);
void gpioDigitalFWrite (GPIO_TypeDef *gpio, uint32_t pin, bool state);
//void gpioFMode(AgTechIOStruct *gpio, uint32_t bitpos1, uint32_t mode);
//bool gpioDigitalFRead(AgTechIOStruct *gpio);
#endif
/******************************************************************************
  * Revision History
  *	@file      	gpio.h
  *****************************************************************************
  * @version    v2.2.0
  * @date       04/08/16
  * @author     D.Lasdoce
  * @changes	
  ********************************************************************************
  * @version    v2.1
  * @date       
  * @author     D.Lasdoce
  * @changes	- created file
  *****************************************************************************
 */

