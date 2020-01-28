/**
  ******************************************************************************
  * @file      	gpio.c
  * @author     Hardware Team
  * @version    v2.2.0
  * @date       04/08/16
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


/* GPIO REGISTER DETAILS
 * y = [0:15]	-stands for register bits as well as pin number

 * MODER;    GPIO port mode register,               Address offset: 0x00
  	  	  	  	--->Bits2y:2y+1 MODERy[1:0] R/W
  	  	  	  		00: Input (reset state)
					01: General purpose output mode
					10: Alternate function mode
					11: Analog mode
  	  	  	  	--->Reset values:
					0xA800 0000 for port A
					0x0000 0280 for port B
					0x0000 0000 for other ports

 * OTYPER;   GPIO port output type register,        Address offset: 0x04
  	  	  	    --->Bits15:0 OTYPERy R/W
  	  	  	    	0: Output push-pull (reset state)
  	  	  	    	1: Output open-drain
  	  	  	    --->Reset values:
  	  	  	    	0x0000 0000

 * OSPEEDR;  GPIO port output speed register,       Address offset: 0x08
  	  	  	    --->Bits2y:2y+1 OSPEEDRy[1:0] R/W
  	  	  	    	00: Low speed
					01: Medium speed
					10: High speed
					11: Very high speed
  	  	  	    --->Reset values:
  	  	  	    	0x0C00 0000 for port A
  	  	  	    	0x0000 00C0 for port B
  	  	  	    	0x0000 0000 for other ports

 * PUPDR;    GPIO port pull-up/pull-down register,  Address offset: 0x0C
  	  	  	  	--->Bits2y:2y+1 PUPDRy[1:0]
  	  	  	  		00: No pull-up, pull-down
  	  	  	  		01: Pull-up
					10: Pull-down
					11: Reserved
				--->Reset values:
					0x6400 0000 for port A
					0x0000 0100 for port B
					0x0000 0000 for other ports

 * IDR;      GPIO port input data register,         Address offset: 0x10
  	  	  	  	--->Bits 15:0 IDRy R
  	  	  	  	--->Reset values:
  	  	  	    	0x0000 XXXX

 * ODR;      GPIO port output data register,        Address offset: 0x14
				--->Bits 15:0 ODRy R/W
				--->Reset values:
  	  	  	    	0x0000 0000

 * BSRR;     GPIO port bit set/reset register,      Address offset: 0x18
  	  	  	  	--->Bits31:16 BRy
  	  	  	  		0: No action on the corresponding ODRy bit
					1: Sets the corresponding ODRy bit (ODRy = 1)
  	  	  	  	--->Bits15:0 BSy
  	  	  	  		0: No action on the corresponding ODRy bit
					1: Resets the corresponding ODRy bit (ODRy = 0)
				--->Reset values:
  	  	  	    	0x0000 0000

 * LCKR;     GPIO port configuration lock register, Address offset: 0x1C
 				--->Reset values:
  	  	  	    		0x0000 0000

 * AFR[2];   GPIO alternate function registers,     Address offset: 0x20-0x24
 	  			--->Reset values:
  	  	  	    	0x0000 0000
*/

/* Defines =================================================================*/
#include <math.h>

#include "agBoardInit.h"
#include "delay.h"
#include "gpio.h"
#include <stm32f7xx_hal_conf.h>

/* Functions =================================================================*/

/* =============================================================================
 * @brief	reset the states of gpio
 * @param	gpio: pointer to GPIO_TypeDef that will be reset
 * ===========================================================================*/
//void gpioReset(GPIO_TypeDef *gpio)
//{
//	if (gpio == GPIOF)
//	{
//		gpio->PUPDR &= 0x00000000;
//		gpio->OTYPER &= 0x00000000;
//		gpio->MODER &= 0x00000000;
//	}
//	gpio->AFR[0] &= 0x00000000;
//	gpio->AFR[1] &= 0x00000000;
//}

/* ============================================================================
 * @brief	sets the gpio settings
 * @param	gpio: 	pointer to GPIO_TypeDef that will be configured
 * @param	otype: 	PUSHPULL/OPENDRAIN
 * @param	pupd: 	UP/DOWN/NONE
 * @param	ospeed	determines the maximum frequncy of the gpio
 * ==========================================================================*/
void gpioConfig(AgTechIOStruct *gpio, uint32_t otype, uint32_t pupd, uint32_t ospeed)
{
//	assert_param(IS_PIN(gpio->pin));
//	assert_param(IS_GPIO_ALL_INSTANCE(gpio->group));
//	assert_param(IS_OTYPE(otype));
//	assert_param(IS_PUPD(pupd));
//	assert_param(IS_OSPEED(ospeed));

	uint32_t bitpos = gpio->pin;
	uint32_t temp = 0x00000000;

	/*
	 * Set if PUPD or Open Drain
	 */
	temp = gpio->group->OTYPER;
	temp &= ~(BM_OTYPER << bitpos);
	temp |= otype << bitpos;
	gpio->group->OTYPER = temp;

	/*
	 * Set if PullUp or PullDown
	 */
	temp = gpio->group->PUPDR;
	temp &= ~(BM_PUPDR << (bitpos * 2));
	temp |= pupd << (bitpos * 2);
	gpio->group->PUPDR = temp;

	/*
	 * Set Pin Speed
	 */
	temp = gpio->group->OSPEEDR;
	temp &= ~(BM_OSPEEDR << (bitpos * 2));
	temp |= ospeed << (bitpos * 2);
	gpio->group->OSPEEDR = temp;
}

/* ===========================================================================
 * @brief	configures the alternate function of the gpio
 * @param	gpio: pointer to pin to be configured
 * @param	function: alternate function selected
 * ==========================================================================*/
void gpioAltFuncSelect(AgTechIOStruct *gpio, uint32_t function)
{
	/*AFR[0] for pins 0-7 and AFR[1] is for pins 8-15*/
	uint32_t temp = gpio->group->AFR[gpio->pin >> 3];
	temp |= function << (gpio->pin & (uint32_t)0x07) * 4;
	gpio->group->AFR[gpio->pin >> 3] = temp;

}

/* ============================================================================
 * @brief sets the gpio as digital input/output or analog output
 * @param	gpio: pointer to pin to be configured
 * @param	mode: determines if input or output
 * ==========================================================================*/
void gpioMode(AgTechIOStruct *gpio, uint32_t mode)
{
	uint32_t temp = 0x00000000;
	uint32_t bitpos = gpio->pin;
	temp = gpio->group->MODER;
	temp &= ~(BM_MODER << (bitpos * 2));
	temp |= mode << (bitpos * 2);
	gpio->group->MODER  = temp;
}

//void gpioFMode(AgTechIOStruct *gpio, uint32_t bitpos1, uint32_t mode)
//{
//	uint32_t temp = 0x00000000;
//	uint32_t bitpos = 8;
//	temp = gpio->group->MODER;
//	temp &= ~(BM_MODER << (bitpos * 2));
//	temp |= mode << (bitpos * 2);
//	gpio->group->MODER  = temp;
//}


/* ============================================================================
 * @brief 	sets the gpio HIGH or LOW
 * @param	gpio: pointer to pin to be configured
 * @param	state: tells wether high or low
 * ==========================================================================*/
void gpioDigitalWrite(AgTechIOStruct *gpio, bool state)
{
	//assert_param(IS_PIN(gpio->pin));
	if (state)
		gpio->group->BSRR = BM_BSSR << gpio->pin;
	else
		gpio->group->BSRR = (BM_BSSR << (gpio->pin))<<16;
}

inline void gpioDigitalSet(GPIO_TypeDef *gpio, uint32_t pin)
{
	//assert_param(IS_PIN(gpio->pin));
  	gpio->BSRR = pin;
	//gpio->BSRR = 0x02;
}

inline void gpioDigitalReset(GPIO_TypeDef *gpio, uint32_t pin)
{
	//assert_param(IS_PIN(gpio->pin));
	gpio->BSRR = pin<<16;
}

#pragma optimize=speed
void gpioDigitalFWrite(GPIO_TypeDef *gpio, uint32_t pin, bool state)
{
	//assert_param(IS_PIN(gpio->pin));
  	if(state)
	  gpio->BSRR = pin;
	else
		gpio->BSRR = pin<<16;
}


/* ============================================================================
 * @brief	enables the interrupt line on the gpio
 * @param	gpio: pointer to pin to be configured
 * @param	interrupt_type: tells whether rising or falling detection
 * ==========================================================================*/
void gpioAttachInt(AgTechIOStruct *gpio, uint32_t interrupt_type)
{
	uint32_t temp = 0x00;
	uint32_t position = gpio->pin;
	//uint32_t bitpos = 1<<position;
	uint32_t iocurrent = 1<<position;

    /*--------------------- EXTI Mode Configuration ------------------------*/
    /* Configure the External Interrupt or event for the current IO */
	 if((interrupt_type & EXTI_MODE) == EXTI_MODE)
	 {
		/* Enable SYSCFG Clock */
		__HAL_RCC_SYSCFG_CLK_ENABLE();

		/*Clear external interrupt register*/
		temp = SYSCFG->EXTICR[position >> 2];
		temp &= ~(((uint32_t)0x0F) << (4 * (position & 0x03)));
		temp |= ((uint32_t)(GPIO_GET_INDEX(gpio->group)) <<
				(4 * (position & 0x03)));
		SYSCFG->EXTICR[position >> 2] = temp;

		/* Clear EXTI line configuration */
		temp = EXTI->IMR;
		temp &= ~((uint32_t)iocurrent);
		if((interrupt_type & GPIO_MODE_IT) == GPIO_MODE_IT)
		{
			temp |= iocurrent;
		}
		EXTI->IMR = temp;

		temp = EXTI->EMR;
		temp &= ~((uint32_t)iocurrent);
		if((interrupt_type & GPIO_MODE_EVT) == GPIO_MODE_EVT)
		{
			temp |= iocurrent;
		}
		EXTI->EMR = temp;

		/* Clear Rising Falling edge configuration */
		temp = EXTI->RTSR;
		temp &= ~((uint32_t)iocurrent);
		if((interrupt_type & RISING_EDGE) == RISING_EDGE)
		{
			temp |= iocurrent;
		}
		EXTI->RTSR = temp;

		temp = EXTI->FTSR;
		temp &= ~((uint32_t)iocurrent);
		if((interrupt_type & FALLING_EDGE) == FALLING_EDGE)
		{
			temp |= iocurrent;
		}
		EXTI->FTSR = temp;
	}
}

/* ============================================================================
 * @brief	remove interrupt on gpio pin
 * @param	gpio: pointer to pin to be configured
 * ==========================================================================*/
void gpioDettachInt(AgTechIOStruct *gpio)
{
	uint32_t iocurrent = 1<<gpio->pin;
	uint32_t temp = EXTI->IMR;
	temp &= ~((uint32_t)iocurrent);
	EXTI->IMR = temp;

}

/* ============================================================================
 * @brief	reads the current digital state of the gpio
 * @param	gpio: pointer to pin to be configured
 * ==========================================================================*/
bool gpioDigitalRead(AgTechIOStruct *gpio)
{
	//assert_param(IS_PIN(gpio->pin));
	if ( gpio->group->IDR & ((uint32_t)0x00000001<<gpio->pin) )
		return 1;
	else
		return 0;
}

//bool gpioDigitalFRead(AgTechIOStruct *gpio)
//{
//	//assert_param(IS_PIN(gpio->pin));
//	if ( gpio->group->IDR & gpio->pin )
//		return 1;
//	else
//		return 0;
//}

/* ============================================================================
 * @brief	reads the current analog value of the gpio
 * @param	gpio: pointer to pin to be configured
 * ==========================================================================*/
float gpioAnalogRead(AgTechAdcStruct *ADCHandlePtr)
{
	uint32_t adc_bits = 0;
	float adc_voltage;
	uint8_t i;
	uint8_t udelay = 100;
	ADC_ChannelConfTypeDef ADCChConDef;
	//ADC_HandleTypeDef *ADCHandlePtr;
	__HAL_RCC_ADC_FORCE_RESET();
	delayMillis(100);
	__HAL_RCC_ADC_RELEASE_RESET();

	ADCChConDef.Channel = ADCHandlePtr->channel;
	ADCChConDef.Offset = 0;
	ADCChConDef.Rank = 1;
	ADCChConDef.SamplingTime = ADC_SAMPLETIME_480CYCLES;

	HAL_ADC_ConfigChannel(ADCHandlePtr->AdcHandle, &ADCChConDef);
	HAL_ADC_Start(ADCHandlePtr->AdcHandle);
	HAL_ADC_PollForConversion(ADCHandlePtr->AdcHandle, 20);
	
	for(i = 0; i < ADC_READSAMPLES; i++)
	{
		adc_bits += HAL_ADC_GetValue(ADCHandlePtr->AdcHandle);
		delayMicros(udelay);
	}
	HAL_Delay(500);
	HAL_ADC_Stop(ADCHandlePtr->AdcHandle);
	adc_voltage = ((float)adc_bits * ADC_REF)/ (4095 * ADC_READSAMPLES);
	adc_voltage = roundf(adc_voltage*10000)/10000;
	return adc_voltage;
}

/*============================================================================
* @brief    
* @param
* @retval 
============================================================================*/
void gpioToggle(AgTechIOStruct *gpio,uint32_t udelay)
{
	while (1)
	{
		gpioDigitalWrite(gpio, 0);
		delayMicros(udelay);
		gpioDigitalWrite(gpio, 1);
		delayMicros(udelay);
	}
}


/******************************************************************************
  * Revision History
  *	@file      	gpio.c
  *****************************************************************************
  * @version    v2.2.0
  * @date       04/08/16
  * @author     D.Lasdoce
  * @changes
  ********************************************************************************
  * @version    v2.1.0
  * @date       12/23/2015
  * @author     D.Lasdoce
  * @changes    created file
  *****************************************************************************
 */
