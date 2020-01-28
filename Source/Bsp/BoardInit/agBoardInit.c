/**
  ******************************************************************************
  * @file      	agBoardInit.c
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

/* Includes =================================================================*/
#include <limits.h>
#include <stdarg.h>
#include "agBoardInit.h"
#include "delay.h"
#include "productCfg.h"
#include "rtc.h"
#include "pexpander.h"
#include "isrMain.h"
#include "sdProcess.h"
#include "bsp_pwm.h"
#include <deployAssert.h>
#include "i2c.h"

/* Global Declarations ======================================================*/
static __IO uint32_t millisTick;
static __IO uint32_t usTick;
int xyz;
/* External Declarations =====================================================*/

UART_HandleTypeDef UART1HandlerDef;
UART_HandleTypeDef UART2HandlerDef;
UART_HandleTypeDef UART3HandlerDef;
UART_HandleTypeDef UART4HandlerDef;
UART_HandleTypeDef UART5HandlerDef;
UART_HandleTypeDef UART6HandlerDef;
UART_HandleTypeDef UART7HandlerDef;
UART_HandleTypeDef UART8HandlerDef;
I2C_HandleTypeDef I2C1HandleDef;
I2C_HandleTypeDef I2C2HandleDef;
ADC_HandleTypeDef ADC3HandlerDef, ADC1HandlerDef;
SPI_HandleTypeDef SPIPwrMonitorHandle;

/* Function Prototypes =======================================================*/

static void gpioInit(void);
static void spiInit(void);
static void i2cInit(void);
static void adcInit(void);
static void pwmInit(void);
static __IO uint32_t millisTick;
void LPTIMInit(void);
void lowSpeedClocksInit(void);

/* Functions =================================================================*/


/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void agBoardInit(void)
{
  	lowSpeedClocksInit();
	gpioInit();
	i2cInit();
	uartInit();
	pwmInit();
	spiInit(); 
	extRTCInit();
	adcInit();
}

void lowSpeedClocksInit(void)
{
  	RCC_OscInitTypeDef        RCC_OscInitStruct;
  	RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
  
  	RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{ 
		while(1);
	}
	
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LPTIM1;
	PeriphClkInitStruct.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSI; 
	if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{ 
		while(1);
	}
	
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{ 
		assert(0);
	}
	
  	/* Configures the External Low Speed oscillator (LSE) drive capability */
  	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH); 
}

void LPTIMInit(void)
{
  	uint32_t LPTIM_PERIOD = 500;
	uint32_t LPTIM_TIMEOUT = 250;

	LptimHandle.Instance = LPTIM1;
	LptimHandle.Init.Clock.Source       = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
	LptimHandle.Init.Clock.Prescaler    = LPTIM_PRESCALER_DIV128;  
	LptimHandle.Init.Trigger.Source     = LPTIM_TRIGSOURCE_SOFTWARE;
	LptimHandle.Init.Trigger.ActiveEdge = LPTIM_ACTIVEEDGE_RISING;
	LptimHandle.Init.CounterSource      = LPTIM_COUNTERSOURCE_INTERNAL;
	
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

static void pwmInit(void)
{
	gpioConfig(&VCON1, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDHIGH);
	gpioMode(&VCON1, MODE_ALTERNATE);
	gpioAltFuncSelect(&VCON1, AF_TIM1_2);
	
	gpioConfig(&VCON2, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDHIGH);
	gpioMode(&VCON2, MODE_ALTERNATE);
	gpioAltFuncSelect(&VCON2, AF_TIM8_9_10_11);
	
	gpioConfig(&VCON3, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDHIGH);
	gpioMode(&VCON3, MODE_ALTERNATE);
	gpioAltFuncSelect(&VCON3, AF_TIM3_4_5);
	
	gpioConfig(&VCON4, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDHIGH);
	gpioMode(&VCON4, MODE_ALTERNATE);
	gpioAltFuncSelect(&VCON4, AF_TIM1_2);
	
	pwmParamSetUp();
}

static void spiInit(void)
{
  	/*SPI2(FLASH and RTC) INIT*/
	gpioDigitalWrite(&MSP430_CS_PIN, HIGH);
	gpioDigitalWrite(&MSP430_SPI_COMING, HIGH);

  	gpioConfig(&FLM25_CS_PIN, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDFAST);
	gpioConfig(&MSP430_CS_PIN, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDFAST);
    gpioConfig(&MSP430_SPI_COMING, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDFAST);

    gpioConfig(&SPI2_CLK_PIN, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDMID);
    gpioConfig(&SPI2_MISO_PIN, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDMID);
    gpioConfig(&SPI2_MOSI_PIN, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDMID);
	gpioConfig(&RTC_ENB, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDMID);
	
	
    gpioMode(&SPI2_CLK_PIN, MODE_ALTERNATE);
    gpioMode(&SPI2_MISO_PIN, MODE_ALTERNATE);
    gpioMode(&SPI2_MOSI_PIN, MODE_ALTERNATE);
	
	gpioAltFuncSelect(&SPI2_CLK_PIN, AF_SPI2);
    gpioAltFuncSelect(&SPI2_MISO_PIN, AF_SPI2);
    gpioAltFuncSelect(&SPI2_MOSI_PIN, AF_SPI2);
	
	gpioMode(&FLM25_CS_PIN, MODE_OUTPUT);
	gpioMode(&MSP430_CS_PIN, MODE_OUTPUT);
        gpioMode(&MSP430_SPI_COMING, MODE_OUTPUT);

    gpioMode(&RTC_ENB, MODE_OUTPUT);
	
	gpioDigitalWrite(&FLM25_CS_PIN, HIGH);
	gpioDigitalWrite(&MSP430_CS_PIN, HIGH);
        gpioDigitalWrite(&MSP430_SPI_COMING, HIGH);

	gpioDigitalWrite(&RTC_ENB, HIGH);
	
	__HAL_RCC_SPI2_CLK_ENABLE();
	SPIFlashRTCHandle.Instance = SPI2;
	SPIFlashRTCHandle.Init.Mode = SPI_MODE_MASTER;
	SPIFlashRTCHandle.Init.Direction = SPI_DIRECTION_2LINES;
	SPIFlashRTCHandle.Init.DataSize = SPI_DATASIZE_8BIT;
	SPIFlashRTCHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
	SPIFlashRTCHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
	SPIFlashRTCHandle.Init.NSS = SPI_NSS_HARD_OUTPUT;
	SPIFlashRTCHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	SPIFlashRTCHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
	SPIFlashRTCHandle.Init.TIMode = SPI_TIMODE_DISABLE;
	SPIFlashRTCHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	SPIFlashRTCHandle.Init.CRCPolynomial = 7;

	HAL_SPI_Init(&SPIFlashRTCHandle);
	__HAL_SPI_ENABLE(&SPIFlashRTCHandle);
		

	
	/*SPI4 (ADS1225) INIT*/
	__HAL_RCC_SPI4_CLK_ENABLE();
    gpioConfig(&SPI4_SCK, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDMID);
    gpioConfig(&SPI4_MOSI, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDMID);
    gpioConfig(&SPI4_MISO, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDMID);

    gpioMode(&SPI4_SCK, MODE_ALTERNATE);
    gpioMode(&SPI4_MOSI, MODE_ALTERNATE);
    gpioMode(&SPI4_MISO, MODE_ALTERNATE);
    
    gpioAltFuncSelect(&SPI4_SCK, AF_SPI4);
    gpioAltFuncSelect(&SPI4_MOSI, AF_SPI4);
    gpioAltFuncSelect(&SPI4_MISO, AF_SPI4);
	
	/*SPI3 Init*/
	__HAL_RCC_SPI3_CLK_ENABLE();
    gpioConfig(&SPI3_SCK, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDMID);
    gpioConfig(&SPI3_MOSI, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDMID);
    gpioConfig(&SPI3_MISO, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDMID);
	gpioConfig(&PM_MAIN, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDMID);
	gpioConfig(&PM_SIG1, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDMID);
	gpioConfig(&PM_SIG2, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDMID);
	gpioConfig(&PM_SIG3, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDMID);
	gpioConfig(&PM_SIG4, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDMID);

	
    gpioMode(&SPI3_SCK, MODE_ALTERNATE);
    gpioMode(&SPI3_MOSI, MODE_ALTERNATE);
    gpioMode(&SPI3_MISO, MODE_ALTERNATE);
	gpioMode(&PM_MAIN, MODE_OUTPUT);
	gpioMode(&PM_SIG1, MODE_OUTPUT);
	gpioMode(&PM_SIG2, MODE_OUTPUT);
	gpioMode(&PM_SIG3, MODE_OUTPUT);
	gpioMode(&PM_SIG4, MODE_OUTPUT);
    
    gpioAltFuncSelect(&SPI3_SCK, AF_SPI3);
    gpioAltFuncSelect(&SPI3_MOSI, AF_SPI3_MOSI);
    gpioAltFuncSelect(&SPI3_MISO, AF_SPI3);
	
	gpioDigitalWrite(&PM_MAIN, HIGH);
	gpioDigitalWrite(&PM_SIG1, HIGH);
	gpioDigitalWrite(&PM_SIG2, HIGH);
	gpioDigitalWrite(&PM_SIG3, HIGH);
	gpioDigitalWrite(&PM_SIG4, HIGH);
	
	SPIPwrMonitorHandle.Instance = SPI3;
	SPIPwrMonitorHandle.Init.Mode = SPI_MODE_MASTER;
	SPIPwrMonitorHandle.Init.Direction = SPI_DIRECTION_2LINES;
	SPIPwrMonitorHandle.Init.DataSize = SPI_DATASIZE_8BIT;
	SPIPwrMonitorHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
	SPIPwrMonitorHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
	SPIPwrMonitorHandle.Init.NSS = SPI_NSS_HARD_OUTPUT;
	SPIPwrMonitorHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	SPIPwrMonitorHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
	SPIPwrMonitorHandle.Init.TIMode = SPI_TIMODE_DISABLE;
	SPIPwrMonitorHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	SPIPwrMonitorHandle.Init.CRCPolynomial = 7;

	HAL_SPI_Init(&SPIPwrMonitorHandle);
	__HAL_SPI_ENABLE(&SPIPwrMonitorHandle);
	
	/*Shutdown All Power Monitors*/
	utilPwrModeCont(&PM_MAIN, false);
	utilPwrModeCont(&PM_SIG1, false);
	utilPwrModeCont(&PM_SIG2, false);
	utilPwrModeCont(&PM_SIG3, false);
	utilPwrModeCont(&PM_SIG4, false);

	
}

void sdInitHardware(void)
{
    gpioConfig(&SD_CMD, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDHIGH);
  	gpioConfig(&SD_D0, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDHIGH);
    gpioConfig(&SD_D1, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDHIGH);
    gpioConfig(&SD_D2, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDHIGH);
    gpioConfig(&SD_D3, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDHIGH);
    gpioConfig(&SD_CLK, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDHIGH);
    gpioConfig(&SD_EN, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDHIGH);
	
    gpioMode(&SD_CMD, MODE_ALTERNATE);
	gpioMode(&SD_D0, MODE_ALTERNATE);
    gpioMode(&SD_D1, MODE_ALTERNATE);
    gpioMode(&SD_D2, MODE_ALTERNATE);
    gpioMode(&SD_D3, MODE_ALTERNATE);
    gpioMode(&SD_CLK, MODE_ALTERNATE);
	gpioMode(&SD_EN, MODE_OUTPUT);
    gpioDigitalWrite(&SD_EN, HIGH);
	
    gpioAltFuncSelect(&SD_CMD, AF_SDIO_D0_D1_D3);
    gpioAltFuncSelect(&SD_D0, AF_SDIO_D0_D1_D3);
    gpioAltFuncSelect(&SD_D1, AF_SDIO_D0_D1_D3);
    gpioAltFuncSelect(&SD_D2, AF_SDIO_D2);
    gpioAltFuncSelect(&SD_D3, AF_SDIO_D0_D1_D3);
    gpioAltFuncSelect(&SD_CLK, AF_SDIO_D0_D1_D3);
}

/*============================================================================
 * @brief
 * @param
 * @retval
============================================================================*/
//static void pexpanderInit(void)
//{
//	pexpanderMode(RS485_RE_ENB, 0, I2CADD_PEXPANDER1);
//	pexpanderMode(RS485_DE_EN, 0, I2CADD_PEXPANDER1);
//	pexpanderMode(RS485_RES_EN, 0, I2CADD_PEXPANDER1);
//	pexpanderMode(RS232_EN, 0, I2CADD_PEXPANDER1);
//
//	pexpanderWrite(RS485_RE_ENB, HIGH, I2CADD_PEXPANDER1);
//	pexpanderWrite(RS485_DE_EN, LOW, I2CADD_PEXPANDER1);
//	pexpanderWrite(RS485_RES_EN, LOW, I2CADD_PEXPANDER1);
//	pexpanderWrite(RS232_EN, LOW, I2CADD_PEXPANDER1);
//
//}
/*============================================================================
 * @brief	Enable RTC
 * @param
 * @retval  
============================================================================*/
void rtcInit(void)
{
  	bool use_bkuptime = true;
	RTCHandleStruct.Instance = RTC;
	RTCHandleStruct.Init.AsynchPrediv = RTC_APRES;
	RTCHandleStruct.Init.HourFormat = RTC_HOURFORMAT_24;
	RTCHandleStruct.Init.OutPut = RTC_OUTPUT_DISABLE;
	RTCHandleStruct.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	RTCHandleStruct.Init.SynchPrediv = RTC_SPRES;
	RTCHandleStruct.Init.OutPutType = RTC_OUTPUT_TYPE_PUSHPULL;

	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_EnableBkUpAccess();

	HAL_RTC_Init(&RTCHandleStruct);
	__HAL_RCC_RTC_ENABLE();

	RTCDateTimeStruct RTC_DateTimeStructSetup;
	exRTCReadTime(&RTC_DateTimeStructSetup);
	
	if(!IS_RTC_YEAR(RTC_DateTimeStructSetup.Year))
	  	 use_bkuptime = false;
	
	if(!IS_RTC_MONTH(RTC_DateTimeStructSetup.Month))
	  		use_bkuptime = false;

	if(!IS_RTC_DATE(RTC_DateTimeStructSetup.Date))
		use_bkuptime = false;
	
	if(!IS_RTC_HOUR24(RTC_DateTimeStructSetup.Hours))
		use_bkuptime = false;
	
	if(!IS_RTC_MINUTES(RTC_DateTimeStructSetup.Minutes))
		use_bkuptime = false;
	
	if(!IS_RTC_SECONDS(RTC_DateTimeStructSetup.Seconds))
		use_bkuptime = false;
	
	if(use_bkuptime == false)
	{
		RTC_DateTimeStructSetup.Year	= DEFAULT_YEAR;
		RTC_DateTimeStructSetup.Month = DEFAULT_MONTH;
		RTC_DateTimeStructSetup.Date = DEFAULT_DATE;
		RTC_DateTimeStructSetup.Hours = DEFAULT_HOURS;
		RTC_DateTimeStructSetup.Minutes = DEFAULT_MINUTES;
		RTC_DateTimeStructSetup.Seconds = DEFAULT_SECONDS;
	}
	
	rtcSetDateTime(&RTC_DateTimeStructSetup);
	exRTCWriteTime(&RTC_DateTimeStructSetup);
}

void agBoardSyncRTCs()
{
  	#if PRINT_PROCESSES_IN_UART
  		logWrite("\nDATE and TIME: Syncing RTCs\n");
	#endif
	RTCDateTimeStruct sDateTime;
	rtcGetDateTime(&sDateTime);
	delayMillis(100);
  	rtcSetDateTime(&sDateTime);
	delayMillis(100);
	exRTCWriteTime(&sDateTime);
	delayMillis(1000);
}

/*============================================================================
 * @brief	
 * @param
 * @retval  
============================================================================*/
static void i2cInit(void)
{
	__HAL_RCC_I2C2_CLK_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();

	gpioConfig(&I2C1_SDA, CFG_OPENDRAIN, CFG_PULLUP, CFG_SPEEDFAST);
	gpioConfig(&I2C1_SCL, CFG_OPENDRAIN, CFG_PULLUP, CFG_SPEEDFAST);
	gpioConfig(&I2C2_SDA, CFG_OPENDRAIN, CFG_PULLUP, CFG_SPEEDFAST);
	gpioConfig(&I2C2_SCL, CFG_OPENDRAIN, CFG_PULLUP, CFG_SPEEDFAST);

	gpioMode(&I2C1_SDA, MODE_ALTERNATE);
	gpioMode(&I2C1_SCL, MODE_ALTERNATE);
	gpioMode(&I2C2_SDA, MODE_ALTERNATE);
	gpioMode(&I2C2_SCL, MODE_ALTERNATE);

	gpioAltFuncSelect(&I2C1_SCL, AF_I2C);
	gpioAltFuncSelect(&I2C1_SDA, AF_I2C);
	gpioAltFuncSelect(&I2C2_SCL, AF_I2C);
	gpioAltFuncSelect(&I2C2_SDA, AF_I2C);

	gpioConfig(&ACCEL_ENB, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDFAST);
	gpioMode(&ACCEL_ENB, MODE_OUTPUT);
	
	gpioConfig(&ACCEL_IRQ, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST);
	gpioMode(&ACCEL_IRQ, MODE_INPUT);
	
	// abarbaso??? Revisit due to changes in I2C_CR1 register
    
    I2C1HandleDef.Instance = I2C1;
	I2C1HandleDef.Init.Timing = 0x40912732;
	//I2C1HandleDef.Init.DutyCycle = I2C_DUTYCYCLE_2;
	I2C1HandleDef.Init.OwnAddress1 = 0x00;
	I2C1HandleDef.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	I2C1HandleDef.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2C1HandleDef.Init.OwnAddress2 = 0x00;
	I2C1HandleDef.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	I2C1HandleDef.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	HAL_I2C_Init(&I2C1HandleDef);

	I2C2HandleDef.Instance = I2C2;
	I2C2HandleDef.Init.Timing = 0x40912732;
	//I2C2HandleDef.Init.ClockSpeed = 100000;
	//I2C2HandleDef.Init.DutyCycle = I2C_DUTYCYCLE_2;
	I2C2HandleDef.Init.OwnAddress1 = 0x00;
	I2C2HandleDef.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	I2C2HandleDef.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2C2HandleDef.Init.OwnAddress2 = 0x00;
	I2C2HandleDef.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	I2C2HandleDef.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	//HAL_I2C_Init(&I2C2HandleDef);
}

/*============================================================================
 * @brief
 * @param
 * @retval  
============================================================================*/
static void gpioInit(void)
{
	/*clock enabling section ================================================*/
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOJ_CLK_ENABLE();
	__HAL_RCC_GPIOK_CLK_ENABLE();

	/*Port's Digital IO =====================================================*/
	gpioConfig(&DIG1, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&DIG2, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&DIG3, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST );
	gpioConfig(&DIG4, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST );
	gpioMode(&DIG1, MODE_INPUT);
	gpioMode(&DIG2, MODE_INPUT);
	gpioMode(&DIG3, MODE_INPUT);
	gpioMode(&DIG4, MODE_INPUT);
	
	gpioConfig(&VCON1_EN, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&VCON2_EN, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&VCON3_EN, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST );
	gpioConfig(&VCON4_EN, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST );
	gpioMode(&VCON1_EN, MODE_OUTPUT);
	gpioMode(&VCON2_EN, MODE_OUTPUT);
	gpioMode(&VCON3_EN, MODE_OUTPUT);
	gpioMode(&VCON4_EN, MODE_OUTPUT);
	gpioDigitalWrite(&VCON1_EN, LOW);
	gpioDigitalWrite(&VCON2_EN, LOW);
	gpioDigitalWrite(&VCON3_EN, LOW);
	gpioDigitalWrite(&VCON4_EN, LOW);
	
	gpioConfig(&LD_USER1_R, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST );
	gpioMode(&LD_USER1_R, MODE_OUTPUT);
	
	gpioConfig(&LD_USER2_G, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST );
	gpioMode(&LD_USER2_G, MODE_OUTPUT);
	
	gpioConfig(&BUZZ_EN, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST );
	gpioMode(&BUZZ_EN, MODE_OUTPUT);
	
	gpioMode(&RS485_EN, MODE_OUTPUT);
	
	gpioConfig(&TP38, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDFAST );
	gpioMode(&TP38, MODE_OUTPUT);


	/*Wakeup Pins ===========================================================*/
	gpioConfig(&XBEE_STATUS, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST);
	gpioMode(&XBEE_STATUS, MODE_INPUT);
	//gpioAttachInt(&XBEE_STATUS, IT_MODE_RISING);
	
	gpioConfig(&XBEE_IRQ, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST);
	gpioMode(&XBEE_IRQ, MODE_INPUT);
	

	gpioConfig(&BLE_IRQ, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST);
	gpioMode(&BLE_IRQ, MODE_INPUT);
	gpioAttachInt(&BLE_IRQ, IT_MODE_FALLING);

	/*One Wire Mux Bus Selection Pins =======================================*/
	gpioConfig(&SIG1_ID, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST );
	gpioConfig(&SIG2_ID, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST );
	gpioConfig(&SIG3_ID, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST );
	gpioConfig(&SIG4_ID, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST );
	gpioMode(&SIG1_ID, MODE_INPUT);
	gpioMode(&SIG2_ID, MODE_INPUT);
	gpioMode(&SIG3_ID, MODE_INPUT);
	gpioMode(&SIG4_ID, MODE_INPUT);

	/*Level Shifter Pins ====================================================*/
	gpioConfig(&DLS_DIR, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST );
	gpioConfig(&DLS_OE, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDFAST);
	gpioConfig(&DLS_VS, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST);
	gpioMode(&DLS_DIR, MODE_OUTPUT);
	gpioMode(&DLS_OE, MODE_OUTPUT);
	gpioMode(&DLS_VS, MODE_OUTPUT);

	/*Interface Selection Pins ==============================================*/
	gpioConfig(&SIG1_SEL, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&SIG2_SEL, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&SIG3_SEL, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&SIG4_SEL, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioMode(&SIG1_SEL, MODE_OUTPUT);
	gpioMode(&SIG2_SEL, MODE_OUTPUT);
	gpioMode(&SIG3_SEL, MODE_OUTPUT);
	gpioMode(&SIG4_SEL, MODE_OUTPUT);
	
	gpioConfig(&SIG1_SEL0, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&SIG2_SEL0, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&SIG3_SEL0, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&SIG4_SEL0, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioMode(&SIG1_SEL0, MODE_OUTPUT);
	gpioMode(&SIG2_SEL0, MODE_OUTPUT);
	gpioMode(&SIG3_SEL0, MODE_OUTPUT);
	gpioMode(&SIG4_SEL0, MODE_OUTPUT);
	
	gpioConfig(&SIG1_SEL1, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&SIG2_SEL1, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&SIG3_SEL1, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&SIG4_SEL1, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioMode(&SIG1_SEL1, MODE_OUTPUT);
	gpioMode(&SIG2_SEL1, MODE_OUTPUT);
	gpioMode(&SIG3_SEL1, MODE_OUTPUT);
	gpioMode(&SIG4_SEL1, MODE_OUTPUT);
	
	gpioConfig(&RSx_A_EN, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&RSx_A0_SEL, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&RSx_A1_SEL, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioMode(&RSx_A_EN, MODE_OUTPUT);
	gpioMode(&RSx_A0_SEL, MODE_OUTPUT);
	gpioMode(&RSx_A1_SEL, MODE_OUTPUT);
	
	gpioConfig(&RSx_B_EN, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&RSx_B0_SEL, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&RSx_B1_SEL, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioMode(&RSx_B_EN, MODE_OUTPUT);
	gpioMode(&RSx_B0_SEL, MODE_OUTPUT);
	gpioMode(&RSx_B1_SEL, MODE_OUTPUT);
	
	gpioDigitalWrite(&RSx_B_EN, LOW);

	gpioConfig(&RS485_RES_EN, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST);
	gpioConfig(&RS232_ENB, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST);
	gpioConfig(&RS485_FEN, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST);
	gpioConfig(&RS485_DXEN, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST);
	gpioConfig(&RS485_RXENB, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST);
	gpioMode(&RS485_RES_EN, MODE_OUTPUT);
	gpioMode(&RS232_ENB, MODE_OUTPUT);
	gpioMode(&RS485_FEN, MODE_OUTPUT);
	gpioMode(&RS485_DXEN, MODE_OUTPUT);
	gpioMode(&RS485_RXENB, MODE_OUTPUT);
	
	gpioDigitalWrite(&RS485_FEN, LOW);
	gpioDigitalWrite(&RS485_DXEN, LOW);
	gpioDigitalWrite(&RS485_RXENB, HIGH);
	

	/*ADC CS Pins ==============================*/
	gpioConfig(&AD_EN, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST);
	gpioConfig(&AD_CONV, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&AD_CS, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&AD_BUSY, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	
	gpioMode(&AD_EN, MODE_OUTPUT);
	gpioMode(&AD_CONV, MODE_OUTPUT);
	gpioMode(&AD_CS, MODE_OUTPUT);
	gpioMode(&AD_BUSY, MODE_INPUT);
	
	gpioDigitalWrite(&AD_EN, HIGH);
	gpioDigitalWrite(&AD_CONV, LOW);
	gpioDigitalWrite(&AD_CS, HIGH);

	/*Power Enabling Pins ===================================================*/
	gpioConfig(&PWR1_A_EN, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&PWR2_A_EN, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&PWR3_A_EN, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&PWR4_A_EN, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);

	gpioMode(&PWR1_A_EN, MODE_OUTPUT);
	gpioMode(&PWR2_A_EN, MODE_OUTPUT);
	gpioMode(&PWR3_A_EN, MODE_OUTPUT);
	gpioMode(&PWR4_A_EN, MODE_OUTPUT);
	
	gpioDigitalWrite(&PWR1_A_EN, LOW);
	gpioDigitalWrite(&PWR2_A_EN, LOW);
	gpioDigitalWrite(&PWR3_A_EN, LOW);
	gpioDigitalWrite(&PWR4_A_EN, LOW);


	/*Valve Trigger Pins ====================================================*/
	gpioConfig(&VALVE_TRIG1_A, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&VALVE_TRIG1_B, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&VALVE_TRIG2_A, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&VALVE_TRIG2_B, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&VCON15_EN, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioMode(&VALVE_TRIG1_A, MODE_OUTPUT);
	gpioMode(&VALVE_TRIG1_B, MODE_OUTPUT);
	gpioMode(&VALVE_TRIG2_A, MODE_OUTPUT);
	gpioMode(&VALVE_TRIG2_B, MODE_OUTPUT);
	gpioMode(&VCON15_EN, MODE_OUTPUT);
	
	/*Banner Radio Pins =====================================================*/
	gpioConfig(&BANNER_EN, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST);
	gpioConfig(&BANNER_RST, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDFAST);
	gpioConfig(&BNR_CTR, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST);
    gpioConfig(&BNR_ECHO0, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST);
	gpioConfig(&BNR_ECHO1, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST);
    
	gpioMode(&BANNER_EN, MODE_OUTPUT);
	gpioMode(&BANNER_RST, MODE_OUTPUT);
	gpioMode(&BNR_CTR, MODE_ANALOG);
	gpioMode(&BNR_ECHO0, MODE_INPUT);
	gpioMode(&BNR_ECHO1, MODE_INPUT);
    
	gpioDigitalWrite(&BANNER_EN, HIGH);
	gpioDigitalWrite(&BANNER_RST, HIGH);

    /*GPS Pins ==============================================================*/
	gpioConfig(&GPS_ON_OFF, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&GPS_SYSON, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST);
	gpioConfig(&GPS_RESET, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDFAST);

	gpioMode(&GPS_ON_OFF, MODE_OUTPUT);
	gpioMode(&GPS_SYSON, MODE_INPUT);
	gpioMode(&GPS_RESET, MODE_OUTPUT);

	gpioDigitalWrite(&GPS_ON_OFF, LOW);
	gpioDigitalWrite(&GPS_RESET, LOW);

	/*Cellular Power Pins ===================================================*/
	gpioConfig(&CELL_ON_OFF, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&CELL_NRESET, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST);
	gpioConfig(&CELL_RTS, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST);
	gpioMode(&CELL_RTS, MODE_OUTPUT);
	gpioMode(&CELL_ON_OFF, MODE_OUTPUT);
	gpioMode(&CELL_NRESET, MODE_OUTPUT);
	
	gpioDigitalWrite(&CELL_NRESET, LOW);
	gpioDigitalWrite(&CELL_ON_OFF, LOW);

	/*Wifi Power Pins =======================================================*/
	gpioConfig(&WIFI_SLEEP, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioConfig(&WIFI_NRESET, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDFAST);
	gpioConfig(&WIFI_IRQ, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST);
	gpioMode(&WIFI_SLEEP, MODE_OUTPUT);
	gpioMode(&WIFI_NRESET, MODE_OUTPUT);
	gpioMode(&WIFI_IRQ, MODE_INPUT);
	
	gpioDigitalWrite(&WIFI_NRESET, HIGH);
	gpioDigitalWrite(&WIFI_SLEEP, LOW);
	
	/*Misc Pins =============================================================*/
	gpioConfig(&TEMP_EXT, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST);	
	gpioConfig(&CHARGE_OK, CFG_PUSHPULL, CFG_PULLNO, CFG_SPEEDFAST);
	gpioMode(&CHARGE_OK, MODE_INPUT);
}

void uartDeInit(void)
{	
   	/*Cell*/
  	HAL_UART_DeInit(&UART1HandlerDef);
	gpioMode(&CELL_RX, MODE_ANALOG);
	gpioMode(&CELL_TX, MODE_ANALOG);	
	
	/*Banner*/
	HAL_UART_DeInit(&UART2HandlerDef);
	gpioMode(&BANNER_RX, MODE_ANALOG);
	gpioMode(&BANNER_TX, MODE_ANALOG);
	
	/*Xbee*/
	HAL_UART_DeInit(&UART4HandlerDef);
	gpioMode(&XBEE_RX, MODE_ANALOG);
	gpioMode(&XBEE_TX, MODE_ANALOG);
	
	/*UART5*/
	HAL_UART_DeInit(&UART5HandlerDef);
	gpioMode(&UART5_RX, MODE_ANALOG);
	gpioMode(&UART5_TX, MODE_ANALOG);
	
	/*Gps*/
	HAL_UART_DeInit(&UART7HandlerDef);
	gpioMode(&GPS_RX, MODE_ANALOG);
	gpioMode(&GPS_TX, MODE_ANALOG);
	
	/*RS485/232*/
	HAL_UART_DeInit(&UART8HandlerDef);
	gpioMode(&RSX_RX, MODE_ANALOG);
	gpioMode(&RSX_TX, MODE_ANALOG);
}

/*============================================================================
 * @brief
 * @param
 * @retval  
============================================================================*/
void uartInit(void)
{
	gpioConfig(&CELL_RX, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDFAST);
	gpioConfig(&CELL_TX, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDFAST);
	gpioConfig(&BANNER_RX, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDFAST);
	gpioConfig(&BANNER_TX, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDFAST);
	gpioConfig(&BLE_RX, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDFAST);
	gpioConfig(&BLE_TX, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDFAST);
	gpioConfig(&XBEE_RX, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDHIGH);
	gpioConfig(&XBEE_TX, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDFAST);
	gpioConfig(&UART5_RX, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDFAST);
	gpioConfig(&UART5_TX, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDFAST);
	gpioConfig(&WIFI_RX, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDFAST);
	gpioConfig(&WIFI_TX, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDFAST);
	gpioConfig(&GPS_RX, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDFAST);
	gpioConfig(&GPS_TX, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDFAST);
	gpioConfig(&RSX_RX, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDFAST);
	gpioConfig(&RSX_TX, CFG_PUSHPULL, CFG_PULLUP, CFG_SPEEDFAST);

	gpioMode(&CELL_RX, MODE_ALTERNATE);
	gpioMode(&CELL_TX, MODE_ALTERNATE);
	gpioMode(&BANNER_RX, MODE_ALTERNATE);
	gpioMode(&BANNER_TX, MODE_ALTERNATE);
	gpioMode(&BLE_RX, MODE_ALTERNATE);
	gpioMode(&BLE_TX, MODE_ALTERNATE);
	gpioMode(&XBEE_RX, MODE_ALTERNATE);
	gpioMode(&XBEE_TX, MODE_ALTERNATE);
	gpioMode(&UART5_RX, MODE_ALTERNATE);
	gpioMode(&UART5_TX, MODE_ALTERNATE);
	gpioMode(&WIFI_RX, MODE_ALTERNATE);
	gpioMode(&WIFI_TX, MODE_ALTERNATE);
	gpioMode(&GPS_RX, MODE_ALTERNATE);
	gpioMode(&GPS_TX, MODE_ALTERNATE);
	gpioMode(&RSX_RX, MODE_ALTERNATE);
	gpioMode(&RSX_TX, MODE_ALTERNATE);

	gpioAltFuncSelect(&CELL_RX, AF_USART1);
	gpioAltFuncSelect(&CELL_TX, AF_USART1);
	gpioAltFuncSelect(&BANNER_RX, AF_UART1_3);
	gpioAltFuncSelect(&BANNER_TX, AF_UART1_3);
	gpioAltFuncSelect(&BLE_RX, AF_UART7);
	gpioAltFuncSelect(&BLE_TX, AF_UART7);
	//gpioAltFuncSelect(&XBEE_RX, AF_UART4_8);
	gpioAltFuncSelect(&XBEE_TX, AF_UART4_8);
	gpioAltFuncSelect(&UART5_RX, AF_UART4_8);
	gpioAltFuncSelect(&UART5_TX, AF_UART4_8);
	gpioAltFuncSelect(&WIFI_RX, AF_UART4_8);
	gpioAltFuncSelect(&WIFI_TX, AF_UART4_8);
	gpioAltFuncSelect(&GPS_RX, AF_UART4_8);
	gpioAltFuncSelect(&GPS_TX, AF_UART4_8);
	gpioAltFuncSelect(&RSX_RX, AF_UART4_8);
	gpioAltFuncSelect(&RSX_TX, AF_UART4_8);

	/*CELLULAR ==============================================================*/
	__USART1_CLK_ENABLE();
	UART1HandlerDef.Instance = USART1;
	UART1HandlerDef.Init.BaudRate = 115200;
	UART1HandlerDef.Init.WordLength = UART_WORDLENGTH_8B;
	UART1HandlerDef.Init.StopBits = UART_STOPBITS_1;
	UART1HandlerDef.Init.Parity = UART_PARITY_NONE;
	UART1HandlerDef.Init.Mode = UART_MODE_TX_RX;
	UART1HandlerDef.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UART1HandlerDef.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_DeInit(&UART1HandlerDef);
	HAL_UART_Init(&UART1HandlerDef);

	/*BANNER  ===============================================================*/
	__USART2_CLK_ENABLE();
	UART2HandlerDef.Instance = USART2;
	UART2HandlerDef.Init.BaudRate = 19200;
	UART2HandlerDef.Init.WordLength = UART_WORDLENGTH_8B;
	UART2HandlerDef.Init.StopBits = UART_STOPBITS_1;
	UART2HandlerDef.Init.Parity = UART_PARITY_NONE;
	UART2HandlerDef.Init.Mode = UART_MODE_TX_RX;
	UART2HandlerDef.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	HAL_UART_DeInit(&UART2HandlerDef);
	HAL_UART_Init(&UART2HandlerDef);

	/*BLE  ==================================================================*/
	__USART3_CLK_ENABLE();
	UART3HandlerDef.Instance = USART3;
	UART3HandlerDef.Init.BaudRate = 9600;
	UART3HandlerDef.Init.WordLength = UART_WORDLENGTH_8B;
	UART3HandlerDef.Init.StopBits = UART_STOPBITS_1;
	UART3HandlerDef.Init.Parity = UART_PARITY_NONE;
	UART3HandlerDef.Init.Mode = UART_MODE_TX_RX;
	UART3HandlerDef.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	HAL_UART_DeInit(&UART3HandlerDef);
	HAL_UART_Init(&UART3HandlerDef);

	/*XBEE  =================================================================*/
	__UART4_CLK_ENABLE();
	UART4HandlerDef.Instance = UART4;
	UART4HandlerDef.Init.BaudRate = 57600;
	UART4HandlerDef.Init.WordLength = UART_WORDLENGTH_8B;
	UART4HandlerDef.Init.StopBits = UART_STOPBITS_1;
	UART4HandlerDef.Init.Parity = UART_PARITY_NONE;
	UART4HandlerDef.Init.Mode = UART_MODE_TX_RX;
	UART4HandlerDef.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	HAL_UART_DeInit(&UART4HandlerDef);
	HAL_UART_Init(&UART4HandlerDef);

	/*FREE  =================================================================*/
	__UART5_CLK_ENABLE();
	UART5HandlerDef.Instance = UART5;
	UART5HandlerDef.Init.BaudRate = 115200;
	UART5HandlerDef.Init.WordLength = UART_WORDLENGTH_8B;
	UART5HandlerDef.Init.StopBits = UART_STOPBITS_1;
	UART5HandlerDef.Init.Parity = UART_PARITY_NONE;
	UART5HandlerDef.Init.Mode = UART_MODE_TX_RX;
	UART5HandlerDef.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	HAL_UART_DeInit(&UART5HandlerDef);
	HAL_UART_Init(&UART5HandlerDef);

	/*WIFI  =================================================================*/
//	__USART6_CLK_ENABLE();
//	UART6HandlerDef.Instance = USART6;
//	UART6HandlerDef.Init.BaudRate = 115200;
//	UART6HandlerDef.Init.WordLength = UART_WORDLENGTH_8B;
//	UART6HandlerDef.Init.StopBits = UART_STOPBITS_1;
//	UART6HandlerDef.Init.Parity = UART_PARITY_NONE;
//	UART6HandlerDef.Init.Mode = UART_MODE_TX_RX;
//	UART6HandlerDef.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//	UART6HandlerDef.Init.OverSampling = UART_OVERSAMPLING_16;
//	HAL_UART_DeInit(&UART6HandlerDef);
//	HAL_UART_Init(&UART6HandlerDef);

	/*GPS  ==================================================================*/
	__UART7_CLK_ENABLE();
	UART7HandlerDef.Instance = UART7;
	UART7HandlerDef.Init.BaudRate = 4800;
	UART7HandlerDef.Init.WordLength = UART_WORDLENGTH_8B;
	UART7HandlerDef.Init.StopBits = UART_STOPBITS_1;
	UART7HandlerDef.Init.Parity = UART_PARITY_NONE;
	UART7HandlerDef.Init.Mode = UART_MODE_RX;
	UART7HandlerDef.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UART7HandlerDef.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	HAL_UART_DeInit(&UART7HandlerDef);
	HAL_UART_Init(&UART7HandlerDef);

	/*RS485  ================================================================*/
	__UART8_CLK_ENABLE();
	UART8HandlerDef.Instance = UART8;
	UART8HandlerDef.Init.BaudRate = 4800;
	UART8HandlerDef.Init.WordLength = UART_WORDLENGTH_8B;
	UART8HandlerDef.Init.StopBits = UART_STOPBITS_1;
	UART8HandlerDef.Init.Parity = UART_PARITY_NONE;
	UART8HandlerDef.Init.Mode = UART_MODE_TX_RX;
	UART8HandlerDef.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UART7HandlerDef.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	HAL_UART_DeInit(&UART8HandlerDef);
	HAL_UART_Init(&UART8HandlerDef);
}

static void adcInit(void)
{
	gpioConfig(&SIG1_0_5, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioMode(&SIG1_0_5, MODE_ANALOG);

	gpioConfig(&SIG2_0_5, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioMode(&SIG2_0_5, MODE_ANALOG);

	gpioConfig(&SIG3_0_5, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioMode(&SIG3_0_5, MODE_ANALOG);

	gpioConfig(&SIG4_0_5, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioMode(&SIG4_0_5, MODE_ANALOG);

	gpioConfig(&THERM1, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioMode(&THERM1, MODE_ANALOG);
	
	gpioConfig(&THERM2, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioMode(&THERM2, MODE_ANALOG);
	
	gpioConfig(&THERM3, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioMode(&THERM3, MODE_ANALOG);
	
	gpioConfig(&THERM4, CFG_PUSHPULL, CFG_PULLDOWN, CFG_SPEEDFAST);
	gpioMode(&THERM4, MODE_ANALOG);
	
	ADC3HandlerDef.Instance = ADC3;
	ADC3HandlerDef.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	ADC3HandlerDef.Init.Resolution = ADC_RESOLUTION_12B;
	ADC3HandlerDef.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	ADC3HandlerDef.Init.ScanConvMode = DISABLE;
	ADC3HandlerDef.Init.EOCSelection = DISABLE;
	ADC3HandlerDef.Init.ContinuousConvMode = ENABLE;
	ADC3HandlerDef.Init.DMAContinuousRequests = DISABLE;
	ADC3HandlerDef.Init.NbrOfConversion = 1;
	ADC3HandlerDef.Init.DiscontinuousConvMode = DISABLE;
	ADC3HandlerDef.Init.NbrOfDiscConversion = 0;
	ADC3HandlerDef.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	ADC3HandlerDef.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	HAL_ADC_Init(&ADC3HandlerDef);

	ADC1HandlerDef.Instance = ADC1;
	ADC1HandlerDef.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	ADC1HandlerDef.Init.Resolution = ADC_RESOLUTION_12B;
	ADC1HandlerDef.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	ADC1HandlerDef.Init.ScanConvMode = DISABLE;
	ADC1HandlerDef.Init.EOCSelection = DISABLE;
	ADC1HandlerDef.Init.ContinuousConvMode = ENABLE;
	ADC1HandlerDef.Init.DMAContinuousRequests = DISABLE;
	ADC1HandlerDef.Init.NbrOfConversion = 1;
	ADC1HandlerDef.Init.DiscontinuousConvMode = DISABLE;
	ADC1HandlerDef.Init.NbrOfDiscConversion = 0;
	ADC1HandlerDef.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	ADC1HandlerDef.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	HAL_ADC_Init(&ADC1HandlerDef);
	__HAL_RCC_ADC3_CLK_ENABLE();
	__HAL_RCC_ADC1_CLK_ENABLE();
	
	adcSpiInit();
}

/*============================================================================
 * @brief
 * @param
 * @retval
============================================================================*/
void irqConfig(void)
{
	/*DAVIS DTH SCK*/
	HAL_NVIC_SetPriority((IRQn_Type)DAVIS_DTH_SCK_IRQn, 0x03, 0);
	HAL_NVIC_EnableIRQ((IRQn_Type)DAVIS_DTH_SCK_IRQn);

	/*BLE*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0x03, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/*RTC Alarm*/
	HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0x02, 0);
	HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);

	/*WDOG Kicker*/
	HAL_NVIC_SetPriority((IRQn_Type)TIM5_IRQn, 0x02, 0x00);
	
	/*RTC WAKEUP*/
	HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0x02, 0);
	#if RTC_BASED_CYCLE
		HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
	#endif

	/*WIFI UART*/
	HAL_NVIC_SetPriority(WIFI_IRQn, 0x0D, 0);
	//HAL_NVIC_EnableIRQ(WIFI_IRQn);

	#if DEVICE_TYPE == DEVICETYPE_NODE
		/*BANNER UART*/
		#if defined(BANNER_RADIO)
			HAL_NVIC_SetPriority(USART2_IRQn, 0x04, 0);
			HAL_NVIC_EnableIRQ(USART2_IRQn);
		#else

		/*XBEE UART*/
		HAL_NVIC_SetPriority(UART4_IRQn, 0x0D, 0);
		//HAL_NVIC_EnableIRQ(UART4_IRQn);
		#endif
	#endif

	/*Davis*/
	HAL_NVIC_SetPriority((IRQn_Type)RS485_IRQn, 0x0C, 0);
	HAL_NVIC_EnableIRQ((IRQn_Type)RS485_IRQn);

	/*DAQ Port 1*/
	HAL_NVIC_SetPriority((IRQn_Type)TIM1_BRK_TIM9_IRQn, 0x01, 0xF);
	//HAL_NVIC_EnableIRQ((IRQn_Type)TIM1_BRK_TIM9_IRQn);
	
	HAL_NVIC_SetPriority((IRQn_Type)TIM2_IRQn, 0x0A, 0xF);
	//HAL_NVIC_EnableIRQ((IRQn_Type)TIM2_IRQn);

	/*DAQ Port 2*/
	HAL_NVIC_SetPriority((IRQn_Type)DAQTIM_P2_IRQn, 0x0F, 0xF);

	/*Watchdog Kicker*/
	HAL_NVIC_SetPriority((IRQn_Type)TIM4_IRQn, 0x04, 0x00);

	/*DAQ Port 4*/
	//HAL_NVIC_SetPriority((IRQn_Type)DAQTIM_P4_IRQn, 0x0F, 0xF);
	
 	/*Cloud Posting*/
  	#if DEVICE_TYPE == DEVICETYPE_GATE
  		HAL_NVIC_SetPriority((IRQn_Type)RADIO_CLOUD_IRQn, 0x0F, 0xF);
  		#if !RTC_BASED_CYCLE
  			//HAL_NVIC_EnableIRQ((IRQn_Type)RADIO_CLOUD_IRQn);
  		#endif
	#endif

  	/*system_status*/
  	HAL_NVIC_SetPriority((IRQn_Type)SYSSTATUS_IRQn, 0x0F, 0xF);
  	//HAL_NVIC_EnableIRQ((IRQn_Type)SYSSTATUS_IRQn);

  	/*XBEE_STATUS*/
	#ifdef EXTI_WAKEUP
		HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
		//HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	#endif

//	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
//	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    
      /* NVIC configuration for SDIO interrupts */
      HAL_NVIC_SetPriority(SDMMC2_IRQn, 3, 0);
      //HAL_NVIC_EnableIRQ(SDMMC2_IRQn);
      
        /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(SD_DMAx_Rx_IRQn, 2, 0);
  //HAL_NVIC_EnableIRQ(SD_DMAx_Rx_IRQn);
  
  /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(SD_DMAx_Tx_IRQn, 2, 0);
  //HAL_NVIC_EnableIRQ(SD_DMAx_Tx_IRQn);
}

/*============================================================================
 * @brief
 * @param
 * @retval
============================================================================*/
void periphEnable(void)
{
	logWrite("\n===ENABLING PERIPHERALS===\n");
	
	logWrite("Turning Radio ON...\n");

	#ifdef BANNER_RADIO
		gpioDigitalWrite(&BANNER_RST, HIGH);
		gpioDigitalWrite(&BANNER_EN, HIGH);
	#endif

	#ifdef XBEE_RADIO
		initializeXBee();
	#endif
	
	logWrite("Turning Main Power Monitor ON...\n");
	
	utilPwrModeCont(&PM_MAIN, true);
	utilPwrModeCont(&PM_SIG1, true);
	utilPwrModeCont(&PM_SIG2, true);
	utilPwrModeCont(&PM_SIG3, true);
	utilPwrModeCont(&PM_SIG4, true);
	
	/*Turning Wifi ON (currently wifi is always off)*/
	logWrite( "Turning Wifi ON...\n");
	logWrite( "===ENABLING PERIPHERALS DONE===\n\n");
}

/*==============================================================================
* @brief     
* @param
* @retval
*=============================================================================*/
void periphDisable(void)
{
	logWrite( "\n===DISABLING PERIPHERALS===\n");
	
	/*Turn off Radio*/
	logWrite("Turning Radio OFF...\n");

	#ifdef BANNER_RADIO
		gpioDigitalWrite(&BANNER_RST, LOW);
		gpioDigitalWrite(&BANNER_EN, LOW);
	#endif

	#ifdef XBEE_RADIO
		initializeXBee();
	#endif

	/*Cutoff Wifi Power*/ 
	logWrite("Turning Wifi OFF...\n");

	gpioDigitalWrite(&WIFI_SLEEP, LOW);
	gpioDigitalWrite(&WIFI_NRESET, HIGH);
	
	/*Put Accelerometer to sleep*/
	logWrite("Turning Accelerometer OFF...\n");

	gpioDigitalWrite(&ACCEL_ENB, HIGH);
	uint8_t cmd_sleep[] = {0x2d, 0x04};
	uint8_t cmd_measure[] = {0x2d, 0x08};
	uint8_t i2c_buff[58];
	if (i2c1Write(0x1D,cmd_sleep, 2) != INTERFACE_OK)
	  	assert(0);
	if (i2c1Read((uint8_t)0x1D,0x00,i2c_buff,58)
					== INTERFACE_ERROR)
	   	assert(0); 
	if(i2c_buff[0x2D] != 0x04)
	   	assert(0);
	gpioDigitalWrite(&ACCEL_ENB, LOW);

	/*Put Pwr Monitors to sleep*/
	logWrite("Turning Main Power Monitor OFF...\n");
	
	utilPwrModeCont(&PM_MAIN, false);
	utilPwrModeCont(&PM_SIG1, false);
	utilPwrModeCont(&PM_SIG2, false);
	utilPwrModeCont(&PM_SIG3, false);
	utilPwrModeCont(&PM_SIG4, false);
	logWrite("===DISABLING PERIPHERALS DONE===\n\n");
}

/*============================================================================
 * @brief
 * @param
 * @retval
============================================================================*/
void radioEnable(bool state)
{
	#ifdef BANNER_RADIO
	if(state)
	{
	  	logWrite("\n===ENABLING Radio===\n");
		gpioDigitalWrite(&BANNER_RST, HIGH);
		gpioDigitalWrite(&BANNER_EN, HIGH);
		logWrite("===ENABLING Radio Done===\n\n");
	}
	else
	{
	  	logWrite("\n===DISABLING Radio===\n");
	  	gpioDigitalWrite(&BANNER_RST, LOW);
		gpioDigitalWrite(&BANNER_EN, LOW);
		logWrite("===DISABLING Radio Done===\n\n");
	}
	#endif


}

/*============================================================================
 * @brief
 * @param
 * @retval
============================================================================*/
uint32_t HAL_GetTick(void)
{
  return millisTick;
}

/*============================================================================
 * @brief
 * @param
 * @retval
============================================================================*/
uint32_t delayGetTick(void)
{
  return usTick;
}
/*============================================================================
 * @brief
 * @param
 * @retval  status
============================================================================*/
void HAL_IncTick(void)
{
  millisTick++;
  if(millisTick == ULONG_MAX)
  	  millisTick = 0;
}

void delayIncTick(void)
{
  usTick++;
  if(usTick == ULONG_MAX)
  	  usTick = 0;
}

/*==============================================================================
* @brief     
* @param
* @retval
*=============================================================================*/
void SystemClock_Config(uint32_t clk_div)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef  ret = HAL_OK;
  
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;  
  RCC_OscInitStruct.PLL.PLLP = clk_div;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 7;
  
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    assert(0);
  }
  
  /* Activate the OverDrive to reach the 216 MHz Frequency */  
//  ret = HAL_PWREx_EnableOverDrive();
//  if(ret != HAL_OK)
//  {
//     assert(0);
//  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; 
  
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  if(ret != HAL_OK)
  {
     assert(0);
  }  
}

void SystemClock_Config_LowPower(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef  ret = HAL_OK;
  
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 64;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 7;
  
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    assert(0);
  }
  
  /* Activate the OverDrive to reach the 216 MHz Frequency */  
//  ret = HAL_PWREx_EnableOverDrive();
//  if(ret != HAL_OK)
//  {
//     assert(0);
//  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; 
  
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  if(ret != HAL_OK)
  {
     assert(0);
  }
  delayInit();
}

void buzzEnable(uint8_t beep_count)
{
	for(uint8_t i = 0; i < beep_count; i++)
	{
		gpioDigitalWrite(&BUZZ_EN, HIGH);
		delayMillis(250);
		gpioDigitalWrite(&BUZZ_EN, LOW);
		delayMillis(250);
	}
}

void buzzWaked(void)
{
	for(uint8_t i = 0; i < 2; i++)
	{
		gpioDigitalWrite(&BUZZ_EN, HIGH);
		delayMillis(100);
		gpioDigitalWrite(&BUZZ_EN, LOW);
		delayMillis(100);
	}
}

void buzzBootSuccess(void)
{
  	gpioDigitalWrite(&BUZZ_EN, HIGH);
	delayMillis(250);
	gpioDigitalWrite(&BUZZ_EN, LOW);
	delayMillis(250);
	for(uint8_t i = 0; i < 2; i++)
	{
		gpioDigitalWrite(&BUZZ_EN, HIGH);
		delayMillis(150);
		gpioDigitalWrite(&BUZZ_EN, LOW);
		delayMillis(150);
	}
}

void buzzSleep(void)
{
	for(uint8_t i = 0; i < 3; i++)
	{
		gpioDigitalWrite(&BUZZ_EN, HIGH);
		delayMillis(100);
		gpioDigitalWrite(&BUZZ_EN, LOW);
		delayMillis(100);
	}
}

#pragma optimize=none
void changeClockSpeed(PowerModeEnum power_mode)
{
  	char status_print[50];
	logWrite("\n*****Changing MCU Speed*****\n");
	
	/*DeInitialize I2C HAL*/
	__HAL_RCC_I2C2_CLK_DISABLE();
	__HAL_RCC_I2C1_CLK_DISABLE();
	HAL_I2C_DeInit(&I2C1HandleDef);
	
	/*DeInitialize UART HAL*/
	__USART1_CLK_DISABLE();
	__USART2_CLK_DISABLE();
	__USART3_CLK_DISABLE();
	__UART4_CLK_DISABLE();
	__UART5_CLK_DISABLE();
	__USART6_CLK_DISABLE();
	__UART7_CLK_DISABLE();
	__UART8_CLK_DISABLE();
	HAL_UART_DeInit(&UART1HandlerDef);
	HAL_UART_DeInit(&UART2HandlerDef);
	HAL_UART_DeInit(&UART3HandlerDef);
	HAL_UART_DeInit(&UART4HandlerDef);
	HAL_UART_DeInit(&UART5HandlerDef);
	HAL_UART_DeInit(&UART7HandlerDef);
	HAL_UART_DeInit(&UART8HandlerDef);
	
	/*DeInitialize ADC HAL*/
	__HAL_RCC_ADC3_CLK_DISABLE();
	__HAL_RCC_ADC1_CLK_DISABLE();
	HAL_ADC_DeInit(&ADC1HandlerDef);
	HAL_ADC_DeInit(&ADC3HandlerDef);
	
	/*DeInitialize PWM HAL*/
	pwmDeInitTim();
	
	/*Initialize SPI  HAL*/
	__HAL_RCC_SPI2_CLK_DISABLE();
	//HAL_SPI_DeInit(&SPIFlashRTCHandle);
	HAL_SPI_DeInit(&SPIPwrMonitorHandle);
	
	HAL_RCC_DeInit();	
	switch(power_mode){
	case POWERMODE_HIGH:
	  	SystemClock_Config(RCC_PLLP_DIV4);
		current_power_mode = POWERMODE_HIGH;
		sprintf(status_print, "*****MCU Speed Changed to HIGH*****\n\n");
	  	break;
	case POWERMODE_BALANCED:
	  	SystemClock_Config(RCC_PLLP_DIV6);
		current_power_mode = POWERMODE_BALANCED;
		sprintf(status_print, "*****MCU Speed Changed to MID*****\n\n");
	  	break;
	case POWERMODE_LOW:
	  	SystemClock_Config_LowPower();
		current_power_mode = POWERMODE_LOW;
		sprintf(status_print, "*****MCU Speed Changed to LOW*****\n\n");
	  	break;
	default:
	  	assert(0);
	}	
	
	/*Initialize I2C HAL*/
	__HAL_RCC_I2C2_CLK_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();
	HAL_I2C_Init(&I2C1HandleDef);
	
	/*Initialize UART HAL*/
	__USART1_CLK_ENABLE();
	__USART2_CLK_ENABLE();
	__USART3_CLK_ENABLE();
	__UART4_CLK_ENABLE();
	__UART5_CLK_ENABLE();
	__USART6_CLK_ENABLE();
	__UART7_CLK_ENABLE();
	__UART8_CLK_ENABLE();
	HAL_UART_Init(&UART1HandlerDef);
	HAL_UART_Init(&UART2HandlerDef);
	HAL_UART_Init(&UART3HandlerDef);
	HAL_UART_Init(&UART4HandlerDef);
	HAL_UART_Init(&UART5HandlerDef);
	HAL_UART_Init(&UART7HandlerDef);
	HAL_UART_Init(&UART8HandlerDef);
	
	/*Initialize ADC HAL*/
	__HAL_RCC_ADC3_CLK_ENABLE();
	__HAL_RCC_ADC1_CLK_ENABLE();
	HAL_ADC_Init(&ADC1HandlerDef);
	HAL_ADC_Init(&ADC3HandlerDef);

	/*Initialize PWM HAL*/
	pwmInitTim();
	
	/*Initialize SPI HAL*/
	__HAL_RCC_SPI2_CLK_ENABLE();
	//HAL_SPI_Init(&SPIFlashRTCHandle);
	HAL_SPI_Init(&SPIPwrMonitorHandle);
	
	delayInit();
	delayMillis(10);
	logWrite(status_print);
}

void __attribute__ ((format (printf, 1, 0)))
logWrite(const char *str_format, ...)

{
	va_list arg_ptr;
	char printout_buff[2000];
	va_start(arg_ptr, str_format);
	vsprintf(printout_buff, str_format, arg_ptr);
	va_end(arg_ptr);
	
	uartTxSys(printout_buff,strlen(printout_buff));
}

#if DEVICE_TYPE == DEVICETYPE_GATE
const char *getDeviceSN(uint16_t modbus_id)
{
	for(uint8_t i = 0; i <= NODE_COUNT; i++)
	{
		if(DEVICE_INFO_LIST[i].rad_modbus_id == modbus_id)
			return DEVICE_INFO_LIST[i].device_sn;
	}
	assert(0);
}

uint16_t getRadModbusID(uint16_t rad_dev_add)
{
	for(uint8_t i = 0; i <= NODE_COUNT; i++)
	{
		if(DEVICE_INFO_LIST[i].rad_dev_add == rad_dev_add)
			return DEVICE_INFO_LIST[i].rad_modbus_id;
	}
	return 0;
}

uint16_t getRadDeviceAddr(uint16_t rad_mb_id)
{
	for(uint8_t i = 0; i <= NODE_COUNT; i++)
	{
		if(DEVICE_INFO_LIST[i].rad_modbus_id == rad_mb_id)
			return DEVICE_INFO_LIST[i].rad_dev_add;
	}
	return 0;
}

bool updateDeviceInfoListMBID(RadioNodeStruct target_node)
{
	for(uint8_t i = 0; i <= NODE_COUNT; i++)
	{
		if( strcmp(DEVICE_INFO_LIST[i].device_sn, target_node.rad_dev_sn)== 0)
		{
			DEVICE_INFO_LIST[i].rad_modbus_id = target_node.rad_mb_id;
			return true;
		}
	}
	return false;
}

bool updateDeviceInfoListDevAddAndState(RadioNodeStruct target_node)
{
	for(uint8_t i = 0; i <= NODE_COUNT; i++)
	{
		if( strcmp(DEVICE_INFO_LIST[i].device_sn, target_node.rad_dev_sn)== 0)
		{
			DEVICE_INFO_LIST[i].rad_dev_add = target_node.rad_dev_addr;
            DEVICE_INFO_LIST[i].network_add_status = NETWORK_ADD_OK;
			return true;
		}
	}
	return false;
}
#endif


/******************************************************************************
  * Revision History
  *	@file      	agBoardInit.c
  *****************************************************************************
  * @version    v2.2.0
  * @date       04/08/16
  * @author     D.Lasdoce
  * @changes
  ********************************************************************************
  * @version    v2.1.0
  * @date       12/23/2015
  * @author     D.Lasdoce
  * @changes	- created file
  *****************************************************************************
  */
