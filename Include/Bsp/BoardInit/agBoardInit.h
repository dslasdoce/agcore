/**
  ******************************************************************************
  * @file      	agBoardInit.h
  * @author		Hardware Team
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

#ifndef __AGBOARDINIT_H_
#define __AGBOARDINIT_H_

/* Includes ==================================================================*/
#include "stm32f7xx_hal.h"
#include "gpio.h"
#include "productCfg.h"

/*Typedes*/
	  
/* Constants/Macros==========================================================*/

/* UART - Module Specific  Functions */
#define UART_TIMEOUT_BANNER ((uint32_t)200)
#define UART_TIMEOUT ((uint32_t)200)

#define uartTxCell(p_data,size) 			(HAL_UART_Transmit(&UART1HandlerDef,\
                                             p_data, size, (uint32_t)2000))
#define uartRxCell(p_data,size) 			(HAL_UART_Receive(&UART1HandlerDef,\
                                             p_data, size, (uint32_t)UART_TIMEOUT))
#define uartRxITCell(p_data,size) 			(HAL_UART_Receive_IT(&UART1HandlerDef,\
                                             p_data, size))

#define uartTxBanner(p_data,size) 			(HAL_UART_Transmit(&UART2HandlerDef,\
                                             p_data, size, (uint32_t)2000))
#define uartRxBanner(p_data,size, timeout)	(HAL_UART_Receive(&UART2HandlerDef,\
                                             p_data, size, (uint32_t)timeout))
#define uartRxITBanner(p_data,size) 		(HAL_UART_Receive_IT(&UART2HandlerDef,\
                                             p_data, size))

#define uartTxBle(p_data,size) 				(HAL_UART_Transmit(&UART3HandlerDef,\
                                             p_data, size, (uint32_t)2000))
#define uartRxBle(p_data,size) 				(HAL_UART_Receive(&UART3HandlerDef,\
                                             p_data, size, (uint32_t)20))
#define uartTxXbee(p_data,size) 			(HAL_UART_Transmit(&UART4HandlerDef,\
                                             p_data, size, (uint32_t)2000))

#define uartRxXbee(p_data,size) 			(HAL_UART_Receive(&UART4HandlerDef,\
                                             p_data, size, (uint32_t)UART_TIMEOUT))
#define uartRxITXbee(p_data,size) 			(HAL_UART_Receive_IT(&UART4HandlerDef,\
                                             p_data, size))

#define uartTxWifi(p_data,size) 			(HAL_UART_Transmit(&UART6HandlerDef,\
                                             p_data, size, (uint32_t)UART_TIMEOUT))
#define uartRxWifi(p_data,size) 			(HAL_UART_Receive(&UART6HandlerDef,\
                                             p_data, size, (uint32_t)UART_TIMEOUT))
#define uartRxITWifi(p_data,size) 			(HAL_UART_Receive_IT(&UART6HandlerDef,\
                                             p_data, size))
#define uartTxGps(p_data,size) 				(HAL_UART_Transmit(&UART7HandlerDef,\
                                             p_data, size, (uint32_t)UART_TIMEOUT))
#define uartRxGps(p_data,size) 				(HAL_UART_Receive(&UART7HandlerDef,\
                                             p_data, size, (uint32_t)1000))

#define uartTxRS485(p_data,size) 			(HAL_UART_Transmit(&UART8HandlerDef,\
                                             p_data, size, (uint32_t)UART_TIMEOUT))
#define uartRxRS485(p_data,size, timeout) 			(HAL_UART_Receive(&UART8HandlerDef,\
                                             p_data, size, (uint32_t)timeout))
#define uartRxITRS485(p_data,size) 			(HAL_UART_Receive_IT(&UART8HandlerDef,\
                                             p_data, size))
#define uartTxSys(p_data,size) 				(HAL_UART_Transmit(&UART5HandlerDef,\
                                             p_data, size, (uint32_t)2000))
#define uartRxSys(p_data,size) 				(HAL_UART_Receive(&UART5HandlerDef,\
                                             p_data, size, (uint32_t)UART_TIMEOUT))
#define UARTRESET(huart)					 __HAL_UART_DISABLE(huart);\
											  __HAL_UART_ENABLE(huart);

#ifdef BANNER_RADIO

	#define radioTx(p_data, size)		     (uartTxBanner(p_data,size))
	#define radioRx(p_data, size)		     (uartRxBanner(p_data,size))

	#define uartSHOW(p_data,size)			(uartTxSys(p_data,size))

#endif

void __attribute__ ((format (printf, 1, 0)))
logWrite(const char *str_format, ...);


#define uartRS485ITEnable(rx_buff)	        if(uartRxITRS485((uint8_t*) rx_buff, 8) != HAL_OK){\
                                            	UART8HandlerDef.RxState = HAL_UART_STATE_READY;\
                                             	if(uartRxITRS485((uint8_t*) rx_buff, 8) != HAL_OK)\
                                              	assert(0);}

#define uartBannerITEnable(rx_buff)	        if(uartRxITBanner((uint8_t*) rx_buff, 8) != HAL_OK){\
                                             UART2HandlerDef.RxState = HAL_UART_STATE_READY;\
                                             if(uartRxITBanner((uint8_t*) rx_buff, 8) != HAL_OK)\
											 assert(0);}

#define MAX_PRINTOUT			            ((uint32_t)100)
#define	NETWORK_ADD_OK						(0xF2)
#define	NETWORK_ADD_EMPTY					(0x00)

/* I2C - Bus Specific Functions */
#define ADC_REF								((float)3.3)
#define ADC_READSAMPLES						((uint32_t)100)

/*
 * Alternate Function Values
 */
#define AF_TIM1_2							((uint32_t)0x00000001)
#define AF_TIM3_4_5							((uint32_t)0x00000002)
#define AF_TIM8_9_10_11						((uint32_t)0x00000003)
#define AF_I2C								((uint32_t)0x00000004)
#define AF_SPI1                             ((uint32_t)0x00000005)
#define AF_SPI2                             ((uint32_t)0x00000005)
#define AF_SPI4                             ((uint32_t)0x00000005)
#define AF_SPI3                             ((uint32_t)0x00000006)
#define AF_SPI3_MOSI                        ((uint32_t)0x00000007)
#define AF_USART1							((uint32_t)0x00000004)
#define AF_UART1_3							((uint32_t)0x00000007)
#define  AF_UART7							((uint32_t)0x00000007)
#define AF_UART4_8							((uint32_t)0x00000008)
#define AF_SDIO_D0_D1_D3                     ((uint32_t)0x0000000B)
#define AF_SDIO_D2	                        ((uint32_t)0x0000000A)

//#define I2Cx_FORCE_RESET()               	__HAL_RCC_I2C1_FORCE_RESET()
//#define I2Cx_RELEASE_RESET()             	__HAL_RCC_I2C1_RELEASE_RESET()
#define ACCEL_ENB							((AgTechIOStruct){GPIOC,13})
#define ACCEL_IRQ							((AgTechIOStruct){GPIOB,5})
/*
* SPI PINS
*/
#define FLM25_CS_PIN                        ((AgTechIOStruct){GPIOB,10})
#define MSP430_CS_PIN                       ((AgTechIOStruct){GPIOI,0})
#define SPI2_CLK_PIN			            ((AgTechIOStruct){GPIOI,1})
#define SPI2_MISO_PIN			            ((AgTechIOStruct){GPIOI,2})
#define SPI2_MOSI_PIN			            ((AgTechIOStruct){GPIOI,3})
#define MSP430_SPI_COMING  		     		((AgTechIOStruct){GPIOD,5})

#define RTC_ENB								((AgTechIOStruct){GPIOD,13})

#define XBEE_IRQ							((AgTechIOStruct){GPIOJ,5})

#define SPI3_SCK							((AgTechIOStruct){GPIOC,10})
#define SPI3_MISO							((AgTechIOStruct){GPIOC,11})
#define SPI3_MOSI							((AgTechIOStruct){GPIOB,2})
#define PM_SIG1								((AgTechIOStruct){GPIOK,1})
#define PM_SIG2								((AgTechIOStruct){GPIOK,2})
#define PM_SIG3								((AgTechIOStruct){GPIOK,3})
#define PM_SIG4								((AgTechIOStruct){GPIOK,4})
#define PM_MAIN								((AgTechIOStruct){GPIOK,0})

#define SPI4_SCK							((AgTechIOStruct){GPIOE,12})
#define SPI4_MISO							((AgTechIOStruct){GPIOE,13})
#define SPI4_MOSI							((AgTechIOStruct){GPIOE,6})

/*
	POWER PWM
*/
#define VCON1								((AgTechIOStruct){GPIOE,14})					
#define VCON2								((AgTechIOStruct){GPIOC,9})
#define VCON3								((AgTechIOStruct){GPIOB,1})
#define VCON4								((AgTechIOStruct){GPIOB,11})
#define VCON1_TIM							(TIM1)
#define VCON2_TIM							(TIM8)
#define VCON3_TIM							(TIM3)
#define VCON4_TIM							(TIM2)
#define VCONx_CH							(TIM_CHANNEL_4)
#define VCON1_EN							((AgTechIOStruct){GPIOJ,11})
#define VCON2_EN							((AgTechIOStruct){GPIOJ,12})
#define VCON3_EN							((AgTechIOStruct){GPIOJ,13})
#define VCON4_EN							((AgTechIOStruct){GPIOJ,14})

#define DUTY_CYCLE_VCON1					(0.25)
#define DUTY_CYCLE_VCON2					(0.15)
#define DUTY_CYCLE_VCON3					(0.25)
#define DUTY_CYCLE_VCON4					(0.25)


#define DUTYCYC_3V							((float)0.54)
#define DUTYCYC_5V							((float)0.47)
#define DUTYCYC_9V							((float)0.34)
#define DUTYCYC_12V							((float)0.23)

/* Interface(Analog/Digital) Selection Values */
#define CODE_INTERFACE_AN					((uint8_t)1)
#define CODE_INTERFACE_DIG					((uint8_t)0)

#define RTC_SPRES							((uint32_t)255)
#define RTC_APRES							((uint32_t)127)
#define RTCHandleStruct						((RTC_HandleTypeDef){RTC,\
											{RTC_HOURFORMAT_24,RTC_APRES,RTC_SPRES,RTC_OUTPUT_DISABLE\
											,RTC_OUTPUT_POLARITY_HIGH,RTC_OUTPUT_TYPE_PUSHPULL}\
											,HAL_UNLOCKED,HAL_RTC_STATE_READY})
//RTC_HandleTypeDef RTCHandleStruct;
/* Digital IO Pins */
#define DIG1					((AgTechIOStruct){GPIOF,11})
#define DIG2					((AgTechIOStruct){GPIOF,12})
#define DIG3					((AgTechIOStruct){GPIOF,13})
#define DIG4					((AgTechIOStruct){GPIOF,14})

/*1-wire demux pins*/
#define SIG1_ID					((AgTechIOStruct){GPIOI,8})
#define SIG2_ID					((AgTechIOStruct){GPIOI,9})
#define SIG3_ID					((AgTechIOStruct){GPIOI,10})
#define	SIG4_ID					((AgTechIOStruct){GPIOI,11})
#define TP38					((AgTechIOStruct){GPIOD,4})
/*
 * Level Shifter Control Pins
 */
//#define LEVELSHIFT_DIR			((AgTechIOStruct){GPIOE,11})
//#define LEVELSHIFT_OENB			((AgTechIOStruct){GPIOE,12})

#define DLS_DIR					((AgTechIOStruct){GPIOD,1})
#define DLS_OE					((AgTechIOStruct){GPIOD,0})
#define DLS_VS					((AgTechIOStruct){GPIOE,7})

/*
 *I2C Interface Pins
 */
#define I2C1_SCL				((AgTechIOStruct){GPIOB,6})
#define I2C1_SDA				((AgTechIOStruct){GPIOB,7})
#define I2C2_SDA				((AgTechIOStruct){GPIOF,0})
#define I2C2_SCL				((AgTechIOStruct){GPIOF,1})

/* UART Pins */
#define CELL_TX					((AgTechIOStruct){GPIOB,14})
#define CELL_RX					((AgTechIOStruct){GPIOB,15})
#define CELL_RTS				((AgTechIOStruct){GPIOH,15})
#define BANNER_TX				((AgTechIOStruct){GPIOA,2})	//very low pull up
#define BANNER_RX				((AgTechIOStruct){GPIOA,3})
#define BLE_TX					((AgTechIOStruct){GPIOD,8})
#define BLE_RX					((AgTechIOStruct){GPIOD,9})
#define XBEE_TX					((AgTechIOStruct){GPIOA,0})
#define XBEE_RX					((AgTechIOStruct){GPIOA,1})
//#define XBEE_RX					((AgTechIOStruct){GPIOA,0x02})
#define UART5_TX				((AgTechIOStruct){GPIOC,12})
#define UART5_RX				((AgTechIOStruct){GPIOD,2})
#define WIFI_TX					((AgTechIOStruct){GPIOC,6})
#define WIFI_RX					((AgTechIOStruct){GPIOC,7})
#define GPS_TX					((AgTechIOStruct){GPIOF,7})
#define GPS_RX					((AgTechIOStruct){GPIOF,6})
#define RSX_TX					((AgTechIOStruct){GPIOE,1})
#define RSX_RX					((AgTechIOStruct){GPIOE,0})

/*SDIO*/
#define SD_CMD	    			((AgTechIOStruct){GPIOD,7})
#define SD_D0	    			((AgTechIOStruct){GPIOG,9})
#define SD_D1		    		((AgTechIOStruct){GPIOG,10})
#define SD_D2   				((AgTechIOStruct){GPIOG,11})
#define SD_D3		    		((AgTechIOStruct){GPIOG,12})
#define SD_CLK  				((AgTechIOStruct){GPIOD,6})
#define SD_EN					((AgTechIOStruct){GPIOJ,8})

/* Analog I/O Pins */
#define THERM1					((AgTechIOStruct){GPIOC,0})
#define THERM2					((AgTechIOStruct){GPIOC,1})
#define THERM3					((AgTechIOStruct){GPIOC,2})
#define THERM4					((AgTechIOStruct){GPIOC,3})
#define THERM1_ADC				((AgTechAdcStruct){&ADC1HandlerDef, 10})
#define THERM2_ADC				((AgTechAdcStruct){&ADC1HandlerDef, 11})
#define THERM3_ADC				((AgTechAdcStruct){&ADC1HandlerDef, 12})
#define THERM4_ADC				((AgTechAdcStruct){&ADC1HandlerDef, 13})

#define SIG1_0_5				((AgTechIOStruct){GPIOF,8})
#define SIG2_0_5				((AgTechIOStruct){GPIOF,9})
#define SIG3_0_5				((AgTechIOStruct){GPIOF,10})
#define SIG4_0_5				((AgTechIOStruct){GPIOF,3})
#define SIG1_0_5_ADC			((AgTechAdcStruct){&ADC3HandlerDef, 6})
#define SIG2_0_5_ADC			((AgTechAdcStruct){&ADC3HandlerDef, 7})
#define SIG3_0_5_ADC			((AgTechAdcStruct){&ADC3HandlerDef, 8})
#define SIG4_0_5_ADC			((AgTechAdcStruct){&ADC3HandlerDef, 9})

#define RS485_EN				((AgTechIOStruct){GPIOE,2})
#define XBEE_STATUS				((AgTechIOStruct){GPIOJ,4})

/* Main Interface Selection Pins (Analog/Digital) */
#define SIG1_SEL				((AgTechIOStruct){GPIOE,11})
#define SIG1_SEL0				((AgTechIOStruct){GPIOE,15})
#define SIG1_SEL1				((AgTechIOStruct){GPIOF,2})

#define SIG2_SEL				((AgTechIOStruct){GPIOF,15})
#define SIG2_SEL0				((AgTechIOStruct){GPIOF,5})
#define SIG2_SEL1				((AgTechIOStruct){GPIOI,4})
   
#define SIG3_SEL				((AgTechIOStruct){GPIOI,6})
#define SIG3_SEL0				((AgTechIOStruct){GPIOI,7})
#define SIG3_SEL1				((AgTechIOStruct){GPIOG,4})
   
#define SIG4_SEL				((AgTechIOStruct){GPIOG,5})
#define SIG4_SEL0				((AgTechIOStruct){GPIOG,6})
#define SIG4_SEL1				((AgTechIOStruct){GPIOG,8})

#define RSx_A_EN                ((AgTechIOStruct){GPIOA,4})
#define RSx_A0_SEL              ((AgTechIOStruct){GPIOA,5})
#define RSx_A1_SEL              ((AgTechIOStruct){GPIOA,7})

#define RSx_B_EN                ((AgTechIOStruct){GPIOA,8})
#define RSx_B0_SEL              ((AgTechIOStruct){GPIOB,13})
#define RSx_B1_SEL              ((AgTechIOStruct){GPIOB,0})

#define RS232_ENB				((AgTechIOStruct){GPIOG,13})
#define RS485_FEN				((AgTechIOStruct){GPIOG,14})
#define RS485_RES_EN			((AgTechIOStruct){GPIOG,15})
#define RS485_DXEN				((AgTechIOStruct){GPIOD,14})
#define RS485_RXENB				((AgTechIOStruct){GPIOD,15}) 


/* Analog Input Fixed Resistor Selection Pins */
#define	AD_EN					((AgTechIOStruct){GPIOI,12})
#define	AD_CONV					((AgTechIOStruct){GPIOI,15})
#define	AD_CS					((AgTechIOStruct){GPIOI,13})
#define	AD_BUSY					((AgTechIOStruct){GPIOI,14})
/* Sensor Power Selection Pins */
#define PWR1_A_EN				((AgTechIOStruct){GPIOG,0})
#define PWR2_A_EN				((AgTechIOStruct){GPIOG,1})
#define PWR3_A_EN				((AgTechIOStruct){GPIOG,2})
#define PWR4_A_EN				((AgTechIOStruct){GPIOG,3})

/* Valve Control Pins */
#define VALVE_TRIG1_A			((AgTechIOStruct){GPIOH,2})
#define VALVE_TRIG1_B			((AgTechIOStruct){GPIOH,3})
#define VALVE_TRIG2_A			((AgTechIOStruct){GPIOH,4})
#define VALVE_TRIG2_B			((AgTechIOStruct){GPIOH,5})
#define VCON15_EN				((AgTechIOStruct){GPIOJ,15})

#define CELL_NRESET				((AgTechIOStruct){GPIOH,13})
#define CELL_ON_OFF				((AgTechIOStruct){GPIOJ,10})
#define TEMP_EXT				((AgTechIOStruct){GPIOF,4})
#define WIFI_SLEEP				((AgTechIOStruct){GPIOJ,7})
#define WIFI_NRESET				((AgTechIOStruct){GPIOJ,9})
#define WIFI_IRQ				((AgTechIOStruct){GPIOJ,9})
//#define PWRN_4V0_EN				((AgTechIOStruct){GPIOD,1})
//#define PWR_12V0_EN				((AgTechIOStruct){GPIOD,0})
     
/* Banner Radio Pins */
#define BANNER_EN				((AgTechIOStruct){GPIOH,6})
#define BANNER_RST				((AgTechIOStruct){GPIOA,10})
#define BNR_CTR					((AgTechIOStruct){GPIOH,7})
#define	BLE_IRQ					((AgTechIOStruct){GPIOB,12})
#define BNR_ECHO0			    ((AgTechIOStruct){GPIOC,4})
#define BNR_ECHO1			    ((AgTechIOStruct){GPIOC,5})
     
/* GPS Pins */
#define GPS_ON_OFF 			   	((AgTechIOStruct){GPIOJ,2}) //!!!!!!ADDED FOR GPS
#define GPS_SYSON  	  			((AgTechIOStruct){GPIOJ,3}) //!!!!!!ADDED FOR GPS
#define GPS_RESET  	  			((AgTechIOStruct){GPIOD,10})
//#define PWR_1V8_EN    			((AgTechIOStruct){GPIOE,15}) //!!!!!!ADDED FOR GPS

#define RTC_ALARM				((AgTechIOStruct){GPIOC,13})
     
#define CHARGE_OK				((AgTechIOStruct){GPIOJ,0})
 
#define LD_USER1_R				((AgTechIOStruct){GPIOK,6})
#define LD_USER2_G				((AgTechIOStruct){GPIOK,7})
   
#define SPI4_SCK				((AgTechIOStruct){GPIOE,12})
#define SPI4_MOSI				((AgTechIOStruct){GPIOE,6})
#define SPI4_MISO				((AgTechIOStruct){GPIOE,13})

#define BUZZ_EN					((AgTechIOStruct){GPIOK,5})

#define PWR_MODE_OFF			((uint16_t)0x0000)
#define PWR_MODE_CONTINUOUS		((uint16_t)0x0001)
#define PWR_MODE_DISCRETE		((uint16_t)0x0014)
#define	PWR_INT_MASK_SET		((uint16_t)0x1000)
#define	PWR_INT_MASK_RESET		((uint16_t)0x00FF)
#define VOUT_MARGIN				(2)

/* Typedefs =================================================================*/
/*
RTC_HandleTypeDef RTC_HandleStruct;
	RTC_HandleStruct.Instance = RTC;
	RTC_HandleStruct.Init.HourFormat = RTC_HOURFORMAT_24;
	RTC_HandleStruct.Init.AsynchPrediv = 102;
	RTC_HandleStruct.Init.SynchPrediv = 2500;
	RTC_HandleStruct.Init.OutPut = RTC_OUTPUT_DISABLE;
	RTC_HandleStruct.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	RTC_HandleStruct.Init.OutPutType = RTC_OUTPUT_TYPE_PUSHPULL;
*/

typedef enum
{
   RADNETMAINTENANCEMODE_DISABLE,
   RADNETMAINTENANCEMODE_ENABLE,
   RADNETMAINTENANCEMODE_TOBEENABLED,
   RADNETMAINTENANCEMODE_TOBEDISABLED
}RadNetMaintenanceModeStatusEnum;

typedef enum
{
	INTERFACE_ERROR,
	INTERFACE_OK
	
}InterfaceStatusEnum;

typedef enum
{
	DAQ_OK,
	DAQ_ERROR,
	DAQ_NO_RESPONSE,
	DAQ_RAW_UOM_ERROR,
	DAQ_PORT_ERROR,
	DAQ_DATA_ERROR
}DaqStatusEnum;

typedef enum
{
	EQUATION_ERROR,
	EQUATION_OK,
	EQUATION_ZERODIV

}EquationStatusEnum;

typedef enum
{
    IFC_ANALOG_V 		= 0,
    IFC_ANALOG_I 		= 1,
    IFC_ANALOG_R 		= 2,
    IFC_ANALOG_V_R 		= 3,
    IFC_ANALOG_V_V		= 4,
    IFC_COUNTER_PU_C	= 5,
    IFC_COUNTER_PU_P	= 6,
    IFC_COUNTER_PD_C	= 7,
    IFC_COUNTER_PD_P 	= 8,
    IFC_COUNTER_NP_C	= 9,
    IFC_COUNTER_NP_P	= 10,
    IFC_SDI12			= 11,
    IFC_RS485			= 12,
    IFC_RS232			= 13,
    IFC_DAVIS_ISS		= 14,
    IFC_DAVIS_TEMP		= 15,
    IFC_AV_CPUC			= 16,
    IFC_AV_CPUP			= 17,
    IFC_AV_CPDC			= 18,
    IFC_AV_CPDP			= 19,
    IFC_AV_CNPC			= 20,
    IFC_AV_CNPP			= 21,
	IFC_DAVIS_DTH		= 22,
	IFC_THERM			= 23,
	IFC_BLE				= 24
}InterfaceCodeEnum;

typedef enum
{
	VALVE_OPEN,
	VALVE_CLOSE
}ValveStateEnum;

typedef enum
{
	POSITIVE_EXCITATION,
	REVERSE_POLARITY
}ValveCtrlModeEnum;

typedef enum
{
	SOIL_MOIST,
	SOIL_TEMP,
	AIR_TEMP,
	AIR_PRES,
	WWATER_PRES,
	UV,
	SOLAR_RAD,
	WIND_SPEED,
	WIND_DIR,
	R_HUMIDITY,
	RAIN_RATE

}SensorTypeEnum;
#define SIZE_SRCHAR_64BYTES		((uint32_t)64)
#define SIZE_SRCHAR_32BYTES		((uint32_t)32)
typedef struct
{
	uint8_t status;
	uint8_t port;
	uint8_t interface;
	uint8_t power_supply;
	uint16_t sensor_code;
	uint16_t interval;
	uint32_t excite_time;
	uint32_t depth_uom;
	float raw_min;
	float raw_max;
	float lower_lim;
	float upper_lim;
	char sensor_type[SIZE_SRCHAR_32BYTES];
	char raw_uom[SIZE_SRCHAR_32BYTES];
	char base_uom[SIZE_SRCHAR_32BYTES];
	char depth_value[SIZE_SRCHAR_64BYTES];
	char data_sequence[SIZE_SRCHAR_64BYTES];
	char command[SIZE_SRCHAR_64BYTES];
	char conversion[SIZE_SRCHAR_64BYTES];
	AgTechIOStruct gpio;
	AgTechIOStruct sub_gpio;
}SensorRegistrationStruct;

typedef struct
{
	SensorRegistrationStruct P1;
	SensorRegistrationStruct P2;
	SensorRegistrationStruct P3;
	SensorRegistrationStruct P4;
}PortConfigStruct;

typedef struct
{
	int8_t Seconds;
	int8_t Minutes;
	int8_t Hours;
	uint8_t Weekday;
	int16_t Month;
	int16_t Date;
	int16_t Year;
	int16_t Millis;
}RTCDateTimeStruct;

typedef  struct
{
	uint16_t			sensor_code;
	uint16_t 			rad_modbus_id;
	uint8_t				sensor_type;
	uint8_t				port;
	uint8_t 			raw_uom;
	uint8_t 			base_uom;
	uint8_t 			depth_uom;	
	uint8_t				backup_flag;
	uint8_t				unsent_flag;
	uint8_t				rsrvd;
	float 				raw_value;
	float 				base_value;
	float 				depth_value;
	RTCDateTimeStruct 	date_time;
} TelemetryStruct;



/* System Status Data Struct*/
typedef struct
{
	uint8_t batt_stat;					/* battery status */
	uint8_t radio_sig_stat;				/* radio signal status */
	uint8_t cell_sig_stat;				/* cell signal status */
	uint8_t acc_sig_stat;				/* accelerometer signal status */
	uint16_t avail_mem;					/* available memory */
	uint16_t orig_mem_size;				/* original memory size */
	float int_temp;						/* internal temperature */
	float ext_temp;						/* external temperature */
	float latitude;						/* gps location 1 */
	float longitude;					/* gps location 2 */
	uint32_t charging_stat	: 1;		/* charging status*/
	uint32_t reg_typ		: 3;		/* registration type */
	uint32_t rsvd_12b		: 12;		/* reserved field */
	uint32_t backup_flag	: 8;		/* back-up flag */
	uint32_t unsent_flag	: 8;		/* unsent flag */
	FWVersionStruct fwver;
	DRVersionStruct drver;
	RTCDateTimeStruct 		date_time;	/* registration date*/
	uint8_t rad_modbus_id;					/* sequence number */
	uint8_t res1;	
	uint16_t res2;
} SystemStatusStruct;

typedef enum
{
  	LED_RUNMODE,
  	LED_RUNMODE_GPS,
	LED_RUNMODE_ACCNTID_SET,
	LED_BLEMODE,
	LED_SLEEPMODE,
} LEDPatternEnum;

#define RADIONODE_MAX_DEVSN_CHARSZ ((uint8_t)19) //max # of char for Banner's ATNA command + end char
typedef struct
{
	uint16_t rad_dev_addr;
	uint16_t rad_mb_id;
	char rad_dev_sn[RADIONODE_MAX_DEVSN_CHARSZ]; 
} RadioNodeStruct;

typedef struct
{
    volatile bool enter_sleepmode;
    volatile uint8_t nmm_timer_min;
    volatile uint8_t nmm_status;
} SleepAttrStruct;
	
#if DEVICE_TYPE == DEVICETYPE_GATE
	extern char *SERIAL_NUMTEST[10];
#endif

/*
typedef struct
{
	float raw_value;
}TelemetryStruct;
*/
//extern SensorRegistrationStruct Port1Config,Port2Config,Port3Config,Port4Config;


//RTC_DateTypeDef RTC_DateStruct;
//RTC_TimeTypeDef RTC_TimeStruct;
//RTC_DateTimeTypeDef RTC_DateTimeStruct;

extern UART_HandleTypeDef UART1HandlerDef;
extern UART_HandleTypeDef UART2HandlerDef;
extern UART_HandleTypeDef UART3HandlerDef;
extern UART_HandleTypeDef UART4HandlerDef;
extern UART_HandleTypeDef UART5HandlerDef;
extern UART_HandleTypeDef UART6HandlerDef;
extern UART_HandleTypeDef UART7HandlerDef;
extern UART_HandleTypeDef UART8HandlerDef;
extern I2C_HandleTypeDef I2C1HandleDef;
extern I2C_HandleTypeDef I2C2HandleDef;
extern ADC_HandleTypeDef ADC3HandlerDef, ADC1HandlerDef;
extern SPI_HandleTypeDef SPIFlashRTCHandle;
extern bool enter_initmode;
extern bool read_gps;
extern bool run_mode;
extern uint32_t ota_fwsize;
extern IWDG_HandleTypeDef IwdgHandle;
extern SPI_HandleTypeDef SPIPwrMonitorHandle;
extern volatile bool davisdth_flasgset;
extern volatile bool davisdth_sck_state;
extern AgTechIOStruct *io_dth_sck;
extern bool gps_running;
extern bool account_id_set;
extern float adc_therm_ref;
extern bool check_battery;
extern PowerModeEnum current_power_mode;
extern LEDPatternEnum led_pattern;
extern bool execute_routine;
extern uint16_t RTC_SLEEPTIME;
extern SleepAttrStruct sleep_attr;


/* Function Prototypes =======================================================*/
void agBoardInit(void);
void sdInitHardware(void);
void irqConfig(void);
//_PTR agMalloc(size_t __size);
void rtcInit(void);
void periphEnable(void);
void radioEnable(bool state);
void periphDisable(void);
void SystemClock_Config(uint32_t clk_div);
void SystemClock_Config_LowPower(void);
void buzzEnable(uint8_t beep_count);
void buzzBootSuccess(void);
void buzzWaked(void);
void buzzSleep(void);
void delayIncTick(void);
void uartDeInit(void);
void uartInit(void);
void LPTIMInit(void);
uint32_t delayGetTick(void);
void changeClockSpeed(PowerModeEnum power_mode);
void agBoardSyncRTCs();
#if DEVICE_TYPE == DEVICETYPE_GATE
	const char *getDeviceSN(uint16_t modbus_id);
	uint16_t getRadModbusID(uint16_t rad_dev_add);
    uint16_t getRadDeviceAddr(uint16_t rad_mb_id);
	bool updateDeviceInfoListMBID(RadioNodeStruct target_node);
	bool updateDeviceInfoListDevAddAndState(RadioNodeStruct target_node);
#endif
#if DEVICETYPE == DEVICETYPE_NODE
	extern uint16_t NODEWAIT_TIMEOUT;
#endif
#endif

/*********************************************************************************
  * Revision History
  *	@file      	agBoardInit.h
  ********************************************************************************
  * @version    v2.2.0
  * @date       04/08/16
  * @author     D.Lasdoce
  * @changes
  ********************************************************************************
  * @version    v2.1.0
  * @date       12/23/2015
  * @author     D.Lasdoce
  * @changes	created file
  ********************************************************************************
  */
