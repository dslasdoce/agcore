/**
  ******************************************************************************
  * @file		extInterrupt.c
  * @author     Hardware Team
  * @version	v2.2.0
  * @date		04/08/16
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
#include <deployAssert.h>
#include <string.h>

#include "productCfg.h"
//#include "agBoardInit.h"
#include "delay.h"
#include "daqSensInterface.h"
#include "radio.h"
#include "ble.h"
#include "isrMain.h"
#include "daqDavis.h"

/* Defines ==================================================================*/
#define PORT1_TICK_IDX	((uint8_t)0x00)
#define PORT2_TICK_IDX	((uint8_t)0x01)
#define PORT3_TICK_IDX	((uint8_t)0x02)
#define PORT4_TICK_IDX	((uint8_t)0x03)

/* External Declarations ===================================================*/
//extern uint8_t rxbuff_xbee[8];
//extern uint8_t rxbuff_rs485[8];
//extern uint8_t rxbuff_banner[8]; /*moved to rdBnrMain.h*/

uint8_t rxbuff_xbee[8];
//uint8_t rxbuff_rs485[8]; /*declared and defines in daqSensInterface*/

extern volatile bool uart_int;

#if DEVICE_TYPE == DEVICETYPE_NODE
	#if !RTC_BASED_CYCLE
		extern uint8_t rx_buff;
	#endif
#endif

/*============================================================================
* @brief    
* @param
* @retval 
============================================================================*/
static uint32_t port_ticks[] = {0,0,0,0};

void extiCfgGPIO(uint8_t port, bool exti_en)
{
//	AgTechIOStruct *gpio[] = {&CTR1, &CTR2, &CTR3, &CTR4};
//	if(exti_en)
//		gpioAttachInt(gpio[port-1], IT_MODE_RISING);
//	else
//		gpioDettachInt(gpio[port-1]);
}

/*============================================================================
* @brief    
* @param
* @retval 
============================================================================*/
void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(0);
}

/*============================================================================
* @brief    
* @param
* @retval 
============================================================================*/
void EXTI1_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(1);
}

/*============================================================================
* @brief    
* @param
* @retval 
============================================================================*/
void EXTI3_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(3);
}

/*============================================================================
* @brief    
* @param
* @retval 
============================================================================*/
void EXTI4_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(4);
}

/*============================================================================
* @brief    
* @param
* @retval 
============================================================================*/
void EXTI15_10_IRQHandler(void)
{
	uint32_t line,gpio_pin;
	line = EXTI->PR;				//check which interrupt line is active
	for(gpio_pin=10; gpio_pin<16; gpio_pin++)
	{
		if(line & (1<<gpio_pin) )
		{
			HAL_GPIO_EXTI_IRQHandler(gpio_pin);
			return;
		}
	}
}
/*
void EXTI6_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(2);
}
#ifdef EXTI_WAKEUP
	void EXTI4_IRQHandler(void)
	{
		HAL_GPIO_EXTI_IRQHandler(4);
	}
#endif


void EXTI9_5_IRQHandler(void)
{
	volatile uint32_t line,gpio_pin;
	line = EXTI->PR;				//check which interrupt line is active
	for(gpio_pin=0; gpio_pin<16; gpio_pin++)
	{
		if(line & (1<<gpio_pin) )
			break;
	}
	HAL_GPIO_EXTI_IRQHandler(gpio_pin);
}
*/

/*============================================================================
* @brief    
* @param
* @retval 
============================================================================*/
uint32_t extiGetPortTicks(uint8_t port)
{
	return port_ticks[port - 1];
}

/*============================================================================
* @brief    
* @param
* @retval 
============================================================================*/
void extiResetPortTicks(uint8_t port)
{
	port_ticks[port - 1] = 0;
}

/*============================================================================
* @brief    
* @param
* @retval 
============================================================================*/
void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
	//const char *output = "\n===MCU WAKING UP===\n";
	switch(gpio_pin)
	{
		case 0:
			//++port_ticks[PORT3_TICK_IDX];
			break;
		case 1:
			//++port_ticks[PORT4_TICK_IDX];
			break;
		case 4:
			++port_ticks[PORT2_TICK_IDX];
			break;
		case 3:
			++port_ticks[PORT1_TICK_IDX];
			break;
		case 11: 
		case 13: 
		case 14:
			davisdth_flasgset = true;
		  	break;
		case 12:
		    uint32_t exticr = SYSCFG->EXTICR[BLE_IRQ.pin >> 2]; //get exticr (1to4)
			uint32_t exti_x = (exticr >> (BLE_IRQ.pin & 0x03)); //get exti line (0 to 15)
			PowerModeEnum last_stat = current_power_mode;
			LEDPatternEnum last_led_pattern = led_pattern;
			/*compare group index to exti_x group indicator, see RM0410 p.237*/
			/*check either INT is from BLE)IRQ or DIG2 because they share INT Line 12 */
		  	if( (exti_x & 0x000F)
			   		== GPIO_GET_INDEX(BLE_IRQ.group))
			{
				if(!run_mode)
				{
					rtcDisableWakeupTimer();
					SystemClock_Config(RCC_PLLP_DIV6);
					SystemCoreClockUpdate();
					delayInit();
					uartInit();
					enter_initmode = true;
					buzzWaked();
					HAL_IWDG_Init(&IwdgHandle);
					#if PRINT_PROCESSES_IN_UART
						const char *msg_sleep = "\n^^^^^ WAKED BY BLE ^^^^^\n";
						uartSHOW((uint8_t*)msg_sleep, strlen(msg_sleep));
					#endif
					HAL_NVIC_EnableIRQ((IRQn_Type)TIM5_IRQn);
					last_led_pattern = LED_RUNMODE;
				}
				led_pattern = LED_BLEMODE;
				changeClockSpeed(POWERMODE_HIGH);
				buzzEnable(2);
				bleDataControl();
				changeClockSpeed(last_stat);
				delayMillis(20);
				led_pattern = last_led_pattern;
			}
			else
			{
			  	davisdth_flasgset = true;
			}
			break;
		#ifdef EXTI_WAKEUP
			case 4:
				__initialize_hardware();
				uartTxWifi((uint8_t*)output, strlen(output));
				delayMillis(2000);
				daqExecute(&Port1Config);
				delayMillis(200);
				daqExecute(&Port2Config);
				delayMillis(200);
				daqExecute(&Port3Config);
				delayMillis(200);
				daqExecute(&Port4Config);
				delayMillis(1000);
				timerCloudPost();
				delayMillis(1000);
				HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
				break;
		#endif
		default:
			break;
	}
}

/*============================================================================
* @brief    
* @param
* @retval 
============================================================================*/
void UART4_IRQHandler(void)
{
	//__HAL_UART_DISABLE_IT(&UART4HandlerDef,UART_IT_RXNE);
	HAL_UART_IRQHandler(&UART4HandlerDef);
	//const char * msg = "===UART4 HANDLER END===\n";
	//uartSHOW((uint8_t*)msg, strlen(msg));
}


/*============================================================================
* @brief    
* @param
* @retval 
============================================================================*/
static int calcrc(char *ptr, int count)
{
    int  crc;
    char i;
    crc = 0;
    while (--count >= 0)
    {
        crc = crc ^ (int) *ptr++ << 8;
        i = 8;
        do
        {
            if (crc & 0x8000)
                crc = crc << 1 ^ 0x1021;
            else
                crc = crc << 1;
        } while(--i);
    }
    return (crc);
}

/*============================================================================
* @brief    
* @param
* @retval 
============================================================================*/
void UART8_IRQHandler(void)
{
	HAL_UART_IRQHandler(&UART8HandlerDef);
}

/*============================================================================
* @brief    
* @param
* @retval 
============================================================================*/
void USART1_IRQHandler(void)
{
	HAL_UART_IRQHandler(&UART1HandlerDef);
	//cell_rx_buff_single = (uint8_t)(UART1HandlerDef.Instance->DR & (uint16_t)0x00FF); /*this is for No Parity Data Decoding*/
	//uartCELLITEnable(&cell_rx_buff_single);
	//cell_rx_buff[cell_idx_buff++] = cell_rx_buff_single;
	return;
}

/*============================================================================
* @brief    
* @param
* @retval 
============================================================================*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//uint8_t rx_buff;
	uint16_t crc_rx = 0;
	uint16_t crc_cl = 0;
	//uint8_t temp_value[8];
	static bool is_first_run = true;
	//static AgTechIOStruct *en_davis = NULL;
	//static SensorRegistrationStruct *davis_config = NULL;
	switch((uint32_t)huart->Instance)
	{
		case (uint32_t)UART4:
			if(uartRxITXbee(rxbuff_xbee,2) != HAL_OK)
			{
                // abarbaso??? Verify correct usage of RxState
				UART4HandlerDef.RxState = HAL_UART_STATE_READY;
				if(uartRxITXbee(rxbuff_xbee,2) != HAL_OK)
				{
					UART4HandlerDef.RxState = HAL_UART_STATE_READY;
					assert(0);
				}
			}
			break;
		case (uint32_t)USART2:
			RdFrameStruct frame;                /*transmitting frame*/
			frame.header = rxbuff_banner.header;
			frame.crc = modbusCrcCalc((uint8_t*)&frame.header, RDHEAD_SZ);	
			if( (frame.crc.crc_hi<<8 | frame.crc.crc_lo) 
							== (rxbuff_banner.crc.crc_hi<<8 | rxbuff_banner.crc.crc_lo))
			{
		  		if(rxbuff_banner.header.mb_id == DEVICE_INFO->rad_modbus_id
				   || rxbuff_banner.header.function == RDFUNC_NETDEVNUMSET
                   || rxbuff_banner.header.function == RDFUNC_MAINTENANCE) 
				{
					#if DEVICE_TYPE == DEVICETYPE_NODE
						radioResponse(RDTYPE_BANNER, rxbuff_banner);
					#endif
				}
				else 
				{
					const char * msg0 = "===UART RADIO NOT===\n";
					uartSHOW((uint8_t*)msg0, strlen(msg0));
				}
			}
			else 
			{
				const char * msg0 = "===Wrong CRC===\n";
				uartSHOW((uint8_t*)msg0, strlen(msg0));
			}
			
			memset(&rxbuff_banner, 0, RDHEAD_SZ);
			uart_int = true;
			//uartBannerITEnable(&rxbuff_banner);
			break;

		case (uint32_t)UART8:
			if(is_first_run)
			{
		//		davis_config = davisGetPortDetails(en_davis);
				is_first_run = false;
			}
			//uartSHOW(rxbuff_rs485, 8);
			crc_rx = (rxbuff_rs485[6]<<8) | rxbuff_rs485[7];
			crc_cl = calcrc((char*)rxbuff_rs485, 6);
			if(crc_rx == crc_cl)
			{
				davis((DavisMainStruct*)rxbuff_rs485, davis_weather_port);
			}
			memset(rxbuff_rs485, 0, 8);
			uartRS485ITEnable(rxbuff_rs485);
			 //uart_int = true;
			break;
	}
}

/*============================================================================
* @brief    
* @param
* @retval 
============================================================================*/
void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&UART2HandlerDef);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uart_int = true;
  /* Prevent unused argument(s) compilation warning */
//	HAL_UART_DeInit(huart);
//	HAL_UART_Init(huart);
	//HAL_UART_RxCpltCallback(huart);
	//uartRS485ITEnable(rxbuff_rs485);
  UNUSED(huart); 
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_ErrorCallback could be implemented in the user file
   */ 
}
/*
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	huart->ErrorCode = HAL_UART_ERROR_NONE;
}
*/

/*********************************************************************************
  * Revision History
  * @file         
  ********************************************************************************
  * @version	v2.2.0	
  * @date		04/08/16
  * @author     D.Lasdoce    
  * @changes     
  *****************************************************************************
  * @version	v2.1.0
  * @date		
  * @author     D. Lasdoce    
  * @changes     created file
  ********************************************************************************
  */