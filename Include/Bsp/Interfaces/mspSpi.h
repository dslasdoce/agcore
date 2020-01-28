/**
  ******************************************************************************
  * @file      	i2c.h
  * @author     Hardware Team
  * @version    v2.1.0
  * @date       12/23/2015
  * @brief
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
  */

#ifndef MSPSPI_H_
#define MSPSPI_H_

/* Includes ==================================================================*/
#include "agBoardInit.h"

typedef struct  {

    uint8_t cmd_stx;               // STX - for SPI Framing
    uint8_t cmd_action;            // CMD1 - CMD
    uint8_t cmd_port_pr;           // Port/Power Rail Config
    uint8_t cmd_mux_pwm_pr;        // Mux/PWM, Power Rail
    uint8_t cmd_data_lsb;          // data sent MSP                     LSB      bits 7-0
    uint8_t cmd_data_low_middle;   // like the interval timer                    bits 15-8
    uint8_t cmd_data_high_middle;  //                                            bits 23-16
    uint8_t cmd_data_msb;          // data sent to MSP                  MSB      bits 31-24
    uint8_t cmd_reserve_1;         // padding to makes packets 12 bytes in size
    uint8_t cmd_reserve_2;         // padding to makes packets 12 bytes in size 
    uint8_t cmd_reserve_3;         // padding to makes packets 12 bytes in size 
    uint8_t cmd_etx;               // ETX - for SPI Framing 
} ArmMspCmdStruct;

// RESPONSE STRUCTURE
typedef struct  {
    
    uint8_t resp_stx;              // STX - 0x02
    uint8_t resp_ack_nak;          // command validation check  either NAK or ACK
    uint8_t resp_data0;            // resp_data0 - such as CNTR1 Reading  LSB                bits 7-0
    uint8_t resp_data1;            // resp_data1 - such as CNTR1 Reading  lower middle bits  bits 15-8
    uint8_t resp_data2;            // resp_data1 - such as CNTR1 Reading  upper middle bits  bits 23-16
    uint8_t resp_data3;            // resp_data3 - such as CNTR1 Reading  MSB                bits 31-24
    uint8_t resp_reserve_1;        // keep packet size the same as command
    uint8_t resp_reserve_2;        // keep packet size the same as command
    uint8_t resp_reserve_3;        // keep packet size the same as command
    uint8_t resp_reserve_4;        // keep packet size the same as command
    uint8_t resp_reserve_5;        // keep packet size the same as command
    uint8_t resp_etx;              // ETX - 0x03


                        // An ACK is 0x06
                        // NACK - 0x15

} ArmMspRespStruct;
	
/* Defines ==================================================================*/

#define 	MSP_STX (uint8_t) 0x02
#define 	MSP_ETX (uint8_t) 0x03
#define 	MSP_NAK (uint8_t) 0x15
#define 	MSP_ACK (uint8_t) 0x06

/*Command MSB*/
#define		MSPCMD1_MCUSLEEP				(0x00)
#define		MSPCMD1_PORT_CFG				(0x01)
#define		MSPCMD1_PORT_START_READ			(0x02)
#define		MSPCMD1_PORT_SEND_DATA			(0x03)
#define		MSPCMD1_PORT_CLEAR_DATA			(0x04)
#define		MSPCMD1_PORT_PWR				(0x05)
#define		MSPCMD1_FW_UPDATE				(0x06)
#define		MSPCMD1_PWR_RAIL				(0x07)
#define         MSPCMD1_LED_TEST_DIAGNOSTIC	(0x08)	 // phil  LED's on/off/diag
#define         MSPCMD1_WATCHDOG_KICK            (0xaa)
#define         MSPCMD1_SEND_SENSOR_MAX_COUNT     (0x09)
#define         MSPCMD1_SEND_SENSOR_MIN_COUNT     (0x0A)
#define         MSPCMD1_SEND_SENSOR_INTERVAL_COUNT     (0x0B)
#define         MSPCMD1_SEND_SENSOR_RUNNING_SUM_COUNT     (0x0C)
#define         MSPCMD1_RESET_SENSOR_DATA             (0x0D)
#define         MSPCMD1_MSP_SOFTWARE_VERSION          (0x0E)



/*Command mid*/
#define		MSPCMD2_PORT1					(0x01)
#define		MSPCMD2_PORT2					(0x02)
#define		MSPCMD2_PORT3					(0x03)
#define		MSPCMD2_PORT4					(0x04)

#define		MSPCMD2_PR_1V8					(0x01)
#define		MSPCMD2_PR_3V3					(0x02)
#define		MSPCMD2_PR_4V					(0x03)
#define		MSPCMD2_PR_5V					(0x04)
#define		MSPCMD2_PR_420					(0x05)
#define		MSPCMD2_PR_ALL					(0x06)
#define		MSPCMD2_PR_3_5					(0x07)
#define		MSPCMD2_PR_2947					(0x08)


// LED's
#define LED1_STAT_RED	     0x01	  // Red LED
#define LED2_STAT_GREEN	     0x02	  // Green LED
#define BOTH_LEDS	     0x03	  // Both LED's
#define IDLE_TASK_LED_TEST   0x04	


/*Command LSB*/
#define		MSPCMD3_MUX_3K_UP				(0x00)
#define		MSPCMD3_MUX_10K_UP				(0x01)
#define		MSPCMD3_MUX_10K_DOWN			(0x02)
#define		MSPCMD3_MUX_3KUP_3V				(0x03)
#define		MSPCMD3_MUX_OFF				        (0x04)


#define		MSPCMD3_PORTPWR_3V3				(0x00)
#define		MSPCMD3_PORTPWR_5V				(0x01)
#define		MSPCMD3_PORTPWR_9V				(0x02)
#define		MSPCMD3_PORTPWR_12V				(0x03)
#define		MSPCMD3_PORTPWR_15V				(0x04)
#define		MSPCMD3_PORTPWR_OFF				(0x05)

#define		MSPCMD3_PR_OFF					(0x00)
#define		MSPCMD3_PR_ON					(0x01)

#define		MSPCMD_SIZE						(sizeof(ArmMspCmdStruct))
#define		MSPCMD_TIMEOUT					(100)
#define		MSPCMD_NULL						(0)
#define		MSPRESP_SIZE					(sizeof(ArmMspRespStruct))

#define		SUPPLY_1V 						((float)1)  //phil
#define		SUPPLY_3V						((float)4)
#define		SUPPLY_5V						((float)6)
#define		SUPPLY_9V						((float)10)
#define		SUPPLY_12V 						((float)13) //phil
#define		SUPPLY_15V						((float)16)  //phil


// LED's -
#define DIAGNOSTIC_PATTERN_1	 0x02		// diagnostic pattern
#define DIAGNOSTIC_PATTERN_2	 0x03
#define DIAGNOSTIC_PATTERN_3	 0x04
#define DIAGNOSTIC_PATTERN_4	 0x05		// enable LED Blink in IDLE Task


void mspPortConfig(uint8_t port, uint8_t mux);
void mspSetPowerRails(uint8_t pwr_rail, bool state);
uint32_t mspSendSensorData(uint8_t port);
void mspClearSensorData(uint8_t port);
void mspStartSensorRead(uint8_t port, uint32_t timerInterval);
void mspSetPortPower(uint8_t port, float supply);
void mspPortPowerReset(void);
#endif

/*********************************************************************************
  * Revision History
  * @file       i2c.h
  ********************************************************************************
  * @version    v2.1.0
  * @date       12/23/2015
  * @author     D.Lasdoce
  * @changes     - created file
  ********************************************************************************
  */
