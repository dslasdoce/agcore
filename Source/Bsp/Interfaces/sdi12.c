/********************************************************************************
  * @file      	sdi12.c
  * @author		Hardware Team
  * @version	v2.1
  * @date		12/23/2015
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


/* Includes ------------------------------------------------------------------*/
#include <agBoardInit.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "sdi12.h"
#include "gpio.h"
#include "delay.h"
#include "utilCalc.h"

/* Macro Definitions ---------------------------------------------------------*/
#define SDISTATE_TRANSMITTING 		((uint8_t)2)
#define SDISTATE_LISTENING 			((uint8_t)3)
#define SDISTATE_HOLDING 			((uint8_t)4)

#define TIME_TIMEOUT  				((uint32_t)2000)
#define TIME_RESPONSE 				((uint32_t)200)
#define TIME_TRIALINTERVAL 			((uint32_t)100)
#define TIME_SETUP_DELAY 			((uint32_t)1000)
#define TIME_BREAK  				((uint32_t)12500)
#define TIME_MARKING 				((uint32_t)8500)
#define TIME_PULSE_PERIOD 			((uint32_t)833)

#define IDX_WAITTIME_TIMECOUNT 		((uint8_t)0x00)
#define IDX_MEASCOUNT_TIMECOUNT 	((uint8_t)0x01)
#define IDX_STARTOFMEAS_RX			((uint8_t)1)
#define IDX_DEVICEADD_RX			((uint8_t)0)

#define SIZE_WAITTIME				((uint8_t)4)
#define SIZE_MEASCOUNT				((uint8_t)2)
#define SIZE_RXBUFFER 				((uint32_t)64)
#define SIZE_VALBUFFER 				((uint32_t)32)

#define SIZE_TEMPVAL 				(20)
#define	SDI_DECPLACES				(4)

/* Global Declarations ======================================================*/
//bool rx_bufferfull;
char rx_buffer[SIZE_RXBUFFER];				/* storage for individual char received from sensor*/
float val_buffer[SIZE_VALBUFFER];			/* storage fro every sensor value */
static uint8_t time_count_buffer[2];		/* storage for waiting time and number of measurements */
uint8_t val_bufferhead = 0;
uint8_t val_buffertail = 0;
uint8_t rx_buffertail;					/* index where next character will be written */


/* Functions ================================================================*/

/* ============================================================================
 * @func	sdiState
 * @brief	controls the state of the data line
 * @param
 * @para
 * @retval	none
 * ==========================================================================*/
static void sdiState(int state, AgTechIOStruct *gpio)
{
	/*set pin as output*/
	if (state == SDISTATE_TRANSMITTING)
	{
		gpioMode(gpio, MODE_OUTPUT);
		/* set level shifter dir pin to LOW
		 * (direction is from bus B[mcu] to bus A[sensor])*/
		gpioDigitalWrite(&DLS_DIR, HIGH);
	}
	/*set pin as input*/
	else if (state == SDISTATE_LISTENING)
	{
		gpioDigitalWrite(gpio, LOW);
		gpioMode(gpio, MODE_INPUT);
		/* set level shifter dir pin to HIGH
		* (direction is from bus A[sensor] to B[mcu])*/
		gpioDigitalWrite(&DLS_DIR, LOW);
	}
	/*set pin as input*/
	else if (state == SDISTATE_HOLDING)
	{
		gpioMode(gpio, MODE_OUTPUT);
		gpioDigitalWrite(gpio, LOW);
	}
}

static inline void sdiWakeSensor(AgTechIOStruct *gpio)		/* issue a signal to notify sensors that command will be issued */
{
	sdiState(SDISTATE_TRANSMITTING,gpio);
	gpioDigitalWrite(gpio,HIGH);
	delayMicros(TIME_BREAK);
	gpioDigitalWrite(gpio, LOW);
	delayMicros(TIME_MARKING);
}

static inline char addEvenParity(char out_char)
{
	char parity_bit = 0x00;
	uint8_t ctr = 0;
	int mask;

	for(mask = 0x01; mask<=0x40; mask<<=1)
	{
		if (out_char & mask)		//check if bit is 1
		{
			++ctr;					//increment current count of 1's
		}
	}
	if (ctr & 0x01)						//check if there is an odd number of 1
	{
		parity_bit = 0x01;
	}
	return (out_char | (char)(parity_bit<<0x07));	//insert parity bit
}

/*function to get command string and transmit it as logic levels*/
static void sdiSendCommand(const char *cmd, AgTechIOStruct *gpio)	//send different commands to data line
{
	char cmdlen;						//command length
	cmdlen = (char)strlen(cmd);
	sdiWakeSensor(gpio);
	int i;
	int mask;
	char out_byte;
	
	for (i=0; i<cmdlen; ++i)					//send command as discrete voltage level in negative logic
	{
		out_byte = addEvenParity(cmd[i]);
		gpioDigitalWrite(gpio, HIGH);			//send start bit
		delayMicros(TIME_PULSE_PERIOD);


		/* send character bits LSB first */
		for(mask = 0x01; mask <= 128; mask<<=1)
		{
			if (!(out_byte & mask))
			{
				gpioDigitalWrite(gpio, HIGH);
			}
			else
			{
				gpioDigitalWrite(gpio, LOW);
			}
			delayMicros(TIME_PULSE_PERIOD);
		}
		gpioDigitalWrite(gpio, LOW); 			/* send stop bit */
		delayMicros(TIME_PULSE_PERIOD);
	}
	sdiState(SDISTATE_LISTENING,gpio);			/* set pin as output to wait for data */
}

static void sdiFlushRxBuff(void)						//clear rx_buffer
{
	memset(rx_buffer,0,SIZE_RXBUFFER);
	//rx_bufferfull = false;
	rx_buffertail = 0;
}

static inline bool sdiUpdateBuffer(char newchar)		//function to insert new received character to the rx_buffer
{
	rx_buffer[rx_buffertail++] = newchar;
	if(rx_buffertail >= SIZE_RXBUFFER)
	{
		sdiFlushRxBuff();
		return false;
	}
	return true;
}




static void sdiFlushTimeNumBuff(void)				//clear buffer of waiting time and number of values
{
	time_count_buffer[IDX_WAITTIME_TIMECOUNT] = 0;
	time_count_buffer[IDX_MEASCOUNT_TIMECOUNT] = 0;
}

static inline void sdiFlushValBuff(void)
{
	uint8_t idx;
	for(idx = 0; idx< SIZE_VALBUFFER; idx++)
	{
		val_buffer[idx] = 0;
	}
	val_buffertail = 0;

}

static char sdiReceiveChar(AgTechIOStruct *gpio)	//read data bits from sensor and convert to ASCII decimal
{
	volatile uint32_t t_start;
	uint8_t mask;
	t_start = HAL_GetTick();
	uint32_t t_current;
	t_current = HAL_GetTick();
	char newchar = 0;
	while (delayTimeDiff(t_start, t_current) < 20 )
	{
		if (gpioDigitalRead(gpio))		/* start bit received */
		{

			delayMicros(TIME_PULSE_PERIOD/2);			/* wait for the pulse to center */
			for(mask = 0x01; mask < 0x80; mask <<= 1) /* get char value in decimal */
			{
				delayMicros(TIME_PULSE_PERIOD);
				if (!(gpioDigitalRead(gpio))) //*************
				{
					newchar |= (char)mask;
				}
				else
				{
					newchar &= (char)~mask;
				}

			}
			delayMicros(TIME_PULSE_PERIOD); /* skip parity bit */
			delayMicros(TIME_PULSE_PERIOD); /* skip stop bit */
			/*
			if((rx_buffertail + 1)%SIZE_RXBUFFER == rx_buffertail)
			{
				//rx_bufferfull = true;
			}
			*/
			return newchar;
		}
		t_current = HAL_GetTick();
	}
	return 0;
}

static bool sdiSaveValue(void)					//convert contents of rx_buffer into actual numerical values
{
	//const uint8_t size_tempval = 20;
	char tempval[SIZE_TEMPVAL];				/* temporary buffer to hold each actual value as array */
	memset(tempval,0,SIZE_TEMPVAL);
	uint8_t idx_tempval;
	idx_tempval = 0;
	uint8_t idx;

	/*check if rx_buffer contents adheres with format*/
	#if SDRIVER_TEST
		uartSHOW(rx_buffer, strlen(rx_buffer));
		uartSHOW("\n", 1);
	#endif
	if( (isdigit((int)rx_buffer[IDX_DEVICEADD_RX]) == 0) ||
		( (rx_buffer[IDX_STARTOFMEAS_RX] != 43) &
		  (rx_buffer[IDX_STARTOFMEAS_RX] != 45) ) )
	{
		return false;
	}

	/*Check for unexpected characters*/
	for(idx=0;idx<strlen(rx_buffer);++idx)
	{
		if ( (isdigit((int)rx_buffer[idx]) == 0) &
			 (rx_buffer[idx] != 45) & (rx_buffer[idx] != 43) &
			 (rx_buffer[idx] != 0) & (rx_buffer[idx] != 46))
		{
			return false;
		}
	}

	for(idx=IDX_STARTOFMEAS_RX; idx<rx_buffertail; ++idx)
	{
		tempval[idx_tempval] = rx_buffer[idx];	/* save char to temporary buffer */
		++idx_tempval;

		if ( (rx_buffer[idx + 1] == 45) || (rx_buffer[idx + 1] == 43) || (rx_buffer[idx + 1] == 0) )	/* check if next if char in ('+','-','') */
		{	
			//val_buffer[val_buffertail] = roundf( strtof(tempval,NULL)*10000 )/10000;	/* convert str to float then update value buffer */
			val_buffer[val_buffertail] = utilCalcRoundDec(strtof(tempval,NULL), SDI_DECPLACES);	/* convert str to float then update value buffer */
			val_buffertail = (uint8_t)(val_buffertail + 1)%SIZE_VALBUFFER;				/* move to next empty val_buffer index */
			memset(tempval,0,SIZE_TEMPVAL);
			idx_tempval = 0;
		}
	}
	return true;
}

static bool sdiGetData(const char *cmd,AgTechIOStruct *gpio)	//receive all char from sensor
{
	char newchar;
	uint32_t t_start,t_lastbyte;
	t_start = t_lastbyte = HAL_GetTick();
	bool CRreceived;
	bool LFreceived;
	CRreceived = false;
	LFreceived = false;
	bool start_rx = false;
	sdiFlushRxBuff();
	sdiSendCommand(cmd,gpio);

	while( !(delayTimeDiff(t_start,HAL_GetTick()) > TIME_TIMEOUT ||
			( (delayTimeDiff(t_lastbyte,HAL_GetTick()) > TIME_RESPONSE) && start_rx) ))

	{
		newchar = sdiReceiveChar(gpio);
		if (newchar)
		{
			start_rx = true;
			t_lastbyte = HAL_GetTick();
			if (newchar == 13)				/* check if char is Carriage Return */
			{
				CRreceived = true;
			}
			else if (newchar == 10)			/* check if char is Line Feed */
			{
				LFreceived = true;
			}
			else
			{
				if(!sdiUpdateBuffer(newchar))	/* add char to buffer */
					return false;
				//sdiUpdateBuffer(newchar);
			}
			if (CRreceived && LFreceived)	/* if true, end of reception */
			{
				if (sdiSaveValue())			/* save the chars in buffer as numeric values */
				{
					return true;
				}
				return false;
			}
		}
	}
	return false;
}

static uint8_t sdiGetWaitTime(const char *cmd_begin,AgTechIOStruct *gpio)	/* get how long should the logger wait and how many values */
{
	uint8_t newchar = 0;
	uint32_t t_start,t_lastbyte, t_current;
	bool start_rx = false;
	t_start = t_lastbyte = t_current = HAL_GetTick();
	bool CRreceived = false;
	bool LFreceived = false;
	//CRreceived = false;
	//LFreceived = false;

	sdiSendCommand(cmd_begin,gpio);			/* send start measurement command */

	while( !(delayTimeDiff(t_start, t_current) > TIME_TIMEOUT ||
			( ((delayTimeDiff(t_lastbyte, t_current) > TIME_RESPONSE) && start_rx)) ))
	{
			newchar = sdiReceiveChar(gpio);		/* read sensor response */
			if (newchar)
			{
				start_rx = true;
				t_lastbyte = HAL_GetTick();
				if (newchar == 0x0D)
				{
					CRreceived = true;
				}
				else if (newchar == 0x0A)
				{
					LFreceived = true;
				}
				else
				{
					if(!sdiUpdateBuffer(newchar))	/* add char to buffer */
						return 0;
				}
				if (CRreceived && LFreceived)	/* if both are true, end of reception */
				{
					#if SDRIVER_TEST
						uartSHOW((uint8_t*)rx_buffer, strlen(rx_buffer));
						uartSHOW("\n", 1);
					#endif
					/* atttn - response format of SDI Start Measurement Command (aM!) */
					uint8_t idxs_wait_time = 1;
					uint8_t idxs_meas_count = 4;
					uint8_t idx;

					/*check if chars in rx_buffer adheres with response format*/
					for(idx = 0; idx < 5; idx++)
					{
						if ( isdigit((int)rx_buffer[idx]) == 0 )
							return 2;
					}

					char wait_time_str[SIZE_WAITTIME];
					memset(wait_time_str,0,SIZE_WAITTIME);
					char meas_count_str[SIZE_MEASCOUNT];
					memset(meas_count_str,0,SIZE_MEASCOUNT);

					strncpy(wait_time_str, &rx_buffer[idxs_wait_time], SIZE_WAITTIME - 1);
					meas_count_str[0] = rx_buffer[idxs_meas_count];

					time_count_buffer[IDX_WAITTIME_TIMECOUNT] = (uint8_t)strtol(wait_time_str,NULL,10);
					time_count_buffer[IDX_MEASCOUNT_TIMECOUNT] = (uint8_t)strtol(meas_count_str,NULL,10);
					delayMicros(20000);
					return 1;
				}
			}
			t_current = HAL_GetTick();
		}
	
	return 3;
}

static uint8_t sdiValAvailable(void)
{
	return val_buffertail;
}


static bool getDataCycle(const char *addr, AgTechIOStruct *gpio)	//cycle for 'send data command'
{
	uint8_t totalcount = 2;
	uint8_t rawcount = 1;
	char senddata_cmd[10];										/* command to issued to SDI sensor for it to send its data */
	int idx;
	char idx_ascii;
	totalcount = time_count_buffer[IDX_MEASCOUNT_TIMECOUNT];
	sdiFlushValBuff();
	for (idx = 0; idx < 9; ++idx)								/* run all possible send data commands to get data */
	{
		idx_ascii = (char)('0' + idx);							/* 0 - 9 */
		snprintf(senddata_cmd,10,"%sD%c!",addr,idx_ascii);		/* 0D0! - 0D9! */

		if (sdiGetData(senddata_cmd,gpio))						/* receive the response from sensor */
		{
			rawcount = sdiValAvailable();						/* check how many characters are in the val_buffer */
			if (rawcount == totalcount)							/* if the number of data based on 'start measurement command' = number of actual received data, return true */
			{
				return true;
			}
			else if (rawcount > totalcount)
			{
				return false;
			}
		}
		else
		{
			return false;
		}

	}
	return false;
}

static inline void sdiFlushAll(void)
{
	sdiFlushValBuff();
	sdiFlushRxBuff();
	sdiFlushTimeNumBuff();
}

uint8_t sdiMain(AgTechIOStruct *gpio, const char *cmd_begin, float *p_values)
{
	int attempt;
	char addr[2];
	uint8_t idx;
	addr[0]	= cmd_begin[0];
	addr[1] = '\0';
	uint8_t sdistat = 0;
	char adc_buff_show[30];
	gpioDigitalWrite(&DLS_OE, LOW);						/* enable level shifter */
	for (attempt = 1; attempt<=3; ++attempt)							/* attempt to read data for 3 times */
	{
		sdiFlushAll();
		rx_buffertail = 0;
		sdistat = sdiGetWaitTime(cmd_begin,gpio);
		//sprintf(adc_buff_show, "MR = %04x\n", sdistat);
	  	//uartSHOW(adc_buff_show, strlen(adc_buff_show));
		  /* clear all buffers */
		if (sdistat == 1)							/* get waiting time and number of data */
		{
			sdiFlushRxBuff();
			if (time_count_buffer[IDX_MEASCOUNT_TIMECOUNT] > 0)		/* if data is available then enter data acquisition */
			{
				uint8_t getdata_attempt;
				delayMillis( (time_count_buffer[IDX_WAITTIME_TIMECOUNT])*1000 + 100);
				for (getdata_attempt=1; getdata_attempt<=3; ++getdata_attempt)	/* attempt to get data from sensor for 3 times */
				{
					if(getDataCycle(addr,gpio))								/* run the command-response cycle for 'send data' command */
					{
						gpioDigitalWrite(&DLS_OE, HIGH);
						for(idx = 0; idx<val_buffertail;++idx)
						{
							p_values[idx] = val_buffer[idx];
						}
						gpioDigitalWrite(&DLS_OE, HIGH);
						return val_buffertail;
					}
					else
						delayMillis(TIME_TRIALINTERVAL);				/* time waiting before trying again */
				}
			}
		}
		delayMillis(TIME_TRIALINTERVAL);
	}
	gpioDigitalWrite(&DLS_OE, HIGH);
	return 0;
}

/*********************************************************************************
  * Revision History
  *	@file      	sdi12.c
  ********************************************************************************
  * @version    v2.1.0
  * @date       12/23/2015
  * @author     D.Lasdoce
  * @changes
  ********************************************************************************
  * @version	v2.0.2
  * @date		09/21/2015
  * @author		D. Ladoce
  * @changes	created file
  ********************************************************************************
  */