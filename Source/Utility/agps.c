/********************************************************************************
 * @file      	:	agps.c
 * @author		:	Hardware Team, Agtech Labs Inc.
 * @version		:	V2.0.1
 * @date		:	10/27/2015
 * @brief		:	Global Positioning System Module
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

/* Includes =================================================================*/
#include "agps.h"

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stm32f7xx.h>

#include "agBoardInit.h"
#include "delay.h"
#include "gpio.h"
#include "rtc.h"
#include "agdatim.h"
#include "utilMem.h"
#include "extRTC.h"
#include "mspSpi.h"

/* Definition ===============================================================*/
#define CLEARVALUE			(uint8_t)0
#define EQUAL				(uint8_t)0
#define INVERT_SIGN			(int8_t)-1
#define LATITUDE_MAXSIZE 	(uint8_t)15
#define LONGITUDE_MAXSIZE 	(uint8_t)15
#define TIME_MAXSIZE 		(uint8_t)11
#define DATE_MAXSIZE 		(uint8_t)7
#define STATUS_MAXSIZE 		(uint8_t)2
#define LATDIR_MAXSIZE 		(uint8_t)2
#define LONGDIR_MAXSIZE 	(uint8_t)2

#define CHKSUM_CHARSIZE     (uint8_t)3	    /*byte size of checksum character*/
#define BUFF_SIZE           (uint8_t)15     /*buffer byte size of rmcfield*/
#define NMEATYP_SIZE        (uint8_t)5 	    /*byte size of NMEA type*/
#define MAX_FRAMESIZE       (uint8_t)100    /*initial buff size of NMEA frame*/

#define MIL_T0_SEC			(uint32_t)1000

/* Typedefs =================================================================*/
typedef struct									 /* structure of GPS raw data*/
{
	char latitude[LATITUDE_MAXSIZE];
	char longitude[LONGITUDE_MAXSIZE];
	char time[TIME_MAXSIZE];
	char date[DATE_MAXSIZE];
	char status[STATUS_MAXSIZE];
	char latdir[LATDIR_MAXSIZE];
	char longdir[LONGDIR_MAXSIZE];
}gpsRawStruct;

/* MACROS ===================================================================*/
/* External Declarations ====================================================*/
GpsSysStatStruct gps_systat_data;			/* external variable of GPS data */

/* Global Declarations ======================================================*/
gpsRawStruct gps; 						  /* global variable of GPS raw data */

/* Functions ================================================================*/
/******************************************************************************
 * @brief	bootup process of GPS module
 * @param	none
 * @retval  GPS SYS-ON pin status
 *****************************************************************************/
static GpsStat gpsBoot(void) 
{
	const uint8_t INBTWN_PULSE_DLY = 100; /* tested delay in between = 100ms */
	const uint16_t REST_PULSE_DLY = 2000;			 /* rest pulse for 1 sec */
	uint32_t t_start;					 	 /* base reference of time delay */

	 /* supply 1.8Vdc to GPS module */
	gpioDigitalWrite(&GPS_RESET, HIGH);
	gpioMode(&GPS_ON_OFF, MODE_OUTPUT);
	/* GPS module booting up process */
	t_start = HAL_GetTick();
	while( delayTimeDiff(t_start, HAL_GetTick()) < GPS_BOOTIME ) {
		/* 0V as GPS on-off pin logic high */
		gpioDigitalWrite(&GPS_ON_OFF, HIGH);
		delayMillis(INBTWN_PULSE_DLY);
		gpioDigitalWrite(&GPS_ON_OFF, LOW);
		
		delayMillis(REST_PULSE_DLY);

		if ( gpioDigitalRead(&GPS_SYSON) == true ) 
        {
            #if PRINT_PROCESSES_IN_UART
                const char *temp_display2 = "$GPS BOOT UP SUCCESS....\n\n";
                uartSHOW((uint8_t*)temp_display2, strlen(temp_display2));
            #endif
			return GPS_SUCCESSFUL;
		}
	}
	return GPS_FAILED;
}

GpsStat gpsPowerON(bool state) 
{
	const uint8_t INBTWN_PULSE_DLY = 100; /* tested delay in between = 100ms */
	const uint16_t REST_PULSE_DLY = 2000;			 /* rest pulse for 1 sec */
	char uartout[50];
	uint32_t t_start;					 	 /* base reference of time delay */
	
	 /* supply 1.8Vdc to GPS module */
	gpioDigitalWrite(&GPS_RESET, HIGH);
	gpioMode(&GPS_ON_OFF, MODE_OUTPUT);
	/* GPS module booting up process */
	t_start = HAL_GetTick();
	
	mspSetPowerRails(MSPCMD2_PR_1V8, state);
	delayMillis(500);
	mspLedDiagnostic(LED1_STAT_RED, state);
    delayMillis(100);
		
	if(state)
	{
		if ( gpioDigitalRead(&GPS_SYSON) == true ) 
		{
			#if PRINT_PROCESSES_IN_UART
				sprintf(uartout, "\n$GPS ALREADY ON....\n\n");
				uartSHOW((uint8_t*)uartout, strlen(uartout));
			#endif
			return GPS_SUCCESSFUL;
		}
		
		while( delayTimeDiff(t_start, HAL_GetTick()) < GPS_BOOTIME ) 
		{
			/* 0V as GPS on-off pin logic high */
			gpioDigitalWrite(&GPS_ON_OFF, HIGH);
			delayMillis(INBTWN_PULSE_DLY);
			gpioDigitalWrite(&GPS_ON_OFF, LOW);
			delayMillis(REST_PULSE_DLY);

			if ( gpioDigitalRead(&GPS_SYSON) == true ) 
			{
				#if PRINT_PROCESSES_IN_UART
					sprintf(uartout, "$GPS BOOT UP SUCCESS....\n\n");
					uartSHOW((uint8_t*)uartout, strlen(uartout));
				#endif
				return GPS_SUCCESSFUL;
			}
		}
	}
	else
	{
		if ( gpioDigitalRead(&GPS_SYSON) == LOW ) 
		{
			#if PRINT_PROCESSES_IN_UART
				sprintf(uartout, "$GPS ALREADY OFF....\n");
				uartSHOW((uint8_t*)uartout, strlen(uartout));
			#endif
			return GPS_SUCCESSFUL;
		}
		
		while( delayTimeDiff(t_start, HAL_GetTick()) < GPS_BOOTIME ) 
		{
			/* 0V as GPS on-off pin logic high */
			gpioDigitalWrite(&GPS_ON_OFF, HIGH);
			delayMillis(INBTWN_PULSE_DLY);
			gpioDigitalWrite(&GPS_ON_OFF, LOW);
			delayMillis(REST_PULSE_DLY);

			if ( gpioDigitalRead(&GPS_SYSON) == LOW ) 
			{
				#if PRINT_PROCESSES_IN_UART
					sprintf(uartout, "$GPS PWR OFF SUCCESS....\n");
					uartSHOW((uint8_t*)uartout, strlen(uartout));
				#endif
				return GPS_SUCCESSFUL;
			}
		}
	}
	return GPS_FAILED;
}

/******************************************************************************
 * @brief	shutoff GPS module by cutting the power supply
 * @param	none
 * @retval  none
 *****************************************************************************/
static void gpsShutDown(void) {
	//gpioDigitalWrite(&PWR_1V8_EN, LOW);
}

/******************************************************************************
 * @brief	validate NMEA frame by its cheksum
 * @param	nmea_ptr: pointer of NMEA frame
 * 			nmea_size: byte size of NMEA frame
 * @retval  GPS SYS-ON pin status
 *****************************************************************************/
static bool gpsValidateNmea(char *nmea_ptr, uint32_t nmea_size) {
	const uint8_t HEXBASE = 16;				  /* number system of string data*/
	char nmeachck[CHKSUM_CHARSIZE];				 /* checksum from NMEA frame */
	char character; 							  /* NMEA character in frame */
	uint8_t chksum;				  /* generated checksum from NMEA characters */
	uint32_t ptr_idx;					 /* index of NMEA character in frame */

	/* initialization */
	memset(nmeachck, CLEARVALUE, sizeof(nmeachck));
	ptr_idx = CLEARVALUE;

	/* cheksum validation process */
	if ( nmea_ptr[ptr_idx++] == '$' ) {
		/* get first character */
		character = nmea_ptr[ptr_idx++];
		chksum = character;
		while ( ( character != '*' ) && ( ptr_idx < nmea_size ) ) {
			/* get next character */
			character = nmea_ptr[ptr_idx++];
			if ( character != '*' ) {
				chksum = chksum ^ character;
			}
		}
		/* at this point we are at * or at end of string */
		strncpy(nmeachck, &nmea_ptr[ptr_idx], sizeof(nmeachck));
		if ( chksum != strtol(nmeachck, NULL, HEXBASE) ) {
			return false;
		}
	}
	else {
		return false;
	}
	return true;
}

/******************************************************************************
 * @brief	Data Logging of GPS data into external variable
 * @param	none
 * @retval  none
 *****************************************************************************/
static void gpsLog(void) {
	const float MINTODEG = 0.6;	  		   /* conversion of minute to degree */
	const int INTTODEC = 100;				 /* decimal adjustment of degree */
	RTCDateTimeStruct conv_datim;		  /* RTC formatted date time form GPS*/
	double longint, latint;					   /* whole value of coordinates */
	double longfract, latfract;				 /* decimal value of coordinates */
	double longvar, latvar;					   /* exact value of coordinates */

	/* initialization */
	memset(&conv_datim, CLEARVALUE, sizeof(RTCDateTimeStruct));
	memset(&gps_systat_data, CLEARVALUE, sizeof(GpsSysStatStruct));

	/* processing of GPS datetime data */
	agdatimGpsParser(gps.date, gps.time, &conv_datim);
	rtcSetDateTime(&conv_datim); 	   /* !!! temporary setup of system time */
	exRTCWriteTime(&conv_datim);
	gps_systat_data.datetime = conv_datim;

	/* processing of GPS coordinates data */
	longfract = modf(strtod(gps.longitude, NULL) / INTTODEC, &longint)
			/ MINTODEG;
	longvar = longint + longfract;
	gps_systat_data.longitude = (strcmp(gps.longdir, "W") == EQUAL)
			? longvar * INVERT_SIGN: longvar;

	latfract = modf(strtod(gps.latitude, NULL) / INTTODEC, &latint) / MINTODEG;
	latvar = latint + latfract;
	gps_systat_data.latitude = (strcmp(gps.latdir, "S") == EQUAL)
			? latvar * INVERT_SIGN: latvar;
}

/******************************************************************************
 * @brief	parsing of NMEA frame into global variable of GPS raw data
 * @param	nmea_ptr: pointer of NMEA frame
 * 			nmea_size: byte size of NMEA frame
 * @retval  none
 *****************************************************************************/
static void gpsNmeaParser(char *nmea_ptr, uint16_t nmea_size) {
	const uint8_t START_IDX = 1;  	  /* index of NMEA frame starting string */
	uint8_t rmcfield[BUFF_SIZE];	/* buffer for every NMEA RMC field value */
	uint8_t rmcfield_size;			   /* size of every NMEA RMC field value */
	uint8_t rmcfield_idx;					/* index of every NMEA RMC field */
	uint8_t rmc_char;				  /* buffer for every NMEA RMC character */
	enum field {										  /* NMEA field type */
		UTCTIME	= 1,
		STATUS,
		LATITUDE,
		LATITUDE_DIRECTION,
		LONGITUDE,
		LONGITUDE_DIRECTION,
		DATE = 9
	};

	/* initialization */
	memset(&gps, CLEARVALUE, sizeof(gps));
	memset(rmcfield, CLEARVALUE, sizeof(rmcfield));
	rmcfield_size = CLEARVALUE;
	rmcfield_idx = CLEARVALUE;

	/* parsing process */
	for ( rmc_char = START_IDX; rmc_char < nmea_size; rmc_char++ ) {
		if ( nmea_ptr[rmc_char] != ',' ) {
			rmcfield[rmcfield_size++] = nmea_ptr[rmc_char];
			continue;
		}
		switch(rmcfield_idx) {
			case UTCTIME:
				memcpy(gps.time, rmcfield, rmcfield_size);
				break;
			case STATUS:
				memcpy(gps.status, rmcfield, rmcfield_size);
				break;
			case LATITUDE:
				memcpy(gps.latitude, rmcfield, rmcfield_size);
				break;
			case LATITUDE_DIRECTION:
				memcpy(gps.latdir, rmcfield, rmcfield_size);
				break;
			case LONGITUDE:
				memcpy(gps.longitude, rmcfield, rmcfield_size);
				break;
			case LONGITUDE_DIRECTION:
				memcpy(gps.longdir, rmcfield, rmcfield_size);
				break;
			case DATE:
				memcpy(gps.date, rmcfield, rmcfield_size);
				break;
			default:
				break;
		}
		memset(rmcfield, CLEARVALUE, sizeof(rmcfield));
		rmcfield_size = CLEARVALUE;
		rmcfield_idx++;
	}
}

/******************************************************************************
 * @brief	full process of GPS
 * @param	process_time: time allowance to complete data acquisition
 * @retval  process result status
 ******************************************************************************/
GpsStat gpsProcess(uint32_t process_time) {
	GpsStat retstat; 					   /* return status of this function */
	char nmea_ptr[MAX_FRAMESIZE];							 /* buffer pointer of NMEA frame */
	uint8_t nmea[NMEATYP_SIZE];						  /* buffer of NMEA type */
	uint8_t gpsrx; 			/* buffer of received character from GPS UART RX */
	uint16_t nmea_size; 					   /* varying size of NMEA frame */
	uint32_t t_start;						 /* base reference of time delay */
	process_time *= MIL_T0_SEC;
	/* initial state*/
	retstat = GPS_RUNNING;
	UART_HandleTypeDef *huart  = &UART7HandlerDef;

	/* initialization */
    nmea_size = CLEARVALUE;

	/* NMEA data acquisition process */
	t_start = HAL_GetTick();
	while( (delayTimeDiff(t_start, HAL_GetTick()) < process_time)
		  && (run_mode == true)) 
    {
		if ( uartRxGps(&gpsrx, sizeof(char)) != HAL_OK ) {
//			continue;
			HAL_UART_DeInit(&UART7HandlerDef);
			HAL_UART_Init(&UART7HandlerDef);
		  	//UARTRESET(huart);
			//__HAL_UART_ENABLE(&UART7HandlerDef);
//		    CLEAR_BIT(huart->Instance->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
// 			 CLEAR_BIT(huart->Instance->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));
//		  __HAL_UART_CLEAR_FEFLAG(&UART7HandlerDef);
//		  UART7HandlerDef.Instance->RQR = UART7HandlerDef.Instance->RQR | 0x08;
		}
		if ( gpsrx == '$' ) {
			memset(nmea_ptr, CLEARVALUE, nmea_size);
			nmea_size = CLEARVALUE;
		}
		nmea_ptr[nmea_size++] = gpsrx;
		if( gpsrx != '\n' ) {
			continue;
		}
        
		memcpy(nmea, nmea_ptr + 1, NMEATYP_SIZE);
		if( memcmp(nmea, "GPRMC", NMEATYP_SIZE) == EQUAL
				&& gpsValidateNmea(nmea_ptr, nmea_size) == true ) {
            #if PRINT_PROCESSES_IN_UART
            uartSHOW((uint8_t*)nmea_ptr, nmea_size);
            #endif
			gpsNmeaParser(nmea_ptr, nmea_size);
			if ( memcmp(gps.status, "A", 1) == EQUAL ) 
            {
                #if PRINT_PROCESSES_IN_UART
                    const char *temp_display3 = "\n&LOGGING COORDINATES....\n";
                    uartSHOW((uint8_t*)temp_display3, strlen(temp_display3));
                #endif
				gpsLog();
				retstat = GPS_SUCCESSFUL;
//				buzzWaked();
//				buzzWaked();
//				buzzWaked();
//				buzzWaked();
				break;
			}
		}
	}

	return retstat;
}

/******************************************************************************
 * @brief	full process of GPS
 * @param	mode: execution mode either on-check or on-boot
 * 			process_time: time allowance to complete data acquisition
 * @retval  process result status
 ******************************************************************************/
GpsStat gpsExec(GpsMode mode,uint32_t process_time) {
	GpsStat retstat;						 /* current state of GPS process */
        
	/* GPS execution process */
    	/* boot-up GPS module*/
	const char *printout1 = "\n\n&GPS start\n";
    uartSHOW((uint8_t*)printout1, strlen(printout1));
	
	if( gpsPowerON(true) != GPS_SUCCESSFUL ) 
		return GPS_FAILED;


//	uartRxGps(&gpsrx, sizeof(char));
//		HAL_UART_DeInit(&UART7HandlerDef);
//		HAL_UART_Init(&UART7HandlerDef);
		retstat = gpsProcess(process_time);
        if(retstat == GPS_SUCCESSFUL)
        {
          delayMillis(15000);
          gpsProcess(process_time);
//          delayMillis(15000);
//          gpsProcess(process_time);     
        }

    /* terminating process */
	if (retstat == GPS_SUCCESSFUL)
	{
		if( gpsPowerON(false) != GPS_SUCCESSFUL ) 
			return GPS_FAILED;
	}
    const char * printout3 = "\n&GPS end\n";
    uartSHOW((uint8_t*)printout3, strlen(printout3));
        
	return retstat;
}

/******************************************************************************
 * Revision History
 *	@file      	agps.c
 ******************************************************************************
 * @version	v2.0.0
 * @date	10/13/2015
 * @author	JWF
 * @changes	- initial version
 * 			- done on booting up gps module
 * 			- done on acquiring data
 ******************************************************************************
 * @version	v2.0.1
 * @date	10/27/2015
 * @author	JWF
 * @changes	- full-functional release
 * 			- internal function:
 * 				+ gpsBoot
 * 				+ gpsShutDown
 * 				+ gpsValidateNmea
 * 				+ gpsLog
 * 				+ gpsNmeaParser
 *				+ gpsProcess
 * 			- external function:
 * 				+ gpsExec
 *****************************************************************************
 * @version	Beta Version 2.3.0
 * @date		11-April-2016
 * @author		JWF
 * @initial release for version 2.3
 *****************************************************************************
 */