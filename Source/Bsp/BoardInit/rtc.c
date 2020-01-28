/**
  *****************************************************************************
  * @file      	rtc.c
  * @author     Hardware Team
  * @version    v2.2.0
  * @date 		04/08/16      
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

/* Includes ==================================================================*/
#include <deployAssert.h>
#include "agBoardInit.h"
#include "rtc.h"

/* Defines ===================================================================*/
#define MAX_MINUTE ((uint8_t)60)

/* External Declarations =====================================================*/
//extern PortConfigStruct PortConfig;
extern bool enter_initmode;
uint32_t count_before_wake;
/* Functions =================================================================*/

/*==============================================================================
 * @brief	converts the value of subsecond register to milli seconds
 * @param	subsec: subsecond value
==============================================================================*/
static uint32_t subSecToMillis(uint32_t subsec)
{
	uint32_t millis = 0;
	millis = (uint32_t)( ((RTC_SPRES - subsec)*1000)/RTC_SPRES);
	return millis;
}

/*============================================================================
 * @brief
 * @param
 * @retval
============================================================================*/
void rtcGetTime(RTC_TimeTypeDef *sTime)
{
	HAL_RTC_GetTime(&RTCHandleStruct, sTime, RTC_FORMAT_BIN);
	sTime->SubSeconds = subSecToMillis(sTime->SubSeconds);
}


/*============================================================================
 * @brief 	Reads the current date and time of rtc
 * @param	sDateTime: pointer to  RTC_DateTimeStruct that will
 *					   hold the values of date and time
 * @retval
============================================================================*/
void rtcGetDateTime(RTCDateTimeStruct *sDateTime)
{
	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_DateTypeDef RTC_DateStruct;
	RTC_TimeStruct.SubSeconds = RTC_SPRES;
	RTC_TimeStruct.TimeFormat = RTC_HOURFORMAT_24;
	RTC_TimeStruct.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	RTC_TimeStruct.StoreOperation = RTC_STOREOPERATION_SET;

	HAL_RTC_GetTime(&RTCHandleStruct, &RTC_TimeStruct, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&RTCHandleStruct, &RTC_DateStruct, RTC_FORMAT_BIN);
	sDateTime->Year = RTC_DateStruct.Year;
	sDateTime->Month = RTC_DateStruct.Month;
	sDateTime->Date = RTC_DateStruct.Date;
	sDateTime->Hours = RTC_TimeStruct.Hours;
	sDateTime->Minutes = RTC_TimeStruct.Minutes;
	sDateTime->Seconds = RTC_TimeStruct.Seconds;
	sDateTime->Millis = subSecToMillis(RTC_TimeStruct.SubSeconds);
}

/*============================================================================
 *@brief 	Sets date and time of rtc
 *@param	sDateTime: pointer to  RTC_DateTimeStruct that will
 *					   hold the values of date and time to be set
============================================================================*/
void rtcSetDateTime(RTCDateTimeStruct *sDateTime)
{
	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_DateTypeDef RTC_DateStruct;
	RTC_DateStruct.Year = sDateTime->Year;
	RTC_DateStruct.Month = sDateTime->Month;
	RTC_DateStruct.Date = sDateTime->Date;
	RTC_DateStruct.WeekDay = DEFAULT_WEEKDAY;

	RTC_TimeStruct.Hours = sDateTime->Hours ;
	RTC_TimeStruct.Minutes = sDateTime->Minutes;
	RTC_TimeStruct.Seconds = sDateTime->Seconds;
	RTC_TimeStruct.SubSeconds = RTC_SPRES;
	RTC_TimeStruct.TimeFormat = RTC_HOURFORMAT_24;
	RTC_TimeStruct.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	RTC_TimeStruct.StoreOperation = RTC_STOREOPERATION_SET;

	HAL_RTC_SetDate(&RTCHandleStruct, &RTC_DateStruct, RTC_FORMAT_BIN);
	HAL_RTC_SetTime(&RTCHandleStruct, &RTC_TimeStruct, RTC_FORMAT_BIN);
}

/*============================================================================
 * @brief 	RTC ALARM Interrupt: set alarm to generate interrupt
 * @param	datetime_ptr: ptr to RTCDateTimeStruct
 * @param	alarm: rtc alarm to be configured (RTC_ALARM_A or RTC_ALARM_B)
============================================================================*/
void rtcSetAlarm(RTCDateTimeStruct *datetime_ptr, uint32_t alarm, uint32_t mask)
{
	RTC_AlarmTypeDef AlarmTypeStruct;
	RTC_TimeTypeDef RTCTimeStruct;

	RTCTimeStruct.Hours = datetime_ptr->Hours;
	RTCTimeStruct.Minutes = datetime_ptr->Minutes;
	RTCTimeStruct.Seconds = datetime_ptr->Seconds;
	RTCTimeStruct.SubSeconds = RTC_SPRES;
	RTCTimeStruct.TimeFormat = RTC_HOURFORMAT_24;
	RTCTimeStruct.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	RTCTimeStruct.StoreOperation = RTC_STOREOPERATION_SET;

	AlarmTypeStruct.AlarmTime = RTCTimeStruct;
	AlarmTypeStruct.Alarm = alarm;
	AlarmTypeStruct.AlarmDateWeekDay = datetime_ptr->Date;
	AlarmTypeStruct.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	AlarmTypeStruct.AlarmMask = mask;
	AlarmTypeStruct.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;

	HAL_RTC_SetAlarm_IT(&RTCHandleStruct, &AlarmTypeStruct, RTC_FORMAT_BIN);
}

/*============================================================================
 * @brief
 * @param
 * @retval
============================================================================*/
void rtcDeactivateAlarm(uint32_t alarm)
{
	HAL_RTC_DeactivateAlarm(&RTCHandleStruct, alarm);
}

/*============================================================================
 * @brief
 * @param
 * @retval
============================================================================*/
void RTC_Alarm_IRQHandler(void)
{
	HAL_RTC_AlarmIRQHandler(&RTCHandleStruct);
}

/*============================================================================
 * @brief
 * @param
 * @retval
============================================================================*/
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	if(__HAL_RTC_ALARM_GET_IT(hrtc, RTC_IT_ALRA))
	{
		rtcDeactivateAlarm(RTC_ALARM_A);
		/*======insert code to be executed for ALARM A here=====*/
		read_gps = true;
	}

	if(__HAL_RTC_ALARM_GET_IT(hrtc, RTC_IT_ALRB))
	{
		rtcDeactivateAlarm(RTC_ALARM_B);
		/*======insert code to be executed for ALARM B here=====*/
	}
	/*======insert code to be executed for both ALARMS here=====*/
}

/*============================================================================
 * @brief 	RTC WAKEUP Interrupt
 * @param
 * @retval
============================================================================*/
void rtcSetWakeupTimer(uint32_t intertval)
{
  	char rtc_warning[50];
  	HAL_RTCEx_DeactivateWakeUpTimer(&RTCHandleStruct);
	if(HAL_RTCEx_SetWakeUpTimer_IT(&RTCHandleStruct, intertval,
		RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
	{
	  	sprintf(rtc_warning, "\nRTC: Set wakeup timer failed, retrying...\n");
		uartSHOW(rtc_warning, strlen(rtc_warning));
		delayMillis(10);
		
	  	if(HAL_RTCEx_SetWakeUpTimer_IT(&RTCHandleStruct, intertval,
		RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
		{
		  	sprintf(rtc_warning, "\nRTC: FAILED TO SET WAKEUP TIMER !!!!!!!\n");
			uartSHOW(rtc_warning, strlen(rtc_warning));
			assert(0);
		}
		uartSHOW(rtc_warning, strlen(rtc_warning));
	}
	sprintf(rtc_warning, "\nRTC: Set wakeup timer SUCCESS!\n");
	uartSHOW(rtc_warning, strlen(rtc_warning));
}

/*============================================================================
 * @brief
 * @param
 * @retval
============================================================================*/
void rtcDisableWakeupTimer(void)
{
	HAL_RTCEx_DeactivateWakeUpTimer(&RTCHandleStruct);
}

/*============================================================================
 * @brief
 * @param
 * @retval
============================================================================*/
void RTC_WKUP_IRQHandler(void)
{
	HAL_RTCEx_WakeUpTimerIRQHandler(&RTCHandleStruct);
	if(!run_mode)
	{
		SystemClock_Config(RCC_PLLP_DIV4);
		SystemCoreClockUpdate();
		delayInit();
		uartInit();
		enter_initmode = true;
		check_battery = true;
		buzzWaked();
		#if PRINT_PROCESSES_IN_UART
			const char *msg_sleep = "\n^^^^^ WAKED BY RTC ^^^^^\n";
			uartSHOW((uint8_t*)msg_sleep, strlen(msg_sleep));
		#endif
	}
	else
	{
	  	#if PRINT_PROCESSES_IN_UART
			const char *msg_sleep = "\n^^^^^ SYSTEM ALREADY AWAKE ^^^^^\n";
			uartSHOW((uint8_t*)msg_sleep, strlen(msg_sleep));
		#endif
	}
	rtcDisableWakeupTimer();
	
	/*======insert code to be executed here=====*/
	if(RTCHandleStruct.State != HAL_RTC_STATE_READY)
	{

		/*trial function*/
	}	
}

/*============================================================================
 * @brief
 * @param
 * @retval
============================================================================*/
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
//  	if(!run_mode)
//	{
//		SystemClock_Config(RCC_PLLP_DIV8);
//		SystemCoreClockUpdate();
//		delayInit();
//		uartInit();
//		enter_initmode = true;
//		check_battery = true;
//		buzzWaked();
//		#if PRINT_PROCESSES_IN_UART
//			const char *msg_sleep = "\n^^^^^ WAKED BY RTC ^^^^^\n";
//			uartSHOW((uint8_t*)msg_sleep, strlen(msg_sleep));
//		#endif
//	}
//	else
//	{
//	  	#if PRINT_PROCESSES_IN_UART
//			const char *msg_sleep = "\n^^^^^ SYSTEM ALREADY AWAKE ^^^^^\n";
//			uartSHOW((uint8_t*)msg_sleep, strlen(msg_sleep));
//		#endif
//	}
//	rtcDisableWakeupTimer();
//	
//	/*======insert code to be executed here=====*/
//	if(hrtc->State != HAL_RTC_STATE_READY)
//	{
//
//		/*trial function*/
//	}	
}

/*============================================================================
 * @brief
 * @param
 * @retval
============================================================================*/
uint8_t rtcMinDiff(int8_t min_start, int8_t min_end)
{
	uint8_t time_diff;
	if(min_start > min_end)
		time_diff = min_end + MAX_MINUTE - min_start;
	else
		time_diff = min_end - min_start;
	return (uint8_t)time_diff;
}


/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void rtcSampleUsage(void)
{
	RTCDateTimeStruct RTCDateTimeStructCurrent;
	RTCDateTimeStruct RTC_DateTimeStructSetup;

	/*set values of RTC_DateTimeStructSetup*/
	RTC_DateTimeStructSetup.Year	= DEFAULT_YEAR;
	RTC_DateTimeStructSetup.Month = DEFAULT_MONTH;
	RTC_DateTimeStructSetup.Date = DEFAULT_DATE;
	RTC_DateTimeStructSetup.Hours = DEFAULT_HOURS;
	RTC_DateTimeStructSetup.Minutes = DEFAULT_MINUTES;
	RTC_DateTimeStructSetup.Seconds = DEFAULT_SECONDS;

	/*set rtc time to RTC_DateTimeStructSetup*/
	rtcSetDateTime(&RTC_DateTimeStructSetup);

	/*get date time and store it to RTCDateTimeStructCurrent*/
	rtcGetDateTime(&RTCDateTimeStructCurrent);
	rtcGetDateTime(&RTCDateTimeStructCurrent);

	/*set alarm to next 5 seconds*/
	RTCDateTimeStructCurrent.Seconds = RTCDateTimeStructCurrent.Seconds + 5;

	/*set alarm*/
	//rtcSetAlarm(&RTCDateTimeStructCurrent, RTC_ALARM_A);

	//delayMillis(4000);
	rtcGetDateTime(&RTCDateTimeStructCurrent);
	//delayMillis(4000);
}


/*******************************************************************************
  * Revision History
  *	@file      	rtc.c
  ******************************************************************************
  * @version    v2.2.0
  * @date       04/08/16
  * @author     D.Lasdoce
  * @changes
  ******************************************************************************
  * @version    v2.1.0
  * @date       12/23/2015
  * @author     D.Lasdoce
  * @changes	created file
  ******************************************************************************
  */
