/*******************************************************************************
  * @file      	main.c
  * @author		Hardware Team
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
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <exceptionHandlers.h>

#include "productCfg.h"
#include "agBoardInit.h"
#include "dmMain.h"
#include "delay.h"
#include "rtc.h"
#include "intFlash.h"
#include "rdBnrMain.h"
#include "port.h"
#include "daqProcess.h"
#include "stm32f7xx_hal.h"
#include "cell.h"
#include "radio.h"
#include "extFlash.h"
#include "bsp_pwm.h"
#include "gpio.h"
#include "extRTC.h"
#include "agps.h"
#include "daqAdcSpi.h"
#include "delay.h"
#include "utilMem.h"
#include "utilCalc.h"
#include "sdMain.h"
#include "tempApiCodedList.h"
#include "port.h"
#include "utilPwrMonitor.h"
#include "daqSensInterface.h"
#include "isrMain.h"
#include "daqDavis.h"
#include "i2c.h"
#include "equations.h"
#include "cloud.h"
#include "mspSpi.h"
#include "bringup_testcodes.h"
#include "ble.h"
#include "daqSysStat.h"

#define BM_RSTF			(1)
#define BP_RSTF_LPWR	(31)
#define BP_RSTF_WWDG	(30)
#define BP_RSTF_IWDG	(29)
#define BP_RSTF_SFT		(28)
#define BP_RSTF_POR		(27)
#define BP_RSTF_PIN		(26)
#define BP_RSTF_BOR		(25)
#define BP_RSTF_RMVF	(24)

/* External Declarations=====================================================*/
bool execute_routine = true;
bool enter_initmode;
bool read_gps;
bool run_mode;
bool gps_running;
bool account_id_set;
bool check_battery = true;
volatile bool uart_int;
volatile bool davisdth_flasgset = false;
volatile bool davisdth_sck_state = false;
float adc_therm_ref = 3.3;
uint8_t rxbuff_rs485[8];
uint32_t ota_fwsize;
PowerModeEnum current_power_mode;
LEDPatternEnum led_pattern;
AgTechIOStruct *io_dth_sck = NULL;
GpsStat gps_time_success;
IWDG_HandleTypeDef IwdgHandle;
DeviceInfoStruct *DEVICE_INFO = NULL;
BleFWVersionStruct BLE_FWVER;
RTCDateTimeStruct RTCDateTimeStructCurrent;
RTCDateTimeStruct RTCDateTimeStructSleep;
SleepAttrStruct sleep_attr;

#if DEVICE_TYPE == DEVICETYPE_GATE
    uint8_t AVAILABLE_MODBUS_ID = RDBNR_BASE_MODBUSID;
	uint32_t NODE_COUNT = 0;
	uint32_t ACCOUNT_ID = 0;
	DeviceInfoStruct *DEVICE_INFO_LIST = NULL;
	SensorCodeStruct *SENSOR_CODE_LIST = NULL;
	SensorTypeStruct *SENSOR_TYPE_LIST;
	SensorUomStruct  *SENSOR_UOM_LIST;
	char *SERIAL_NUMTEST[10];
#endif



#if RTC_BASED_CYCLE
	uint16_t RTC_SLEEPTIME = 600;	//seconds
	uint16_t NODEWAIT_TIMEOUT = 30;	//minutes
	bool is_devnum_set;
#else
	bool exec_daq1, exec_daq2, exec_daq3, exec_daq4, exec_sysstat, exec_poll_post;
#endif
	

/* Functions Prototypes =====================================================*/
static void displayTime(void);
static void postLoop(RecordTypEnum rec_type);
static void setPowerRails(bool state);
static void pollLoop(RdApiEnum radio_api);
static void CPU_CACHE_Enable(void);
static void delaySeconds(uint32_t sec);
static void checkResetRegister(void);
void sleepCheck(void);
void checkBattery(void);


#if RTC_BASED_CYCLE
	static void rtcTimerRoutine(void);
	static void enterSleepMode(void);
	void enterInitMode(void);
#else
	static void timerInterruptRoutine(void);
#endif

#pragma optimize=none
/*==============================================================================
* @brief     
* @param
* @retval
*=============================================================================*/
int main(int argc, char* argv[])
{
	/******************************INITIALIZATIONS*****************************/
  	CPU_CACHE_Enable();
	HAL_Init();
	SystemClock_Config(RCC_PLLP_DIV4);
	SystemCoreClockUpdate();
	current_power_mode = POWERMODE_HIGH;
	bool is_first_run = true;
	run_mode = true;
	led_pattern = LED_RUNMODE;
	enter_initmode = true;
	is_devnum_set = false;
    sleep_attr.nmm_status = RADNETMAINTENANCEMODE_DISABLE;
    sleep_attr.nmm_timer_min = 0;
	
	#if RTC_BASED_CYCLE
		sleep_attr.enter_sleepmode = uart_int = false;
		gps_running = false;
		account_id_set = false;
	#else
		exec_daq1 = exec_daq2 = exec_daq3 = exec_daq4 = exec_poll_post = false;
		exec_sysstat = true;
	#endif

	/***************************Set Watchdog Timer*****************************/
	IwdgHandle.Instance = IWDG;
  	IwdgHandle.Init.Prescaler = IWDG_PRESCALER_256;
  	IwdgHandle.Init.Reload    = 1750;
	IwdgHandle.Init.Window = IWDG_WINDOW_DISABLE;
	#if ENABLE_WATCHDOG
		HAL_IWDG_Init(&IwdgHandle);
		HAL_IWDG_Refresh(&IwdgHandle);
	#endif
	
	delayInit();
	agBoardInit();
	irqConfig();
	rtcInit();
	delayMillis(2000);
	buzzBootSuccess();	

	logWrite("\n\n\n******************************Program Start******************************\n");
	
	/**********************Port, Databuff, WDOG Kicker, SD init****************/
	timerIrqEnable();	
	dmInitialize();
	portInit();
	LPTIMInit();
	
	#if DEVICE_TYPE == DEVICETYPE_NODE
		logWrite("DEFAULT NODE WAIT TO: %d\n", NODEWAIT_TIMEOUT);	
	#endif
	logWrite("DEFAULT SLEEP PERIOD: %d\n\n", RTC_SLEEPTIME);

	/*Display Reasons of reset*/
	checkResetRegister();
	memset(&gps_systat_data, 0, sizeof(GpsSysStatStruct));
	
	#if CELLTEST
		cellON_OFF(true);
		cellModemTestMode();
	#endif
		
//	#if DEVICE_TYPE == DEVICETYPE_GATE
//		sdUpdateAcctID(0);
//	#endif
	
	#if ENABLE_AUTOASSOC
		sdUpdateDeviceRadioModbusID(MODBUS_DEFAULT);
	#endif
	//cellUpdateCheck();

	mspLedDiagnostic(LED1_STAT_RED ,MSPCMD3_PR_OFF);
	delayMillis(100);
	mspLedDiagnostic(LED2_STAT_GREEN, MSPCMD3_PR_OFF);
	delayMillis(500);
		
	mspLedDiagnostic(LED1_STAT_RED ,MSPCMD3_PR_ON);
	delayMillis(500);
	mspLedDiagnostic(LED2_STAT_GREEN, MSPCMD3_PR_ON);
	delayMillis(500);
	mspLedDiagnostic(LED1_STAT_RED ,MSPCMD3_PR_OFF);
	delayMillis(500);
	mspLedDiagnostic(LED2_STAT_GREEN, MSPCMD3_PR_OFF);
	delayMillis(500);
	
	/****************************Device Info Display***************************/
	const char *printout2 =
			"\nRUNNING!!!";
	
	#if RTC_BASED_CYCLE
		#if DEVICE_TYPE == DEVICETYPE_NODE
			const char *msg_sleep = "\nNO RESPONSE FROM GATE\n";
			int8_t min_prevcycle = 0;
			int8_t sec_uart_it_check = 0;
		#endif
	#endif

	/*************************Power Status Guard*******************************/
	checkBattery();
	
	/******************BLE and Radio Parameter Setting*************************/
	#if ENABLE_RADIO 
		#if DEVICE_TYPE == DEVICETYPE_GATE
        	AVAILABLE_MODBUS_ID = RDBNR_BASE_MODBUSID;
		#endif
		radioEnable(true);
		radioInit();
	#endif
		
	#if ENABLE_BLE
		bleInit();
	#endif
		
	buzzEnable(1);
	cellForcedShutDown();
	
	#if ENABLE_RADIO 
		delayMillis(500);
		radioEnable(false);
	#endif

	bool state = 0;
	

    logWrite("DEVICE_SN: %s\nARM FW Version %d.%d.%d.%d\n"\
				"MODBUS_ID: 0x%02x",
            	DEVICE_INFO->device_sn, 
				DEVICE_INFO->arm_fwver.fwver_main,
				DEVICE_INFO->arm_fwver.fwver_submain,
				DEVICE_INFO->arm_fwver.fwver_minor,
				DEVICE_INFO->arm_fwver.fwver_dev,
				DEVICE_INFO->rad_modbus_id);
	
	char ver_str[13];
	memset(ver_str, 0, 13);
	mspSoftwareVersion(&ver_str[0]);
	logWrite("\nMSP FW ");
	logWrite(ver_str);
	
	logWrite("\n\n1 0x%04x %d\n2 0x%04x %d\n3 0x%04x %d\n4 0x%04x %d\n\n"
						,PortConfig.P1.sensor_code, PortConfig.P1.status
						,PortConfig.P2.sensor_code, PortConfig.P2.status
						,PortConfig.P3.sensor_code, PortConfig.P3.status
						,PortConfig.P4.sensor_code, PortConfig.P4.status);
	
	logWrite("NetworkID = %d\n", DEVICE_INFO->network_id);
	
	#if DEVICE_TYPE == DEVICETYPE_GATE
		logWrite("ACCOUNT_ID = %d\n", ACCOUNT_ID);
		
		logWrite("NODE_COUNT = %d\n", NODE_COUNT);
		
		for(uint8_t i = 1; i <= NODE_COUNT; i++)
		{
		  	logWrite("%s\n", DEVICE_INFO_LIST[i].device_sn);
		}
	#endif
	displayTime();
	

	/*****************Cell Modem and Cloud Initialization**********************/
	setPowerRails(true);
	#if DEVICE_TYPE == DEVICETYPE_GATE
		#if ENABLE_POSTING
			cellON_OFF(true);
			cellSetupParameters();
			#if CELLTYPE == CELLTYPE_CDMA
				cellProvision();
			#endif
			#if ENABLE_AUTONODELIST			
				if(ACCOUNT_ID == 0)
				{
					account_id_set = true;
					cloudGateInfoGet(DEVICE_INFO->device_sn);
				}
				else
				{
					delayMillis(2000);
					cellON_OFF(false);
				}
			#endif
		#endif
 	#endif	
	
	/***********************************GPS************************************/
	changeClockSpeed(POWERMODE_LOW);
	#if ENABLE_GPS
		buzzEnable(1);
		gps_running = true;
		led_pattern = LED_RUNMODE_GPS;
		gps_time_success = gpsExec(0, GPS_TIMEOUT);
		read_gps = false;
		RTCDateTimeStruct *alarmForGPS;
		alarmForGPS = memAlloc(sizeof(RTCDateTimeStruct));
		alarmForGPS->Seconds = 0;
		alarmForGPS->Minutes = 0;
		alarmForGPS->Hours = 7;
		alarmForGPS->Date = 1;
		displayTime();
		rtcSetAlarm(alarmForGPS, RTC_ALARM_A, RTC_ALARMMASK_DATEWEEKDAY);
		free(alarmForGPS);
		gps_running = false;
		led_pattern = LED_RUNMODE;
	#endif
	
	#if MOBILE_TEST
		buzzWaked();
		while(1);
	#endif
	
	/***********************Cell Date and Time Setting*************************/
	#if DEVICE_TYPE == DEVICETYPE_GATE
		changeClockSpeed(POWERMODE_HIGH);
		#if ENABLE_CELLGETTIME
			if(gps_time_success != GPS_SUCCESSFUL)
			{
			  	cellON_OFF(true);
				cellSetDateTime();
			}
		#endif
 	#endif
			
	#if DEVICE_TYPE == DEVICETYPE_NODE
		changeClockSpeed(POWERMODE_BALANCED);
		rtcGetDateTime(&RTCDateTimeStructCurrent);
	#endif

	while(1)
	{
		if(enter_initmode)
		{
			enter_initmode = false;
            uart_int = true;
			#if RTC_BASED_CYCLE				
				run_mode = true;
				led_pattern = LED_RUNMODE;
				execute_routine = true;
			#endif 
			enterInitMode();
			
            #if RTC_BASED_CYCLE
				#if DEVICE_TYPE == DEVICETYPE_NODE
					rtcGetDateTime(&RTCDateTimeStructCurrent);
					min_prevcycle = RTCDateTimeStructCurrent.Minutes;
					sec_uart_it_check = RTCDateTimeStructCurrent.Seconds;
				#endif
            #endif
					
			#if ENABLE_GPS
				if(read_gps)
				{				  
					gps_running = true;
					led_pattern = LED_RUNMODE_GPS;
					displayTime();
					gps_time_success = gpsExec(0, GPS_RE_TIMEOUT);
					read_gps = false;
					gps_running = false;
					led_pattern = LED_RUNMODE;
				}
			#endif
		}

		#if DEVICE_TYPE == DEVICETYPE_NODE
			if(uart_int)
			{
				uart_int = false;

				__HAL_UART_DISABLE_IT(&UART2HandlerDef, UART_IT_RXNE);
				HAL_UART_DeInit(&UART2HandlerDef);
				HAL_UART_Init(&UART2HandlerDef);
			  	CLEAR_BIT(UART2HandlerDef.Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
			  	CLEAR_BIT(UART2HandlerDef.Instance->CR3, USART_CR3_EIE);
			  	UART2HandlerDef.RxState = HAL_UART_STATE_READY;
				rtcGetDateTime(&RTCDateTimeStructCurrent);
				sec_uart_it_check = RTCDateTimeStructCurrent.Seconds;
				uartBannerITEnable(&rxbuff_banner);
			}

			rtcGetDateTime(&RTCDateTimeStructCurrent);
            #if RTC_BASED_CYCLE
			if(rtcMinDiff(min_prevcycle, RTCDateTimeStructCurrent.Minutes) >= NODE_TIMEOUT
			   && run_mode
               && sleep_attr.nmm_status == RADNETMAINTENANCEMODE_DISABLE)
			{
				#if PRINT_PROCESSES_IN_UART
					logWrite(msg_sleep);
				#endif
				sleep_attr.enter_sleepmode = true;
			}
			
			
			if(rtcMinDiff(sec_uart_it_check, RTCDateTimeStructCurrent.Seconds) >= 10)
			{
			  	sec_uart_it_check = RTCDateTimeStructCurrent.Seconds;
				__HAL_UART_DISABLE_IT(&UART2HandlerDef, UART_IT_RXNE);
				HAL_UART_DeInit(&UART2HandlerDef);
				HAL_UART_Init(&UART2HandlerDef);
			  	CLEAR_BIT(UART2HandlerDef.Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
			  	CLEAR_BIT(UART2HandlerDef.Instance->CR3, USART_CR3_EIE);
			  	UART2HandlerDef.RxState = HAL_UART_STATE_READY;
				uartBannerITEnable(&rxbuff_banner);
				//sdUpdateDeviceRadioModbusID(0);
			}
            #endif
		#endif

		#if RTC_BASED_CYCLE
            if(sleep_attr.enter_sleepmode 
               && sleep_attr.nmm_status == RADNETMAINTENANCEMODE_DISABLE)
			{
				#if DEVICE_TYPE == DEVICETYPE_NODE
					rtcGetDateTime(&RTCDateTimeStructCurrent);
					min_prevcycle =  RTCDateTimeStructCurrent.Minutes;
				#endif
				sleep_attr.enter_sleepmode = false;
				enterSleepMode();
			}
            
			if(execute_routine && run_mode)
			{
				execute_routine = false;
				rtcTimerRoutine();
                displayTime();
				
				#if ENABLE_GPS
					if(gps_time_success != GPS_SUCCESSFUL)
					{
					  	read_gps = true;
						gpsPowerON(true);
					}
				#endif
				
				#if POWER_TEST_MODE
                 	if(sleep_attr.nmm_status == RADNETMAINTENANCEMODE_DISABLE)
                        sleep_attr.enter_sleepmode = true;
					delayMillis(10000);
				#endif
			}
			sleepCheck();
		#else
			timerInterruptRoutine();
		#endif
	}
}

/*==============================================================================
* @brief     
* @param
* @retval
*=============================================================================*/
static void displayTime(void)
{
	RTCDateTimeStruct RTCDateTimeStructCurrentI;
	RTCDateTimeStruct RTCDateTimeStructCurrentE;
	rtcGetDateTime(&RTCDateTimeStructCurrentI);
	exRTCReadTime(&RTCDateTimeStructCurrentE);
	
	logWrite("\nI: %d-%d-%d %d:%d:%d\n",
			RTCDateTimeStructCurrentI.Month,
			RTCDateTimeStructCurrentI.Date,
			RTCDateTimeStructCurrentI.Year + 2000,
			RTCDateTimeStructCurrentI.Hours,
			RTCDateTimeStructCurrentI.Minutes,
			RTCDateTimeStructCurrentI.Seconds);
	
	
	logWrite("E: %d-%d-%d %d:%d:%d\n\n",
			RTCDateTimeStructCurrentE.Month,
			RTCDateTimeStructCurrentE.Date,
			RTCDateTimeStructCurrentE.Year + 2000,
			RTCDateTimeStructCurrentE.Hours,
			RTCDateTimeStructCurrentE.Minutes,
			RTCDateTimeStructCurrentE.Seconds);
}

/*==============================================================================
* @brief     
* @param
* @retval
*=============================================================================*/
void enterInitMode(void)
{
	#if PRINT_PROCESSES_IN_UART
		const char *msg_sleep = "\n^^^^^ Init Mode HW ^^^^^\n";
		logWrite(msg_sleep);
	#endif

	setPowerRails(true);
 	buzzWaked();

	displayTime();
	radioEnable(true);
	portInitPaths();
	timerIrqEnable();
			
	#if DEVICE_TYPE == DEVICETYPE_NODE
		uartBannerITEnable(&rxbuff_banner);
	#endif
		
	#if ASSOCTEST
		uint32_t default_net_id = 0;
		static uint32_t test_net_id = 1;
		rdBnrMainSetNetId(test_net_id);

		rdBnrMainGetNetId(&default_net_id);
		if(default_net_id != test_net_id)
			assert(0);
		
		delayMillis(500);
		gpioDigitalWrite(&BANNER_EN, LOW);
		delayMillis(2000);
		gpioDigitalWrite(&BANNER_EN, HIGH);
		test_net_id++;
	#endif
}

#if RTC_BASED_CYCLE

/*==============================================================================
* @brief     
* @param
* @retval
*=============================================================================*/

static void enterSleepMode(void)
{
	if(!run_mode)
		return;
	changeClockSpeed(POWERMODE_BALANCED);
	 __HAL_UART_DISABLE_IT(&UART2HandlerDef, UART_IT_RXNE);
	uint16_t sleep_duration = RTC_SLEEPTIME;

	#if DEVICE_TYPE == DEVICETYPE_NODE
		delayMillis(10000);
		sleep_duration = RTC_SLEEPTIME + 20;
	#endif
	
	#if PRINT_PROCESSES_IN_UART
		logWrite("\n^^^^^ Entering SLEEP MODE :%d^^^^^\n", sleep_duration);
	#endif
	radioEnable(false);
	
	#if DEVICE_TYPE == DEVICETYPE_GATE
		cellON_OFF(false);
	#endif
	
	/*Turn off all power rails*/
	setPowerRails(false);
	
	/*Set ports to default signal path*/
	portDeInitPaths();
	
	/*Enable wake-up timer*/
	rtcSetWakeupTimer(sleep_duration);	
	/*execute sleep beep*/
	buzzSleep();
	run_mode = false;
	
	rtcGetDateTime(&RTCDateTimeStructSleep);
	agBoardSyncRTCs();
	displayTime();
	led_pattern = LED_SLEEPMODE;
	
	#if PRINT_PROCESSES_IN_UART
		logWrite("\n^^^^^ MCU SLEEP ^^^^^\n");
	#endif
		
	/*Disable uarts except for BLE and sys print*/
	uartDeInit();
		
	/*Enter Low Power Mode*/
	HAL_PWREx_EnableFlashPowerDown();
	HAL_PWREx_EnterUnderDriveSTOPMode( PWR_LOWPOWERREGULATOR_UNDERDRIVE_ON,PWR_SLEEPENTRY_WFI);
}

static void postLoop(RecordTypEnum rec_type)
{
	for(uint8_t i = 0; i < 10; i++)
	{
		cloudPost(rec_type);
		if(!cloud_post_try)
			break;
	}
}
#if DEVICE_TYPE == DEVICETYPE_GATE
static void pollLoop(RdApiEnum radio_api)
{
	const char *print_poll_tel = "\n*POLLING TELEMETRY\n";
	const char *print_poll_sys = "\n*POLLING SYSTEMSTATUS\n";
	
	if(radio_api == RDAPI_TELEM)
	   	logWrite((uint8_t*)print_poll_tel);
	else
	  	logWrite((uint8_t*)print_poll_sys);
	
	for(uint8_t i = 1; i <= NODE_COUNT; i++)
	{
		if(DEVICE_INFO_LIST[i].network_add_status == NETWORK_ADD_OK)
		{
			if( radioQuery(DEVICE_INFO_LIST[i].rad_modbus_id, RDFUNC_GET, radio_api) != RDSTAT_OK)
			{
				/*query failed*/
				//DEVICE_INFO_LIST[i].network_add_status = NETWORK_ADD_EMPTY;
				delayMillis(RD_TRANSMIT_DELAY);
				
				if (radioQuery(DEVICE_INFO_LIST[i].rad_modbus_id, RDFUNC_GET, radio_api) == RDSTAT_OK)
				{
					/*query retry successful*/
					//DEVICE_INFO_LIST[i].network_add_status = NETWORK_ADD_OK;
				}
			}
			delayMillis(RD_TRANSMIT_DELAY);
		}
	}
}
#endif
/*==============================================================================
* @brief     
* @param
* @retval
*=============================================================================*/
static void rtcTimerRoutine(void)
{
  	changeClockSpeed(POWERMODE_HIGH);
  	/*************************Get Sensor Data********************************/
	#if ENABLE_DAQ
    SensorRegistrationStruct *sr_ptr = (SensorRegistrationStruct*)&PortConfig;
    for(uint8_t i = 0; i < MAX_SENSORPORT; i++)
    {
    	if(sr_ptr->status == PORT_ACTIVE_M || sr_ptr->status == PORT_ACTIVE_A)
		{
			daqExecute(sr_ptr);
		}
		sr_ptr++;
    }
	#endif
		
	/*************************Get System Status********************************/
	#if DEVICE_TYPE == DEVICETYPE_NODE
		daqSysAcquire();
	#endif
	
	#if DEVICE_TYPE == DEVICETYPE_GATE
		displayTime();
		cellON_OFF(true);
		changeClockSpeed(POWERMODE_LOW);
		#if ENABLE_POLLING
			#if ENABLEGATEWAIT
                rdBnrMainReset();
				delaySeconds(TIME_GATEWAIT);
			#endif
			changeClockSpeed(POWERMODE_HIGH);	
			daqSysAcquire();
			cellON_OFF(false);
			displayTime();
			
			#if ENABLE_AUTOASSOC
				rdBnrGateNetworkInit();
			#endif
			
			changeClockSpeed(POWERMODE_LOW);
			/*POLL SYS and TELEM*/
			pollLoop(RDAPI_DEVREG);
			pollLoop(RDAPI_TELEM);
			pollLoop(RDAPI_DEVREG);
			pollLoop(RDAPI_TELEM);
			
			logWrite("\n*SENDING SLEEP CMD\n");

			for(uint8_t i = 1; i <= NODE_COUNT; i++)
			{
				if(DEVICE_INFO_LIST[i].network_add_status == NETWORK_ADD_OK)
				{
					radioQuery(DEVICE_INFO_LIST[i].rad_modbus_id, RDFUNC_SLEEP);
					delayMillis(500);
				}
			}
		#endif
		
		#if ENABLE_POSTING
			changeClockSpeed(POWERMODE_HIGH);
			logWrite("\n###CELL POSTING START###\n");
			cellON_OFF(true);
			postLoop(TELEMETRY);
			postLoop(SYSTEM_STAT);		
			cellON_OFF(false);
			logWrite("\n###CELL POSTING END###\n");
		#else
			  delaySeconds(30);
		#endif
			  
		#if CLOUD_FORMAT_TEST
			cloudPost(RDAPI_TELEM);
		#endif
        if(sleep_attr.nmm_status == RADNETMAINTENANCEMODE_DISABLE)
            sleep_attr.enter_sleepmode = true;
	#else
		if(DEVICE_INFO->rad_modbus_id != MODBUS_DEFAULT)
		  	changeClockSpeed(POWERMODE_LOW);
	#endif
		
}
#else
/*==============================================================================
* @brief     
* @param
* @retval
*=============================================================================*/
static void timerInterruptRoutine(void)
{
	if(exec_daq1)
	{
		exec_daq1 = false;
		daqExecute(&PortConfig.P1);
	}

	if(exec_daq2)
	{
		exec_daq2 = false;
		daqExecute(&PortConfig.P2);
	}

	if(exec_daq3)
	{
		exec_daq3 = false;
		daqExecute(&PortConfig.P3);
	}

	if(exec_daq4)
	{
		exec_daq4 = false;
		daqExecute(&PortConfig.P4);
	}

	if(exec_sysstat)
	{
		exec_sysstat = false;
		daqSysAcquire();
        displayTime();
	}

	#if DEVICE_TYPE == DEVICETYPE_GATE
	if(exec_poll_post)
	{
		cellON_OFF(true);
		radioPollPost(TELEMETRY);

		radioPollPost(SYSTEM_STAT);
		cellON_OFF(false);
	}
	#endif
}
#endif

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;
  
  /*##-1- Configure the I2C clock source. The clock is derived from the SYSCLK #*/
  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
}

void assert_failed (uint8_t* file, uint32_t line)
{
    logWrite("\r\nassert_failed(). file: %s, line: %d\r\n", file, line );
	//__DEBUG_BKPT();
#if HW_DBG_MODE
    while(1);
    {}
#else
	NVIC_SystemReset();
#endif
}

/*==============================================================================
* @brief     
* @param
* @retval
*=============================================================================*/
void __reset_hardware(void)
{
	logWrite("\n\n\n******************************\
							Program Restarted*********************\
							*********\n\n\n");
	//loggerBackUpRecords();
	//intFlSaveSettings();
	delayMillis(50);
	logWrite("\n\n\nSTOP!!!!!\n\n\n");
	NVIC_SystemReset();
}

static void setPowerRails(bool state)
{	
  	gpioDigitalWrite(&FLM25_CS_PIN, HIGH);
	gpioDigitalWrite(&MSP430_CS_PIN, HIGH);
	gpioDigitalWrite(&RTC_ENB, HIGH);
    gpioDigitalWrite(&MSP430_SPI_COMING, HIGH);

	delayMillis(10);


	mspSetPowerRails(MSPCMD2_PR_3V3, state);
	delayMillis(500);
	
	mspSetPowerRails(MSPCMD2_PR_4V, state);
	delayMillis(500);
	
	mspSetPowerRails(MSPCMD2_PR_5V, state);
	delayMillis(500);
	
	mspLedDiagnostic(LED2_STAT_GREEN, state);
    delayMillis(100);
	
	if(state)
		logWrite("\nPower Rails ON...\n");
	else
		logWrite("\nPower Rails OFF...\n");
}

static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

/*==============================================================================
* @brief     
* @param
* @retval
*=============================================================================*/
static void delaySeconds(uint32_t sec)
{
	for(uint32_t i = 0; i < sec; i++)
	{
		delayMillis(1000);
	}
}

#pragma optimize=none
void checkBattery(void)
{
	uint8_t battery_level = 0;
	
  	while(1)
	{ 	
	  	if(check_battery)
		{
		  	check_battery = false;
			logWrite("Checking Battery Status...\n");
			utilPwrModeCont(&PM_MAIN, true);
			
			logWrite("Reading Battery Status...\n");
			sysPwrMntrRead(&battery_level);
			
			logWrite("Battery Level = %d\n", battery_level);
			utilPwrModeCont(&PM_MAIN, false);
			
			if(battery_level >= BATTERY_MARGIN)
			{
			  	logWrite("Battery level ok...\n");
				led_pattern = LED_RUNMODE;
				break;
			}
			else
			{
			  	logWrite("Battery too low, unit will sleep...\n");
				rtcGetDateTime(&RTCDateTimeStructCurrent);		
				radioEnable(false);
				sysSleepAcce();
			
				run_mode = false;
				portDeInitPaths();
				setPowerRails(false);
				changeClockSpeed(POWERMODE_LOW);
				rtcSetWakeupTimer(RTC_SLEEPTIME);		
				agBoardSyncRTCs();
				
				#if PRINT_PROCESSES_IN_UART
					logWrite("\n^^^^^ Entering SLEEP MODE :%d^^^^^\n", RTC_SLEEPTIME);
				#endif
				
				displayTime();
				rtcGetDateTime(&RTCDateTimeStructSleep);
				uartDeInit();
				buzzSleep();
				led_pattern = LED_SLEEPMODE;
				
			  	HAL_PWREx_EnableFlashPowerDown();
				HAL_PWREx_EnterUnderDriveSTOPMode( PWR_LOWPOWERREGULATOR_UNDERDRIVE_ON,PWR_SLEEPENTRY_WFI);		
			}		  
		}
		sleepCheck();
	}
}

#pragma optimize=none
void sleepCheck(void)
{
  	if(!run_mode)
	{
	  	HAL_IWDG_Refresh(&IwdgHandle);
		//rtcGetDateTime(&RTCDateTimeStructCurrent);
		exRTCReadTime(&RTCDateTimeStructCurrent);
		if(rtcMinDiff(RTCDateTimeStructSleep.Minutes, RTCDateTimeStructCurrent.Minutes)
			> (RTC_SLEEPTIME/60 + NODEWAIT_TIMEOUT) )
		{
			SystemClock_Config(RCC_PLLP_DIV8);
			SystemCoreClockUpdate();
			rtcDisableWakeupTimer();
			delayInit();
			uartInit();
			enter_initmode = true;
			check_battery = true;
			buzzWaked();
			#if PRINT_PROCESSES_IN_UART
				logWrite("\n^^^^^ WAKED BY MAIN ^^^^^\n");
			#endif
		}
	}
}
			 
static void checkResetRegister(void)
{
	__IO uint32_t rcc_csr = RCC->CSR; //;=  RCC->CSR  | (0x01 << 24);
	logWrite("Reset Flags:\n");
	logWrite("RCC_CSR: 0x%08X\n", rcc_csr);
	logWrite("LPWR: %d\n", (rcc_csr & (BM_RSTF << BP_RSTF_LPWR)) >> BP_RSTF_LPWR );
	logWrite("WWDG: %d\n", (rcc_csr & (BM_RSTF << BP_RSTF_WWDG)) >> BP_RSTF_WWDG );
	logWrite("IWDG: %d\n", (rcc_csr & (BM_RSTF << BP_RSTF_IWDG)) >> BP_RSTF_IWDG );
	logWrite("SFT: %d\n", (rcc_csr & (BM_RSTF << BP_RSTF_SFT)) >> BP_RSTF_SFT );
	logWrite("---\n");
	logWrite("POR: %d\n", (rcc_csr & (BM_RSTF << BP_RSTF_POR)) >> BP_RSTF_POR );
	logWrite("PIN: %d\n", (rcc_csr & (BM_RSTF << BP_RSTF_PIN)) >> BP_RSTF_PIN );
	logWrite("BOR: %d\n", (rcc_csr & (BM_RSTF << BP_RSTF_BOR)) >> BP_RSTF_BOR );
	logWrite("RMVF: %d\n", (rcc_csr & (BM_RSTF << BP_RSTF_RMVF)) >> BP_RSTF_RMVF );
		 
	RCC->CSR =  rcc_csr  | (BM_RSTF << BP_RSTF_RMVF);
	logWrite("ClearReset Flags\nRCC_CSR: 0x%08X\n\n", RCC->CSR);
}


/****************************** END *********************************/

/*******************************************************************************
  * Revision History
  * @file		main.c
  ******************************************************************************
  * @version	v2.2.0
  * @date		04/08/16
  * @author		D. Lasdoce
  * @changes     
  ******************************************************************************
  * @version	v2.1.0
  * @date
  * @author
  * @changes     
  ******************************************************************************
  * @version	v2.0.0
  * @date
  * @author         
  * @changes    created file
  ******************************************************************************
  */
