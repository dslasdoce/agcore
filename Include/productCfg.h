/********************************************************************************
  * @file      	productCfg.h
  * @author		Hardware Team
  * @version	v2.2.0
  * @date		04/08/16
  * @author    	D. Lasdoce  
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

#ifndef __PRODUCTCFG_H_
#define __PRODUCTCFG_H_

/* Defines ===================================================================*/  
#include "sensors.h"
#include "stdint.h"

#define DEVICETYPE_GATE (0)
#define DEVICETYPE_NODE (1)

#define CELLTYPE_HSPA   (0)
#define CELLTYPE_CDMA	(1)

/* ============================================================================
 * DEVICE DETAILS
 * ==========================================================================*/
#define				DEVICE_TYPE			(DEVICETYPE_GATE)	
#define				CELLTYPE			(CELLTYPE_HSPA)
#define				DEFAULT_NETID		(0x01)
#define				MODBUS_DEFAULT		(0x01)
#define				MODBUS_GATE			(0x0B)
	 
/* ============================================================================
 * FW INFO
 * ==========================================================================*/
/*FW Info*/
#define				FW_VER_MAIN			((uint8_t)3)
#define				FW_VER_SUBMAIN		((uint8_t)6)
#define				FW_VER_MINOR		((uint8_t)0)
#define				FW_VER_DEV			((uint8_t)8)
#define				FW_VERSION			((FWVersionStruct){FW_VER_MAIN,FW_VER_SUBMAIN,FW_VER_MINOR, FW_VER_DEV})

/*DR Info*/
#define				DR_VER_MAIN			((uint8_t)3)
#define				DR_VER_SUB			((uint8_t)2)
#define				DR_VER_MINOR		((uint8_t)0)
#define				DR_VERSION			((DRVersionStruct){DR_VER_MAIN,DR_VER_SUB,DR_VER_MINOR})

/* ============================================================================
 * DEBUG MODE - DONT FORGET TO SET TO "0" during RELEASE !!!!!!!!!!!!!
 * ==========================================================================*/
#define 			AGRICAPTURE_DEBUG_MODE			(0)
	 
/* ============================================================================
 * PROCESS SETTINGS
 * ==========================================================================*/
#define				RTC_BASED_CYCLE             	(1)
#define			    ENABLE_POSTING              	(1)		/*set to 0 to disable cloud posting*/
#define			    ENABLE_POLLING					(1)   	/*set to 0 to disable cloud posting*/
#define 			ENABLE_DAQ                  	(1)   	/*set to 0 to disable data acquisition*/
#define				ENABLE_GPS                  	(1)		/*set to 0 to disable gps*/
#define				ENABLE_BLE                  	(1)		/*set to 0 to disable gps*/
#define				ENABLE_RADIO                  	(1)		/*set to 0 to disable gps*/
#define				ENABLEGATEWAIT					(1) 	/*set to 0 to disable time delay of gate before polling*/
#define				ENABLE_WATCHDOG             	(1)		/*set to 0 to disable gps*/
#define 			PRINT_PROCESSES_IN_UART			(1)		/*set to 0 to disable serial printing*/
#define 			PRINT_RDPROC_IN_UART			(1)		/*set to 0 to disable serial printing*/
#define				ENABLE_CELLGETTIME				(1)		/*set to 0 to disable time assignment via gate*/
#define				ENABLE_AUTOASSOC				(1)		/*set to 0 to disable auto assignment of modbus id*/
#define				ENABLE_AUTONETID				(1)		/*set to 0 to disable setting of default network id*/
#define				ENABLE_AUTONODELIST				(1)		/*set to 0 to disable gate pulling of nodelist from cloud*/
#define				ENABLE_CELLPOST					(1)		/*set to 0 to disable gate pulling of nodelist from cloud*/
#define				BATTERY_MARGIN					(20)
#define				MAXNODE							(20)
/* ============================================================================
 * CLOUD DETAILS
 * ==========================================================================*/
#define				AUTHENTICATION			((const char *)"Bearer 08e937186390a12ed16294706e974830ccbc749d")
#define				CLOUDHOST				((const char *)"dev-core.agtechindustries.com/")

/* ============================================================================
 * RAM BUFFER SIZE
 * ==========================================================================*/
#define				TELEM_MAX_REC_CNT_BUFF		(400)
#define				SYSSTAT_MAX_REC_CNT_BUFF	(50)
#define				MAX_REC_PER_POST			(40)

/* ============================================================================
 * TEST MODES
 * ==========================================================================*/
#define				USE_FW_PORTACTIVATION	(0)	/*set to 0 for normal operation*/
#define				POWER_TEST_MODE			(0) /*set to 0 for normal operation*/
#define				MOBILE_TEST				(0) /*set to 0 for normal operation*/
#define				HWTEST					(0)
#define				CELLTEST				(0)
#define				ASSOCTEST				(0)
#define				CLOUD_FORMAT_TEST		(0)
#define				HWTEST_MODE				(0)
#define				HW_DBG_MODE				(0)
#define				SDRIVER_TEST			(1)

/* ============================================================================
 * TIMING DETAILS
 * ==========================================================================*/
#define				NODE_TIMEOUT	 		(30)	/*Minutes*/
#define				TIME_GATEWAIT			(75)	/*Seconds*/
#define				GPS_TIMEOUT				(600)	/*Seconds*/
#define				GPS_RE_TIMEOUT			(30)	/*Seconds*/
#define				RD_TRANSMIT_DELAY 		(100)	/*Milliseconds*/
#define				RD_DATATX_DELAY 		(500)	/*Milliseconds*/
   
typedef struct
{
  uint8_t fwver_main;
  uint8_t fwver_submain;
  uint8_t fwver_minor;
  uint8_t fwver_dev;
}FWVersionStruct;

typedef struct
{
  uint8_t drver_main;
  uint8_t drver_sub;
  uint8_t drver_minor;
  uint8_t reserved;
}DRVersionStruct;

typedef struct
{
  uint8_t blefwver_main;
  uint8_t blefwver_sub;
  uint8_t blefwver_minor;
  uint8_t blefwver_build;
}BleFWVersionStruct;

typedef struct
{
  	char *device_sn;
	FWVersionStruct arm_fwver;
	uint32_t network_id;
	uint16_t network_add_status;
	uint16_t rad_dev_add;
	uint16_t rad_modbus_id;
	uint16_t reserved;
}DeviceInfoStruct;

typedef struct
{
  uint32_t sc_code;
  const char *sc_name;
}SensorCodeStruct;

typedef struct
{
  uint32_t hexcode;
  const char *name;
}SensorTypeStruct;

typedef struct
{
  uint32_t hexcode;
  const char *name;
}SensorUomStruct;

/*Transmision Parameters*/
extern DeviceInfoStruct *DEVICE_INFO;	/*information for individual device*/
extern BleFWVersionStruct BLE_FWVER; 

#if DEVICE_TYPE == DEVICETYPE_GATE
    extern uint8_t AVAILABLE_MODBUS_ID;
	extern uint32_t NODE_COUNT;
	extern uint32_t ACCOUNT_ID;
	extern DeviceInfoStruct *DEVICE_INFO_LIST;		/*list of information for the gate and all associated nodes*/
	extern SensorCodeStruct *SENSOR_CODE_LIST;
	extern SensorTypeStruct *SENSOR_TYPE_LIST;
	extern SensorUomStruct 	*SENSOR_UOM_LIST;
#endif

/* ============================================================================
 * Specify Radio to be used
 * ==========================================================================*/
//#define 			XBEE_RADIO			/* Uncomment if Radio deployed is XBEE*/
#define 			BANNER_RADIO		/* Uncomment if Radio deployed is BANNER*/
	

/* ============================================================================
 * Specify Internet Medium
 * ==========================================================================*/
#define				CELL				/* Uncomment if device connects to web via CELLULAR*/
//#define			WIFI				/* Uncomment if device connects to web via WIFI*/


/* ============================================================================
 * Specify Read Interval in MINUTES for each PORT. NOT USED IN THIS VERSION
 * ==========================================================================*/
#define				PORT1_INTERVAL		((uint32_t) 1)
#define				PORT2_INTERVAL		((uint32_t) 1)
#define				PORT3_INTERVAL		((uint32_t) 1)
#define				PORT4_INTERVAL		((uint32_t) 1)
#define				SYS_INTERVAL		((uint32_t) 1)
#define				POLL_POST_INTERVAL	((uint32_t) 1)

/* ============================================================================
 * For Development Use ONLY!!!!
 * ==========================================================================*/
#define				PORT1_STAT		(false)
#define				PORT2_STAT		(false)
#define				PORT3_STAT		(false)
#define				PORT4_STAT		(false)

#define				PORT1_SENSOR		((uint16_t)0xFFFF)
#define				PORT2_SENSOR		((uint16_t)0xFFFF)
#define				PORT3_SENSOR		((uint16_t)0xFFFF)
#define				PORT4_SENSOR		((uint16_t)0xFFFF)

#if ENABLE_DAQ
	#define			PORT1_ACTIVE		(PORT1_STAT)
	#define			PORT2_ACTIVE		(PORT2_STAT)
	#define			PORT3_ACTIVE		(PORT3_STAT)
	#define			PORT4_ACTIVE		(PORT4_STAT)
#else
	#define			PORT1_SENSOR		((uint16_t)0x00)
	#define			PORT2_SENSOR		((uint16_t)0x00)
	#define			PORT3_SENSOR		((uint16_t)0x00)
	#define			PORT4_SENSOR		((uint16_t)0x00)
#endif

     
#define			SD_FNAME_ACCNTID	    ((const char *)"AcctID.txt")
#define			SD_FNAME_RADMBID	    ((const char *)"RadModbusID.txt")
#define			SD_FNAME_NETID		    ((const char *)"RadNetId.txt")
#define			SD_FNAME_NODELIST	    ((const char *)"NodeDevSn.txt")
#define			SD_FNAME_NL_BNRDEVADDR	((const char *)"NodeBnrRadDevAddr.txt")
#define			SD_FNAME_ACCNTID	    ((const char *)"AcctID.txt")
#define			SD_FNAME_INFOSERIAL	    ((const char *)"DeviceSerial.txt")
#define			SD_FNAME_INFOSTAT	    ((const char *)"DeviceStatus.txt")
#define			SD_FNAME_SENSLIST	    ((const char *)"SensCode.txt")
#define			SD_FNAME_STYPELIST	    ((const char *)"SensTyp.txt")
#define			SD_FNAME_UOMLIST	    ((const char *)"RawUom.txt")
#define			SD_FNAME_SLEEPTIME	    ((const char *)"SleepTimeSeconds.txt")
#define			SD_FNAME_WAITTIMEOUT    ((const char *)"WaitTimeoutMinutes.txt")
	
/* Typedef Declarations --------------------------------------------------------*/

typedef enum
{
	TELEMETRY,
	SYSTEM_STAT,
	VALVE_SCHED,
	VALVE_CMD,
	VALVE_REG,
	VALVE_DRIVER,
	SENSOR_REG,
	SENSOR_DRIVER,
	FIRMWARE_UPDATE,
	GATEINFO
} RecordTypEnum;


typedef enum
{
	FAILED,
	SUCCESSFUL,
	PENDING,
	NOT_AVAIL,
	NO_MEM_AVAIL
}ReturnStatusEnum;

typedef enum
{
	POWERMODE_HIGH,
	POWERMODE_BALANCED,
	POWERMODE_LOW
}PowerModeEnum;

#endif /* PRODUCTCFG_H_ */

/*******************************************************************************
  * Revision History
  * @file         productCfg.h
  ******************************************************************************
  * @version	v2.2.0
  * @date		04/08/16
  * @author    	D. Lasdoce  
  * @changes    
  ******************************************************************************
 * @version		v2.0.1
  * @date		09/14/2015
  * @version	v2.2.0
  * @changes    created file
  ******************************************************************************
 
  */
