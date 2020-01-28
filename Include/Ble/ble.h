/**
  ******************************************************************************
  * @file    	ble.h
  * @author  	Hardware Team
  * @version 	V2.2.0
  * @date    	13-January-2016
  * @brief   	Header file of ble module.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BLE_H
#define __BLE_H

/* Macro Definitions ========================================================*/
#define BLE_DEBUG_MODE      (false)     //true to enable debug logs false if not
#define BLE_INFO_MODE       (true)      //true to enable debug logs false if not

#define	MAX_REC_READ		((uint8_t)20)	//temporary
#define REC_PER_PORT        ((uint8_t)3)
#define PORTALL				((uint8_t)4)
#define ITERATION			((uint8_t)5)   //number of iterations for notification

#define BLEATTR_DEVICENAME          ((uint8_t)0x00)
#define BLEATTR_MANUFACTURERNAME    ((uint8_t)0x01)
#define BLEATTR_MODELNUM            ((uint8_t)0x02)
#define BLEATTR_SERIALNUM           ((uint8_t)0x03)
#define BLEATTR_HWVERSION           ((uint8_t)0x04)
#define BLEATTR_FWVERSION           ((uint8_t)0x05)
#define BLEATTR_DRVERSION           ((uint8_t)0x06)

#define BLEMODE_SENSOR              ((uint8_t)0x00)
#define BLEMODE_MOBILE              ((uint8_t)0x01)

/* Typedefs =================================================================*/
typedef struct
{
	uint8_t			port;
	uint8_t			sensor_type;
	uint16_t		sensor_code;
	uint8_t			depth_value;
	uint8_t			depth_unit;
	uint16_t		interval_min;
	uint16_t		interval_max;
	uint8_t			alert;
    uint8_t			reserved;
} BleSensorConfigStruct;

typedef struct
{
	uint8_t			port;
	uint8_t			actuator_type;
	uint16_t		actuator_code;
	uint8_t			power;
	uint16_t		psi_filltime;
	uint8_t			psi_target;
	uint8_t			psi_range;
	uint16_t		galpermin_filltime;
	uint8_t			galpermin_target;
	uint8_t			galpermin_range;
	uint8_t			alert;
} BleControlConfigStruct;

typedef struct
{
	BleSensorConfigStruct 	SP1;
	BleSensorConfigStruct 	SP2;
	BleSensorConfigStruct 	SP3;
	BleSensorConfigStruct 	SP4;
	BleControlConfigStruct	VP1;
	BleControlConfigStruct	VP2;
} BlePortStruct;

typedef enum
{
	BLE_FAILED			= 0x00,
	BLE_SUCCESSFUL		= 0x01,
	BLE_RUNNING			= 0x02
} BleStat;


/* Function Prototypes ======================================================*/

BleStat bleSysConfig(uint8_t buff[15], BlePortStruct *ble_config_ptr);
BleStat blePortModeStatus(void);
BleStat bleTelemPush(uint8_t blerx);
BleStat bleSensor(void);
BleStat bleDataControl(void);
BleStat bleReadPressureSensor(float *a_data1, float *a_data2, float *a_data3);
BleStat bleChangeAttrValue(uint8_t a_id, char *a_data_value, uint8_t a_data_sz);
BleStat bleChangeMode(uint8_t arg_mode);
void bleInit(void);

#endif
