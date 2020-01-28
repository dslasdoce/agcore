/********************************************************************************
  * @file      	ble.c
  * @author		Hardware Team
  * @version	v2.0.0
  * @date		02/20/2015
  * @brief		Module for data communications via
  * 			Bluetooth Low Energy interface (BluGiga BLE113-256k)
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
/* Includes ==================================================================*/
#include <deployAssert.h>
#include <string.h>
#include <stdlib.h>

#include "stm32f7xx.h"
#include "utilMem.h"
#include "agBoardInit.h"
#include "port.h"
#include "delay.h"
#include "rtc.h"
#include "loggerMain.h"
#include "ble.h"
#include "equations.h"
#include "daqProcess.h"
#include "productCfg.h"
#include "radio.h"
#include "rdBnrMain.h" //FIXME: temporary
#include "math.h"

/* Defines ===================================================================*/
//Basics
#define TERMCHAR	                ((uint8_t)0x7F)
#define PORT_BITMASK                ((uint32_t)0x0f)
#define BLE_CMD_DLY 			    ((uint32_t)500)
#define BLE_SENDINGDATA_DLY 	    ((uint32_t)500)
#define BLE_RX_WAITTIME 			((uint32_t)3000)
#define BLE_BTSENSOR_CONNECTINGTIME ((uint32_t)15000)
#define BLE_RX_WAITTIME_LONG 		((uint32_t)6000)
#define BLE_REPLY_WAITTIME		    ((uint32_t)3000)
#define BLE_MAX_PAYLOAD_SZ          ((uint8_t)30)
#define BLE_FRAME_DIR_ARMTOBLE  	((uint8_t)0x01)

#define BLE_DIRECTION_SEND          ((uint8_t)0x00)
#define BLE_DIRECTION_REPLY         ((uint8_t)0x01)

//Operation
#define BLEHDR_MODECHANGE           ((uint8_t)0xF1)
#define BLEHDR_SENSORCONN           ((uint8_t)0xF2)
#define BLEHDR_SENSORCONNTOANY      ((uint8_t)0xF3)
#define BLEHDR_SENSORREAD           ((uint8_t)0xF4)
#define BLEHDR_CHANGEDEVINFO        ((uint8_t)0xF5)
#define BLEHDR_SENDREQUEST          ((uint8_t)0xF6)
#define BLEHDR_READFWVER            ((uint8_t)0xF7)
#define BLEHDR_READATTBYHANDLE      ((uint8_t)0xF8)

//Mobile Access Flag Id
#define BLE_ACCESSFLAG_HOME         ((uint8_t)0x00)
#define BLE_ACCESSFLAG_SYSDIAG      ((uint8_t)0x01)
#define BLE_ACCESSFLAG_SYSCONFIG    ((uint8_t)0x02)
#define BLE_ACCESSFLAG_NETID        ((uint8_t)0x03)

//Startflag - to be removed
#define SFLAG_STARTCHAR             ((uint8_t)0x01)
#define SFLAG_SYSCFG_EN             ((uint8_t)0x02)
#define SFLAG_PORT_SELECTED         ((uint8_t)0x03)
#define SFLAG_SYSDIAG_EN            ((uint8_t)0x04)
#define SFLAG_SYSCFG_READY          ((uint8_t)0x05)
#define SFLAG_NETID_READ            ((uint8_t)0x06)
#define SFLAG_NETID_WRITE           ((uint8_t)0x07)
#define SFLAG_NETID_WRITE_READY     ((uint8_t)0x08)

//Header
#define BLEHDR_SPORT                ((uint8_t)0xA0)
#define BLEHDR_SPORT1               ((uint8_t)0xA1)
#define BLEHDR_SPORT2               ((uint8_t)0xA2)
#define BLEHDR_SPORT3               ((uint8_t)0xA3)
#define BLEHDR_SPORT4               ((uint8_t)0xA4)
#define BLEHDR_VPORT                ((uint8_t)0xB0)
#define BLEHDR_VPORT1               ((uint8_t)0xB1)
#define BLEHDR_VPORT2               ((uint8_t)0xB2)

//Command Header
#define BLEHDR_ACCESSFLAG           ((uint8_t)0xFA)
#define BLEHDR_TELEMREQ             ((uint8_t)0xFB)
#define BLEHDR_CONFIGMODE           ((uint8_t)0xFC)
#define BLEHDR_NET_ID    	        ((uint8_t)0xE1)
#define BLEHDR_NET_MODE   	        ((uint8_t)0xE2) 
#define BLEHDR_ARM_RESPONSE  	    ((uint8_t)0xEA)
#define BLEHDR_ARM_MOBL_CONN_STAT	((uint8_t)0xEB)

//Other - to be fixed
#define SYSCFG_HEADER               ((uint8_t)0)
#define SYSCFG_SENSOR_TYPE          ((uint8_t)1) 
#define SYSCFG_SENSOR_CODE1         ((uint8_t)2) 
#define SYSCFG_SENSOR_CODE0         ((uint8_t)3) 
#define SYSCFG_DEPTH                ((uint8_t)4) 
#define SYSCFG_DEPTH_UOM            ((uint8_t)5) 
#define SYSCFG_INTERVAL_MIN1        ((uint8_t)6)
#define SYSCFG_INTERVAL_MIN0        ((uint8_t)7) 
#define SYSCFG_INTERVAL_MAX1        ((uint8_t)8) 
#define SYSCFG_INTERVAL_MAX0        ((uint8_t)9) 
#define SYSCFG_ALERT                ((uint8_t)10)

#define SYSDIAG_HEADER              ((uint8_t)0)
#define SYSDIAG_BATSTAT             ((uint8_t)1)
#define SYSDIAG_RSS                 ((uint8_t)2)
#define SYSDIAG_CELL_SIGSTAT        ((uint8_t)3)
#define SYSDIAG_MEMSIZE_TOTAL       ((uint8_t)4)
#define SYSDIAG_MEMSIZE_AVAIL       ((uint8_t)5)
#define SYSDIAG_TEMP_INT            ((uint8_t)6)
#define SYSDIAG_TEMP_EXT            ((uint8_t)7)
#define SYSDIAG_AZIMUTH             ((uint8_t)8)
#define SYSDIAG_LATiTUDE            ((uint8_t)9)
#define SYSDIAG_LONGITUDE           ((uint8_t)10)
#define SYSDIAG_CHARGING_STAT       ((uint8_t)11)

#define STATUS_SPORT1                ((uint8_t)0)
#define STATUS_SPORT2                ((uint8_t)1)
#define STATUS_SPORT3                ((uint8_t)2)
#define STATUS_SPORT4                ((uint8_t)3)

const uint8_t SCHAR_SYSDIAG[12] =
	{0x00, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xDB};
const uint8_t SCHAR_ACCESSCTRL[5] =
	{0xFA, 0xFB, 0xFC, 0xFD, 0xFE};
const uint8_t SCHAR_SYSCONFIG[6] =
	{BLEHDR_SPORT1, BLEHDR_SPORT2, BLEHDR_SPORT3, BLEHDR_SPORT4, 
     BLEHDR_VPORT1, BLEHDR_VPORT2};
const uint8_t SCHAR_TELEMDATA[4] =
	{0xC1, 0xC2, 0xC3, 0xC4};

typedef enum
{
    BLE_RESCODE_NULL            = 0x00,
    BLE_RESCODE_READ_SUCCESS    = 0x01,
    BLE_RESCODE_WRITE_SUCCESS   = 0x02,
    BLE_RESCODE_ERROR           = 0x03
} BleReplyResultCodeEnum;

typedef struct
{
    uint8_t header;
    uint8_t function;
    uint8_t result_code;
    uint8_t end_char;
} BleReplyStruct;

/* External Declarations =====================================================*/
//PortConfigStruct PortConfig;

/* Internal Functions ========================================================*/
#pragma optimize=none
static BleStat bleReceiveData(uint8_t a_datarx[BLE_MAX_PAYLOAD_SZ]
                              , uint32_t a_wait_time_ms)
{
    BleStat rx_success_flag = BLE_FAILED;
    uint8_t ble_rxbyte = 0x00;
	uint16_t ble_bytecnt = 0;
    uint32_t t_start = 0;

    //data receiving stage
    t_start = HAL_GetTick();
    while (delayTimeDiff(t_start, HAL_GetTick()) < a_wait_time_ms
           && ble_bytecnt < BLE_MAX_PAYLOAD_SZ)
    {
        if (uartRxBle(&ble_rxbyte, sizeof(uint8_t)) == HAL_OK)
        {
            a_datarx[ble_bytecnt++] = ble_rxbyte;
            if(ble_rxbyte == TERMCHAR)
            {
                rx_success_flag = BLE_SUCCESSFUL;
                break;
            }
        }
    }

    HAL_UART_DeInit(&UART3HandlerDef);
    HAL_UART_Init(&UART3HandlerDef);
    return rx_success_flag;
}

#pragma optimize=none
static BleStat bleSendCommand(uint8_t *a_cmd_ptr, uint8_t a_cmd_byte_sz)
{
    BleStat retval = BLE_FAILED;
    uint8_t rxframe[4];
    uint8_t rxbyte;
    uint8_t rxbyte_count;
    uint8_t cmd_header = *a_cmd_ptr;
    uint32_t t_start = 0; 	            /*lap time reference*/
    bool startflag = false;
    
    memset(&rxframe, 0x00, sizeof(rxframe));
    uartTxBle(a_cmd_ptr, a_cmd_byte_sz);
    t_start = HAL_GetTick();
    while(delayTimeDiff(t_start, HAL_GetTick()) < BLE_REPLY_WAITTIME)
    {
        if(uartRxBle(&rxbyte, sizeof(rxbyte)) != HAL_OK) {
            HAL_UART_DeInit(&UART3HandlerDef);
            HAL_UART_Init(&UART3HandlerDef);
            continue;
        }
        
        if(rxbyte == cmd_header && startflag == false)
        {
            rxbyte_count = 0;
            startflag = true;
        }
        
        if(startflag == true)
        {
            rxframe[rxbyte_count++] = rxbyte;
            if(rxbyte == TERMCHAR)
            {
                HAL_UART_DeInit(&UART3HandlerDef);
                HAL_UART_Init(&UART3HandlerDef);
                break;
            }
        }
    }
     
    if(rxframe[0] == cmd_header && rxframe[3] == TERMCHAR)
    {
        if(BLE_DEBUG_MODE)
        {
            logWrite("debug: bleCommand(header[%02x]) SUCCESS\n", cmd_header);
        }
        retval = BLE_SUCCESSFUL;
    } 
    else {
        if(BLE_DEBUG_MODE)
        {
            logWrite("debug: bleCommand(header[%02x]) ERROR\n", cmd_header);      
        }
    }
 
    return retval;
}

#pragma optimize=none
static void bleReply(uint8_t a_function, uint8_t a_result_code)
{  
    BleReplyStruct rxframe;
    
    rxframe.header = BLEHDR_ARM_RESPONSE;
    rxframe.function = a_function;
    rxframe.result_code = a_result_code;
    rxframe.end_char = TERMCHAR;
    
    bleSendCommand((uint8_t*)&rxframe, sizeof(rxframe));
    
    if(BLE_INFO_MODE)
    {
        logWrite("info: Replied to Mobile, function[0x%02x], result code[%d]\n"
                , a_function, a_result_code);
    }
}

#pragma optimize=none
static void bleEnableArmMobileConnectionState(void)
{
    const uint8_t CONN_STAT = 1;
    BleReplyStruct rxframe;
    
    rxframe.header = BLEHDR_ARM_MOBL_CONN_STAT;
    rxframe.function = NULL;
    rxframe.result_code = CONN_STAT;
    rxframe.end_char = TERMCHAR;
    
    bleSendCommand((uint8_t*)&rxframe, sizeof(rxframe));
     
    logWrite("bleEnableArmMobileConnectionState() DONE!\n");
}

#pragma optimize=none
static BleStat bleSensorConnect(uint8_t arg_addr[6])
{
    uint8_t ble_txframe[] = {BLEHDR_SENSORCONN, 0x00, 0x42, 0x7F, 0x24, 0x4E, 0x29, 0xCB, TERMCHAR};
    uartTxBle((uint8_t*)ble_txframe, sizeof(ble_txframe));

    return BLE_SUCCESSFUL;
}

#pragma optimize=none
static BleStat bleSensorConnectToAny(void)
{
    BleStat retval = BLE_FAILED;
    uint8_t ble_rxframe[4];
    
    uint8_t ble_txframe[] = {BLEHDR_SENSORCONNTOANY, 0x00, 0x00, TERMCHAR};
    uartTxBle(ble_txframe, 4);
    
    if(bleReceiveData(ble_rxframe, BLE_BTSENSOR_CONNECTINGTIME) == BLE_SUCCESSFUL
       && ble_rxframe[0] == BLEHDR_SENSORCONNTOANY)
    {
        logWrite("bleSensorConnectToAny[CONNECTED]\n"); //TODO: temo - remove me
        retval = BLE_SUCCESSFUL;
    } 
    else 
    {
        logWrite("bleSensorConnectToAny[ERROR]\n"); //TODO: temo - remove me
        HAL_UART_DeInit(&UART3HandlerDef);
        HAL_UART_Init(&UART3HandlerDef);
    }
     
    return retval;
}


static void bleCopyToSR(SensorRegistrationStruct *sr_ptr
						,BleSensorConfigStruct *ble_config_ptr
						,uint8_t to_update)
{
	for(uint8_t i = 0; i < 4; i++)
	{
		//sr_ptr->port = ble_config_ptr->port&0x0f;
		if(to_update & (1 << i))
		{
			//sr_ptr->depth_uom = ble_config_ptr->depth_unit;
			//sr_ptr->interval = ble_config_ptr->interval_min;
			sr_ptr->status = PORT_ACTIVE_M;
		}
		++sr_ptr;
		++ble_config_ptr;
	}
}

#pragma optimize=none
static void bleReadAccessFlag(void) // BleGetAccessFlag(uint8_t *a_access_flag) 
{
    uint8_t txframe[] = {BLEHDR_ACCESSFLAG
                        , BLE_DIRECTION_SEND
                        , 0x00
                        , TERMCHAR};
    
    logWrite("---Read current Access Flag\n");
    uartTxBle((uint8_t*)txframe, sizeof(txframe));  
}

static BleStat bleSysDiag(void)
{
	SystemStatusStruct *sys_buff_ptr;
	uint16_t notifydata = 0;
	uint8_t notifygps[4];

	sys_buff_ptr = (SystemStatusStruct *)memAlloc(SYSSTAT_REC_SIZE_BYTES);
	daqSysAcquireApp(sys_buff_ptr);

    for (uint8_t n=1; n<12; n++)
    {
        switch (n)
        {
        case SYSDIAG_BATSTAT: //battery status
            notifydata = sys_buff_ptr->batt_stat; 
            break;
        #if DEVICE_TYPE == DEVICETYPE_NODE
        case SYSDIAG_RSS:
            notifydata = m_radio_signal_strength;
        #endif
        case SYSDIAG_CELL_SIGSTAT: //cell signal status
            #if DEVICE_TYPE == DEVICETYPE_GATE
                notifydata = sys_buff_ptr->cell_sig_stat;
            #else
                notifydata = sys_buff_ptr->radio_sig_stat;
            #endif
                break;
        case SYSDIAG_MEMSIZE_TOTAL: //memory status - original
            notifydata = sys_buff_ptr->orig_mem_size; 
            break;
        case SYSDIAG_MEMSIZE_AVAIL: //memory status - available
            notifydata = sys_buff_ptr->avail_mem; 
            break;
        case SYSDIAG_TEMP_INT: //internal temp
            notifydata = (int16_t)sys_buff_ptr->int_temp; 
            break;
        case SYSDIAG_TEMP_EXT: //external temp
            notifydata = (int16_t)sys_buff_ptr->ext_temp; 
            break;
        case SYSDIAG_AZIMUTH: //orientation/accelerometer status
            notifydata = sys_buff_ptr->acc_sig_stat; 
            break;
        case SYSDIAG_LATiTUDE: //GPS coordinate: latitude
            memcpy(notifygps, &(sys_buff_ptr->latitude), 4); 
            break;
        case SYSDIAG_LONGITUDE: //GPS coordinate: longitude
            memcpy(notifygps, &(sys_buff_ptr->longitude), 4); 
            break;
        case SYSDIAG_CHARGING_STAT: //charging status
            notifydata = (uint8_t)sys_buff_ptr->charging_stat; 
            break;
        default:
            notifydata = 0;
            break;
        }
        
        #if DEVICE_TYPE == DEVICETYPE_GATE
        if (n==2) {continue;} //exclude radio RSS
        #endif
        if (n==9 || n==10) //GPS coordinates data
        {
            uint8_t framegps[] = {SCHAR_SYSDIAG[n], notifygps[3], notifygps[2], notifygps[1], notifygps[0], TERMCHAR};
            bleSendCommand(framegps, sizeof(framegps));
            memset(framegps,0,sizeof(framegps));
            memset(notifygps,0,sizeof(notifygps));
        }
        else //all other system diagnostics
        {
            uint8_t framedat[] = {SCHAR_SYSDIAG[n], notifydata>>8, notifydata, TERMCHAR};
            bleSendCommand(framedat, sizeof(framedat));
            memset(framedat,0,sizeof(framedat));
        }
    }
	
	free(sys_buff_ptr);
	return BLE_SUCCESSFUL;
}

static void bleSetNetId(uint32_t a_net_id)
{
    uint8_t ble_txframe[6];
    
    //Update Network Id in BLE
    ble_txframe[0] = BLEHDR_NET_ID;
    ble_txframe[1] = a_net_id >> 24;
    ble_txframe[2] = a_net_id >> 16;
    ble_txframe[3] = a_net_id >> 8;
    ble_txframe[4] = a_net_id;
    ble_txframe[5] = TERMCHAR;
    bleSendCommand((uint8_t*)ble_txframe, sizeof(ble_txframe));
    
    logWrite("Radio network ID updated[%d]\n", a_net_id);
}

static void bleUpdateNetId(void)
{
    uint32_t radio_net_id_toset = 0;
    
    rdBnrMainGetNetId(&radio_net_id_toset);
    bleSetNetId(radio_net_id_toset);
}

static BleStat bleUpdateMaintenanceMode(uint8_t a_cmdcode)
{
    uint8_t ble_txframe[6];
    
    //Update Network Maintenance Mode in BLE 
    ble_txframe[0] = BLEHDR_NET_MODE;
    ble_txframe[1] = BLE_FRAME_DIR_ARMTOBLE;
    ble_txframe[2] = a_cmdcode;
    ble_txframe[3] = sleep_attr.nmm_status;
    ble_txframe[4] = sleep_attr.nmm_timer_min;
    ble_txframe[5] = TERMCHAR;
    return bleSendCommand((uint8_t*)ble_txframe, sizeof(ble_txframe));
}

#pragma optimize=none
static BleStat bleReadFirmwareVersion()
{
    BleStat retval = BLE_FAILED;
    uint8_t ble_rxframe[10];
    uint8_t ble_txframe[] = {BLEHDR_READFWVER, 0x00, 0x00, TERMCHAR};
    uint8_t printout[100];
    

    for (uint8_t i=0; i<3; ++i)
    {
        memset(ble_rxframe, 0, 10);        
        uartTxBle(ble_txframe, sizeof(ble_txframe));
        if(bleReceiveData(ble_rxframe, BLE_RX_WAITTIME) != BLE_SUCCESSFUL)
        {
            continue;
        }
                    
        if(ble_rxframe[0] == BLEHDR_READFWVER && ble_rxframe[9] == TERMCHAR)
        {
            retval = BLE_SUCCESSFUL;
            BLE_FWVER.blefwver_main = ble_rxframe[2];
            BLE_FWVER.blefwver_minor = ble_rxframe[4];
            BLE_FWVER.blefwver_sub = ble_rxframe[6];
            BLE_FWVER.blefwver_build = ble_rxframe[8];
            logWrite("bleReadFirmwareVersion[%c.%c.%c(%c)] SUCCESS!\n"
                    , BLE_FWVER.blefwver_main
                    , BLE_FWVER.blefwver_minor
                    , BLE_FWVER.blefwver_sub
                    , BLE_FWVER.blefwver_build);
            break;
        }
    }
       
    return retval;  
}

#pragma optimize=none
static BleStat bleGetPressureExponent(uint16_t a_handle, int8_t *a_exponent)
{
    BleStat retval = BLE_FAILED;
    uint8_t ble_rxframe[10];
    uint8_t ble_txframe[] = {BLEHDR_READATTBYHANDLE, 0x00, a_handle, a_handle >> 8, TERMCHAR};
    uint8_t printout[100];
    
    sprintf(printout, "bleGetPressureExponent[?] ERROR\n");
    for (uint8_t i=0; i<3; ++i)
    {
        memset(ble_rxframe, 0, 10);        
        uartTxBle(ble_txframe, sizeof(ble_txframe));
        if(bleReceiveData(ble_rxframe, BLE_RX_WAITTIME_LONG) != BLE_SUCCESSFUL)
        {
            continue;
        }
                    
        if(ble_rxframe[0] == BLEHDR_READATTBYHANDLE)
        {
            retval = BLE_SUCCESSFUL;
            *a_exponent = ble_rxframe[4];
            sprintf(printout, "bleGetPressureExponent[%d]\n", *a_exponent);
            break;
        }
    }
    
    uartSHOW((uint8_t*)printout, strlen(printout));
    return retval; 
}

/* Functions =================================================================*/
bool portProcessBLE(uint16_t *sensor_codes, SensorRegistrationStruct *sr_ptr);
/******************************************************************************
 * @brief	System Diagnostics (Notify to BLE remote device)
 * @param   none
 * @retval  BleStat
 ******************************************************************************/

/******************************************************************************
 * @brief	System/Manual Configuration of ports function
 * @param   buff (system config frame from BLE remote device)
 * @retval  BleStat
 ******************************************************************************/
BleStat bleSysConfig(uint8_t buff[15], BlePortStruct *ble_config_ptr)
{
	assert(ble_config_ptr);
    volatile BleSensorConfigStruct *sr_ptr = NULL;
	switch (buff[SYSCFG_HEADER] & 0xf0){
		case BLEHDR_SPORT:
            //++sr_ptr ;//= (BleSensorConfigStruct*)ble_config_ptr + (buff[SYSCFG_HEADER] & PORT_BITMASK - 1)*sizeof(BleSensorConfigStruct)/4;
            sr_ptr = (BleSensorConfigStruct*)ble_config_ptr + ((buff[SYSCFG_HEADER] & PORT_BITMASK) - 1);
			sr_ptr->port = buff[SYSCFG_HEADER] & PORT_BITMASK; 
			sr_ptr->sensor_type = buff[SYSCFG_SENSOR_TYPE]; 
			sr_ptr->sensor_code = (buff[SYSCFG_SENSOR_CODE1]<<8) 
                                               | (buff[SYSCFG_SENSOR_CODE0]); 
			sr_ptr->depth_value = buff[SYSCFG_DEPTH]; 
			sr_ptr->depth_unit = buff[SYSCFG_DEPTH_UOM];
			sr_ptr->interval_min = (buff[SYSCFG_INTERVAL_MIN1]<<8) 
                                                | (buff[SYSCFG_INTERVAL_MIN0]); 
			sr_ptr->interval_max = (buff[SYSCFG_INTERVAL_MAX1]<<8)
                                                | (buff[SYSCFG_INTERVAL_MAX0]); 
			sr_ptr->alert = buff[SYSCFG_ALERT]; 
			break;

		case BLEHDR_VPORT:
			ble_config_ptr->VP1.port = buff[0]; //1b
			ble_config_ptr->VP1.actuator_type = buff[1]; //1b
			ble_config_ptr->VP1.actuator_code = (buff[2]<<8)+(buff[3]); //2b
			ble_config_ptr->VP1.power = buff[4]; //1b
			ble_config_ptr->VP1.psi_filltime = (buff[5]<<8)+(buff[6]); //2b
			ble_config_ptr->VP1.psi_target = buff[7]; //1b
			ble_config_ptr->VP1.psi_range = buff[8]; //1b
			ble_config_ptr->VP1.galpermin_filltime = (buff[9]<<8)+(buff[10]); //2b
			ble_config_ptr->VP1.galpermin_target = buff[11]; //1b
			ble_config_ptr->VP1.galpermin_range = buff[12]; //1b
			ble_config_ptr->VP1.alert = buff[13]; //1b
			break;
            
		default:
			break;
		}

	return BLE_SUCCESSFUL;
}

#pragma optimize=none
/******************************************************************************
 * @brief	Function to send Status and Mode of each port (Notify to BLE remote device)
 * @param   none
 * @retval  BleStat
 ******************************************************************************/
BleStat blePortModeStatus(void)
{
	/*Check each port for mode:
	 * 0x00 = Manual
	 * 0x01 = Automatic
	 *
	 * Check each port status:
	 * 0x00 = Active
	 * 0x01 = Inactive
	 * 0x02 = Available */

	uint8_t local_portstat[6];
	uint8_t ble_cfgmode[6];
	uint8_t ble_portstat[6];
	uint8_t lim = 20;
	uint8_t buff[4][12];
	uint8_t flag_sensortype = 0; //1 = flag for more than 1 sensor type; 0 = for only 1 sensor type
    SensorRegistrationStruct *sr_ptr = (SensorRegistrationStruct*)&PortConfig;
    DataProcessingStruct *dataproc_ptr = (DataProcessingStruct*)&PortDataProcessing;
	local_portstat[STATUS_SPORT1] = PortConfig.P1.status;
	local_portstat[STATUS_SPORT2] = PortConfig.P2.status;
	local_portstat[STATUS_SPORT3] = PortConfig.P3.status;
	local_portstat[STATUS_SPORT4] = PortConfig.P4.status;
	local_portstat[4] = 4; //placeholder only for testing without valve ports
	local_portstat[4] = 4; //placeholder only for testing without valve ports

	for (uint8_t z = 0; z < 4; z++)
	{
		switch (local_portstat[z]){
		case 0: //#define PORT_INACTIVE   ((uint8_t)0)
			ble_cfgmode[z] = 0; //Manual
			ble_portstat[z] = 2; //Available
			break;
		case 1: //#define PORT_ACTIVE_A   ((uint8_t)1)
			ble_cfgmode[z] = 1; //Auto
			ble_portstat[z] = 0; //Active
			break;
		case 2:
			break;
		case 3: //#define PORT_ACTIVE_M   ((uint8_t)3)
			ble_cfgmode[z] = 1; //Manual
			ble_portstat[z] = 0; //Active
			break;
		default:
			ble_cfgmode[z] = 0; //Manual
			ble_portstat[z] = 1; //Inactive
			//uncomment only when valve is not yet used or supported
			ble_cfgmode[4] = 0;
			ble_cfgmode[5] = 0;
			break;
		}
        //parse auto port config details
        buff[z][SYSCFG_HEADER] = SCHAR_SYSCONFIG[z]; //buff[0] = auto port config header
        buff[z][SYSCFG_INTERVAL_MAX1] = 0;
        buff[z][SYSCFG_INTERVAL_MAX0] = 0;
        buff[z][SYSCFG_ALERT] = 0;

        //buff[1] = sensor type
        //temp fix for multiple sensor types in 1 sensor
        if (dataproc_ptr->sensor_type[0]==0x01 
            && dataproc_ptr->sensor_type[1]==0x0F)
            buff[z][SYSCFG_SENSOR_TYPE] = 0x65; //SM,ST
        else if (dataproc_ptr->sensor_type[0]==0x07 
                 && dataproc_ptr->sensor_type[1]==0x06)
            buff[z][SYSCFG_SENSOR_TYPE] = 0x66; //WS,WD
        else
            buff[z][SYSCFG_SENSOR_TYPE] = dataproc_ptr->sensor_type[0];
        //***
        buff[z][SYSCFG_SENSOR_CODE1] = (sr_ptr->sensor_code)<<8; //buff[2],buff[3] = sensor code
        buff[z][SYSCFG_SENSOR_CODE0] = sr_ptr->sensor_code;
        //buff[4] = depth value
        //temp fix for multiple depths in 1 sensor (e.g. soil moisture probe)
        if (sr_ptr->sensor_code==0x33)
            buff[z][SYSCFG_DEPTH] = 0x65;
        else if (sr_ptr->sensor_code==0x27)
            buff[z][SYSCFG_DEPTH] = 0x66;
        else if (sr_ptr->sensor_code==0x26)
            buff[z][SYSCFG_DEPTH] = 0x67;
        else
            buff[z][SYSCFG_DEPTH] = dataproc_ptr->depth_values[0];
        //***
        buff[z][SYSCFG_DEPTH_UOM] = sr_ptr->depth_uom; //buff[5] = depth unit
        buff[z][SYSCFG_INTERVAL_MIN1] = (sr_ptr->interval)>>8; //buff[6], buff[7] = interval
        buff[z][SYSCFG_INTERVAL_MIN0] = sr_ptr->interval & 0xff;
        ++sr_ptr;
        ++dataproc_ptr;
	}

    //prepare data frame for port modes setting
	uint8_t frame_mode[] = {SCHAR_ACCESSCTRL[2], ble_cfgmode[STATUS_SPORT1]
                            ,ble_cfgmode[STATUS_SPORT2], ble_cfgmode[STATUS_SPORT3]
                            ,ble_cfgmode[STATUS_SPORT4], ble_cfgmode[4]
                            ,ble_cfgmode[5], TERMCHAR};
    
    //prepare data frame for port status setting
	uint8_t frame_stat[] = {SCHAR_ACCESSCTRL[3], ble_portstat[STATUS_SPORT1]
                            ,ble_portstat[STATUS_SPORT2], ble_portstat[STATUS_SPORT3]
                            ,ble_portstat[STATUS_SPORT4], ble_portstat[4]
                            ,ble_portstat[5], TERMCHAR};

	for (uint8_t s=0; s<ITERATION; s++) //send ports' mode and status iter-times
	{
		uartTxBle((uint8_t*)frame_mode,sizeof(frame_mode)); //notify ports' mode
		delayMillis(BLE_SENDINGDATA_DLY);
		uartTxBle((uint8_t*)frame_stat,sizeof(frame_stat)); //notify ports' status
		delayMillis(BLE_SENDINGDATA_DLY);
	}

	for (uint8_t m=0; m<4; m++)
	{
//		if (ble_cfgmode[m]) //notify port configuration if mode is automatic
//		{
			uint8_t frame_auto[] = {SCHAR_SYSCONFIG[m], buff[m][SYSCFG_SENSOR_TYPE]
                                    ,buff[m][SYSCFG_SENSOR_CODE1] 
                                    ,buff[m][SYSCFG_SENSOR_CODE0]
                                    ,buff[m][SYSCFG_DEPTH]
                                    ,buff[m][SYSCFG_DEPTH_UOM]
                                    ,buff[m][SYSCFG_INTERVAL_MIN1]
                                    ,buff[m][SYSCFG_INTERVAL_MIN0]
                                    ,buff[m][SYSCFG_INTERVAL_MAX1]
                                    ,buff[m][SYSCFG_INTERVAL_MAX0]
                                    ,buff[m][SYSCFG_ALERT], TERMCHAR};

//			for (uint8_t s=0; s<ITERATION; s++) //notify ports' configuration iter-times
//			{
				uartTxBle((uint8_t*)frame_auto,sizeof(frame_auto)); //notify automatic ports' configuration
				delayMillis(BLE_SENDINGDATA_DLY);
//			}
			memset(frame_auto,0,sizeof(frame_auto));
//		}
	}
    
	//free(port_config);
	return BLE_SUCCESSFUL;
}


/******************************************************************************
 * @brief	Latest telemetry data push to BLE remote device
 * @param   none
 * @retval  BleStat
 ******************************************************************************/
BleStat bleTelemPush (uint8_t blerx)
{
	//uint8_t blerx;
	uint8_t rxflag = 0;
	uint8_t portval[PORTALL][MAX_REC_READ];
	uint8_t portcnt[5] = {0, 0, 0, 0, 0};
	uint8_t pnum = 0;
	uint8_t floatbuff[4];
	uint8_t ble_telem_count = 0;

	TelemetryStruct ble_telem_buff[MAX_REC_READ];

	ble_telem_count = loggerGetRecFromRingBuff((uint32_t*)ble_telem_buff, TELEMETRY, MAX_REC_READ, NULL); //retrieve data
	for (uint8_t rec=MAX_REC_READ; rec>0; rec--) //sort each record according to portnum group
	{
		pnum = ble_telem_buff[rec-1].port;
		portval[pnum][portcnt[pnum]] = rec-1;
		portcnt[pnum]++; //number of entries for each port: {#port1, #port2, #port3, #port4}
	}
	pnum=0;
	for (uint8_t datcnt=0; datcnt<REC_PER_PORT; datcnt++) //save each data frame in buffer
	{
		uint8_t buff[13];
		uint8_t index = portval[blerx][datcnt];
		buff[0] = datcnt; //record number or data count (for mobile app)
		buff[1] = (ble_telem_buff[index].sensor_code)<<8;
		buff[2] = ble_telem_buff[index].sensor_code;
		memset(floatbuff,0,sizeof(floatbuff));
		memcpy(floatbuff, &(ble_telem_buff[index].raw_value), 4);
		buff[3] = floatbuff[3];
		buff[4] = floatbuff[2];
		buff[5] = floatbuff[1];
		buff[6] = floatbuff[0];
		memset(floatbuff,0,sizeof(floatbuff));
		memcpy(floatbuff, &(ble_telem_buff[index].base_value), 4);
		buff[7] = floatbuff[3];
		buff[8] = floatbuff[2];
		buff[9] = floatbuff[1];
		buff[10] = floatbuff[0];
		buff[11] = ble_telem_buff[index].raw_uom;
		buff[12] = ble_telem_buff[index].base_uom;

		uint8_t frame_telem[] = {SCHAR_TELEMDATA[blerx-1], buff[0], buff[1], buff[2], buff[3], buff[4], buff[5],
							buff[6], buff[7], buff[8], buff[9], buff[10], buff[11], buff[12],TERMCHAR};

		for (uint8_t s=0; s<ITERATION; s++) //notify ports' configuration iter-times
		{
			uartTxBle((uint8_t*)frame_telem,sizeof(frame_telem)); //notify automatic ports' configuration
			delayMillis(20);
		}
		memset(frame_telem,0,sizeof(frame_telem));
		memset(buff,0,sizeof(buff));
	}

	return SUCCESSFUL;
}


BleStat bleChangeMode(uint8_t arg_mode)
{
    BleStat retval = BLE_FAILED;
    uint32_t t_start;
    bool start_rx_flag = false;
    uint8_t ble_rxframe[4];
    uint8_t uart_blerx = 0;
    uint8_t rx_ctr = 0;
    const char *BLE_MODE_STR[] = {"SENSOR", "MOBILE"};

    for(int i=0; i<3; i++)
    {
        memset(ble_rxframe, 0xFF, 4);

        uint8_t ble_txframe[] = {BLEHDR_MODECHANGE, 0x00, arg_mode, TERMCHAR};
        uartTxBle((uint8_t*)ble_txframe, sizeof(ble_txframe));

        t_start = HAL_GetTick();
        while(delayTimeDiff(t_start, HAL_GetTick()) < BLE_RX_WAITTIME) 
        {
            if(uartRxBle(&uart_blerx, sizeof(uart_blerx)) != HAL_OK) 
            {
                continue;
            }

            if (start_rx_flag==false && uart_blerx==BLEHDR_MODECHANGE) 
            {
                start_rx_flag = true;
            }

            if (start_rx_flag==true) 
            {
                ble_rxframe[rx_ctr++] = uart_blerx;
                if(uart_blerx == TERMCHAR) 
                {
                  break;
                }
            }
        }
        
        if(ble_rxframe[0] == BLEHDR_MODECHANGE && ble_rxframe[3] == TERMCHAR)
        {
            retval = BLE_SUCCESSFUL;
            logWrite("bleChangeMode[%s] = SUCCESS\n", BLE_MODE_STR[arg_mode]);
            break;
        }
        else
        {
            retval = BLE_FAILED;
            HAL_UART_DeInit(&UART3HandlerDef);
            HAL_UART_Init(&UART3HandlerDef);
            logWrite("bleChangeMode[%s] = ERROR\n", BLE_MODE_STR[arg_mode]);
        }
    }

    return retval;
}

/******************************************************************************
 * @brief	BLE-based sensor wireless data acquisition
 * @param   none
 * @retval  BleStat
 ******************************************************************************/
BleStat bleSensorRead(uint8_t arg_bleaddr[6], uint8_t *arg_uuid, uint8_t arg_typ)
{
    uint8_t ble_txframe[] = {BLEHDR_SENSORREAD, 0x00, 0x1C, 0x2A, TERMCHAR};
    
    //set BLE to sensor mode
    bleChangeMode(BLEMODE_SENSOR);
    delayMillis(20);
    
    //connect to BLE sensor
    bleSensorConnect(arg_bleaddr);
    
    //read data from BLE sensor
    uartTxBle((uint8_t*)ble_txframe, sizeof(ble_txframe));
    
    //set back the BLE into mobile mode
    bleChangeMode(BLEMODE_MOBILE);
                   
	return BLE_SUCCESSFUL;
}

#pragma optimize=none
BleStat bleChangeAttrValue(uint8_t a_id, char *a_data_value, uint8_t a_data_sz)
{
    BleStat retval = BLE_FAILED;
    const uint8_t END_OF_TRIES = 3;
    uint8_t *attr_txframe;
    uint8_t send_req_txframe[4];
    uint8_t ble_rxframe[4];
    uint8_t tx_size;
    const uint8_t *attr_name[] = 
    {
        "Device Name"
        , "Manufacture Name"
        , "Model Number"
        , "Serial Number"
        , "Hardware Version"
        , "Firmware Version"
        , "Driver Version"
    };
    
    //Initialize frames to transmit
    tx_size = sizeof(BLEHDR_CHANGEDEVINFO)
                + sizeof(BLE_DIRECTION_SEND)
                + sizeof(a_id)
                + sizeof(tx_size)
                + strlen(a_data_value) 
                + sizeof(TERMCHAR);
    
    send_req_txframe[0] = BLEHDR_SENDREQUEST;
    send_req_txframe[1] = 0x00;
    send_req_txframe[2] = tx_size;
    send_req_txframe[3] = TERMCHAR;
    
    attr_txframe = (uint8_t*) malloc(tx_size);
    attr_txframe[0] = BLEHDR_CHANGEDEVINFO;
    attr_txframe[1] = BLE_DIRECTION_SEND;
    attr_txframe[2] = a_id;
    attr_txframe[3] = strlen(a_data_value);
    memcpy(attr_txframe+4, a_data_value, strlen(a_data_value));
    *(attr_txframe+tx_size-1) = TERMCHAR;
    
    //Process
    for (uint8_t i=0; i<END_OF_TRIES; ++i)
    {
        memset(ble_rxframe, 0, 4);        
        uartTxBle(send_req_txframe, sizeof(send_req_txframe));
        if(bleReceiveData(ble_rxframe, BLE_RX_WAITTIME) != BLE_SUCCESSFUL)
        {
            continue;
        }
        
        if(BLE_DEBUG_MODE)
        {
            logWrite("debug: 1 Rx[0x%02x 0x%02x 0x%02x 0x%02x]\n"
                    , ble_rxframe[0], ble_rxframe[1], ble_rxframe[2], ble_rxframe[3]);
        }
                    
        if(ble_rxframe[0] == BLEHDR_SENDREQUEST && ble_rxframe[3] == TERMCHAR)
        {
            for (uint8_t j=0; j<END_OF_TRIES; ++j)
            {
                memset(ble_rxframe, 0, 4);                
                uartTxBle((uint8_t*)attr_txframe, tx_size);
                if(bleReceiveData(ble_rxframe, BLE_RX_WAITTIME) != BLE_SUCCESSFUL)
                {
                    continue;
                }
                
                if(BLE_DEBUG_MODE)
                {
                    logWrite("debug: 2 Rx[0x%02x 0x%02x 0x%02x 0x%02x]\n"
                            , ble_rxframe[0], ble_rxframe[1], ble_rxframe[2], ble_rxframe[3]);
                }
                
                if(ble_rxframe[0] == BLEHDR_CHANGEDEVINFO && ble_rxframe[3] == TERMCHAR)
                {
                    retval = BLE_SUCCESSFUL;
                    logWrite("Update Attribute %s[%s] SUCCESS!\n"
                            , attr_name[a_id], a_data_value);
                    i = END_OF_TRIES;
                    break;
                }
            }
        }
    }
    
    if(retval != BLE_SUCCESSFUL)
    {
        logWrite("Update Attribute %s[%s] ERROR!\n"
                , attr_name[a_id], a_data_value);
    }
        
    free(attr_txframe);
    return retval;
}

#pragma optimize=none
/******************************************************************************
 * @brief	BLE-based sensor wireless data acquisition
 * @param   none
 * @retval  BleStat
 ******************************************************************************/
BleStat bleReadPressureSensor(float *a_data1, float *a_data2, float *a_data3) //TODO: this is a temporary patch
{        
    BleStat retval = BLE_FAILED;
    uint8_t bletx_press_cmd[] = {BLEHDR_SENSORREAD, 0x02, 0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xbd, 0xa5, 0xe3, 0x11, 0xe4, 0x51, 0xc0, 0xb4, 0x5a, 0x83, TERMCHAR}; //FIXME!!
  
    uint8_t uart_blerx;
    uint8_t ble_rxframe[20];
    uint8_t trial;
    uint32_t t_start; 	            /*lap time reference*/
    size_t rx_ctr;		            /*counter of receiving byte*/
    const uint8_t MAX_ATTEMPT_CNT = 3;
    uint8_t attempts;
    char printout[100];
    int16_t raw_pressure_data;
    int8_t pressure_exponent;

    uart_blerx = 0;
    rx_ctr = 0;
    memset(ble_rxframe, 0xFF, 20);
    
    logWrite("\n===== bleReadPressureSensor() Start =====\n");

    //set BLE to sensor mode
    for(attempts = 0; attempts < MAX_ATTEMPT_CNT; attempts++)
    {
        retval = bleChangeMode(BLEMODE_SENSOR);
        if (retval == BLE_SUCCESSFUL)
        {
            break;
        }
        delayMillis(BLE_CMD_DLY);
    }

    if(retval != BLE_SUCCESSFUL)
    {
        logWrite("===== bleReadPressureSensor() Ends =====\n");
        return retval;
    }

    for(uint8_t i=0; i<3; ++i)
    {
         //connect to BLE sensor
        retval = BLE_FAILED;
        trial = 0;
        while (retval != BLE_SUCCESSFUL && trial < 3)
        {
            trial++;
            logWrite("Connecting to sensor... attempt[%d]\n", trial);
            delayMillis(BLE_CMD_DLY);
            retval = bleSensorConnectToAny();
        }
        
        if (retval == BLE_SUCCESSFUL) 
        {
            // get exponent of pressure reading
            retval = bleGetPressureExponent(0x000C, &pressure_exponent);
        }
        
        if (retval == BLE_SUCCESSFUL) 
        {
            break;
        }
    }
   

    if (retval == BLE_SUCCESSFUL) 
    {    
        //read data from BLE 
        retval = BLE_FAILED;        //reset status
        for (uint8_t j=0; j<6; ++j)
        {
            logWrite("Reading data from sensor...\n");
            memset(ble_rxframe, 0xFF, 20);
            delayMillis(BLE_RX_WAITTIME);
            uartTxBle((uint8_t*)bletx_press_cmd, sizeof(bletx_press_cmd));
            t_start = HAL_GetTick();
            while(delayTimeDiff(t_start, HAL_GetTick()) < BLE_RX_WAITTIME_LONG) 
            {
                if(uartRxBle(&uart_blerx, sizeof(uart_blerx)) != HAL_OK)
                {
                    continue;
                }
                ble_rxframe[rx_ctr++] = uart_blerx;
                if(uart_blerx == TERMCHAR)
                {
                    raw_pressure_data = ble_rxframe[4] | ble_rxframe[3] << 8;
                    *a_data1 = (float)raw_pressure_data;
                    *a_data1 *= pow(10,pressure_exponent);
                    *a_data2 = *a_data1;
                    *a_data3 = *a_data1;
                        
//                        if(*data_arr[dataset] < 0)
//                        {
//                            *data_arr[dataset] = 0;
//                        }
                    logWrite("Data Read: [%.2f] psi\n", *a_data1);
                    retval = BLE_SUCCESSFUL;
                    break;
                }
            }
            if (retval == BLE_SUCCESSFUL)
            {
                break;
            }

            logWrite("No data received!\n");
            HAL_UART_DeInit(&UART3HandlerDef);
            HAL_UART_Init(&UART3HandlerDef);
        }
    }

    //set back the BLE into mobile mode
    delayMillis(BLE_RX_WAITTIME);
    bleChangeMode(BLEMODE_MOBILE);

    if (PRINT_PROCESSES_IN_UART == 1)
    {
        logWrite("===== bleReadPressureSensor() Ends =====\n");
    }
    return retval;
}

#pragma optimize=none
/******************************************************************************
 * @brief	Function controlling access and actions to/from BLE module
 * @param   none
 * @retval  BleStat
 ******************************************************************************/
BleStat bleDataControl(void)
{
    uint8_t ble_rxframe[BLE_MAX_PAYLOAD_SZ];
    uint8_t ble_rxheader;
    uint8_t is_updateport = 0;
    uint8_t databuff[15];
    uint32_t radio_net_id;
    uint8_t access_flag = BLE_ACCESSFLAG_HOME;
    bool update_access_flag = false;
    bool mobile_arm_connect_state = false;
    uint32_t radio_net_id_toset = 0;
    uint16_t rx_sensorcode;
    uint16_t sd_sensorcode;
    BlePortStruct BLEPortConfig;
    BleReplyResultCodeEnum result_code;
    
    #if PRINT_PROCESSES_IN_UART
    	logWrite("\n\n\n*******BLE Start******\n\n\n");
    #endif
    
	/**/
	portDeInitPower();
	
    // initialize
    radioEnable(true);
        
    // mobile-bluetooth connected state
    while(!gpioDigitalRead(&BLE_IRQ))
    {
        //update mobile-arm connection state
        if(mobile_arm_connect_state == false)
        {
            bleEnableArmMobileConnectionState();
            mobile_arm_connect_state = true;
        }
        
        if(update_access_flag == true)
        {
            bleReadAccessFlag();
        }
        
        //data receiving stage
        if(bleReceiveData(ble_rxframe, BLE_RX_WAITTIME) != BLE_SUCCESSFUL)
        {
            continue;
        }
        update_access_flag = false;
        
        //received data handling stage
        ble_rxheader = ble_rxframe[0];
        switch(ble_rxheader)
        {
        case BLEHDR_ACCESSFLAG:
            logWrite("---BLE Received Header[Access Flag]--->\n");
            access_flag = ble_rxframe[1];
            switch(access_flag)
            {
            case BLE_ACCESSFLAG_HOME:
                logWrite("---ACCESS FLAG[HOME]--->\n");
                bleUpdateNetId();
                bleReply(BLEHDR_ACCESSFLAG, BLE_RESCODE_READ_SUCCESS);
                break;
            case BLE_ACCESSFLAG_SYSDIAG:
                logWrite("---ACCESS FLAG[SYS DIAG]--->\n");
                bleSysDiag();
                update_access_flag = true;
                break;
            case BLE_ACCESSFLAG_SYSCONFIG:
                logWrite("---ACCESS FLAG[SYS CONFIG]--->\n");
                blePortModeStatus();
                logWrite("\n====Port Read====\n1 0x%04x %d\n2"\
                            " 0x%04x %d\n3 0x%04x %d\n4 0x%04x "\
                            "%d\n=================\n"
                        ,PortConfig.P1.sensor_code, PortConfig.P1.status
                        ,PortConfig.P2.sensor_code, PortConfig.P2.status
                        ,PortConfig.P3.sensor_code, PortConfig.P3.status
                        ,PortConfig.P4.sensor_code, PortConfig.P4.status);
                bleReply(BLEHDR_ACCESSFLAG, BLE_RESCODE_READ_SUCCESS);
                break;
            case BLE_ACCESSFLAG_NETID:
                logWrite("---ACCESS FLAG[NETWORK ID]--->\n");
                bleUpdateNetId();
                bleReply(BLEHDR_ACCESSFLAG, BLE_RESCODE_READ_SUCCESS);
                break;
            default:
                logWrite("---ACCESS FLAG[UNKNOWN]--->\n");
                break;
            }
            break;
        case BLEHDR_TELEMREQ:
            logWrite("---BLE Received Header[Telemetry Request]--->\n");
            break;
        case BLEHDR_CONFIGMODE:
            logWrite("---BLE Received Header[Configuration Mode]--->\n");
            break;
        case BLEHDR_NET_ID:
            logWrite("---BLE Received Header[Set Network Id]--->\n");
            memcpy(&radio_net_id_toset,&ble_rxframe[1],sizeof(uint32_t));
            radio_net_id_toset = ((radio_net_id_toset>>24)&0xff) | // move byte 3 to byte 0
                     ((radio_net_id_toset<<8)&0xff0000) | // move byte 1 to byte 2
                     ((radio_net_id_toset>>8)&0xff00) | // move byte 2 to byte 1
                     ((radio_net_id_toset<<24)&0xff000000); // byte 0 to byte 3
            radioNetSet(radio_net_id_toset);	//TODO: Enable this.
            rdBnrMainGetNetId(&radio_net_id);
            if(radio_net_id == radio_net_id_toset)
            {
                bleReply(BLEHDR_NET_ID, BLE_RESCODE_WRITE_SUCCESS);
            }
            else
            {
                bleReply(BLEHDR_NET_ID, BLE_RESCODE_ERROR);
            }
            logWrite("bleDataControl: BLE DB - Net ID Updated\n");
            break;
        case BLEHDR_NET_MODE:
            logWrite("---BLE Received Header[Network Maintenance Mode]--->\n");
            logWrite("---BLE Received Command[0x%02x]--->\n");
            result_code = BLE_RESCODE_ERROR;
            switch(ble_rxframe[2])
            {
            case RDNETMAINTENANCEMODETYP_STARTCMD:
            case RDNETMAINTENANCEMODETYP_STOPCMD:
            case RDNETMAINTENANCEMODETYP_EDITCMD:                
                if(rdBnrMainUpdateMaintenanceModeState(ble_rxframe[2]
                                                    , ble_rxframe[4]) == RDSTAT_OK)
                {
                    result_code = BLE_RESCODE_WRITE_SUCCESS;
                }
                break;
            case RDNETMAINTENANCEMODETYP_CHECKCMD:
                result_code = BLE_RESCODE_READ_SUCCESS;
                break;
            }
            
            if(result_code != BLE_RESCODE_ERROR)
            {
                if(bleUpdateMaintenanceMode(ble_rxframe[2]) != BLE_SUCCESSFUL)
                {
                    result_code = BLE_RESCODE_ERROR;
                }
            }
            #if DEVICE_TYPE == DEVICETYPE_NODE
            switch(sleep_attr.nmm_status)
            {
            case RADNETMAINTENANCEMODE_TOBEENABLED:
                //TODO: Modify Interrupt priorities, Radio over Ble
                break;
            case RADNETMAINTENANCEMODE_TOBEDISABLED:
                //TODO: Modify Interrupt priorities, Ble over Radio
                break;
            }
            #endif
            bleReply(BLEHDR_NET_MODE, result_code);
            break;
        case BLEHDR_SPORT1:
        case BLEHDR_SPORT2:
        case BLEHDR_SPORT3:
        case BLEHDR_SPORT4:
            logWrite("---BLE Received Header[SENSOR/VALVE PORT]--->\n");
            if(access_flag == BLE_ACCESSFLAG_SYSCONFIG)
            {
                logWrite("Configuring port...\n");
                rx_sensorcode = (ble_rxframe[2] << 8) + ble_rxframe[3];
                memcpy(databuff,ble_rxframe,15); 		//copy bleframe to register databuff
                BLEPortConfig.SP1.sensor_code = PortConfig.P1.sensor_code;
                BLEPortConfig.SP2.sensor_code = PortConfig.P2.sensor_code;
                BLEPortConfig.SP3.sensor_code = PortConfig.P3.sensor_code;
                BLEPortConfig.SP4.sensor_code = PortConfig.P4.sensor_code;
                bleSysConfig(databuff, &BLEPortConfig);
                portProcessBLE((uint16_t[]){BLEPortConfig.SP1.sensor_code
                    ,BLEPortConfig.SP2.sensor_code, BLEPortConfig.SP3.sensor_code
                    ,BLEPortConfig.SP4.sensor_code}
                    ,(SensorRegistrationStruct*)&PortConfig);
                is_updateport = is_updateport | 
                    (1 << ((databuff[SYSCFG_HEADER] & PORT_BITMASK) - 1));
                bleCopyToSR((SensorRegistrationStruct*)&PortConfig,
                        (BleSensorConfigStruct*)&BLEPortConfig, is_updateport);
                portSRSave();
                portSRRead(&PortConfig);
				portDeInitPower();
                portInitParams();
                portInitPaths();
                //blePortModeStatus();
                logWrite("\n===Port Configured!===\n1 0x%04x %d\n2 0x%04x %d\n3 "\
                             "0x%04x %d\n4 0x%04x %d"\
                             "\n=======================\n"
                        ,PortConfig.P1.sensor_code, PortConfig.P1.status
                        ,PortConfig.P2.sensor_code, PortConfig.P2.status
                        ,PortConfig.P3.sensor_code, PortConfig.P3.status
                        ,PortConfig.P4.sensor_code, PortConfig.P4.status);
                
                switch(ble_rxheader)
                {
                    case BLEHDR_SPORT1:
                        sd_sensorcode = PortConfig.P1.sensor_code;
                        break;
                    case BLEHDR_SPORT2:
                        sd_sensorcode = PortConfig.P2.sensor_code;
                        break;
                    case BLEHDR_SPORT3:
                        sd_sensorcode = PortConfig.P3.sensor_code;
                        break;
                    case BLEHDR_SPORT4:
                        sd_sensorcode = PortConfig.P4.sensor_code;
                        break;
                }
                
                if(sd_sensorcode == rx_sensorcode 
                   || (sd_sensorcode==0x0000  && rx_sensorcode==0xFFFF ))
                {
                    logWrite("Sensor Code Compared, result[SAME]\n");
                    bleReply(BLEHDR_ACCESSFLAG, BLE_RESCODE_WRITE_SUCCESS);
                }
                else
                {
                    logWrite("Sensor Code Compared, result[NOT SAME]\n");
                    logWrite("Received[0x%04x] != Configured[0x%04x]\n"
                            ,rx_sensorcode, sd_sensorcode);
                    bleReply(BLEHDR_ACCESSFLAG, BLE_RESCODE_ERROR);
                }
                
                buzzEnable(1);
            }
            break;
        default:
            logWrite("---BLE Received Header[0x%02x] UNKNOWN!-->\n", ble_rxheader);
            update_access_flag = true; //solution to keep mobile app moving
            break;
        }
    }
    
    buzzEnable(3);
    #if PRINT_PROCESSES_IN_UART
    logWrite("\n\n\n*******BLE END******\n\n\n");
    #endif

    if(is_updateport > 0)
    {
        logWrite("Port Configuration Changed! - Reset DAQ\n");
		execute_routine = true;
        //NVIC_SystemReset();
    }
    

    return BLE_SUCCESSFUL;
}

#pragma optimize=none
/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
bool portProcessBLE(uint16_t *sensor_codes, SensorRegistrationStruct *sr_ptr)
{
	bool copy_stat = false;
	//AgTechIOStruct sub_ios[4] = {CTR1, CTR2, CTR3, CTR4};
	for(uint8_t i = 0; i < MAX_SENSORPORT; i++)
	{
		sr_ptr->port = i + 1;
        
		if(sensor_codes[i] != 0)
		{
			/*new sensor id detected*/
			if(sensor_codes[i] != sr_ptr->sensor_code)
			{					
				/*driver search*/
				memset(sr_ptr, 0, sizeof(SensorRegistrationStruct));
				if (sdCfgTestCopySensDrvr(sensor_codes[i], sr_ptr) != SUCCESSFUL
					|| sensor_codes[i] == 0xFFFF)
				{
					sr_ptr->status = PORT_INACTIVE;
					sdCfgTestCopySensDrvr(0x00, sr_ptr);
				}
				sr_ptr->status = PORT_ACTIVE_M;
			}
		}
		
		sr_ptr->port = i + 1;
		++sr_ptr;
	}
	return copy_stat;
}

#pragma optimize=none
void bleInit(void)
{    
    char *device_sn = DEVICE_INFO->device_sn;
    char *hw_ver    = "3.3.2";
    char version_str[20];
    
    logWrite("\n===== bleInit() Start =====\n");
    
    /*Read BLE Firmware Version*/
    bleReadFirmwareVersion();
    
    /*Set Device Name*/
    bleChangeAttrValue(BLEATTR_DEVICENAME, device_sn, strlen(device_sn));
    
    /*Set Serial Number*/
    bleChangeAttrValue(BLEATTR_SERIALNUM, device_sn, strlen(device_sn));
    
    /*Set HW Version*/
    bleChangeAttrValue(BLEATTR_HWVERSION, hw_ver, strlen(hw_ver));
    
    /*Set FW Version*/
    sprintf(version_str,"%d.%d.%d", FW_VER_MAIN, FW_VER_SUBMAIN, FW_VER_MINOR);
    bleChangeAttrValue(BLEATTR_FWVERSION, version_str, strlen(version_str));

    /*Set DR Version*/
	sprintf(version_str,"%d.%d.%d", DR_VER_MAIN, DR_VER_SUB, DR_VER_MINOR);
    bleChangeAttrValue(BLEATTR_DRVERSION, version_str, strlen(version_str));
    
    /*Update Network ID*/
    bleSetNetId(DEVICE_INFO->network_id);
    
    /*Enable BLE Broadcast for Mobile App*/
    bleChangeMode(BLEMODE_MOBILE);
    
    logWrite("===== bleInit() Ends =====\n");
}
