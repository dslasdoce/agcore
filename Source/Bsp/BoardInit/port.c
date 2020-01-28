/**
  ******************************************************************************
  * @file      	port.c
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


/* Includes ==================================================================*/
#include <string.h>
#include <deployAssert.h>
#include <math.h>
#include <stdlib.h>

#include "gpio.h"
#include "isrMain.h"
#include "intFlash.h"
#include "delay.h"
#include "sigpathSetup.h"
#include "agBoardInit.h"
#include "oneWire.h"
#include "daqProcess.h"
#include "port.h"
#include "utilMem.h"
#include "sdMain.h"
#include "productCfg.h"  
#include "bsp_pwm.h"
#include "ff.h"
#include "utilPwrMonitor.h"
#include "mspSpi.h"    
/* Defines ===================================================================*/
#define CODE_MUXID_COUNT 			((uint8_t)4)
#define SIZE_SENSOR_ID				((uint8_t)20)
//#define CODE_MUXRES_75				((uint8_t)0x00)
//#define CODE_MUXRES_360				((uint8_t)0x01)
//#define CODE_MUXRES_10k				((uint8_t)0x02)
//#define CODE_MUXRES_PULLUP			((uint8_t)0x03)
//#define MAX_CURRENT_5mA				((uint8_t)5)
//#define MAX_CURRENT_20mA			((uint8_t)20)
#define MAXSIZE_SR_S 			((uint32_t)32)
#define MAXSIZE_SR_L 			((uint32_t)64)
#define BYTES_PER_WORD 			((uint8_t)4)

#define	SUPPLY_3V				((float)4)
#define	SUPPLY_5V				((float)6)
#define	SUPPLY_9V				((float)10)
#define	SUPPLY_12V				((float)12)

#define SIGx_MUX_DIG    		(0)
#define SIGx_MUX_I      		(1)
#define SIGx_MUX_V   	  		(2)
#define SIGx_THERM	     		(3)

#define CTRx_MUX_PU_3    		(0)
#define CTRx_MUX_PU_10   		(1)
#define CTRx_MUX_PD_10   		(2)
#define CTRx_MUX_PD_GND  		(3)

/* Global Declarations =======================================================*/
uint32_t sr_next_free_address = ADDR_FLASH_SECTOR_5;
const uint32_t sr_max_address = ADDR_FLASH_SECTOR_5 +
								sizeof(PortConfigStruct)*5;
bool exec_daq1, exec_daq2, exec_daq3, exec_daq4, exec_sysstat, exec_poll_post;

PortDataProcStruct PortDataProcessing;

PortConfigStruct PortConfig;

/* Functions =================================================================*/

#if !RTC_BASED_CYCLE
/*============================================================================
 * @brief
 * @param
 * @retval
============================================================================*/
static void portEnableDaq(SensorRegistrationStruct *sensordetails_ptr)
{
//	if(sensordetails_ptr->status == true)
//		switch(sensordetails_ptr->port)
//		{
//			case 1:
//				HAL_NVIC_EnableIRQ(DAQTIM_P1_IRQn);
//                exec_daq1 = true;
//				break;
//			case 2:
//				HAL_NVIC_EnableIRQ(DAQTIM_P2_IRQn);
//                exec_daq2 = true;
//				break;
//			case 3:
//				HAL_NVIC_EnableIRQ(DAQTIM_P3_IRQn);
//                exec_daq3 = true;
//				break;
//			case 4:
//				HAL_NVIC_EnableIRQ(DAQTIM_P4_IRQn);
//                exec_daq4 = true;
//				break;
//			default:
//				return;
//		}
}
#endif

/* ============================================================================
 * @brief	save sensor registration value to flash
 * ==========================================================================*/
//void portSRSave(void)
//{
//	if(sr_next_free_address == sr_max_address)
//	{
//		/*reset*/
//		sr_next_free_address = ADDR_FLASH_SECTOR_5;
//                intFlErase(FLASH_SECTOR_5, 1);
//	}
//
//	if (intFlDmaExec((uint32_t)&PortConfig, sr_next_free_address,
//			   sizeof(PortConfigStruct)/BYTES_PER_WORD) != SUCCESSFUL)
//	{
//	   assert(0);
//	   return;
//	}
//	sr_next_free_address += sizeof(PortConfigStruct);
//}

void portSRSave(void)
{
	SensorRegistrationStruct *sr_ptr = (SensorRegistrationStruct*)&PortConfig;
	for(uint8_t i = 1; i <= MAX_SENSORPORT; i++)
	{
		if (sr_ptr->sensor_code == 0)
		{
			sdCfgTestCopySensDrvr(0x00, sr_ptr);
			sr_ptr->port = i;
			delayMillis(10);
		}
		sr_ptr->interval &= PWR_INT_MASK_RESET;
		sdUpdateSensDrvrOrRegn(SENSREGN, sr_ptr, NULL);
		delayMillis(10);
		sr_ptr++;
	}
//	sdUpdateSensDrvrOrRegn(SENSREGN, &PortConfig.P1, NULL);
//    sdUpdateSensDrvrOrRegn(SENSREGN, &PortConfig.P2, NULL);
//    sdUpdateSensDrvrOrRegn(SENSREGN, &PortConfig.P3, NULL);
//    sdUpdateSensDrvrOrRegn(SENSREGN, &PortConfig.P4, NULL);
}

//void portSRRead(PortConfigStruct* dest_addr)
//{
//	intFlDmaExec(((sr_next_free_address == ADDR_FLASH_SECTOR_5) ?
//				sr_next_free_address: sr_next_free_address -
//				sizeof(PortConfigStruct)), (uint32_t)dest_addr,
//				sizeof(SensorRegistrationStruct));
//}

void portSRRead(PortConfigStruct* dest_addr)
{
	memset(dest_addr, 0, sizeof(PortConfigStruct));
	dest_addr->P1.port = 1;
	dest_addr->P2.port = 2;
	dest_addr->P3.port = 3;
	dest_addr->P4.port = 4;
	SensorRegistrationStruct *sr_ptr = (SensorRegistrationStruct*)dest_addr;
	for(uint8_t i = 1; i <= MAX_SENSORPORT; i++)
	{
		sdGetSensDrvrOrRegn(SENSREGN, sr_ptr, NULL);
		sr_ptr++;
	}
	//sdGetSensDrvrOrRegn(SENSREGN, &dest_addr->P2, NULL);
	//sdGetSensDrvrOrRegn(SENSREGN, &dest_addr->P3, NULL);
	//sdGetSensDrvrOrRegn(SENSREGN, &dest_addr->P4, NULL);
}

/* ============================================================================
 * @brief	scan for attached sensor for each port
 * ==========================================================================*/
void portScan(uint16_t *sensor_codes)
{
	AgTechIOStruct *id_onewire[] = {&SIG1_ID, &SIG2_ID, &SIG3_ID, &SIG4_ID};
	
	for(uint8_t i = 0; i < CODE_MUXID_COUNT; i++)
	{
		if (!portReadID(sensor_codes + i, id_onewire[i]))
			sensor_codes[i] = 0;
		oneWireReset(id_onewire[i]);
		delayMillis(100);

	}
}

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
bool portProcess(uint16_t *sensor_codes, SensorRegistrationStruct *sr_ptr)
{
	bool copy_stat = false;
	//AgTechIOStruct sub_ios[4] = {CTR1, CTR2, CTR3, CTR4};
    uint8_t prev_stat = 0;
	for(uint8_t i = 0; i < MAX_SENSORPORT; i++)
	{
		sr_ptr->port = i + 1;
        
		if(sensor_codes[i] != 0)
		{
			/*new sensor id detected*/
			if(sensor_codes[i] != sr_ptr->sensor_code)
			{				
				sr_ptr->status = PORT_ACTIVE_A;
				copy_stat = true;
			}
			/*same sensor id detected*/
			else
			{
				if((sr_ptr->status != PORT_ACTIVE_A) 
				   && (sr_ptr->status != PORT_ACTIVE_M))
				{
					sr_ptr->status = PORT_ACTIVE_A;
					copy_stat = true;
				}

			}
             prev_stat = sr_ptr->status;
		}
		else
		{
			if(sr_ptr->status != PORT_ACTIVE_M)
			{
				sr_ptr->status = PORT_INACTIVE;
				memset(sr_ptr, 0, sizeof(SensorRegistrationStruct));
				copy_stat = true;
			}
		}
		
		if(copy_stat == true && sensor_codes[i] != 0)
		{
		  	/*driver search*/
		  	memset(sr_ptr, 0, sizeof(SensorRegistrationStruct));
		  	if (sdCfgTestCopySensDrvr(sensor_codes[i], sr_ptr) != SUCCESSFUL)
			{
			  	sr_ptr->status = PORT_INACTIVE;
				sdCfgTestCopySensDrvr(0x00, sr_ptr);
			}
			else
            	sr_ptr->status = prev_stat;
		}
		if(sr_ptr->sensor_code == 0)
		  	sr_ptr->status = PORT_INACTIVE;
		sr_ptr->port = i + 1;
		++sr_ptr;
	}
	return copy_stat;
}

/* =============================================================================
 * @brief	initialize each port
 * ===========================================================================*/
void portInitParams(void)
{
    SensorRegistrationStruct *sr_ptr = (SensorRegistrationStruct*)&PortConfig;
    DataProcessingStruct *dproc_ptr = (DataProcessingStruct*)&PortDataProcessing;
    memset(dproc_ptr, 0, sizeof(PortDataProcStruct));
	for(uint8_t i = 0; i < MAX_SENSORPORT; i++)
    {
        if(sr_ptr->status == PORT_ACTIVE_M || sr_ptr->status == PORT_ACTIVE_A)
		{		
			daqSetProcessingParams(dproc_ptr, sr_ptr);	
		}
		sr_ptr++;
		dproc_ptr++;
    }
}

#pragma optimize=none
void portDeInitPower(void)
{
	PortConfig.P1.interval = PortConfig.P1.interval & PWR_INT_MASK_RESET;
	PortConfig.P2.interval = PortConfig.P2.interval & PWR_INT_MASK_RESET;
	PortConfig.P3.interval = PortConfig.P3.interval & PWR_INT_MASK_RESET;
	PortConfig.P4.interval = PortConfig.P4.interval & PWR_INT_MASK_RESET;
	pwmPwrOFFAllPorts();
	mspPortPowerReset();
}
	

void portInitPaths(void)
{
    SensorRegistrationStruct *sr_ptr = (SensorRegistrationStruct*)&PortConfig;
    for(uint8_t i = 0; i < MAX_SENSORPORT; i++)
    {
        if(sr_ptr->status == PORT_ACTIVE_M || sr_ptr->status == PORT_ACTIVE_A)
		{		
			portSetup(sr_ptr);
		}
		sr_ptr++;
    }
}

void portDeInitPaths(void)
{
  	gpioDigitalWrite(&SIG1_SEL, LOW);
	gpioDigitalWrite(&SIG2_SEL, LOW);
	gpioDigitalWrite(&SIG3_SEL, LOW);
	gpioDigitalWrite(&SIG4_SEL, LOW);
	
  	gpioDigitalWrite(&SIG1_SEL0, LOW);
	gpioDigitalWrite(&SIG2_SEL0, LOW);
	gpioDigitalWrite(&SIG3_SEL0, LOW);
	gpioDigitalWrite(&SIG4_SEL0, LOW);
	
	gpioDigitalWrite(&SIG1_SEL1, LOW);
	gpioDigitalWrite(&SIG2_SEL1, LOW);
	gpioDigitalWrite(&SIG3_SEL1, LOW);
	gpioDigitalWrite(&SIG4_SEL1, LOW);
	
	gpioDigitalWrite(&RSx_A0_SEL,LOW);
	gpioDigitalWrite(&RSx_A1_SEL, LOW);
	
	gpioDigitalWrite(&RSx_B0_SEL, LOW);
	gpioDigitalWrite(&RSx_B1_SEL, LOW);
}

/* =============================================================================
 * @brief	initialize each port
 * ===========================================================================*/
void portInit(void)
{
    bool is_save = false;
	/*Available drivers
	 * DVIS_TMP_HMD, DVIS_TMP, DVIS_RAIN, WP_1000, HSTI_40,AQCHK */
	uint16_t sensor_codes[4];
	
    portSRRead(&PortConfig);
	portScan(sensor_codes);
	
	#if USE_FW_PORTACTIVATION
		sensor_codes[0] = PORT1_SENSOR; 
		sensor_codes[1] = PORT2_SENSOR; 
		sensor_codes[2] = PORT3_SENSOR;
		sensor_codes[3] = PORT4_SENSOR;
	#endif

	is_save = portProcess(sensor_codes, (SensorRegistrationStruct*)&PortConfig);  
	if(is_save)
		portSRSave();
	
//    /*Set True to activate port*/
//	#if USE_FW_PORTACTIVATION
//		PortConfig.P1.status = PORT1_ACTIVE;
//		PortConfig.P2.status = PORT2_ACTIVE;
//		PortConfig.P3.status = PORT3_ACTIVE;
//		PortConfig.P4.status = PORT4_ACTIVE;
//	#endif
	
	
	portInitParams();

 	delayMillis(500);
 	#if !RTC_BASED_CYCLE
		portEnableDaq(&PortConfig.P1);
		portEnableDaq(&PortConfig.P2);
		portEnableDaq(&PortConfig.P3);
		portEnableDaq(&PortConfig.P4);
	#endif
}

/* ============================================================================
 * @brief 	turn off the power on the port
 * @param 	port: port number where the power will be cut
 * ==========================================================================*/
void portPowerReset(uint8_t port)
{
	pwmPwrOFF(port);
}

#pragma optimize=none
/* ============================================================================
 * @brief	turn on power on the port
 * @param	port: port number where the power will be cut
 * @param	suuply_code: indicates the hex code of the supply to be used
 * ==========================================================================*/
void portPowerSet(uint8_t port, float supply)
{
	float duty_cycle = 60;
	float voltage_out = 0;
	float v_diff = 0;
	AgTechIOStruct *cs[5] = {NULL, &PM_SIG1, &PM_SIG2, &PM_SIG3, &PM_SIG4};
	
	utilPwrModeCont(cs[port], true);
	if(supply <= SUPPLY_3V)
	{
		supply = 3.3;
		duty_cycle = DUTYCYC_3V;
	}
	else if(supply > SUPPLY_3V && supply <= SUPPLY_5V)
		duty_cycle = DUTYCYC_5V;
	
	else if(supply > SUPPLY_5V && supply <= SUPPLY_9V)
		duty_cycle = DUTYCYC_9V;
	
	else if(supply > SUPPLY_9V && supply <= SUPPLY_12V)
		duty_cycle = DUTYCYC_12V;
	else
		assert(0);
	
	pwmStartPwr(port, duty_cycle);
	
	voltage_out = utilPwrGetVoltage(cs[port]);
	v_diff = voltage_out - supply;
	v_diff = (float)fabs(v_diff);
	
	/*Display Vout*/
	char output[50];
	memset(output, 0, 50);
	double intpart, fracpart_tmp, fracpart;
        
	snprintf(output,50,"ARM: Setting Vout at Port %d...\n", port);
        uartSHOW((uint8_t*)output, strlen(output));
	
    fracpart_tmp = modf(supply,&intpart);
	modf(fracpart_tmp*100000, &fracpart);
	fracpart = abs(fracpart);
    snprintf(output,50,"Target= %6d.%05d\n", (int)intpart, (int)fracpart);
    uartSHOW((uint8_t*)output, strlen(output));
        
	memset(output, 0, 50);
	fracpart_tmp = modf(voltage_out,&intpart);
	modf(fracpart_tmp*100000, &fracpart);
	fracpart = abs(fracpart);
	snprintf(output,50,"Actual= %6d.%05d\n\n", (int)intpart, (int)fracpart);
	uartSHOW((uint8_t*)output, strlen(output));
	
	utilPwrModeCont(cs[port], false);
	if(v_diff > VOUT_MARGIN)
		assert(0);
}

/* =============================================================================
 * @brief	write sensor id to eeprom
 * @param	id_str: string to be written
 * ===========================================================================*/
void portWriteID(uint16_t id, AgTechIOStruct *gpio)
{
	//AgTechIOStruct *gpio = &ID_1WIRE;
	uint8_t cmd_write_scratchpad = 0x0F;
	uint8_t target_address[] = {0x00, 0x00};
	uint8_t crc[2];
	uint8_t addr[8];
	uint8_t auth_code[3];
	uint8_t data[8];
	uint8_t id_str[8];
	id_str[0] = id & 0xFF;
	id_str[1] = id >> 8;
	memset(data, 0, 8);
	memset(crc, 0, 2);
	uint16_t crc2 = 0;
	//crc16(data, 16, crc2);
	volatile uint8_t copy_status = 0;
	volatile uint8_t search_result;
	
	delayMillis(100);
	oneWireReset(gpio);
	delayMillis(100);
	
//    gpioDigitalWrite(&ID_1OENB, 0);
//    gpioDigitalWrite(&ID_SEL0, 0);
//    gpioDigitalWrite(&ID_SEL1,0);
    
	search_result = oneWireSearch(gpio, addr);
	if(!search_result)
		return;

	/*reset bus*/
	oneWireReset(gpio);
	/*skip rom*/
	oneWireWrite(gpio, 0xCC, 0);
	/*issue write scratchpad command*/
	oneWireWrite(gpio, cmd_write_scratchpad, 0);
	/*issue target address*/
	oneWireWriteBytes(gpio, target_address,
					  sizeof(target_address)/sizeof(uint8_t), 0);
	/*issue data*/
	oneWireWriteBytes(gpio, (uint8_t *)id_str, 8, 1);
	/*read crc*/
	oneWireReadBytes(gpio, crc, sizeof(crc)/sizeof(uint8_t));

	/*reset bus*/
	oneWireReset(gpio);
	/*skip rom*/
	oneWireWrite(gpio, 0xCC, 0);
	/*issue read scratchpad command*/
	oneWireWrite(gpio, 0xAA, 0);
	/*read 3 byte auth code*/
	oneWireReadBytes(gpio, auth_code, sizeof(auth_code)/sizeof(uint8_t));
	/*read data*/
	oneWireReadBytes(gpio, data, sizeof(data)/sizeof(uint8_t));
	/*read crc*/
	oneWireReadBytes(gpio, crc, sizeof(crc)/sizeof(uint8_t));

	/*reset bus*/
	oneWireReset(gpio);
	/*skip rom*/
	oneWireWrite(gpio, 0xCC, 0);
	/*issue copy scratchpad command*/
	oneWireWrite(gpio, 0x55, 0);
	/*issue auth code*/
	oneWireWriteBytes(gpio, auth_code,
					  sizeof(auth_code)/sizeof(uint8_t), 0);
	delayMillis(13);
	copy_status = oneWireRead(gpio);
	if(copy_status != 0xAA)
		return;

	oneWireReset(gpio);
	/*skip rom*/
	oneWireWrite(gpio, 0xCC, 1);
	/*read memory*/
	oneWireWrite(gpio, 0xF0, 1);
	/*send target address*/
	oneWireWriteBytes(gpio, target_address,
					  sizeof(target_address)/sizeof(uint8_t), 1);
	/*read data*/
	oneWireReadBytes(gpio, data, 2);

	oneWireReset(gpio);
	//uint16_t xxx;
	//delayMillis(1000);
	//portReadID(&xxx);
}

/* ============================================================================
 * @brief	read sensor id from eeprom
 * ==========================================================================*/
bool portReadID(uint16_t *id, AgTechIOStruct *gpio)
{
	//AgTechIOStruct *gpio = &ID_1WIRE;
	uint8_t data[2];
	uint8_t target_address[] = {0x00, 0x00};
	volatile uint8_t search_result;
	uint8_t crc_code;
	uint8_t addr[8];
	
	delayMillis(100);
	oneWireReset(gpio);
	delayMillis(100);


	search_result = oneWireSearch(gpio, addr);
	if (search_result)
	{
		crc_code = crc8(addr, 7);
		if ( crc_code == addr[7])
		{
			oneWireReset(gpio);
			/*skip rom*/
			oneWireWrite(gpio, 0xCC, 0);
			/*read memory*/
			oneWireWrite(gpio, 0xF0, 0);
			/*send target address*/
			oneWireWriteBytes(gpio, target_address,
							  sizeof(target_address)/sizeof(uint8_t), 0);
			/*read data*/
			oneWireReadBytes(gpio, data, sizeof(data)/sizeof(uint8_t));

			oneWireReset(gpio);
			//strncpy((char*)id_str, (char*)data, 8);
			*id = data[1]<<8 | data[0];
			return true;
		}
	}
	return false;
}

/* ============================================================================
 * @brief	configure which IO pin will be used by a port
 * @param	pointer to port configuration
 * ==========================================================================*/
static void portSetIO(SensorRegistrationStruct *sensordetails_ptr)
{
	switch(sensordetails_ptr->port)
	{
		case 1:
			sensordetails_ptr->gpio = DIG1;
			break;
		case 2:
			sensordetails_ptr->gpio = DIG2;
			break;
		case 3:
			sensordetails_ptr->gpio = DIG3;
			break;
		case 4:
			sensordetails_ptr->gpio = DIG4;
			break;
		default:
			break;
	}
}

static InterfaceStatusEnum portSetInterface(uint8_t port,
								InterfaceCodeEnum code_interface)
{
	AgTechIOStruct *sigx_sel_ptr[] = {&SIG1_SEL, &SIG2_SEL, &SIG3_SEL ,&SIG4_SEL};
    AgTechIOStruct *sigx_sel0_ptr[] = {&SIG1_SEL0, &SIG2_SEL0, &SIG3_SEL0 ,&SIG4_SEL0};
    AgTechIOStruct *sigx_sel1_ptr[] = {&SIG1_SEL1, &SIG2_SEL1, &SIG3_SEL1 ,&SIG4_SEL1};
    InterfaceStatusEnum if_stat = INTERFACE_OK;
    uint8_t sigx_mux_code = 0;
    uint8_t msp_ctrx_mux_cmd = 0; /*under MSP430*/
    //uint8_t rsx_mux_code = 0;
    
    bool ctrx_pupd_en = false; /*under MSP430*/
    bool sigx_en = false;
    //bool rsx_en = false;
    
	switch(code_interface){
		case IFC_ANALOG_V:
            sigx_mux_code = SIGx_MUX_V;
            sigx_en = true;
			break;
		case IFC_ANALOG_I:
            sigx_mux_code = SIGx_MUX_I;
            sigx_en = true;
			break;
		case IFC_ANALOG_R: //Currently sigx_therm
		  	sigx_mux_code = SIGx_THERM;
            sigx_en = true;
			break;
		case IFC_SDI12:
            sigx_mux_code = SIGx_MUX_DIG;
            sigx_en = true;
			break;
		case IFC_ANALOG_V_R: //Unavailable
			break;
		case IFC_ANALOG_V_V:
            sigx_mux_code = SIGx_MUX_V;
            sigx_en = true;
			break;
		case IFC_COUNTER_PU_C:
		case IFC_COUNTER_PU_P:
            msp_ctrx_mux_cmd = MSPCMD3_MUX_10K_UP;
            ctrx_pupd_en = true;
			break;
		case IFC_COUNTER_PD_C:
		case IFC_COUNTER_PD_P:
            msp_ctrx_mux_cmd = MSPCMD3_MUX_10K_DOWN;
            ctrx_pupd_en = true;
			break;
        case IFC_AV_CPUC:
		case IFC_AV_CPUP:
			sigx_mux_code = SIGx_MUX_V;
            sigx_en = true;
			msp_ctrx_mux_cmd = MSPCMD3_MUX_10K_UP;
            ctrx_pupd_en = true;
			break;
        case IFC_AV_CPDC:
		case IFC_AV_CPDP:
			sigx_mux_code = SIGx_MUX_V;
            sigx_en = true;
			msp_ctrx_mux_cmd = MSPCMD3_MUX_10K_DOWN;
            ctrx_pupd_en = true;
			break;
		case IFC_AV_CNPC:
		case IFC_AV_CNPP:
			sigx_mux_code = SIGx_MUX_V;
            sigx_en = true;
			break;
		case IFC_COUNTER_NP_C:
        case IFC_COUNTER_NP_P:
		case IFC_RS485:
        case IFC_DAVIS_ISS:
        case IFC_RS232:
        case IFC_DAVIS_TEMP:
			break;
		case IFC_DAVIS_DTH:
		  	sigx_mux_code = SIGx_MUX_DIG;
            sigx_en = true;
			break;
		case IFC_THERM:
		  	sigx_mux_code = SIGx_THERM;
            sigx_en = true;
		  	break;
		case IFC_BLE:
		  	break;
		default:
			if_stat = INTERFACE_ERROR;
			break;
	}

	gpioDigitalWrite(sigx_sel_ptr[port - 1], sigx_en);
	gpioDigitalWrite(sigx_sel0_ptr[port - 1], sigx_mux_code&0x01);
	gpioDigitalWrite(sigx_sel1_ptr[port - 1], sigx_mux_code>>1);
	portRSXSet(port);
	if(ctrx_pupd_en)
	  	mspPortConfig(port, msp_ctrx_mux_cmd);
}

void portRSXSet(uint8_t port)
{
	gpioDigitalWrite(&RSx_A0_SEL, (port - 1)&0x01);
	gpioDigitalWrite(&RSx_A1_SEL, (port - 1)>>1);
	
	gpioDigitalWrite(&RSx_B0_SEL, (port - 1)&0x01);
	gpioDigitalWrite(&RSx_B1_SEL, (port - 1)>>1);
}

void rs232_485Select(InterfaceCodeEnum interface)
{
  	gpioDigitalWrite(&RS485_FEN, HIGH);
	gpioDigitalWrite(&RS485_RES_EN, LOW);
	switch(interface){
    	case IFC_RS485: 
    	case IFC_DAVIS_ISS:
		  	gpioDigitalWrite(&RS232_ENB, HIGH);
      		break;
	  	case IFC_RS232:
		  	gpioDigitalWrite(&RS232_ENB, LOW);
		  	break;
		default:
		  	assert(0);
	}
}

/* ============================================================================
 * @brief setup signal path and IO to be used by a port
 * @param	sensordetails_ptr: ptr to SensorRegistrationStruct
 * ==========================================================================*/
void portSetup(SensorRegistrationStruct *sensordetails_ptr)
{  
	//sigpathAnalogEn(sensordetails_ptr->port, false);
	portSetInterface(sensordetails_ptr->port, 
                    (InterfaceCodeEnum)sensordetails_ptr->interface);
	//if(status == INTERFACE_OK)
		portSetIO(sensordetails_ptr);
	//sigpathAnalogEn(sensordetails_ptr->port, analog_en_state);
	//return status;
}

void sdtest()
{	ReturnStatusEnum sd_stat;
	FRESULT f_res;
	char *fname_ptr = (char *)SD_FNAME_ACCNTID;
	FIL file_obj;
	char *acct_id_ptr = NULL;
	//char acct_id_ptr[10];
	/*malloc poiner for account ID */
	//acct_id_ptr = NULL; /*Null value initially*/
	f_res = f_chdir("/Cfg/AcctDev");
	if(!f_res)
		/*open file*/
		sd_stat = sdReadInit(fname_ptr, &file_obj);
	
	if(sd_stat)
	{
		acct_id_ptr = (char *)memAlloc(10);
		f_gets(acct_id_ptr, 10, &file_obj);
		acct_id_ptr = (char *)memReAlloc(acct_id_ptr , strlen(acct_id_ptr));

		if( isdigit(acct_id_ptr[strlen(acct_id_ptr) - 1]) == 0)
			acct_id_ptr[strlen(acct_id_ptr) - 1] = '\0';
	}
	//f_gets(acct_id_ptr, 9, &file_obj);
	/*close file*/
	f_res = f_close(&file_obj);
	if(f_res)
		sd_stat = FAILED;
	
//	#ifdef __HWTEST__
//		ACCOUNT_ID = 9;		/*Look for associated account number based on property*/
//	#else
//		ACCOUNT_ID = (uint32_t)strtoul(acct_id_ptr, NULL, 10);
//	#endif
	if(acct_id_ptr)
		free(acct_id_ptr);
}
/*******************************************************************************
  * Revision History
  *	@file       port.c
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
