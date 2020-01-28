/******************************************************************************
  * @file		rdBnrNode.c
  * @author		Hardware Team
  * @version	v2.2.0
  * @date		04/08/16
  * @brief		General function module for Banner Radio
  *****************************************************************************
  * @attention
  *
  * COPYRIGHT(c) 2016 AgTech Labs, Inc.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright
  *   	 notice, this list of conditions and the following disclaimer in the
  *   	 documentation and/or other materials provided with the distribution.
  *   3. Neither the name of AgTech Labs, Inc. nor the names of its
  *   	 contributors may be used to endorse or promote products derived from
  *   	 this software without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  * POSSIBILITY OF SUCH DAMAGE.
  *****************************************************************************
  */

/* Includes =================================================================*/
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <deployAssert.h>

#include "agBoardInit.h"
#include "mspSpi.h"
#include "utilPwrMonitor.h"
#include "port.h"

#pragma optimize=none

	ArmMspRespStruct resp_counter;
uint32_t mspSendSensorData(uint8_t port)
{
  	ArmMspCmdStruct cmd;
//	ArmMspRespStruct resp;
	
	cmd.cmd_stx = MSP_STX;
	cmd.cmd_etx = MSP_ETX;
	cmd.cmd_action = MSPCMD1_PORT_SEND_DATA;
	cmd.cmd_port_pr = port;
	cmd.cmd_mux_pwm_pr = MSPCMD_NULL;	
        
        //phil tell the MSP a message is coming so it can reset its spi state machine
	gpioDigitalWrite(&MSP430_SPI_COMING, LOW);
        delayMillis(1);//phil without this the pulse is only 0.2 micro seconds wide not sure the MSP will see it.
        gpioDigitalWrite(&MSP430_SPI_COMING, HIGH);

	delayMillis(500);

	
	gpioDigitalWrite(&MSP430_CS_PIN, LOW);
	delayMillis(10);
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t*)&cmd, MSPCMD_SIZE, MSPCMD_TIMEOUT) != HAL_OK)
	 	assert(0);
	delayMillis(10);
	
	if(HAL_SPI_Receive(&SPIFlashRTCHandle, (uint8_t*)&resp_counter, MSPRESP_SIZE, MSPCMD_TIMEOUT) != HAL_OK)
	 	assert(0);
	
	gpioDigitalWrite(&MSP430_CS_PIN, HIGH);
	return (resp_counter.resp_data3<<24 | resp_counter.resp_data2<<16 | resp_counter.resp_data1<<8 | resp_counter.resp_data0);
}

void mspClearSensorData(uint8_t port)
{
  	ArmMspCmdStruct cmd;
	cmd.cmd_stx = MSP_STX;
	cmd.cmd_etx = MSP_ETX;
	cmd.cmd_action = MSPCMD1_PORT_CLEAR_DATA;
	cmd.cmd_port_pr = port;
	cmd.cmd_mux_pwm_pr = MSPCMD_NULL;
        
        //phil tell the MSP a message is coming so it can reset its spi state machine
	gpioDigitalWrite(&MSP430_SPI_COMING, LOW);
        delayMillis(1);//phil without this the pulse is only 0.2 micro seconds wide not sure the MSP will see it.
        gpioDigitalWrite(&MSP430_SPI_COMING, HIGH);

	delayMillis(500);

	gpioDigitalWrite(&MSP430_CS_PIN, LOW);
	delayMillis(10);
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t*)&cmd, MSPCMD_SIZE, MSPCMD_TIMEOUT) != HAL_OK)
	 	assert(0);
	gpioDigitalWrite(&MSP430_CS_PIN, HIGH);
}

void mspStartSensorRead(uint8_t port, uint32_t timerInterval)
{
  	ArmMspCmdStruct cmd;
	cmd.cmd_stx = MSP_STX;
	cmd.cmd_etx = MSP_ETX;
	cmd.cmd_action = MSPCMD1_PORT_START_READ;
	cmd.cmd_port_pr = port;
	cmd.cmd_mux_pwm_pr = MSPCMD_NULL;
        cmd.cmd_data_lsb=timerInterval & 0xff;                 // holds int to send to MSP 
        cmd.cmd_data_low_middle=(timerInterval >> 8) & 0xff;   // like the interval timer
        cmd.cmd_data_high_middle=(timerInterval >> 16) & 0xff;
        cmd.cmd_data_msb= (timerInterval>> 24) & 0xff;

        //phil tell the MSP a message is coming so it can reset its spi state machine
	gpioDigitalWrite(&MSP430_SPI_COMING, LOW);
        delayMillis(1);//phil without this the pulse is only 0.2 micro seconds wide not sure the MSP will see it.
        gpioDigitalWrite(&MSP430_SPI_COMING, HIGH);

	delayMillis(500);

	
	gpioDigitalWrite(&MSP430_CS_PIN, LOW);
	delayMillis(10);
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t*)&cmd, MSPCMD_SIZE, MSPCMD_TIMEOUT) != HAL_OK)
	 	assert(0);
	gpioDigitalWrite(&MSP430_CS_PIN, HIGH);
}

	ArmMspRespStruct resp_max;
uint32_t mspSendSensorMaxCountData(uint8_t port)
{
  	ArmMspCmdStruct cmd;
//	ArmMspRespStruct resp;
	
	cmd.cmd_stx = MSP_STX;
	cmd.cmd_etx = MSP_ETX;
	cmd.cmd_action = MSPCMD1_SEND_SENSOR_MAX_COUNT;
	cmd.cmd_port_pr = port;
	cmd.cmd_mux_pwm_pr = MSPCMD_NULL;	
        
        //phil tell the MSP a message is coming so it can reset its spi state machine
	gpioDigitalWrite(&MSP430_SPI_COMING, LOW);
        delayMillis(1);//phil without this the pulse is only 0.2 micro seconds wide not sure the MSP will see it.        
        gpioDigitalWrite(&MSP430_SPI_COMING, HIGH);

	delayMillis(500);

	
	gpioDigitalWrite(&MSP430_CS_PIN, LOW);
	delayMillis(10);
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t*)&cmd, MSPCMD_SIZE, MSPCMD_TIMEOUT) != HAL_OK)
	 	assert(0);
	delayMillis(10);
	
	if(HAL_SPI_Receive(&SPIFlashRTCHandle, (uint8_t*)&resp_max, MSPRESP_SIZE, MSPCMD_TIMEOUT) != HAL_OK)
	 	assert(0);
	
	gpioDigitalWrite(&MSP430_CS_PIN, HIGH);
	return (resp_max.resp_data3<<24 | resp_max.resp_data2<<16 | resp_max.resp_data1<<8 | resp_max.resp_data0);
}

	ArmMspRespStruct resp_min;

uint32_t mspSendSensorMinCountData(uint8_t port)
{
  	ArmMspCmdStruct cmd;
	ArmMspRespStruct resp;
	
	cmd.cmd_stx = MSP_STX;
	cmd.cmd_etx = MSP_ETX;
	cmd.cmd_action = MSPCMD1_SEND_SENSOR_MIN_COUNT;
	cmd.cmd_port_pr = port;
	cmd.cmd_mux_pwm_pr = MSPCMD_NULL;

        //phil tell the MSP a message is coming so it can reset its spi state machine
	gpioDigitalWrite(&MSP430_SPI_COMING, LOW);
        delayMillis(1);//phil without this the pulse is only 0.2 micro seconds wide not sure the MSP will see it.
        gpioDigitalWrite(&MSP430_SPI_COMING, HIGH);

	delayMillis(500);
	
	
	gpioDigitalWrite(&MSP430_CS_PIN, LOW);
	delayMillis(10);
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t*)&cmd, MSPCMD_SIZE, MSPCMD_TIMEOUT) != HAL_OK)
	 	assert(0);
	delayMillis(10);
	
	if(HAL_SPI_Receive(&SPIFlashRTCHandle, (uint8_t*)&resp_min, MSPRESP_SIZE, MSPCMD_TIMEOUT) != HAL_OK)
	 	assert(0);
	
	gpioDigitalWrite(&MSP430_CS_PIN, HIGH);
	return (resp_min.resp_data3<<24 | resp_min.resp_data2<<16 | resp_min.resp_data1<<8 | resp_min.resp_data0);
}
	ArmMspRespStruct resp_interval;

uint32_t mspSendSensorIntervalCountData(uint8_t port)
{
  	ArmMspCmdStruct cmd;
//	ArmMspRespStruct resp;
	
	cmd.cmd_stx = MSP_STX;
	cmd.cmd_etx = MSP_ETX;
	cmd.cmd_action = MSPCMD1_SEND_SENSOR_INTERVAL_COUNT;
	cmd.cmd_port_pr = port;
	cmd.cmd_mux_pwm_pr = MSPCMD_NULL;

        //phil tell the MSP a message is coming so it can reset its spi state machine
	gpioDigitalWrite(&MSP430_SPI_COMING, LOW);
        delayMillis(1);//phil without this the pulse is only 0.2 micro seconds wide not sure the MSP will see it.
        gpioDigitalWrite(&MSP430_SPI_COMING, HIGH);

	delayMillis(500);
	
	
	gpioDigitalWrite(&MSP430_CS_PIN, LOW);
	delayMillis(10);
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t*)&cmd, MSPCMD_SIZE, MSPCMD_TIMEOUT) != HAL_OK)
	 	assert(0);
	delayMillis(10);
	
	if(HAL_SPI_Receive(&SPIFlashRTCHandle, (uint8_t*)&resp_interval, MSPRESP_SIZE, MSPCMD_TIMEOUT) != HAL_OK)
	 	assert(0);
	
	gpioDigitalWrite(&MSP430_CS_PIN, HIGH);
	return (resp_interval.resp_data3<<24 | resp_interval.resp_data2<<16 | resp_interval.resp_data1<<8 | resp_interval.resp_data0);
}

	ArmMspRespStruct resp_sum;

uint32_t mspSendSensorRunningSumData(uint8_t port)
{
  	ArmMspCmdStruct cmd;
//	ArmMspRespStruct resp;
	
	cmd.cmd_stx = MSP_STX;
	cmd.cmd_etx = MSP_ETX;
	cmd.cmd_action = MSPCMD1_SEND_SENSOR_RUNNING_SUM_COUNT;
	cmd.cmd_port_pr = port;
	cmd.cmd_mux_pwm_pr = MSPCMD_NULL;
        
        //phil tell the MSP a message is coming so it can reset its spi state machine
	gpioDigitalWrite(&MSP430_SPI_COMING, LOW);
        delayMillis(1);//phil without this the pulse is only 0.2 micro seconds wide not sure the MSP will see it.
        gpioDigitalWrite(&MSP430_SPI_COMING, HIGH);

	delayMillis(500);
	
	
	gpioDigitalWrite(&MSP430_CS_PIN, LOW);
	delayMillis(10);
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t*)&cmd, MSPCMD_SIZE, MSPCMD_TIMEOUT) != HAL_OK)
	 	assert(0);
	delayMillis(10);
	
	if(HAL_SPI_Receive(&SPIFlashRTCHandle, (uint8_t*)&resp_sum, MSPRESP_SIZE, MSPCMD_TIMEOUT) != HAL_OK)
	 	assert(0);
	
	gpioDigitalWrite(&MSP430_CS_PIN, HIGH);
	return (resp_sum.resp_data3<<24 | resp_sum.resp_data2<<16 | resp_sum.resp_data1<<8 | resp_sum.resp_data0);
}

void mspResetSensorData(uint8_t port)
{
  	ArmMspCmdStruct cmd;
	cmd.cmd_stx = MSP_STX;
	cmd.cmd_etx = MSP_ETX;
	cmd.cmd_action = MSPCMD1_RESET_SENSOR_DATA;
	cmd.cmd_port_pr = port;
	cmd.cmd_mux_pwm_pr = MSPCMD_NULL;
        
        //phil tell the MSP a message is coming so it can reset its spi state machine
	gpioDigitalWrite(&MSP430_SPI_COMING, LOW);
        delayMillis(1);//phil without this the pulse is only 0.2 micro seconds wide not sure the MSP will see it.
        gpioDigitalWrite(&MSP430_SPI_COMING, HIGH);

	delayMillis(500);

	
	gpioDigitalWrite(&MSP430_CS_PIN, LOW);
	delayMillis(10);
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t*)&cmd, MSPCMD_SIZE, MSPCMD_TIMEOUT) != HAL_OK)
	 	assert(0);
	gpioDigitalWrite(&MSP430_CS_PIN, HIGH);
}
#pragma optimize=none
void mspSetPowerRails(uint8_t pwr_rail, bool state)
{
  	ArmMspCmdStruct cmd;
	ArmMspRespStruct resp;
	
	cmd.cmd_stx = MSP_STX;
	cmd.cmd_etx = MSP_ETX;
        cmd.cmd_data_lsb=0;          // holds unsigned int to send to MSP 
        cmd.cmd_data_low_middle=0;   // like the interval timer
        cmd.cmd_data_high_middle=0;
        cmd.cmd_data_msb=0;

	cmd.cmd_action = MSPCMD1_PWR_RAIL;
	cmd.cmd_port_pr = pwr_rail;
	cmd.cmd_mux_pwm_pr = state;
        
        //phil tell the MSP a message is coming so it can reset its spi state machine
	gpioDigitalWrite(&MSP430_SPI_COMING, LOW);
        delayMillis(1);//phil without this the pulse is only 0.2 micro seconds wide not sure the MSP will see it.
        gpioDigitalWrite(&MSP430_SPI_COMING, HIGH);

	delayMillis(500);
        
	
	gpioDigitalWrite(&MSP430_CS_PIN, LOW);
	delayMillis(10);
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t*)&cmd, MSPCMD_SIZE, MSPCMD_TIMEOUT) != HAL_OK)
	 	assert(0);
//	if(HAL_SPI_Receive(&SPIFlashRTCHandle, (uint8_t*)&resp, MSPCMD_SIZE, MSPCMD_TIMEOUT) != HAL_OK)
//	 	assert(0);
	gpioDigitalWrite(&MSP430_CS_PIN, HIGH);
}

void mspPortConfig(uint8_t port, uint8_t mux)
{
  	ArmMspCmdStruct cmd;
	cmd.cmd_stx = MSP_STX;
	cmd.cmd_action = MSPCMD1_PORT_CFG;
	cmd.cmd_port_pr = port;
	cmd.cmd_mux_pwm_pr = mux;
	cmd.cmd_etx = MSP_ETX;
        
        //phil tell the MSP a message is coming so it can reset its spi state machine
	gpioDigitalWrite(&MSP430_SPI_COMING, LOW);
        delayMillis(1);//phil without this the pulse is only 0.2 micro seconds wide not sure the MSP will see it.
        gpioDigitalWrite(&MSP430_SPI_COMING, HIGH);

	delayMillis(500);

	
	gpioDigitalWrite(&MSP430_CS_PIN, LOW);
	delayMillis(10);
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t*)&cmd, MSPCMD_SIZE, MSPCMD_TIMEOUT) != HAL_OK)
	 	assert(0);
	gpioDigitalWrite(&MSP430_CS_PIN, HIGH);
}

#pragma optimize=none
void mspPortPowerReset(void)
{
  	float voltage_out = 0;
	float v_diff = 0;
	float supply = 0;
  	ArmMspCmdStruct cmd;
	uint8_t pwr_code = MSPCMD3_PORTPWR_OFF;
	AgTechIOStruct *vcon_en[] = {NULL, &VCON1_EN, &VCON2_EN, &VCON3_EN, &VCON4_EN};
	AgTechIOStruct *pwr_en[] = {NULL, &PWR1_A_EN, &PWR2_A_EN, &PWR3_A_EN, &PWR4_A_EN};
	AgTechIOStruct *cs[5] = {NULL, &PM_SIG1, &PM_SIG2, &PM_SIG3, &PM_SIG4};
	
	for(uint8_t port = 1; port <= MAX_SENSORPORT; port++)
	{
		gpioDigitalWrite(vcon_en[port], LOW);
		gpioDigitalWrite(pwr_en[port], LOW);
		delayMillis(10);
		
		if(supply < SUPPLY_1V )  //phil
			 pwr_code = MSPCMD3_PORTPWR_OFF;  //phil
		else if(supply > SUPPLY_1V && supply <= SUPPLY_3V)
		{
			supply = 3.3;
			pwr_code = MSPCMD3_PORTPWR_3V3;
		}
		else if(supply > SUPPLY_3V && supply <= SUPPLY_5V)
			pwr_code = MSPCMD3_PORTPWR_5V;	
		else if(supply > SUPPLY_5V && supply <= SUPPLY_9V)
			pwr_code = MSPCMD3_PORTPWR_9V;
		else if(supply > SUPPLY_9V && supply <= SUPPLY_12V)
			pwr_code = MSPCMD3_PORTPWR_12V;
		else if(supply > SUPPLY_12V && supply <= SUPPLY_15V) //phil
			pwr_code = MSPCMD3_PORTPWR_15V; //phil
		else
			assert(0);

		cmd.cmd_stx = MSP_STX;
		cmd.cmd_etx = MSP_ETX;
		cmd.cmd_action = MSPCMD1_PORT_PWR;
		cmd.cmd_port_pr = port;
		cmd.cmd_mux_pwm_pr = pwr_code;

		gpioDigitalWrite(&MSP430_SPI_COMING, LOW);
		delayMillis(1);
		gpioDigitalWrite(&MSP430_SPI_COMING, HIGH);
		delayMillis(100);
		
		gpioDigitalWrite(&MSP430_CS_PIN, LOW);
		delayMillis(10);
		if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t*)&cmd, MSPCMD_SIZE, MSPCMD_TIMEOUT) != HAL_OK)
			assert(0);
		gpioDigitalWrite(&MSP430_CS_PIN, HIGH);
		delayMillis(10);
	}
	
}

#pragma optimize=none
void mspSetPortPower(uint8_t port, float supply)
{
  	float voltage_out = 0;
	float v_diff = 0;
  	ArmMspCmdStruct cmd;
	uint8_t pwr_code = MSPCMD3_PORTPWR_OFF;
	AgTechIOStruct *vcon_en[] = {NULL, &VCON1_EN, &VCON2_EN, &VCON3_EN, &VCON4_EN};
	AgTechIOStruct *pwr_en[] = {NULL, &PWR1_A_EN, &PWR2_A_EN, &PWR3_A_EN, &PWR4_A_EN};
	AgTechIOStruct *cs[5] = {NULL, &PM_SIG1, &PM_SIG2, &PM_SIG3, &PM_SIG4};
	
	utilPwrModeCont(cs[port], true);
	gpioDigitalWrite(vcon_en[port], LOW);
	gpioDigitalWrite(pwr_en[port], LOW);
	delayMillis(500);
	
    if(supply < SUPPLY_1V )  //phil
         pwr_code = MSPCMD3_PORTPWR_OFF;  //phil
	else if(supply > SUPPLY_1V && supply <= SUPPLY_3V)
	{
		supply = 3.3;
		pwr_code = MSPCMD3_PORTPWR_3V3;
	}
	else if(supply > SUPPLY_3V && supply <= SUPPLY_5V)
		pwr_code = MSPCMD3_PORTPWR_5V;	
	else if(supply > SUPPLY_5V && supply <= SUPPLY_9V)
		pwr_code = MSPCMD3_PORTPWR_9V;
	else if(supply > SUPPLY_9V && supply <= SUPPLY_12V)
		pwr_code = MSPCMD3_PORTPWR_12V;
    else if(supply > SUPPLY_12V && supply <= SUPPLY_15V) //phil
		pwr_code = MSPCMD3_PORTPWR_15V; //phil
	else
		assert(0);

    cmd.cmd_stx = MSP_STX;
	cmd.cmd_etx = MSP_ETX;
	cmd.cmd_action = MSPCMD1_PORT_PWR;
	cmd.cmd_port_pr = port;
	cmd.cmd_mux_pwm_pr = pwr_code;

	gpioDigitalWrite(&MSP430_SPI_COMING, LOW);
    delayMillis(1);
    gpioDigitalWrite(&MSP430_SPI_COMING, HIGH);
	delayMillis(500);
	
	gpioDigitalWrite(&MSP430_CS_PIN, LOW);
	delayMillis(10);
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t*)&cmd, MSPCMD_SIZE, MSPCMD_TIMEOUT) != HAL_OK)
		assert(0);
	gpioDigitalWrite(&MSP430_CS_PIN, HIGH);
	 delayMillis(100);
	 
	gpioDigitalWrite(vcon_en[port], HIGH);
	delayMillis(1000);
	gpioDigitalWrite(pwr_en[port], HIGH);
	delayMillis(1000);
	
	voltage_out = utilPwrGetVoltage(cs[port]);
	v_diff = voltage_out - supply;
	v_diff = (float)fabs(v_diff);
	
	/*Display Vout*/
	char output[50];
	memset(output, 0, 50);
	double intpart, fracpart_tmp, fracpart;
        
	snprintf(output,50,"MSP: Setting Vout at Port %d...\n", port);
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

void mspSleep(void)
{
  	ArmMspCmdStruct cmd;
	cmd.cmd_action = MSPCMD1_MCUSLEEP;
	cmd.cmd_port_pr = MSPCMD_NULL;
	cmd.cmd_mux_pwm_pr = MSPCMD_NULL;
	cmd.cmd_stx = MSP_STX;//phil
	cmd.cmd_etx = MSP_ETX;//phil
        
        //phil tell the MSP a message is coming so it can reset its spi state machine
	gpioDigitalWrite(&MSP430_SPI_COMING, LOW);
        delayMillis(1);//phil without this the pulse is only 0.2 micro seconds wide not sure the MSP will see it.
        gpioDigitalWrite(&MSP430_SPI_COMING, HIGH);

	delayMillis(500);
        
	gpioDigitalWrite(&MSP430_CS_PIN, LOW);
	delayMillis(10);
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t*)&cmd, MSPCMD_SIZE, MSPCMD_TIMEOUT) != HAL_OK)
	 	assert(0);
	gpioDigitalWrite(&MSP430_CS_PIN, HIGH);
}

void mspLedDiagnostic(uint8_t LedPattern, uint8_t DiagnosticPattern)
{
  	ArmMspCmdStruct cmd;
	cmd.cmd_action = MSPCMD1_LED_TEST_DIAGNOSTIC;
	cmd.cmd_port_pr = LedPattern;
	cmd.cmd_mux_pwm_pr = DiagnosticPattern;
        cmd.cmd_data_lsb=0;          // holds unsigned int to send to MSP 
        cmd.cmd_data_low_middle=0;   // like the interval timer
        cmd.cmd_data_high_middle=0;
        cmd.cmd_data_msb=0;
	cmd.cmd_stx = MSP_STX;//phil
	cmd.cmd_etx = MSP_ETX;//phil
        
        //phil tell the MSP a message is coming so it can reset its spi state machine
	gpioDigitalWrite(&MSP430_SPI_COMING, LOW);
        delayMillis(1);//phil without this the pulse is only 0.2 micro seconds wide not sure the MSP will see it.
        gpioDigitalWrite(&MSP430_SPI_COMING, HIGH);

	delayMillis(500);
        
	gpioDigitalWrite(&MSP430_CS_PIN, LOW);
	delayMillis(10);
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t*)&cmd, MSPCMD_SIZE, MSPCMD_TIMEOUT) != HAL_OK)
	 	assert(0);
	gpioDigitalWrite(&MSP430_CS_PIN, HIGH);
}

uint32_t mspSoftwareVersion(char *VersionString)
{
  	ArmMspCmdStruct cmd;
//	ArmMspRespStruct resp;
	
	cmd.cmd_stx = MSP_STX;
	cmd.cmd_etx = MSP_ETX;
	cmd.cmd_action = MSPCMD1_MSP_SOFTWARE_VERSION;
	cmd.cmd_port_pr = MSPCMD_NULL;
	cmd.cmd_mux_pwm_pr = MSPCMD_NULL;	
        
        //phil tell the MSP a message is coming so it can reset its spi state machine
	gpioDigitalWrite(&MSP430_SPI_COMING, LOW);
        delayMillis(1);//phil without this the pulse is only 0.2 micro seconds wide not sure the MSP will see it.        
        gpioDigitalWrite(&MSP430_SPI_COMING, HIGH);

	delayMillis(500);

	
	gpioDigitalWrite(&MSP430_CS_PIN, LOW);
	delayMillis(10);
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t*)&cmd, MSPCMD_SIZE, MSPCMD_TIMEOUT) != HAL_OK)
	 	assert(0);
	delayMillis(10);
	
	if(HAL_SPI_Receive(&SPIFlashRTCHandle, (uint8_t*)VersionString, MSPRESP_SIZE, MSPCMD_TIMEOUT) != HAL_OK)
	 	assert(0);
	
	gpioDigitalWrite(&MSP430_CS_PIN, HIGH);
}

void mspwatchdogkick(void)
{
        ArmMspCmdStruct cmd;
	cmd.cmd_action = MSPCMD1_WATCHDOG_KICK;
	cmd.cmd_port_pr = MSPCMD_NULL;
	cmd.cmd_mux_pwm_pr = MSPCMD_NULL;
	cmd.cmd_stx = MSP_STX;//phil
	cmd.cmd_etx = MSP_ETX;//phil
        
        //phil tell the MSP a message is coming so it can reset its spi state machine
	gpioDigitalWrite(&MSP430_SPI_COMING, LOW);
        delayMillis(1);//phil without this the pulse is only 0.2 micro seconds wide not sure the MSP will see it.
        gpioDigitalWrite(&MSP430_SPI_COMING, HIGH);

	delayMillis(500);
        
	gpioDigitalWrite(&MSP430_CS_PIN, LOW);
	delayMillis(10);
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t*)&cmd, MSPCMD_SIZE, MSPCMD_TIMEOUT) != HAL_OK)
	 	assert(0);
	gpioDigitalWrite(&MSP430_CS_PIN, HIGH);
}