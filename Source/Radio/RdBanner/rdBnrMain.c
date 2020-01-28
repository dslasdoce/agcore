/******************************************************************************
  * @file		rdBnrMain.c
  * @author		Hardware Team, Agtech Labs Inc.
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
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "agBoardInit.h"
#include "loggerMain.h"
#include "modbus.h"
#include "radio.h"
#include "rdBnrMain.h"
#include "utilMem.h"
#include "dmMain.h"
#include "agstring.h"

/* Typedefs =================================================================*/
typedef enum 
{                      
	BNRRD_AT_OPEN,
	BNRRD_AT_CLOSE,
	BNRRD_AT_RESET
} rdBnrAtEnum;                      /*AT Mode Action*/

/* Defines ==================================================================*/
#define AT_MAXTRIES   		3	    /*max try to receive data*/
#define BUFF_ATRX_SZ 	 	15	    /*max byte size of receiving data*/
#define BUFF_RX_SZ          8	    /*max byte size of status response*/
#define BINDCODE_SZ  		11	    /*byte size of of AT bind code*/

/* Constants ================================================================*/
const uint16_t POLL_REC_COUNT[] = /*Dynamic max record count by record type*/
{ 
    RECSZ_PER_POLL_TELEM,
    RECSZ_PER_POLL_SYSSTAT,
    RECSZ_PER_POLL_VLVSCHED,
    RECSZ_PER_POLL_VLVCMD,
    RECSZ_PER_POLL_VLVREG,
    RECSZ_PER_POLL_VLVDRV,
    RECSZ_PER_POLL_SENREG,
    RECSZ_PER_POLL_SENDRV,
}; /*WARNING! Do note rearrange this!*/

const uint8_t BYTESIZE[] = /*Dynamic byte size by record type*/
{
	TELEM_REC_SIZE_BYTES,
	SYSSTAT_REC_SIZE_BYTES//,
	//VLVSCHED_REC_SIZE_BYTES,
	//VLVCMD_REC_SIZE_BYTES,
	//VLVREG_REC_SIZE_BYTES,
	//VLVDRV_REC_SIZE_BYTES,
	//SENREG_REC_SIZE_BYTES,
	//SENDRV_REC_SIZE_BYTES,
}; /*WARNING! Do note rearrange this!*/

const uint8_t MAX_FRAME_REC[] = /*Dynamic max record count by record type*/
{ 
	MAX_REC_PER_FRAME_TELEM,
	MAX_REC_PER_FRAME_SYSSTAT	//,
	//MAX_REC_PER_FRAME_VLVSCHED,
	//MAX_REC_PER_FRAME_VLVCMD,
	//MAX_REC_PER_FRAME_VLVREG,
	//MAX_REC_PER_FRAME_VLVDRV,
	//MAX_REC_PER_FRAME_SENREG,
	//MAX_REC_PER_FRAME_SENDRV,
}; /*WARNING! Do note rearrange this!*/

#if DEVICE_TYPE == DEVICETYPE_NODE
uint8_t m_radio_signal_strength;
#endif

/* Functions ================================================================*/

#pragma optimize=none
/*============================================================================
* @brief    Receiving AT response from Banner radio
* @param	rdiobuff_ptr: pointer of receiving data
* @retval	Function status
============================================================================*/
static RdStatEnum rdBnrMainAtRx(uint8_t *arg_rxdata, uint32_t arg_rxsize) 
{
    const bool FUNCTION_DEBUGMODE = false; //FIXME: set this to true to show bug
    RdStatEnum retval = RDSTAT_ERROR;   /*return status*/   
	uint32_t t_start = 0;               /*lap time reference*/
	uint8_t uart_rx = 0;                /*buffer of receiving byte from UART*/
	uint8_t rx_ctr = 0;                 /*counter of receiving byte*/
    char printout[50];
    
    rdClear(*arg_rxdata);
	t_start = HAL_GetTick();

	HAL_UART_DeInit(&UART2HandlerDef);
	HAL_UART_Init(&UART2HandlerDef);
	while(delayTimeDiff(t_start, HAL_GetTick()) < BNRRADIO_RXDLY) 
	{
		if(uartRxBanner(&uart_rx, sizeof(uart_rx), UART_TIMEOUT_BANNER) == HAL_OK) 
		{
		  	arg_rxdata[rx_ctr++] = uart_rx;
			if(uart_rx == '\r') 
			{
				retval = RDSTAT_OK;
				break;
			}
		}
		else
		{
			HAL_UART_DeInit(&UART2HandlerDef);
			HAL_UART_Init(&UART2HandlerDef);
		}

        if(rx_ctr > arg_rxsize)
        {
            #if PRINT_PROCESSES_IN_UART
            sprintf(printout, "rdBnrMainAtRx() MAXOUT Receiving Data Buffer\n");
            uartSHOW((uint8_t*)printout, strlen(printout));
            #endif
            break;
        }
	}
    
    if(FUNCTION_DEBUGMODE == true)
    {
        if(retval != RDSTAT_OK) 
        {
            sprintf(printout, "rdBnrMainAtRx() ERROR\n");
        }
        else
        {
            sprintf(printout, "rdBnrMainAtRx() OK\n");
        }
        uartSHOW((uint8_t*)printout, strlen(printout));
    }
                
	return retval;
}

/*============================================================================
* @brief    Portal of Banner radio's AT command mode
* @param	action: Either entering or exiting AT command mode
* @retval	Function status
============================================================================*/
static RdStatEnum rdBnrMainAtMode(rdBnrAtEnum action)
{
    RdStatEnum retval = RDSTAT_ERROR;   /*return status*/
    uint8_t tries;                      /*current number of try*/
	uint8_t at_resp[BUFF_ATRX_SZ];      /*buffer of AT response*/
	const char *CMD[] =                 /*dynamic AT command by action*/
    {           
        "###",                          /*Enter AT Command mode*/
        "ATCN\r",                       /*Close AT Command mode*/
        "ATRM\r"                        /*Restart AT Command mode*/
	};
    const char *CMD_STR[] = {"AT OPEN", "AT CLOSE", "Radio RESET"};
	const char *RESP[] =                /*dynamic AT response by action*/
    {           
        "ENTER CMD MODE\r",             /*Enter AT Command mode response*/
        "EXIT CMD MODE\r",              /*Close AT Command mode response*/
        "OK\r"                          /*Restart AT Command mode response*/
	};
    char printout[50];

    retval = RDSTAT_ERROR;
	rdClear(tries);
	rdClear(at_resp);
    
    for( tries=0; tries<=AT_MAXTRIES; tries++ )
    {
		uartTxBanner((uint8_t*)CMD[action], strlen(CMD[action]));
		if(rdBnrMainAtRx(at_resp, sizeof(at_resp)) == RDSTAT_OK) 
        {
            if (strstr((char*)at_resp, RESP[action]) != NULL)
            {
                retval = RDSTAT_OK;
                if(action == BNRRD_AT_RESET)
                {
                    delayMillis(1000); //FIXME: temporary delay until LED Mimic of Banner is available, 30 sec to assure module is already booted up
                }
                break;
            }
		}
	}
    
    if(retval == RDSTAT_OK) 
    {
        sprintf(printout, "rdBnrMainAtMode(%s) SUCCESS\n", CMD_STR[action]);
        uartSHOW((uint8_t*)printout, strlen(printout));
    }
    else
    {
        sprintf(printout, "rdBnrMainAtMode(%s) ERROR\n", CMD_STR[action]);
        uartSHOW((uint8_t*)printout, strlen(printout));
    }

	return retval;
}

static RdStatEnum rdBnrMainGetLifeStatus()
{
    if (rdBnrMainAtMode(BNRRD_AT_OPEN) == RDSTAT_OK)
    {
        rdBnrMainAtMode(BNRRD_AT_CLOSE);
        return RDSTAT_OK;
    }
    
    return RDSTAT_ERROR;
}

/*============================================================================
* @brief    Receiving full frame with data via Banner radio
* @param	arg_headptr:    header of receiving frame
            arg_dataptr:    pointer of receiving data 
* @retval	Function status
============================================================================*/
RdStatEnum rdBnrMainDataRx(RdHeadStruct arg_headptr, uint8_t *arg_dataptr) 
{
	RdStatEnum retval = RDSTAT_ERROR;			    /*return status*/
	uint8_t *frame_ptr;             /*buffer of data to receive*/
	uint8_t frame_sz;               /*bytesize of buffer of data to receive*/
	uint8_t *rx_dataptr;
	frame_ptr = NULL;
    ModbusCrcStruct rdiobuff_crc;
	uint8_t databuff_size = ((arg_headptr.type == TELEMETRY) 
                             ? sizeof(TelemetryStruct) 
                             : sizeof(SystemStatusStruct));
	frame_sz = arg_headptr.byt_sz +  RDFRAME_SZ;
	frame_ptr = (uint8_t*)memAlloc(frame_sz);
	assert(frame_ptr);
	memcpy(frame_ptr, &arg_headptr, RDHEAD_SZ);

	TelemetryStruct testthis[4];
	SystemStatusStruct testthis2[4];
	
	if(HAL_UART_Receive(&UART2HandlerDef
						, (uint8_t*)(frame_ptr + RDHEAD_SZ)
						, frame_sz - RDHEAD_SZ
						, DATARX_TIMEOUT) == HAL_OK)
	{
        rdiobuff_crc = modbusCrcCalc(frame_ptr, frame_sz - 2);
        if((rdiobuff_crc.crc_hi<<8 | rdiobuff_crc.crc_lo) 
			== (frame_ptr[frame_sz - 2]<<8 | frame_ptr[frame_sz - 1]))
        {
			if(arg_headptr.type == TELEMETRY)
			{
				memcpy((uint8_t*)testthis
						, frame_ptr + RDHEAD_SZ
						, TELEM_REC_SIZE_BYTES*arg_headptr.value1);
				for(uint8_t i = 0; i < arg_headptr.value1; i++)
				{
					rx_dataptr = (uint8_t*)loggerWrRecToRingBuff(NULL
							, 1
							, (RecordTypEnum)arg_headptr.type);	

					memcpy(rx_dataptr, testthis + i, databuff_size);
				}
			}
		  	else
			{
				memcpy((uint8_t*)testthis2
						, frame_ptr + RDHEAD_SZ
						, SYSSTAT_REC_SIZE_BYTES*arg_headptr.value1);
				for(uint8_t i = 0; i < arg_headptr.value1; i++)
				{
					rx_dataptr = (uint8_t*)loggerWrRecToRingBuff(NULL
							, 1
							, (RecordTypEnum)arg_headptr.type);	

					 memcpy(rx_dataptr, testthis2 + i, databuff_size);
				}
		  	}
			retval = RDSTAT_OK;
        }

	}
	else 
	{
		HAL_UART_DeInit(&UART2HandlerDef);
		HAL_UART_Init(&UART2HandlerDef);
		retval = RDSTAT_ERROR;
	}
    
	free(frame_ptr);
	return retval;
}

/*============================================================================
* @brief    Receiving header from radio
* @param	*arg_headptr:    pointer of header to be written from UART-Rx
* @retval	Function status
============================================================================*/
RdStatEnum rdBnrMainHeadRx(RdHeadStruct *arg_headptr) 
{
	RdStatEnum retval;                  /*return status*/

    retval = RDSTAT_NULL;
	rdClear(*arg_headptr);
	
	if(HAL_UART_Receive(&UART2HandlerDef
						, (uint8_t*)arg_headptr
						, RDHEAD_SZ
						, HEADERRX_TIMEOUT) == HAL_OK) 
	{
        #if DEVICE_TYPE == DEVICETYPE_GATE   
        rdBnrGateRxFrameNmmChecker(*arg_headptr);
        #endif
		retval = RDSTAT_OK;
	}
	else
	{
		HAL_UART_DeInit(&UART2HandlerDef);
		HAL_UART_Init(&UART2HandlerDef);
	}
    
    if(retval != RDSTAT_OK) 
	{
        retval = RDSTAT_ERROR;
    }

	return retval;
}

RdStatEnum rdBnrMainRxResp(RdHeadStruct *a_headptr, uint32_t a_timeout_ms) 
{
	RdStatEnum retval;                  /*return status*/

    retval = RDSTAT_NULL;
	rdClear(*a_headptr);
	
	if(HAL_UART_Receive(&UART2HandlerDef
						, (uint8_t*)a_headptr
						, RDHEAD_SZ
						, a_timeout_ms) == HAL_OK) 
	{
		retval = RDSTAT_OK;
	}
	else
	{
		HAL_UART_DeInit(&UART2HandlerDef);
		HAL_UART_Init(&UART2HandlerDef);
	}
    
    if(retval != RDSTAT_OK) 
	{
        retval = RDSTAT_ERROR;
    }

	return retval;
}

/*============================================================================
* @brief    Receiving header from radio
* @param	*arg_headptr:    pointer of header to be written from UART-Rx
* @retval	Function status
============================================================================*/
RdStatEnum rdBnrMainBlankFrRx(RdHeadStruct *arg_headptr, uint32_t timeout)
{
	RdStatEnum retval = RDSTAT_ERROR;                  /*return status*/
	RdHeadStruct rx_header_b;
    retval = RDSTAT_NULL;
	rdClear(*arg_headptr);
    ModbusCrcStruct rdiobuff_crc;
	ModbusCrcStruct rdcalc_crc;

	if(uartRxBanner((uint8_t*)&rx_header_b, RDHEAD_SZ, timeout) == HAL_OK) 
	{
		if(uartRxBanner((uint8_t*)&rdiobuff_crc, MB_CRC_SZ, 100) == HAL_OK)
		{
			rdcalc_crc = modbusCrcCalc((uint8_t*)&rx_header_b, 6);
			if( (rdiobuff_crc.crc_hi<<8 | rdiobuff_crc.crc_lo) 
				 == (rdcalc_crc.crc_hi<<8 | rdcalc_crc.crc_lo))
			{
				memcpy(arg_headptr, &rx_header_b, RDHEAD_SZ);
                #if DEVICE_TYPE == DEVICETYPE_GATE   
                rdBnrGateRxFrameNmmChecker(*arg_headptr);
                #endif
				retval = RDSTAT_OK;
			}
		
		}
		else
		{
			HAL_UART_DeInit(&UART2HandlerDef);
			HAL_UART_Init(&UART2HandlerDef);
		}
	}
	else
		{
			HAL_UART_DeInit(&UART2HandlerDef);
			HAL_UART_Init(&UART2HandlerDef);
		}
    
	return retval;
}

/*============================================================================
* @brief    Transmit full frame with data via Banner radio
* @param	arg_head:       header of transmitting frame
            arg_dataptr:    pointer of transmitting data
            arg_bytsz:      byte size of transmitting data    
* @retval	Function status
============================================================================*/
RdStatEnum rdBnrMainDataTx(RdHeadStruct arg_head
                           , uint8_t *arg_dataptr
                           , uint32_t arg_bytsz) 
{
    RdStatEnum retval;                  /*return status*/
	ModbusCrcStruct frame_crc;          /*crc of frame*/
	uint8_t *frame_ptr;                 /*pointer of frame to be transmitted*/
	uint8_t frame_sz;                   /*frame byte size*/

    retval = RDSTAT_NULL;
	frame_sz = RDFRAME_SZ + arg_bytsz;
	frame_ptr = memAlloc(frame_sz);
    
	memcpy(frame_ptr, &arg_head, RDFRAME_SZ);
	memcpy(frame_ptr + RDHEAD_SZ, arg_dataptr, arg_bytsz);
	frame_crc = modbusCrcCalc(frame_ptr, frame_sz - RDCRC_SZ);
	memcpy(frame_ptr + frame_sz - RDCRC_SZ, &frame_crc, RDCRC_SZ);

	if(modbusCrcChk(frame_ptr, frame_sz) != MODBUS_OK)
    {
		retval = RDSTAT_ERROR;
	}

    if(retval != RDSTAT_ERROR) 
    {  
        uartTxBanner(frame_ptr, frame_sz);
        delayMillis(BNRRADIO_DLY);
        retval = RDSTAT_OK;
    }

	free(frame_ptr);
	return retval;
}
#pragma optimize=none
/*============================================================================
* @brief    Transmit full frame without data via Banner radio
* @param	buff_head:  header of transmitting frame
* @retval	Function status
============================================================================*/
RdStatEnum rdBnrMainHeadTx(RdHeadStruct arg_head)
{
    RdStatEnum retval = RDSTAT_ERROR;   /*return status*/
	RdFrameStruct frame;                /*transmitting frame*/

    retval = RDSTAT_NULL;
	frame.header = arg_head;
	frame.crc = modbusCrcCalc((uint8_t*)&frame.header, RDHEAD_SZ);
    delayMillis(20);
	if(uartTxBanner((uint8_t*)&frame, RDFRAME_SZ) == HAL_OK) 
	{
        retval = RDSTAT_OK;
    }
    else {
        retval = RDSTAT_ERROR;
    }
    
    return retval;
}

void rdBnrMainReset(void)
{
    rdBnrMainAtMode(BNRRD_AT_RESET);
}

/*============================================================================
* @brief    Initializer of Banner radio
* @param	none
* @retval	Function status
============================================================================*/
RdStatEnum rdBnrMainInit(void) 
{
    const char AT_RESP[] = "OK\r";	/*expected AT response*/  
    RdStatEnum retval = RDSTAT_ERROR;	/*return status*/
    uint8_t tries;			/*current number of try*/
    uint8_t at_cnt;			/*count of implemented AT command*/
    uint8_t at_resp[BUFF_RX_SZ];	/*AT response*/
    uint8_t init_atcmdset_cnt;		/*number of AT cmd to initialize*/
    
    char at_bindcode[BINDCODE_SZ];	/*buffer of AT bind code*/
    char at_agdevsn[24];
    uint8_t at_devsn_sz = sizeof(at_agdevsn)-1;
    bool net_id_config_state = false;
    char printout[50];
    
    sprintf(printout, "\n***rdBnrMainInit(): Banner Set Mode Start***\n");
    uartSHOW((uint8_t*)printout, strlen(printout));
    
    //initialization via AT commands
    rdClear(at_agdevsn);
    snprintf(at_agdevsn, at_devsn_sz, "ATNA%s                "
             , DEVICE_INFO->device_sn);
    at_agdevsn[at_devsn_sz-1] = '\r';
    at_agdevsn[at_devsn_sz] = 0x00;
    //strcat(at_agdevsn, "\r");
    
    rdClear(at_bindcode);  
    #if DEVICE_TYPE == DEVICETYPE_GATE
        snprintf(at_bindcode, BINDCODE_SZ, "ATBM%06u", DEVICE_INFO->network_id);
    #else
        snprintf(at_bindcode, BINDCODE_SZ, "ATBC%06u", DEVICE_INFO->network_id);
        m_radio_signal_strength = 0;
    #endif /*Unit Type*/
    strcat(at_bindcode, "\r");
    
    const char *INIT_AT_CMD_SET[] =	        /*list of AT command*/
    {
        at_agdevsn      /*Agricapture Device SN*/
        ,"ATMR1\r"	    /*Modbus ID*/
        ,"ATDK0\r"	    /*DIP Switch Control*/
        ,"ATBD0\r"	    /*19200 Baud Rate*/
        ,"ATMT4A\r"	    /*Byte Timeout*/
        ,"ATRP0\r"     /*1 Watt power*/
        #if ENABLE_AUTONETID
            ,at_bindcode    /*Bind Code as Network ID*/
        #endif
        #if DEVICE_TYPE == DEVICETYPE_GATE
            ,"ATDI1\r"	    /*Radio Type Pin1*/
            ,"ATDJ0\r" 	    /*Ratio Type Pin2*/
            ,"ATACB\r"	    /*App Level Control*/
            ,"ATBM\r"       /*Read Bind Code*/
        #else
            ,"ATDI0\r"	    /*Radio Type Pin1*/
            ,"ATDJ1\r" 	    /*Ratio Type Pin2*/
            ,"ATAC9\r"	    /*App Level Control*/
            ,"ATBC\r"       /*Read Bind Code*/
        #endif
	};
    const char *INIT_AT_CMD_SET_STR[] =	        /*list of AT command*/
    {    
        "Agricapture Device SN"
        ,"Modbus ID"
        ,"Disabled DIP Switch Control"
        ,"19200 Baud Rate"
        ,"Byte Timeout 0x4A"
        ,"1.00 Watt Transmit Power"
        #if ENABLE_AUTONETID
            ,"Radio Network ID"
        #endif
        #if DEVICE_TYPE == DEVICETYPE_GATE
            ,"Radio Type Pin1 (Master)"
            ,"Ratio Type Pin2 (Master)"
            ,"App Level Control (Master)"
            ,"Radio Network ID"
        #else
            ,"Radio Type Pin1 (Slave)"
            ,"Ratio Type Pin2 (Slave)"
            ,"App Level Control (Slave)"
            ,"Radio Network ID"
        #endif
	};
    init_atcmdset_cnt = sizeof(INIT_AT_CMD_SET) / sizeof(*(INIT_AT_CMD_SET));

    gpioDigitalWrite(&BANNER_EN, HIGH);
    if(rdBnrMainAtMode(BNRRD_AT_OPEN) == RDSTAT_OK) 
    {
        at_cnt = 0;
        tries = 0;
        rdClear(at_resp);
        while((at_cnt < init_atcmdset_cnt) && (tries < AT_MAXTRIES))
        {
            tries++;
            if(at_cnt == 10) //10 = Get radio network ID
            {
                sprintf(printout, "Get %s ", INIT_AT_CMD_SET_STR[at_cnt]);
            }
            else
            {
                sprintf(printout, "Set %s [%s", INIT_AT_CMD_SET_STR[at_cnt]
                        , INIT_AT_CMD_SET[at_cnt]+4);
                printout[strlen(printout)-1] = 0x5D; //"]" in character
            }
            uartSHOW((uint8_t*)printout, strlen(printout));
            
            //send AT command to radio
            if(at_cnt == 0) //1 = Agricapture Device SN AT Command index
            {
                uartTxBanner((uint8_t*)INIT_AT_CMD_SET[at_cnt], at_devsn_sz);
            }
            else
            {
                uartTxBanner((uint8_t*)INIT_AT_CMD_SET[at_cnt], strlen(INIT_AT_CMD_SET[at_cnt]));
            }
            rdBnrMainAtRx(at_resp, sizeof(at_resp));
            if(memcmp(at_resp, AT_RESP, strlen(AT_RESP)) == 0 || (at_cnt == 10)) //10 = Get radio network ID
			{
                if(at_cnt == 6) //6 = net id AT command index
                {
                    net_id_config_state = true;
                }
                
                if(at_cnt == 10)  //10 = Get radio network ID
                {
                    DEVICE_INFO->network_id = atoi((char*)at_resp);
                    sprintf(printout, "[%d] SUCCESS\n", DEVICE_INFO->network_id);
                }
                else
                {
                    sprintf(printout, " SUCCESS\n");
                }
                uartSHOW((uint8_t*)printout, strlen(printout));
                at_cnt++;
                rdClear(tries);
                rdClear(at_resp);
            }
            else
            {
                sprintf(printout, " ERROR\n");
                uartSHOW((uint8_t*)printout, strlen(printout));
            }
        }
        if(tries == AT_MAXTRIES)
        {
            retval = RDSTAT_ERROR;
            retval = rdBnrMainAtMode(BNRRD_AT_CLOSE);
        }
        else 
		{
            delayMillis(1000); //FIXME: temporary delay until LED Mimic of Banner is available, 30 sec to assure module is already booted up
        }
    }   
    
    rdBnrMainAtMode(BNRRD_AT_RESET);
    
    if(net_id_config_state == false)
    {
        sprintf(printout, "FAILED to config Network ID to Banner Radio!\n");
		uartSHOW((uint8_t*)printout, strlen(printout));
        assert(0);
    }
	
    sprintf(printout, "***Banner Set Mode End***\n\n");
    uartSHOW((uint8_t*)printout, strlen(printout));
    
	return retval;
}

/*============================================================================
* @brief    Setup of Network ID (binding code)
* @param	arg_netid: network id to be set
* @retval	Function status
============================================================================*/
RdStatEnum rdBnrMainSetNetId(uint32_t arg_netid) 
{
	RdStatEnum retval;			        /*return status*/
	const char AT_RESP[] = "OK\r";		/*expected AT response*/
	char at_bindcode[BINDCODE_SZ];		/*buffer of AT bind code*/
	char at_resp[BUFF_RX_SZ];		/*buffer of AT response*/

	#if PRINT_PROCESSES_IN_UART
		char printout[50];
        sprintf(printout, "\n*** rdBnrMainSetNetId(%d) START ***\n", arg_netid);
        uartSHOW((uint8_t*)printout, strlen(printout));
    #endif
	
	retval = rdBnrMainAtMode(BNRRD_AT_OPEN);
	if(retval == RDSTAT_OK)
    {
		rdClear(at_bindcode);
        rdClear(at_resp);
        #if DEVICE_TYPE == DEVICETYPE_GATE
        snprintf(at_bindcode, BINDCODE_SZ, "ATBM%06u", arg_netid);
        #else
        snprintf(at_bindcode, BINDCODE_SZ, "ATBC%06u", arg_netid);
        #endif /*Unit Type*/
        strcat(at_bindcode, "\r");
        uartTxBanner((uint8_t*)at_bindcode, BINDCODE_SZ);
        rdBnrMainAtRx(at_resp, sizeof(at_resp));
        if(memcmp(at_resp, AT_RESP, strlen(AT_RESP)) != 0)
		{
            retval = RDSTAT_ERROR;
        }
        rdBnrMainAtMode(BNRRD_AT_CLOSE);
	}

	#if PRINT_PROCESSES_IN_UART
        {
            if (retval == RDSTAT_OK)
            {
                sprintf(printout, "SUCCESS!\n");
            }
            else
            {
                strcat(at_resp, "\r");
                sprintf(printout, "ERROR: Data received [%s]\n", at_resp);
            }
            uartSHOW((uint8_t*)printout, strlen(printout));
        }
        sprintf(printout, "*** rdBnrMainSetNetId() END ***\n");
        uartSHOW((uint8_t*)printout, strlen(printout));

        sprintf(printout, "***rdBnrMainNetSet End***\n");
		uartSHOW((uint8_t*)printout, strlen(printout));
   	#endif
   	
	return retval;
}

RdStatEnum rdBnrMainGetNetId(uint32_t *arg_netid_ptr) 
{
	RdStatEnum retval = RDSTAT_ERROR;		/*return status*/
	uint8_t at_resp[BUFF_RX_SZ];			/*buffer of AT response*/
    uint8_t tries;							/*number of try to read*/
    
    #if DEVICE_TYPE == DEVICETYPE_GATE
    const char AT_READ_NET_ID[] = "ATBM\r";
    #else
    const char AT_READ_NET_ID[] = "ATBC\r";
    #endif /*Unit Type*/

    #if PRINT_PROCESSES_IN_UART
		char printout[50];
		sprintf(printout, "\n\n\n*******rdBnrMainNetGet Start******\n");
	    uartSHOW((uint8_t*)printout, strlen(printout));
    #endif

    if(rdBnrMainAtMode(BNRRD_AT_OPEN) == RDSTAT_OK) 
    {
        for( tries=0; tries <= AT_MAXTRIES; tries++ )
        {
            rdClear(at_resp);
            
            uartTxBanner((uint8_t*)AT_READ_NET_ID, strlen(AT_READ_NET_ID));
            if(rdBnrMainAtRx(at_resp, sizeof(at_resp)) == RDSTAT_OK)
            {
                *arg_netid_ptr = atoi((char*)at_resp);
                retval = RDSTAT_OK;
                break;
            }
        }
    }
    rdBnrMainAtMode(BNRRD_AT_CLOSE);

    #if PRINT_PROCESSES_IN_UART
		sprintf(printout, "current network id [%d]\n", *arg_netid_ptr);
		uartSHOW((uint8_t*)printout, strlen(printout));
		if(retval == RDSTAT_OK)
		{
	        sprintf(printout, "SUCCESS!\n");
		}
		else
        {
            sprintf(printout, "ERROR!\n");
        }
        uartSHOW((uint8_t*)printout, strlen(printout));
		sprintf(printout, "*******rdBnrMainNetGet END******\n\n");
		uartSHOW((uint8_t*)printout, strlen(printout));
    #endif

	return retval;
}

uint8_t rdBnrMainGetLedStat(void)
{
    const uint32_t MAX_READ_TIME = 15000;
    const char *STAT_STR[] = {"No Radio","Green","Red","Amber",[0xFF]="ERROR"};
    
    uint8_t led;                    /*LED status code*/
    uint32_t t_start;               /*lap time reference*/
    char printout[50];
        
    led = 0;
    t_start = HAL_GetTick();
    while(delayTimeDiff(t_start, HAL_GetTick()) < MAX_READ_TIME)
    {
        if ( gpioDigitalRead(&BNR_ECHO0) == true )
        {
            led += 1;
        }
        
        if ( gpioDigitalRead(&BNR_ECHO1) == true )
        {
            led += 2;
        }
        
        if ( led != 0 )
        {
            break;
        }
    }
    
    if (led == 0)
    {
        sprintf(printout, "Check Radio Banner Life\n");
        uartSHOW((uint8_t*)printout, strlen(printout));
        if (rdBnrMainGetLifeStatus() == RDSTAT_OK)
        {
            sprintf(printout, "Radio Banner is Alive!\n");
            uartSHOW((uint8_t*)printout, strlen(printout));
            led = 0xFF;
        }
    }
    
    #if PRINT_PROCESSES_IN_UART
    sprintf(printout, "rdBnrMainGetLedStat: LED status[%s] code[%d]\n"
            , STAT_STR[led], led);
    uartSHOW((uint8_t*)printout, strlen(printout));
    #endif
    
    return led;
}

#pragma optimize=none
RdStatEnum rdBnrMainUpdateMaintenanceModeState(RdNetMaintenanceModeTypEnum a_code
                                            , uint8_t a_remaining_time)
{
    RdStatEnum retval = RDSTAT_ERROR;
    
    switch(a_code)
    {
    case RDNETMAINTENANCEMODETYP_STARTCMD:
        switch(sleep_attr.nmm_status)
        {
        case RADNETMAINTENANCEMODE_TOBEDISABLED:
        case RADNETMAINTENANCEMODE_DISABLE:
        case RADNETMAINTENANCEMODE_TOBEENABLED:
            sleep_attr.nmm_timer_min = (a_remaining_time == 0) 
                ? RDBNR_DEFAULT_NETMAINTENANCEMODE_TIME_MIN : a_remaining_time;
            sleep_attr.nmm_status = RADNETMAINTENANCEMODE_TOBEENABLED;
            retval = RDSTAT_OK;
            break;
        }
        break;
    case RDNETMAINTENANCEMODETYP_STOPCMD:
        sleep_attr.nmm_timer_min = 0;
        if(sleep_attr.nmm_status == RADNETMAINTENANCEMODE_TOBEENABLED
           || sleep_attr.nmm_status == RADNETMAINTENANCEMODE_ENABLE)
        {
            sleep_attr.nmm_status = RADNETMAINTENANCEMODE_TOBEDISABLED;
            retval = RDSTAT_OK;
        }
        break;
    case RDNETMAINTENANCEMODETYP_EDITCMD:
        if(sleep_attr.nmm_status == RADNETMAINTENANCEMODE_TOBEENABLED
           || sleep_attr.nmm_status == RADNETMAINTENANCEMODE_ENABLE)
        {
            sleep_attr.nmm_timer_min = a_remaining_time;
            retval = RDSTAT_OK;
        }
        break;
    default:
        break;
    }
    return retval;
}

#pragma optimize=none
void rdBnrMainSwitchMaintenanceMode(void)
{
    switch(sleep_attr.nmm_status)
    {
    case RADNETMAINTENANCEMODE_TOBEENABLED:
        if(sleep_attr.nmm_timer_min == 0)
            sleep_attr.nmm_status = RADNETMAINTENANCEMODE_DISABLE;
        else
            sleep_attr.nmm_status = RADNETMAINTENANCEMODE_ENABLE;
        break;
    case RADNETMAINTENANCEMODE_TOBEDISABLED:
        sleep_attr.nmm_timer_min = 0;
        sleep_attr.nmm_status = RADNETMAINTENANCEMODE_DISABLE;
        break;
    default:
        break;
    }
}

#pragma optimize=none
void rdBnrMainUpdateNmmTimer(uint32_t a_endtime)
{
    uint32_t t_current = HAL_GetTick();
    
    if(t_current < a_endtime)
        sleep_attr.nmm_timer_min = ((a_endtime - t_current)*60000)+1;
    else
        sleep_attr.nmm_timer_min = 0;
}

/******************************************************************************
  * Revision History
  *	@file      	rdBnrMain.c
  *****************************************************************************
  * @version	v2.2.0
  * @date		04/08/16
  * @author		J. Fajardo
  * @changes	initial release for V2.1 board
  *****************************************************************************
  * @version	v2.1
  * @date		01/12/2015
  * @author		J. Fajardo
  * @changes	initial release for V2.1 board
  *****************************************************************************
  */