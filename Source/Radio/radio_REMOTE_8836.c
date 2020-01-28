/**
  ****************************************************************************
  * @file		radio.c
  * @author		Hardware Team, Agtech Labs Inc.
  * @version	v2.2.0
  * @date		04/08/16
  * @brief		Function Module for Radio Interactions
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
#include <stdarg.h>

#include "productCfg.h"
#include "tempUnitList.h"
#include "radio.h"
#include "agboardInit.h"

#ifdef BANNER_RADIO
	#include "rdBnrMain.h"
	#if DEVICE_TYPE == DEVICETYPE_GATE
		#include "rdBnrGate.h"
	#else
		#include "rdBnrNode.h"
	#endif /*Unit Type*/
#endif /*BANNER_RADIO*/

/* External Declarations ====================================================*/
RdFrameStruct rxbuff_banner;

/* Functions ================================================================*/
/*============================================================================
* @brief    Initializer of Radio Module
* @param	none
* @retval	Function status
============================================================================*/
RdStatEnum radioInit(void) {
	RdStatEnum retval;				/*return status*/
	uint32_t default_net_id = 0;
    retval = RDSTAT_NULL;

	#ifdef BANNER_RADIO
    retval = rdBnrMainInit();
	
	rdBnrMainSetNetId(DEFAULT_NETID);
	
	rdBnrMainGetNetId(&default_net_id);
	if(default_net_id != DEFAULT_NETID)
	  	assert(0);
	#endif /* BANNER_RADIO */

//	#ifdef XBEE_RADIO
//  retval = xbRadioInit(); //under construction
//	#endif /* XBEE_RADIO */

	return retval;
}

/*============================================================================
* @brief    Setup of Radio's Network ID
* @param	arg_netid:  network id (banner: 0-0xFFFFFF, xbee: 0-0x7FFF)
* @retval	Function status
============================================================================*/
RdStatEnum radioNetSet(uint32_t arg_netid) {
	RdStatEnum retval;	            /*return status*/
	
    retval = RDSTAT_NULL;

	#ifdef BANNER_RADIO
    retval = rdBnrMainSetNetId(arg_netid);
	#endif /* BANNER_RADIO */

//	#ifdef XBEE_RADIO
//  retval = xbRadioNetSet(arg_netid); //under construction
//	#endif /* XBEE_RADIO */

	return retval;
}

#if DEVICE_TYPE == DEVICETYPE_GATE
/*============================================================================
* @brief    Gate's query to a specific Node
* @param	arg_id:     unit id
            arg_func:   type of query function (i.e. get, update, sleep, check)
            ...:        variable arguments/parameter
                >> va_arg1 = api type (for get & update only)
                >> va_arg2 = firmware size (for update-firmware only)   
* @retval	Function status
============================================================================*/
RdStatEnum radioQuery(uint8_t mb_id, RdFuncEnum arg_func,...) {
    RdStatEnum retval;              /*return status*/
	//UnitListStruct unit;            /*unit info from Node list*/
	va_list valist;                 /*variable arguments of this function*/

    retval = RDSTAT_NULL;
	//unit = getUnitInfo(arg_id);
//	if(unit.status != UNIT_STAT_ACTIVE) {
//		retval = RDSTAT_ERROR;
//	}

	char printout[30];
	snprintf(printout, 30, "\n*Radio Query %02d : 0x%02x\n", mb_id - 0x24, mb_id);
	uartSHOW((uint8_t*)printout, strlen(printout));
    
//    if(retval != RDSTAT_ERROR) 
//	{
        va_start(valist, arg_func);
//        switch(unit.rd_type) {
//            case RDTYPE_BANNER:
//            case RDTYPE_BOTH:
                retval = rdBnrGateQuery(mb_id, arg_func, valist);
 //               break;
//          case RDTYPE_XBEE: //under construction
//        		xbRdGateQuery(unit.unit_id, arg_func, apitype); 
//              break;
//            default:
//                retval = RDSTAT_ERROR;
//        }
        va_end(valist);
//    }
    	UART2HandlerDef.RxXferSize = 0;
		UART2HandlerDef.RxXferCount = 0;
    return retval;
}

/*============================================================================
* @brief    Gate's query to all active Pod within its network by polling
* @param	arg_func:   type of radio function (i.e. get, update, sleep, check)
            ...:         variable arguments/parameter
                >> va_arg1 = api type (for get & update only)
                >> va_arg2 = firmware size (for update-firmware only)   
* @retval	Function status
============================================================================*/
RdStatEnum radioPollQuery(RdFuncEnum arg_func,...) {
    RdStatEnum retval;              /*return status*/
    va_list valist;                 /*variable arguments of this function*/
	uint32_t idx;                   /*index number of a loop as unit id*/

    retval = RDSTAT_NULL;
	va_start(valist, arg_func);
	for(idx = 1; idx <= UNIT_CNT; idx++) {
		radioQuery(idx, arg_func, va_arg(valist,int));
	}
	va_end(valist);
    
    retval = RDSTAT_OK;
    return retval;
}

#else
/*============================================================================
* @brief    General function of Node's radio from a radio interrupt function
* @param	arg_rdtype: type/brand of radio (i.e. Banner, XBee)
            arg_head:   received radio frame header from interrupt function
* @retval	Function status
============================================================================*/
RdStatEnum radioResponse(RdTypeEnum arg_rdtyp, RdFrameStruct arg_frame) {
	RdStatEnum retval;	        /*return status*/
    RdHeadStruct arg_head = arg_frame.header;
	retval = RDSTAT_NULL;
	switch(arg_rdtyp) {
        case RDTYPE_BANNER:
            #if defined(BANNER_RADIO)
            retval = rdBnrNodeReceiver(arg_head);
            #else
            retval = RDSTAT_ERROR;
            #endif
            break;
//        case RDTYPE_XBEE:
//            #if defined(XBEE_RADIO) && defined(NODE)
//            retval = rdXbNodeReceiver(arg_head);
//            #else
//            retval = RDSTAT_ERROR;
//            #endif
//            retval = ????;
//            break;
        default:
            retval = RDSTAT_ERROR;
	}
    
	return retval;
}
#endif /*Unit Type*/

/******************************************************************************
  * Revision History
  *	@file      	radio.c
  *****************************************************************************
  * @version	Beta Version 2.2.0
  * @date		04/08/16
  * @author		J. Fajardo
  * @changes 	
  *****************************************************************************
  * @version	v2.1
  * @date		01/12/2015
  * @author		J. Fajardo
  * @changes 	initial release for V2.1 board
  *****************************************************************************
  */
