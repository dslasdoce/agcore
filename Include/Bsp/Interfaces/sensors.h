/*******************************************************************************
  * @file      	sensor.h
  * @author     Hardware Team
  * @version    v2.2.0
  * @date       04/08/16
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

#ifndef SENSORS_H_
#define SENSORS_H_

/* Defines ==================================================================*/
#define RAW_UOM_V		((const char*)"01")
#define RAW_UOM_OHM		((const char*)"13")
#define RAW_UOM_A		((const char*)"02")

#define UOM_V			((uint32_t)0x01)
#define UOM_MA			((uint32_t)0x02)
#define UOM_PSI			((uint32_t)0x03)
#define UOM_KPA			((uint32_t)0x04)
#define UOM_VWC			((uint32_t)0x05)
#define UOM_C			((uint32_t)0x06)
#define UOM_F			((uint32_t)0x07)
#define UOM_E			((uint32_t)0x08)
#define UOM_PULSE		((uint32_t)0x09)
#define UOM_PERCENT		((uint32_t)0x0A)
#define UOM_MPH			((uint32_t)0x0B)
#define UOM_MM			((uint32_t)0x0C)
#define UOM_CFM			((uint32_t)0x0D)
#define UOM_W_M2		((uint32_t)0x0E)
#define UOM_IN			((uint32_t)0x0F)
#define UOM_FT			((uint32_t)0x10)
#define UOM_M			((uint32_t)0x11)
#define UOM_CM			((uint32_t)0x12)
#define UOM_OHM			((uint32_t)0x13)
#define UOM_DEG			((uint32_t)0x14)
#define UOM_IN_HR		((uint32_t)0x15)
#define UOM_MEDS		((uint32_t)0x16)
#define UOM_EC			((uint32_t)0x17)

#define ST_SM			((uint32_t)0x01)
#define ST_WP			((uint32_t)0x02)
#define ST_WF			((uint32_t)0x03)
#define ST_TP			((uint32_t)0x04)
#define ST_RH			((uint32_t)0x05)
#define ST_WD			((uint32_t)0x06)
#define ST_WS			((uint32_t)0x07)
#define ST_RG			((uint32_t)0x08)
#define ST_LW			((uint32_t)0x09)
#define ST_SF			((uint32_t)0x0A)
#define ST_SR			((uint32_t)0x0B)
#define ST_LT			((uint32_t)0x0C)
#define ST_WM			((uint32_t)0x0D)
#define ST_IM			((uint32_t)0x0E)
#define ST_ST			((uint32_t)0x0F)
#define ST_AT			((uint32_t)0x10)
#define ST_UV			((uint32_t)0x11)
#define ST_EC			((uint32_t)0x12)

#define SC_WS_WD		((uint16_t)0x001F)
#define SC_UV			((uint16_t)0x37)
#define SC_RG			((uint16_t)0x0021)
#define SC_SR			((uint16_t)0x0020)
#define SC_AT			((uint16_t)0x002D)
#define SC_RH			((uint16_t)0x002D)


/*===========================================================================*/
#define DVIS_WEATHER 	        ((uint16_t)0x00)
#define DVIS_SR 		        ((uint16_t)0x02)
#define DCGN_5TE	 	        ((uint16_t)0x0B)
#define GPLP2	 	            ((uint16_t)0x19)
#define AQCHK_24 	            ((uint16_t)0x1A)
#define DVIS_WD_WS 		        ((uint16_t)0x1F)
#define DVIS_RAIN 	            ((uint16_t)0x20)
#define AQCHK_32 	            ((uint16_t)0x24)
#define AQCHK_60                ((uint16_t)0x27)
#define DVIS_TMP 	            ((uint16_t)0x2A)
#define CDWST_TMP_HMD 	        ((uint16_t)0x31)
#define HSTI_40 	            ((uint16_t)0x33)
#define AQCHK_16 	            ((uint16_t)0x39)
#define HSTI_24 	            ((uint16_t)0x3A)
#define WP_1000 		        ((uint16_t)0x34)
#define TST_AN_0_3 		        ((uint16_t)0x90)
#define TST_AN_0_6 		        ((uint16_t)0x91)
#define TST_ANC_0_3 		    ((uint16_t)0x92)

#endif

/*********************************************************************************
  * Revision History
  * @file        sensors.h 
  ********************************************************************************
  * @version    v2.2.0
  * @date       04/08/16
  * @author     D.Lasdoce
  * @changes
  ********************************************************************************
  * @version    v2.1.0
  * @date       12/23/2015
  * @author     D.Lasdoce
  * @changes     - created file
  ********************************************************************************
  */
