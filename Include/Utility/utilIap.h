//==============================================================================
// COPYRIGHT(c) 2015 AgTech Labs, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//  3. Neither the name of AgTech Labs, Inc. nor the names of its contributors
//     may be used to endorse or promote products derived from this software
//     without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Source: 
// Revision: 
// Author: 
// Date: 
//==============================================================================

#if !defined(_UTILIAP_H__)
#define _UTILIAP_H__

#include <stdint.h>

//==============================================================================
// Local Constant Macros
//==============================================================================
#define IAP_PARM_DATA_SZ        0x4

#define ONE_SECTOR              0x1

#define ON                      0x1
#define OFF                     0x0

#define STAT_FAILED             0x0
#define STAT_OK                 0x1

//==============================================================================
// Local Type Definitions
//==============================================================================

//==============================================================================
// Local Structure Definitions
//==============================================================================
typedef struct IapParmStruct
{
    uint8_t FwValid             :1;        // Firmware Valid         [bit 0]
    uint8_t FwErr               :1;        // Firmware Error         [bit 1]
    uint8_t FwUpgDone           :1;        // Firmware Upgrade Done  [bit 2]
    uint8_t Reserved            :4;        // Reserved               [bits 3~6]
    uint8_t IapFlagEn           :1;        // IAP Flag Enable        [bit 7]
    uint32_t DataSz             :24;       // Data Size              [bytes 1~3]
} IAP_PARM_STRUCT;

//==============================================================================
// Function Macros
//==============================================================================

//==============================================================================
// Function Prototypes
//==============================================================================
uint32_t iapWriteParmData (uint32_t Size);
void iapResetDevice (void);

#endif