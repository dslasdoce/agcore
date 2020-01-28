/********************************************************************************
  * @file      	utilIap.c
  * @author     Hardware Team
  * @version    
  * @date       
  * @brief		Handles requests for IAP related transactions.
  ******************************************************************************
  * @attention
  *
  * COPYRIGHT(c) 2017 AgTech Labs, Inc.
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
#include "stdlib.h"
#include "assert.h"
#include "delay.h"
#include "utilMem.h"
#include "extFlash.h"
#include "utilIap.h"

/* Defines ===================================================================*/

/* Functions =================================================================*/

//==============================================================================
// Function:    iapWriteParmData
// Description: This function prepares then writes the Parameter Data to the
//              first 4 bytes of Sector 0 of External Flash SPI
// Parameters:  Size - FW data size
// Return(s):   STAT_OK
//              STAT_FAILED
//==============================================================================
uint32_t iapWriteParmData (uint32_t Size)
{
    IAP_PARM_STRUCT *ParmBfrPtr;

    ParmBfrPtr = (IAP_PARM_STRUCT *)memAlloc(IAP_PARM_DATA_SZ);

    // Erase Sector 0
    if (extFlSectorErase(ONE_SECTOR,
                         EXT_FL_SEC_0_ADDR) != SUCCESSFUL)
    {
        assert(0);
    }

    // Prepare Parameter Data
    ParmBfrPtr->FwValid = ON;
    ParmBfrPtr->FwErr = OFF;
    ParmBfrPtr->FwUpgDone = OFF;
    ParmBfrPtr->Reserved = OFF;
    ParmBfrPtr->IapFlagEn = ON;
    ParmBfrPtr->DataSz = Size;

    // Write to External Flash SPI
    if (extFlWrite((int8_t *)&ParmBfrPtr,
                   EXT_FL_SEC_0_ADDR,
                   IAP_PARM_DATA_SZ) != SUCCESSFUL)
    {
        assert(0);
    }

    delayMillis(10);

    free(ParmBfrPtr);

    return STAT_OK;
}

//==============================================================================
// Function:    iapResetDevice
// Description: This function is called after receiving all the FW data from
//              the Gate device.
// Parameters:  None
// Return(s):   None
//==============================================================================
void iapResetDevice (void)
{
    const char *iaprst = "\n****** Execute System Reset ******\n";
	uartSHOW((uint8_t*)iaprst, strlen(iaprst));

	NVIC_SystemReset();
}
