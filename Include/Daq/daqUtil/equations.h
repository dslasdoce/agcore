/********************************************************************************
  * @file      	equations.h
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

#include <agboardInit.h>
#ifndef __EQUATIONS_H_
#define __EQUATIONS_H_


#define MAX_CMN_DPSIZE 		((uint8_t)6)
#define MAX_SINGLE_READ		((uint8_t)20)

/* Typedefs ==================================================================*/
typedef struct
{
	float raw_value;
	float base_value;
	float supplementary_values[MAX_CMN_DPSIZE];
}EquationParameterStruct;

typedef EquationStatusEnum (*equationHandler)(EquationParameterStruct*);

typedef struct
{
	const char *code;
	equationHandler equation_ptr;	/*ptr to equation to be used*/
}EquationCodeStruct;

typedef struct
{
	equationHandler equation_ptr;
	float supplementary_values[MAX_CMN_DPSIZE];
}EquationActiveStruct;

typedef struct
{
	uint8_t exp_datacount;
	uint8_t raw_uoms[MAX_CMN_DPSIZE];
	uint8_t base_uoms[MAX_CMN_DPSIZE];
	uint8_t depth_values[MAX_SINGLE_READ];
	uint8_t datasequence[MAX_SINGLE_READ];
	uint8_t sensor_type[MAX_CMN_DPSIZE];
	EquationActiveStruct ActiveEquations[MAX_CMN_DPSIZE];
}DataProcessingStruct;

typedef struct
{
	DataProcessingStruct P1;
    DataProcessingStruct P2;
    DataProcessingStruct P3;
    DataProcessingStruct P4;
}PortDataProcStruct;

/* Function Prototypes =======================================================*/
EquationStatusEnum eqLinearScaling(EquationParameterStruct *params_ptr);
EquationStatusEnum eqRatiometric(EquationParameterStruct *params_ptr);
EquationStatusEnum eqTopps(EquationParameterStruct *params_ptr);
EquationStatusEnum eqDcgnEC5(EquationParameterStruct *params_ptr);
EquationStatusEnum eqDcgnMAS1(EquationParameterStruct *params_ptr);
EquationStatusEnum eqDcgn10HS(EquationParameterStruct *params_ptr);
EquationStatusEnum eqCtoF(EquationParameterStruct *params_ptr);
EquationStatusEnum eqThermistor(EquationParameterStruct *params_ptr);
uint8_t eqStringSplit(char **buffer, char *string, const char * delim);
void eqCodeParser(char *conversion_str, EquationActiveStruct *eq_all);
void eqStringSplitFree(char **buffer, uint8_t size);
EquationStatusEnum eqMapValue(float *output_value, float input_value,
							  float in_min, float in_max, 
							  float out_min, float out_max);


#endif


/*******************************************************************************
  * Revision History
  *	@file      	equations.h
  ******************************************************************************
  * @version	v2.2.0
  * @date		04/08/16
  * @author		D. Lasdoce
  * @changes	- created file
  ******************************************************************************
  */
