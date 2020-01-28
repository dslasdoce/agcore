/********************************************************************************
  * @file      	equations.c
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
#include <math.h>
#include <equations.h>
#include <agboardInit.h>
#include <string.h>
#include <stdlib.h>

#include "port.h"

EquationCodeStruct eq_table[] = {
				{"AA", &eqLinearScaling},
				{"AB", &eqRatiometric},
				{"AC", &eqTopps},
				{"AD", &eqDcgnEC5},
				{"AE", &eqDcgnMAS1},
				{"AF", &eqDcgn10HS},
				{"AG", &eqCtoF},
				{"AH", &eqThermistor},
				{NULL,NULL}
};

static char *strsep(stringp, delim)
	register char **stringp;
	register const char *delim;
{
	register char *s;
	register const char *spanp;
	register int c, sc;
	char *tok;

	if ((s = *stringp) == NULL)
		return (NULL);
	for (tok = s;;) {
		c = *s++;
		spanp = delim;
		do {
			if ((sc = *spanp++) == c) {
				if (c == 0)
					s = NULL;
				else
					s[-1] = 0;
				*stringp = s;
				return (tok);
			}
		} while (sc != 0);
	}
	/* NOTREACHED */
}

/*
 * @brief	split string by the occurrences of pattern
 * @param	buffer:	pointer to where the set of strings after split will be saved
 * @param	delim:	pattern that will indicate where to split
 * @retval	number of strings placed in @buffer
 */
uint8_t eqStringSplit(char **buffer, char *string, const char * delim)
{
	char *token;
	char *stringp = strdup(string);
    char *stringp_firstadd = stringp;

	uint8_t i = 0;
	//buffer = malloc(sizeof(char*) * strlen(stringp));
	while(stringp !=NULL )
	{
		token = strsep(&stringp, delim);
		buffer[i++] = strdup(token);
	}
	free(stringp_firstadd);
	return i;
}

void eqStringSplitFree(char **buffer, uint8_t size)
{
	uint8_t idx;
	for(idx=0; idx<size; idx++)
	{
            if(buffer[idx])
		free(buffer[idx]);
	}
}

/*
 * @brief	search for function representing the eq_code
 * @param	eq_code: string of equation code to be searched
 * @param	eq_active_ptr: ptr to EquationActiveStruct that will store the equation functio ptr
 */
static void eqSearch(char *eq_code, EquationActiveStruct *eq_active_ptr)
{
	char *eq_details[10];
	char *eq_code_tmp = strdup(eq_code);

	/*split equation code and equation parameters*/
	uint8_t count_details = eqStringSplit(eq_details,eq_code_tmp,"-");
	EquationCodeStruct *code_ptr = NULL;
	int i;
	for(i=0; i<count_details; i++)
	{
		/*string is equaion code(first string after split)*/
		if(i == 0)
		{
			for(code_ptr = eq_table; code_ptr->code !=NULL; ++code_ptr )
			{
				/*code matches the one in the table*/
				if(strcmp(code_ptr->code, *eq_details) == 0)
				{
					eq_active_ptr->equation_ptr = *(code_ptr->equation_ptr);
					break;
				}
			}
		}
		/*string is equation parameter*/
		else
		{
			eq_active_ptr->supplementary_values[i-1] = strtof(*(eq_details+i), NULL);
		}
	}
	eqStringSplitFree(eq_details, count_details);
	free(eq_code_tmp);
}

/*
 * @brief	convert a string of equation codes and its params to function pointers that represent each equation
 * @param	conversion_str: string of equation codes and its params
 * @param	eq_all: ptr to array of EquationActiveStruct
 */
void eqCodeParser(char *conversion_str, EquationActiveStruct *eq_all)
{

	char *conv_str_tmp = strdup(conversion_str);
	char *eq_codes[20];
	/*split all equations contained in conversion_str */
	volatile uint8_t eq_count = eqStringSplit(eq_codes,conv_str_tmp,",");
	uint8_t i;
	/*search for eqch equation code and save them to respective EquationActiveStruct */
	for(i=0; i<eq_count; ++i)
	{
		eqSearch(eq_codes[i], &eq_all[i]);
	}
	eqStringSplitFree(eq_codes, eq_count);
	free(conv_str_tmp);
}

/*
 * @brief
 */
EquationStatusEnum eqLinearScaling(EquationParameterStruct *params_ptr)
{
	float input_value = params_ptr->raw_value;
	float in_min = params_ptr->supplementary_values[0];
	float in_max = params_ptr->supplementary_values[1];
	float out_min = params_ptr->supplementary_values[2];
	float out_max = params_ptr->supplementary_values[3];
    float ex_range = in_max - in_min;
    float temp_value = 0;

    if (ex_range == 0)
        return EQUATION_ZERODIV;

    if(input_value < in_min)
    {
    	temp_value = out_min;
    }

    else if(input_value > in_max)
    {
    	temp_value = out_max;
    }
    else
    {
    	temp_value = (input_value-in_min)*(out_max-out_min)/(in_max-in_min) + out_min;
    }
    params_ptr->base_value = temp_value;
    return EQUATION_OK;
}

/*
 * @brief
 */
EquationStatusEnum eqRatiometric(EquationParameterStruct *params_ptr)
{
	params_ptr->base_value = (params_ptr->raw_value)*params_ptr->supplementary_values[0];
	return EQUATION_OK;
}

/*
 * @brief	convert celcius to fahrenheit
 */
EquationStatusEnum eqCtoF(EquationParameterStruct *params_ptr)
{
	params_ptr->base_value = (params_ptr->raw_value)*9/5 + 32;
	return EQUATION_OK;
}

/*
 * @brief	topp's equation
 */
EquationStatusEnum eqTopps(EquationParameterStruct *params_ptr)
{
	float temp_base = 0;
	double e = params_ptr->raw_value;
	temp_base = (float)((4.3E-6)*pow(e,3) - (5.5E-4)*pow(e,2) +
							 (2.92E-2)*e - 5.3E-2)*100;
	if(temp_base < 0)
		temp_base = 0;
	if(temp_base > 100)
		temp_base = 100;
	params_ptr->base_value = temp_base;
	return EQUATION_OK;
}

/*
 * @brief 	decagon mV to VWC conversion
 */
EquationStatusEnum eqDcgnEC5(EquationParameterStruct *params_ptr)
{
	EquationParameterStruct local_param;
	float mV = params_ptr->raw_value;
	float e;
	e = 1/((-3.3326E-9)*pow(mV,3) + (7.0218E-6)*pow(mV,2) - (5.11647E-3)*mV + 1.30746);
	local_param.raw_value = e;
	eqTopps(&local_param);
	params_ptr->base_value = local_param.base_value;
	return EQUATION_OK;
}

EquationStatusEnum eqDcgnMAS1(EquationParameterStruct *params_ptr)
{
	float mA = params_ptr->raw_value;
	params_ptr->base_value = 0.00328*pow(mA,2) - 0.0244*mA- 0.00565;
	return EQUATION_OK;
}

/*
 * @brief
 */
EquationStatusEnum eqDcgn10HS(EquationParameterStruct *params_ptr)
{
	EquationParameterStruct local_param;
	float mV = params_ptr->raw_value;
	float e;
	e = (2.589E-10)*pow(mV, 4) - (5.010E-7)*pow(mV, 3) +
		(3.523E-4)*pow(mV, 2) - (9.135E-2)*mV + 7.457;
	local_param.raw_value = e;
	eqTopps(&local_param);
	params_ptr->base_value = local_param.base_value;
	return EQUATION_OK;
}

EquationStatusEnum eqThermistor(EquationParameterStruct *params_ptr)
{
	const float r_fixed = 200000;
	const float ref_voltage = adc_therm_ref;
	float temp_c = 0;
	float raw_voltage = params_ptr->raw_value;
	float a = 8.54942e-4;
	float b = 2.57305e-4;
	float c = 1.65368e-7;
	//volatile float r_parallel = 5000;
	volatile float r_thermistor = r_fixed*raw_voltage/(ref_voltage - raw_voltage);
	//volatile float r_thermistor = 10000*r_parallel/(10000 - r_parallel);
	temp_c = 1/(a + b*log(r_thermistor) + c*pow(log(r_thermistor), 3))
			- 273.15;
	if(temp_c < -40 || temp_c > 200)
	{
		params_ptr->raw_value = 0;
		params_ptr->base_value = 0;
	}
	else
	{
		params_ptr->base_value = temp_c*9/5 + 32;
		params_ptr->raw_value = r_thermistor;
	}
	return EQUATION_OK;
}


/* ============================================================================
 * @brief	linear mapping of one set of value to specified new limits
 * @param	output_value: ptr to array that will contain new value
 * @param	input_value: actual value to be mapped
 * @param	in_min: minimum value of
 * ==========================================================================*/
EquationStatusEnum eqMapValue(float *output_value, float input_value,
	float in_min, float in_max, float out_min, float out_max)
{
    float ex_range = in_max - in_min;
    if (ex_range == 0)
        return EQUATION_ERROR;
    else
    	*output_value = (input_value-in_min)*
    					(out_max-out_min)/(in_max-in_min) + out_min;
        return EQUATION_OK;
}

/*******************************************************************************
  * Revision History
  *	@file      	equations.c
  ******************************************************************************
  * @version	v2.2.0
  * @date		04/08/16
  * @author		D. Lasdoce
  * @changes	- created file
  ******************************************************************************
  */