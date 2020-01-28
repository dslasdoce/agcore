#include "agBoardInit.h"
#include "daqAdcSpi.h"
#include "delay.h"
#include <math.h>
#include <deployAssert.h>
#include "utilCalc.h"

SPI_HandleTypeDef adc_spi_hdle;

#define COMR_CMD_READ			(0x40)
#define COMR_CMD_WRITE			(0x00)
#define BITLOC_REG_ADD			(3)

#define ADC_22BITMASK			(0x003FFFFF)
#define	ADC_VREF				((float)2.5)
#define	ADC_RES					((float)4194304)
#define	ADC_LSBWEIGHT			(ADC_VREF/ADC_RES)
#define	ADC_MINVAL				((float)-5)
#define	ADC_MAXVAL				((float)5)
#define	ADC_DECPLACES			(4)

typedef struct
{
	uint8_t sr;
	uint8_t mr;
	uint8_t cr;
	uint8_t dr;
	uint8_t idr;
	uint8_t res;
	uint8_t or;
	uint8_t fsr;
}ADCRegisterStruct;

ADCRegisterStruct adcRegSel;

static void adcRegInit(void)
{
	adcRegSel.sr = 0;
	adcRegSel.mr = 1;
	adcRegSel.cr = 2;
	adcRegSel.dr = 3;
	adcRegSel.idr = 4;
	adcRegSel.res = 5;
	adcRegSel.or = 6;
	adcRegSel.fsr = 7;
}

ReturnStatusEnum adcSpiInit(void)
{
	//initialize flash SPI
  	adcRegInit();
	adc_spi_hdle.Instance = SPI4;
	adc_spi_hdle.Init.Mode = SPI_MODE_MASTER;
	adc_spi_hdle.Init.Direction = SPI_DIRECTION_2LINES;
	adc_spi_hdle.Init.DataSize = SPI_DATASIZE_8BIT;
	adc_spi_hdle.Init.CLKPolarity = SPI_POLARITY_LOW;
	adc_spi_hdle.Init.CLKPhase = SPI_PHASE_1EDGE;
	adc_spi_hdle.Init.NSS = SPI_NSS_HARD_OUTPUT;
	adc_spi_hdle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	adc_spi_hdle.Init.FirstBit = SPI_FIRSTBIT_MSB;
	adc_spi_hdle.Init.TIMode = SPI_TIMODE_DISABLE;
	adc_spi_hdle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	adc_spi_hdle.Init.CRCPolynomial = 7;

	if(HAL_SPI_Init(&adc_spi_hdle) != HAL_OK)
		assert(0);	
	
	__HAL_SPI_ENABLE(&adc_spi_hdle);

	HAL_Delay(500);
	return SUCCESSFUL;
}

static void adsSpiReset(void)
{
  	gpioDigitalWrite(&AD_CONV, LOW);
	
  	gpioDigitalWrite(&AD_EN, LOW);
	delayMillis(50);
	gpioDigitalWrite(&AD_EN, HIGH);
	delayMillis(50);
	
	gpioDigitalWrite(&AD_EN, LOW);
	delayMillis(50);
	gpioDigitalWrite(&AD_EN, HIGH);
	delayMillis(50);
}
ReturnStatusEnum adcSpiRead(float *adc_val, AgTechIOStruct *cs, uint8_t channel)
{
  	ReturnStatusEnum adc_stat = FAILED;
	uint8_t comr_cmd = COMR_CMD_READ | (adcRegSel.dr << BITLOC_REG_ADD);
	uint8_t dr_val[3];
	uint8_t dr_val0;
	uint8_t dr_val1;
	uint8_t dr_val2;
	uint16_t adc_dig = 0;
	*adc_val = -1;
	int16_t conv_result = 0;
	bool signbit = 0;

	adsSpiReset();
	gpioDigitalWrite(&AD_EN, LOW);
	delayMillis(500);
	
	gpioDigitalWrite(&AD_CONV, HIGH);
	delayMillis(100);
	gpioDigitalWrite(&AD_CONV, LOW);
	while(gpioDigitalRead(&AD_BUSY));
	
	gpioDigitalWrite(cs, LOW);
	delayMicros(100);

	
//	while(gpioDigitalRead(&SPI4_MISO));
	for(uint8_t i = 0; i < 4; i++)
	{
		if(HAL_SPI_Receive(&adc_spi_hdle, (uint8_t *)&dr_val0,
							1, 1000) != HAL_OK)
			 assert(0);
		if(HAL_SPI_Receive(&adc_spi_hdle, (uint8_t *)(&dr_val1),
						1, 100) != HAL_OK)
			 assert(0);
		if(HAL_SPI_Receive(&adc_spi_hdle, (uint8_t *)(&dr_val2),
						1, 100) != HAL_OK)
			 assert(0);
		
		conv_result = (dr_val0 << 8) | dr_val1;
		if( ((dr_val2>>3) & 0x03) ==  channel)
		{
			*adc_val = conv_result*(float)5/pow(2, 15);
			*adc_val = utilCalcRoundDec(*adc_val, ADC_DECPLACES);

			if( *adc_val > ADC_MAXVAL || *adc_val < ADC_MINVAL)
				assert(0);
			adc_stat = SUCCESSFUL;
		}
	}
	gpioDigitalWrite(cs, HIGH);
	gpioDigitalWrite(&AD_EN, HIGH);
	delayMillis(500);
	
	return adc_stat;
}
							  
ReturnStatusEnum adcSpiReadNone(float *adc_val, AgTechIOStruct *cs)
{
	uint8_t comr_cmd = COMR_CMD_READ | (adcRegSel.dr << BITLOC_REG_ADD);
	uint8_t dr_val[3];
	uint8_t dr_val0;
	uint8_t dr_val1;
	uint8_t dr_val2;
	uint32_t adc_dig = 0;
	*adc_val = -1;
	bool signbit = 0;

	delayMillis(100);	
	if(HAL_SPI_Receive(&adc_spi_hdle, (uint8_t *)&dr_val0,
						1, 1000) != HAL_OK)
		 assert(0);
	if(HAL_SPI_Receive(&adc_spi_hdle, (uint8_t *)(&dr_val1),
					1, 100) != HAL_OK)
		 assert(0);
	if(HAL_SPI_Receive(&adc_spi_hdle, (uint8_t *)(&dr_val2),
					1, 100) != HAL_OK)
		 assert(0);
	

	return SUCCESSFUL;
}









