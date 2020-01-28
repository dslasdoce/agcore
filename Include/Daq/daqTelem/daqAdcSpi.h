#ifndef __DAQADCSPI_H_
#define __DAQADCSPI_H_

#include <stdint.h>
#include "productCfg.h"
#include "gpio.h"

ReturnStatusEnum adcSpiInit(void);
ReturnStatusEnum adcSpiRead(float *adc_val, AgTechIOStruct *cs, uint8_t channel);
ReturnStatusEnum adcSpiReadNone(float *adc_val, AgTechIOStruct *cs);
#endif