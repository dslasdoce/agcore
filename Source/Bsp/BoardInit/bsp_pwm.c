#include <deployAssert.h>

#include "productCfg.h"
#include "agboardInit.h"
#include "dmMain.h"
#include "stm32f779xx.h"
#include "delay.h"
#include "cloud.h"
#include "isrMain.h"
#include "port.h"

TIM_HandleTypeDef 	timer1Handle, timer2Handle, timer3Handle,
					timer4Handle, timer8Handle;
TIM_OC_InitTypeDef sConfig;


#define  PERIOD_VALUE       (uint32_t)(133 - 1)

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  //GPIO_InitTypeDef   GPIO_InitStruct;
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIMx Peripheral clock enable */
	switch((uint32_t)htim->Instance){
	case (uint32_t)TIM1:
  		__HAL_RCC_TIM1_CLK_ENABLE();
		break;
	case (uint32_t)TIM2:
		__HAL_RCC_TIM2_CLK_ENABLE();
		break;
	case (uint32_t)TIM3:
		__HAL_RCC_TIM3_CLK_ENABLE();
		break;
	case (uint32_t)TIM8:
		__HAL_RCC_TIM8_CLK_ENABLE();
		break;
	default:
		assert(0);
	}
}

void pwmInitTim(void)
{
	/* Set the pulse value for channel 1 */
	if (HAL_TIM_PWM_Init(&timer1Handle) != HAL_OK)
		assert(0);
	if (HAL_TIM_PWM_Init(&timer2Handle) != HAL_OK)
		assert(0);
	if (HAL_TIM_PWM_Init(&timer3Handle) != HAL_OK)
		assert(0);
	if (HAL_TIM_PWM_Init(&timer8Handle) != HAL_OK)
		assert(0);
}

void pwmDeInitTim(void)
{
	if (HAL_TIM_PWM_DeInit(&timer1Handle) != HAL_OK)
		assert(0);
	if (HAL_TIM_PWM_DeInit(&timer2Handle) != HAL_OK)
		assert(0);
	if (HAL_TIM_PWM_DeInit(&timer3Handle) != HAL_OK)
		assert(0);
	if (HAL_TIM_PWM_DeInit(&timer8Handle) != HAL_OK)
		assert(0);
}

void pwmParamSetUp(void)
{
	uint32_t uhPrescalerValue = (uint32_t)((SystemCoreClock/2) / 20000000) - 1;
	timer1Handle.Instance 				= TIM1;
	timer1Handle.Init.Prescaler         = uhPrescalerValue;
	timer1Handle.Init.Period            = PERIOD_VALUE;
	timer1Handle.Init.ClockDivision     = 0;
	timer1Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	timer1Handle.Init.RepetitionCounter = 0;
	
	timer2Handle.Instance 				= TIM2;
	timer2Handle.Init.Prescaler         = uhPrescalerValue;
	timer2Handle.Init.Period            = PERIOD_VALUE;
	timer2Handle.Init.ClockDivision     = 0;
	timer2Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	timer2Handle.Init.RepetitionCounter = 0;
	
	timer3Handle.Instance 				= TIM3;
	timer3Handle.Init.Prescaler         = uhPrescalerValue;
	timer3Handle.Init.Period            = PERIOD_VALUE;
	timer3Handle.Init.ClockDivision     = 0;
	timer3Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	timer3Handle.Init.RepetitionCounter = 0;
	
	timer8Handle.Instance 				= TIM8;
	timer8Handle.Init.Prescaler         = uhPrescalerValue;
	timer8Handle.Init.Period            = PERIOD_VALUE;
	timer8Handle.Init.ClockDivision     = 0;
	timer8Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	timer8Handle.Init.RepetitionCounter = 0;
	
	pwmInitTim();
}

void pwmStartPwr(uint8_t port, float duty_cycle)
{
	sConfig.OCMode       = TIM_OCMODE_PWM1;
	sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
	sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
	sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
	sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
	TIM_HandleTypeDef *pwm_tim_handle_ar[] = {NULL, &timer1Handle, &timer8Handle
											 ,&timer3Handle, &timer2Handle};
	AgTechIOStruct *vcon_en[] = {NULL, &VCON1_EN, &VCON2_EN, &VCON3_EN, &VCON4_EN};
	AgTechIOStruct *pwr_en[] = {NULL, &PWR1_A_EN, &PWR2_A_EN, &PWR3_A_EN, &PWR4_A_EN};

	gpioDigitalWrite(vcon_en[port], LOW);
	gpioDigitalWrite(pwr_en[port], LOW);
	
	//pwmInitTim();
		
	sConfig.Pulse = PERIOD_VALUE*duty_cycle;
	
	if (HAL_TIM_PWM_Stop(pwm_tim_handle_ar[port], TIM_CHANNEL_4) != HAL_OK)
		assert(0);
	
	if (HAL_TIM_PWM_ConfigChannel(pwm_tim_handle_ar[port], &sConfig, TIM_CHANNEL_4) != HAL_OK)
		assert(0);
	
	if (HAL_TIM_PWM_Start(pwm_tim_handle_ar[port ], TIM_CHANNEL_4) != HAL_OK)
		assert(0);

    delayMillis(100);
	gpioDigitalWrite(vcon_en[port], HIGH);
    delayMillis(1000);
	gpioDigitalWrite(pwr_en[port], HIGH);
	delayMillis(1000);
}

void pwmPwrOFF(uint8_t port)
{
	TIM_HandleTypeDef *pwm_tim_handle_ar[] = {NULL, &timer1Handle, &timer8Handle
											 ,&timer3Handle, &timer2Handle};
	AgTechIOStruct *vcon_en[] = {NULL, &VCON1_EN, &VCON2_EN, &VCON3_EN, &VCON4_EN};
	AgTechIOStruct *pwr_en[] = {NULL, &PWR1_A_EN, &PWR2_A_EN, &PWR3_A_EN, &PWR4_A_EN};
	
	gpioDigitalWrite(vcon_en[port], LOW);
	gpioDigitalWrite(pwr_en[port], LOW);
	
	if (HAL_TIM_PWM_Stop(pwm_tim_handle_ar[port], TIM_CHANNEL_4) != HAL_OK)
		assert(0);
	delayMillis(1000);
}

void pwmPwrOFFAllPorts(void)
{
	TIM_HandleTypeDef *pwm_tim_handle_ar[] = {NULL, &timer1Handle, &timer8Handle
											 ,&timer3Handle, &timer2Handle};
	AgTechIOStruct *vcon_en[] = {NULL, &VCON1_EN, &VCON2_EN, &VCON3_EN, &VCON4_EN};
	AgTechIOStruct *pwr_en[] = {NULL, &PWR1_A_EN, &PWR2_A_EN, &PWR3_A_EN, &PWR4_A_EN};
	for(uint8_t port = 1; port <= MAX_SENSORPORT; port++)
    {
		gpioDigitalWrite(vcon_en[port], LOW);
		gpioDigitalWrite(pwr_en[port], LOW);

		if (HAL_TIM_PWM_Stop(pwm_tim_handle_ar[port], TIM_CHANNEL_4) != HAL_OK)
			assert(0);
	}
	delayMillis(100);
}
// sets the Vconx_EN enable line high for the LTC3111 which starts the buck converter.
// sets the PWRx_A_EN line which high which connects the buck converter to the load.
// the MSP runs the PWM for the LTC31111

void pwmPwrMSP_ON(uint8_t port)
{

	AgTechIOStruct *vcon_en[] = {NULL, &VCON1_EN, &VCON2_EN, &VCON3_EN, &VCON4_EN};
	AgTechIOStruct *pwr_en[] = {NULL, &PWR1_A_EN, &PWR2_A_EN, &PWR3_A_EN, &PWR4_A_EN};
        
        // I had to add a delay between Vconn2_EN and PWR2_EN in order to 
        // get 15 volts to work.  I think there is a hardware issue at 15 volts.  5/17/2017
	gpioDigitalWrite(vcon_en[port], HIGH);
        delayMillis(1000);
	gpioDigitalWrite(pwr_en[port], HIGH);

}
// sets the Vconx_EN enable line low for the LTC3111 which stops the buck converter.
// sets the PWRx_A_EN line which low which disconnects the buck converter to the load.
// the MSP runs the PWM for the LTC31111

void pwmPwrMSP_OFF(uint8_t port)
{
	AgTechIOStruct *vcon_en[] = {NULL, &VCON1_EN, &VCON2_EN, &VCON3_EN, &VCON4_EN};
	AgTechIOStruct *pwr_en[] = {NULL, &PWR1_A_EN, &PWR2_A_EN, &PWR3_A_EN, &PWR4_A_EN};
	
	gpioDigitalWrite(vcon_en[port], LOW);
	gpioDigitalWrite(pwr_en[port], LOW);
      
 
	delayMillis(1000);

}