#ifndef BSP_PWM_H_
#define BSP_PWM_H_
	//void pwmInitTim();
	void pwmParamSetUp(void);
	void pwmStartPwr(uint8_t port, float duty_cycle);
	void pwmPwrOFF(uint8_t port);
	void pwmInitTim(void);
	void pwmDeInitTim(void);
	void pwmPwrMSP_ON(uint8_t port);
    void pwmPwrMSP_OFF(uint8_t port);
	void pwmPwrOFFAllPorts(void);
	extern TIM_HandleTypeDef 	timer1Handle, timer2Handle, timer3Handle,
					timer4Handle, timer8Handle;
#endif