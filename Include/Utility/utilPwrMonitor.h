#ifndef _UTIL_PWRMONITOR_H_
#define _UTIL_PWRMONITOR_H_
	void utilPwrModeCont(AgTechIOStruct *cs, bool mode);
	float utilPwrGetVoltage(AgTechIOStruct *cs);
	float utilPwrGetTemp(AgTechIOStruct *cs);
#endif