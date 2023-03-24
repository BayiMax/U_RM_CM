#ifndef __USER_ADC_H
#define __USER_ADC_H

#include "user_c.h"

extern void init_vrefint_reciprocal(void);
extern fp32 get_temprate(void);
extern fp32 get_battery_voltage(void);
extern uint8_t get_hardware_version(void);

#endif /*__USER_ADC_H*/
