#ifndef __DELAY_H
#define __DELAY_H

#include "user_c.h"

void delay_init(void);
void delay_us(uint16_t nus);
void delay_ms(uint16_t nms);

#endif /*__DELAY_H*/
