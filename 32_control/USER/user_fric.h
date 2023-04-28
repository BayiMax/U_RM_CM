#ifndef USER_FRIC_H
#define USER_FRIC_H

#include "user_c.h"

#define FRIC_MAX 1320

#define FRIC_UP 1315
#define FRIC_DOWN 1300
#define FRIC_OFF 1000

extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);

#endif /*USER_FRIC_H*/
