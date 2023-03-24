#ifndef __POWER_TASK_H
#define __POWER_TASK_H

#include "user_c.h"
/**
 * @brief          电源采样和计算电源百分比
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
extern void battery_voltage_task(void const *argument);
/**
 * @brief          获取电量
 * @param[in]      void
 * @retval         电量, 单位 1, 1 = 1%
 */
extern uint16_t get_battery_percentage(void);
#endif /*__POWER_TASK_H*/
