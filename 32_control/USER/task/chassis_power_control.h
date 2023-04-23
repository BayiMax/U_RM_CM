/**
 * @file chassis_power_control.h
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief 电机功率控制
 * @version 0.1
 * @date 2023-02-19
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef __CHASSIS_POWER_CONTROL_H
#define __CHASSIS_POWER_CONTROL_H

#include "chassis_task.h"
#include "main.h"
/**
 * @brief          限制功率，主要限制电机电流
 * @param[in]      chassis_power_control: 底盘数据
 * @retval         none
 */
extern void chassis_power_control(chassis_move_t *chassis_power_control);

#endif
