/**
 * @file chassis_behaviour.h
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief 底盘控制
 * @version 0.1
 * @date 2023-02-20
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H

#include "struct_typedef.h"
#include "chassis_task.h"
#include "user_c.h"

typedef enum
{
  CHASSIS_ZERO_FORCE, // 底盘无力, 跟没上电那样

  CHASSIS_NO_MOVE, // 底盘保持不动

  CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW, // 正常步兵底盘跟随云台

  CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, // 工程底盘角度控制底盘，由于底盘未有陀螺仪，故而角度是减去云台角度而得到，
                                       // 如果有底盘陀螺仪请更新底盘的yaw，pitch，roll角度 在chassis_feedback_update函数中

  CHASSIS_NO_FOLLOW_YAW, // 底盘不跟随角度，角度是开环的，但轮子是有速度环

  CHASSIS_OPEN, //  遥控器的值乘以比例成电流值 直接发送到can总线上

  CHASSIS_SPIN,

} chassis_behaviour_e;

#define CHASSIS_OPEN_RC_SCALE 10 // 在chassis_open 模型下，遥控器乘以该比例发送到can上

/**
 * @brief          通过逻辑判断，赋值"chassis_behaviour_mode"成哪种模式
 * @param[in]      chassis_move_mode: 底盘数据
 * @retval         none
 */
extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);

/**
 * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
 * @param[out]     vx_set, 通常控制纵向移动.
 * @param[out]     vy_set, 通常控制横向移动.
 * @param[out]     wz_set, 通常控制旋转运动.
 * @param[in]      chassis_move_rc_to_vector,  包括底盘所有信息.
 * @retval         none
 */

extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
