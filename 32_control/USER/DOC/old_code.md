/**
 * @file old_code.c
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief user_task 可控代码
 * @version 0.2
 * @date 2023-02-22
 * @copyright Copyright (c) 2023
 */

#include "stdio.h"

/*********************************2023年2月22日19:50:18****************************************/
/**
 * @file user_task.c
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief 测试任务(乱七八糟)
 * @version 0.1
 * @date 2023-02-19
 *
 * @copyright Copyright (c) 2023
 *
 */
 
//#include "user_task.h"
//#include "user_c.h"

// 死区限制宏定义
#define rc_deadband_limit(input, output, dealine)    \
  {                                                  \
    if ((input) > (dealine) || (input) < -(dealine)) \
    {                                                \
      (output) = (input);                            \
    }                                                \
    else                                             \
    {                                                \
      (output) = 0;                                  \
    }                                                \
  }

// 底盘运动数据 结构体
chassis_move_t BBX_chassis_move;

/**
 * @brief 更新底盘数据
 *
 * @param chassis_move_update
 */
void user_chassis_feedback_update(chassis_move_t *chassis_move_update)
{
  if (chassis_move_update == NULL)
  {
    return;
  }

  uint8_t i = 0;
  for (i = 0; i < 4; i++)
  {
    // 更新电机速度，加速度是速度的PID微分
    chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
    chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
  }

  // 更新底盘纵向速度 x， 平移速度y，旋转速度wz，坐标系为右手系
  chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
  chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
  chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

  // 计算底盘姿态角度 ?底盘姿态?
  chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - chassis_move_update->chassis_yaw_motor->relative_angle);
  chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
  chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
}
/**
 * @brief          根据遥控器通道值，计算纵向和横移速度
 *
 * @param[out]     vx_set: x轴速度
 * @param[out]     vy_set: y轴速度
 * @param[out]     chassis_move_rc_to_vector:底盘结构体
 * @retval         none
 */
void user_chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
  {
    return;
  }

  int16_t vx_channel, vy_channel;
  fp32 vx_set_channel, vy_set_channel;

  // 死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
  rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
  rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

  vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
  vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

  // 键盘控制
  if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
  {
    vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
  }
  else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
  {
    vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
  }

  if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
  {
    vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
  }
  else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
  {
    vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
  }

  // 一阶低通滤波 代替 斜波作为底盘速度输入
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);

  // 停止信号，不需要缓慢加速，直接减速到零
  if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
  {
    chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
  }

  if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
  {
    chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
  }

  *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
  *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}
/**
 * @brief 控制量设置
 *
 * @param chassis_move_control
 */
void user_chassis_set_control(chassis_move_t *chassis_move_control)
{
  if (chassis_move_control == NULL)
  {
    return;
  }

  fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;

  user_chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_move_control);
  angle_set = -CHASSIS_WZ_RC_SEN * chassis_move_control->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];

  // 角速度设置
  chassis_move_control->wz_set = angle_set;
  // x轴速度
  chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
  // y轴速度
  chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
}
/**
 * @brief          四个麦轮速度是通过三个参数计算出来的
 * @param[in]      vx_set: 纵向速度
 * @param[in]      vy_set: 横向速度
 * @param[in]      wz_set: 旋转速度
 * @param[out]     wheel_speed: 四个麦轮速度
 * @retval         none
 */
void user_chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
  // 旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
  wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}
/**
 * @brief
 *
 * @param chassis_move_control_loop
 */
void user_chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
  fp32 max_vector = 0.0f, vector_rate = 0.0f;
  fp32 temp = 0.0f;
  fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  uint8_t i = 0;

  // 麦轮运动分解
  user_chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                             chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

  if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
  {

    for (i = 0; i < 4; i++)
    {
      chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
    }
    // raw控制直接返回
    return;
  }

  // 计算轮子控制最大速度，并限制其最大速度
  for (i = 0; i < 4; i++)
  {
    chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
    temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
    if (max_vector < temp)
    {
      max_vector = temp;
    }
  }

  if (max_vector > MAX_WHEEL_SPEED)
  {
    vector_rate = MAX_WHEEL_SPEED / max_vector;
    for (i = 0; i < 4; i++)
    {
      chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
    }
  }

  // pid计算速度
  for (i = 0; i < 4; i++)
  {
    PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
  }

  // // 裁判系统功率控制
  // chassis_power_control(chassis_move_control_loop);

  // 电机电流值
  for (i = 0; i < 4; i++)
  {
    chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
  }
}

/**
 * @brief 底盘初始化
 *
 * @param chassis_move_init
 */
void User_chassis_init(chassis_move_t *chassis_move_init)
{
  // 底盘速度环pid值
  const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
  // 底盘角度pid值
  const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};

  const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
  const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};

  uint8_t i;
  // 设置底盘开机状态 底盘由旋转速度控制
  chassis_move_init->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
  // 获取遥控器指针
  chassis_move_init->chassis_RC = get_remote_control_point();
  // 获取陀螺仪姿态指针
  // none
  // 根据电机数据指针初始化PID
  for (i = 0; i < 4; i++)
  {
    chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
    PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
  }
  // 初始化角度PID
  PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
  // 用一阶滤波代替斜波函数生成
  first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
  first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
  // 最大 最小速度
  chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
  chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

  chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
  chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

  user_chassis_feedback_update(chassis_move_init);
}

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t user_high_water;
#endif
/**
 * @brief 测试任务
 *
 * @param pvParameters
 */
int test_value = 0;
const RC_ctrl_t *user_RC_buf = NULL;
const char motor_speed_data = 3.0f;
void usere_test_task(void const *pvParameters)
{
  // 等待其他完成
  vTaskDelay(300);
  User_chassis_init(&BBX_chassis_move);
  // 判断电机是否在线
  while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE))
  {
    vTaskDelay(CHASSIS_CONTROL_TIME_MS);
  }

  while (1)
  {
    // 更新数据
    user_chassis_feedback_update(&BBX_chassis_move);
    // 控制设置
    user_chassis_set_control(&BBX_chassis_move);
    // PID计算
    user_chassis_control_loop(&BBX_chassis_move);

    if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE)))
    {
      if (toe_is_error(DBUS_TOE))
      {
        CAN_cmd_chassis(0, 0, 0, 0);
      }
      else
      {
        CAN_cmd_chassis(BBX_chassis_move.motor_chassis[0].give_current, BBX_chassis_move.motor_chassis[1].give_current,
                        BBX_chassis_move.motor_chassis[2].give_current, BBX_chassis_move.motor_chassis[3].give_current);
      }
    }
    vTaskDelay(2);
#if INCLUDE_uxTaskGetStackHighWaterMark
    user_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
  }
}
/*****************************************************************************/