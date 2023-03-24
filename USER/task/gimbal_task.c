/**
 * @file gimbal_task.c
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief
 * @version 0.1
 *        完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
 *        故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
 *        状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
 *        状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
 * @date 2023-02-20
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "gimbal_task.h"

#include "user_c.h"

// 定义角度旋转是否有限制
#define ANGLE_AREA 0

#define User_yaw 1
#define User_pitch 2

// 电机编码值规整 0—8191
#define ecd_format(ecd)    \
  {                        \
    if ((ecd) > ECD_RANGE) \
      (ecd) -= ECD_RANGE;  \
    else if ((ecd) < 0)    \
      (ecd) += ECD_RANGE;  \
  }

/**
 * @brief 清除云台PID计算值
 */
#define gimbal_total_pid_clear(gimbal_clear)                                               \
  {                                                                                        \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
    PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
                                                                                           \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
    PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
  }

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
 */
static void gimbal_init(gimbal_control_t *init);
/**
 * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
 */
static void gimbal_set_mode(gimbal_control_t *set_mode);
/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 */
static void gimbal_feedback_update(gimbal_control_t *feedback_update);
/**
 * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
 */
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);
/**
 * @brief          计算ecd与offset_ecd之间的相对角度
 */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
/**
 * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
 */
static void gimbal_set_control(gimbal_control_t *set_control);
/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 */
static void gimbal_control_loop(gimbal_control_t *control_loop);
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_RAW，电流值直接发送到CAN总线.
 */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor);
/**
 * @brief          在GIMBAL_MOTOR_GYRO模式，限制角度设定,防止超过最大
 */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add, unsigned char set);
/**
 * @brief          在GIMBAL_MOTOR_ENCONDE模式，限制角度设定,防止超过最大
 */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add, unsigned char set);
/**
 * @brief          云台角度PID初始化, 因为角度范围在(-pi,pi)，不能用PID.c的PID
 */
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 intergral_limit, fp32 kp, fp32 ki, fp32 kd);
/**
 * @brief          云台PID清除，清除pid的out,iout
 * @param[out]     pid_clear:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_PID_clear(gimbal_PID_t *pid_clear);
/**
 * @brief          云台角度PID计算, 因为角度范围在(-pi,pi)，不能用PID.c的PID
 */
static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);
/**
 * @brief          云台校准计算
 */
static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);

#if GIMBAL_TEST_MODE
// j-scope 帮助pid调参
static void J_scope_gimbal_test(void);
#endif

// 云台控制所有相关数据
gimbal_control_t gimbal_control;

/**
 * @brief
 * @param yaw_can_set_current: yaw(偏航角)设置电流
 * @param pitch_can_set_current: pitch(俯仰角)设置电流
 * @param shoot_can_set_current: 射击电机设置电流
 *
 */
static int16_t yaw_can_set_current = 0, pitch_can_set_current = 0, shoot_can_set_current = 0;

/**
 * @brief          云台任务，间隔 1ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void gimbal_task(void const *pvParameters)
{
  // 等待陀螺仪任务更新陀螺仪数据
  vTaskDelay(GIMBAL_TASK_INIT_TIME);
  // 云台初始化
  gimbal_init(&gimbal_control);

  // 射击初始化
  shoot_init();

  // 判断电机是否都上线
  while (toe_is_error(YAW_GIMBAL_MOTOR_TOE) || toe_is_error(PITCH_GIMBAL_MOTOR_TOE))
  {
    vTaskDelay(GIMBAL_CONTROL_TIME);
    // 更新云台数据
    gimbal_feedback_update(&gimbal_control);
  }

  while (1)
  {
    // 设置云台控制模式
    gimbal_set_mode(&gimbal_control);
    // 控制模式切换 控制数据过渡
    gimbal_mode_change_control_transit(&gimbal_control);

    // 云台数据反馈
    gimbal_feedback_update(&gimbal_control);
    // 设置云台控制量
    gimbal_set_control(&gimbal_control);
    // 云台控制PID计算
    gimbal_control_loop(&gimbal_control);

    // 射击任务控制循环
    shoot_can_set_current = shoot_control_loop();

// 偏航角是否翻转
#if YAW_TURN
    yaw_can_set_current = -gimbal_control.gimbal_yaw_motor.given_current;
#else
    yaw_can_set_current = gimbal_control.gimbal_yaw_motor.given_current;
#endif
// 俯仰角是否翻转
#if PITCH_TURN
    pitch_can_set_current = -gimbal_control.gimbal_pitch_motor.given_current;
#else
    pitch_can_set_current = gimbal_control.gimbal_pitch_motor.given_current;
#endif
    // 判断电机是否在线,至少有一个电机在线
    if (!(toe_is_error(YAW_GIMBAL_MOTOR_TOE) && toe_is_error(PITCH_GIMBAL_MOTOR_TOE) && toe_is_error(TRIGGER_MOTOR_TOE)))
    {
      // 电机连接出错
      if (toe_is_error(DBUS_TOE))
      {
        // 发送0电流
        CAN_cmd_gimbal(0, 0, 0, 0);
      }
      else
      {
        // 发送计算电流值
        // CAN_cmd_gimbal(yaw_can_set_current, pitch_can_set_current, 0, 0);
        CAN_cmd_gimbal(yaw_can_set_current, pitch_can_set_current, shoot_can_set_current, 0);
      }
    }

#if GIMBAL_TEST_MODE
    J_scope_gimbal_test();
#endif

    vTaskDelay(GIMBAL_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
    // 检查当前任务的堆栈
    gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
  }
}

/**
 * @brief          云台校准设置，将校准的云台中值以及最小最大机械相对角度
 * @param[in]      yaw_offse:yaw 中值
 * @param[in]      pitch_offset:pitch 中值
 * @param[in]      max_yaw:max_yaw:yaw 最大相对角度
 * @param[in]      min_yaw:yaw 最小相对角度
 * @param[in]      max_yaw:pitch 最大相对角度
 * @param[in]      min_yaw:pitch 最小相对角度
 * @retval         返回空
 * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
 */
void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch)
{
  gimbal_control.gimbal_yaw_motor.offset_ecd = yaw_offset;
  gimbal_control.gimbal_yaw_motor.max_relative_angle = max_yaw;
  gimbal_control.gimbal_yaw_motor.min_relative_angle = min_yaw;

  gimbal_control.gimbal_pitch_motor.offset_ecd = pitch_offset;
  gimbal_control.gimbal_pitch_motor.max_relative_angle = max_pitch;
  gimbal_control.gimbal_pitch_motor.min_relative_angle = min_pitch;
}

/**
 * @brief          云台校准计算，将校准记录的中值,最大 最小值返回
 * @param[out]     yaw 中值 指针
 * @param[out]     pitch 中值 指针
 * @param[out]     yaw 最大相对角度 指针
 * @param[out]     yaw 最小相对角度 指针
 * @param[out]     pitch 最大相对角度 指针
 * @param[out]     pitch 最小相对角度 指针
 * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
 * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
 */
bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
  if (gimbal_control.gimbal_cali.step == 0)
  {
    gimbal_control.gimbal_cali.step = GIMBAL_CALI_START_STEP;
    // 保存进入时候的数据，作为起始数据，来判断最大，最小值
    gimbal_control.gimbal_cali.max_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
    gimbal_control.gimbal_cali.max_pitch_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
    gimbal_control.gimbal_cali.max_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
    gimbal_control.gimbal_cali.max_yaw_ecd = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
    gimbal_control.gimbal_cali.min_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
    gimbal_control.gimbal_cali.min_pitch_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
    gimbal_control.gimbal_cali.min_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
    gimbal_control.gimbal_cali.min_yaw_ecd = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
    return 0;
  }
  else if (gimbal_control.gimbal_cali.step == GIMBAL_CALI_END_STEP)
  {
    calc_gimbal_cali(&gimbal_control.gimbal_cali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
    (*max_yaw) -= GIMBAL_CALI_REDUNDANT_ANGLE;
    (*min_yaw) += GIMBAL_CALI_REDUNDANT_ANGLE;
    (*max_pitch) -= GIMBAL_CALI_REDUNDANT_ANGLE;
    (*min_pitch) += GIMBAL_CALI_REDUNDANT_ANGLE;
    gimbal_control.gimbal_yaw_motor.offset_ecd = *yaw_offset;
    gimbal_control.gimbal_yaw_motor.max_relative_angle = *max_yaw;
    gimbal_control.gimbal_yaw_motor.min_relative_angle = *min_yaw;
    gimbal_control.gimbal_pitch_motor.offset_ecd = *pitch_offset;
    gimbal_control.gimbal_pitch_motor.max_relative_angle = *max_pitch;
    gimbal_control.gimbal_pitch_motor.min_relative_angle = *min_pitch;
    gimbal_control.gimbal_cali.step = 0;
    return 1;
  }
  else
  {
    return 0;
  }
}

/**
 * @brief          云台校准计算，将校准记录的中值,最大 最小值
 * @param[out]     yaw 中值 指针
 * @param[out]     pitch 中值 指针
 * @param[out]     yaw 最大相对角度 指针
 * @param[out]     yaw 最小相对角度 指针
 * @param[out]     pitch 最大相对角度 指针
 * @param[out]     pitch 最小相对角度 指针
 * @retval         none
 */
static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
  if (gimbal_cali == NULL || yaw_offset == NULL || pitch_offset == NULL || max_yaw == NULL || min_yaw == NULL || max_pitch == NULL || min_pitch == NULL)
  {
    return;
  }

  int16_t temp_max_ecd = 0, temp_min_ecd = 0, temp_ecd = 0;

#if YAW_TURN
  temp_ecd = gimbal_cali->min_yaw_ecd - gimbal_cali->max_yaw_ecd;

  if (temp_ecd < 0)
  {
    temp_ecd += ecd_range;
  }
  temp_ecd = gimbal_cali->max_yaw_ecd + (temp_ecd / 2);

  ecd_format(temp_ecd);
  *yaw_offset = temp_ecd;
  *max_yaw = -motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
  *min_yaw = -motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#else

  temp_ecd = gimbal_cali->max_yaw_ecd - gimbal_cali->min_yaw_ecd;

  if (temp_ecd < 0)
  {
    temp_ecd += ECD_RANGE;
  }
  temp_ecd = gimbal_cali->max_yaw_ecd - (temp_ecd / 2);

  ecd_format(temp_ecd);
  *yaw_offset = temp_ecd;
  *max_yaw = motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
  *min_yaw = motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#endif

#if PITCH_TURN

  temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
  temp_max_ecd = gimbal_cali->max_pitch_ecd + temp_ecd;
  temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
  temp_min_ecd = gimbal_cali->min_pitch_ecd + temp_ecd;

  ecd_format(temp_max_ecd);
  ecd_format(temp_min_ecd);

  temp_ecd = temp_max_ecd - temp_min_ecd;

  if (temp_ecd > HALF_ECD_RANGE)
  {
    temp_ecd -= ECD_RANGE;
  }
  else if (temp_ecd < -HALF_ECD_RANGE)
  {
    temp_ecd += ECD_RANGE;
  }

  if (temp_max_ecd > temp_min_ecd)
  {
    temp_min_ecd += ECD_RANGE;
  }

  temp_ecd = temp_max_ecd - temp_ecd / 2;

  ecd_format(temp_ecd);

  *pitch_offset = temp_ecd;

  *max_pitch = -motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
  *min_pitch = -motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);

#else
  temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
  temp_max_ecd = gimbal_cali->max_pitch_ecd - temp_ecd;
  temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
  temp_min_ecd = gimbal_cali->min_pitch_ecd - temp_ecd;

  ecd_format(temp_max_ecd);
  ecd_format(temp_min_ecd);

  temp_ecd = temp_max_ecd - temp_min_ecd;

  if (temp_ecd > HALF_ECD_RANGE)
  {
    temp_ecd -= ECD_RANGE;
  }
  else if (temp_ecd < -HALF_ECD_RANGE)
  {
    temp_ecd += ECD_RANGE;
  }

  temp_ecd = temp_max_ecd - temp_ecd / 2;

  ecd_format(temp_ecd);

  *pitch_offset = temp_ecd;

  *max_pitch = motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
  *min_pitch = motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);
#endif
}

/**
 * @brief          返回yaw 电机数据指针
 * @param[in]      none
 * @retval         yaw电机指针
 */
const gimbal_motor_t *get_yaw_motor_point(void)
{
  return &gimbal_control.gimbal_yaw_motor;
}

/**
 * @brief          返回pitch 电机数据指针
 * @param[in]      none
 * @retval         pitch
 */
const gimbal_motor_t *get_pitch_motor_point(void)
{
  return &gimbal_control.gimbal_pitch_motor;
}

/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
 * @param[out]     init:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_init(gimbal_control_t *init)
{
  // 俯仰角 速度PID
  static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
  // 偏航角 数据PID
  static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};

  // 电机数据指针获取
  // 偏航角数据
  init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
  // 俯仰角数据
  init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();

  // 陀螺仪数据
  init->gimbal_INT_angle_point = get_INS_angle_point();

  // 角速度数据
  init->gimbal_INT_gyro_point = get_gyro_data_point();

  // 遥控器数据指针获取
  init->gimbal_rc_ctrl = get_remote_control_point();

  // 初始化电机模式

  // 偏航角模式 陀螺仪角度控制
  init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
  // 俯仰角模式 陀螺仪角度控制
  init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_GYRO;

  // 初始化yaw电机pid
  gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
  gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
  PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
  // 初始化pitch电机pid
  gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
  // 初始化 电机PID
  gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
  // PID初始化
  PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);

  // 清除所有PID
  gimbal_total_pid_clear(init);
  // 更新数据
  gimbal_feedback_update(init);

  // 初始化数据
  // 初始化偏航角数据
  init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
  init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
  init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;
  // 初始化俯仰角数据
  init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
  init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle;
  init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;
}

/**
 * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
 * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_mode(gimbal_control_t *set_mode)
{
  if (set_mode == NULL)
  {
    return;
  }
  // 云台模式设置
  gimbal_behaviour_mode_set(set_mode);
}

/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
  if (feedback_update == NULL)
  {
    return;
  }
  // 云台数据更新
  feedback_update->gimbal_pitch_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);

#if PITCH_TURN
  feedback_update->gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                  feedback_update->gimbal_pitch_motor.offset_ecd);
#else

  feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                 feedback_update->gimbal_pitch_motor.offset_ecd);
#endif

  feedback_update->gimbal_pitch_motor.motor_gyro = *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);

  feedback_update->gimbal_yaw_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);

#if YAW_TURN
  feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                feedback_update->gimbal_yaw_motor.offset_ecd);

#else
  feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                               feedback_update->gimbal_yaw_motor.offset_ecd);
#endif
  feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET)) - arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
}

/**
 * @brief          计算ecd与offset_ecd之间的相对角度
 * @param[in]      ecd: 电机当前编码
 * @param[in]      offset_ecd: 电机中值编码
 * @retval         相对角度，单位rad
 */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
  int32_t relative_ecd = ecd - offset_ecd;
  if (relative_ecd > HALF_ECD_RANGE)
  {
    relative_ecd -= ECD_RANGE;
  }
  else if (relative_ecd < -HALF_ECD_RANGE)
  {
    relative_ecd += ECD_RANGE;
  }

  return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
 * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
 * @param[out]     gimbal_mode_change:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
  if (gimbal_mode_change == NULL)
  {
    return;
  }
  // yaw电机状态机切换保存数据
  if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
  {
    gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
  }
  else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
  {
    gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
  }
  else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
  {
    gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
  }
  gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

  // pitch电机状态机切换保存数据
  if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
  {
    gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
  }
  else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
  {
    gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
  }
  else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
  {
    gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
  }
  gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}

/**
 * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
 * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_control(gimbal_control_t *set_control)
{
  if (set_control == NULL)
  {
    return;
  }

  fp32 add_yaw_angle = 0.0f;
  fp32 add_pitch_angle = 0.0f;

  gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);
  // yaw电机模式控制
  if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
  {
    // raw模式下，直接发送控制值
    set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
  }
  else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
  {
    // gyro模式下，陀螺仪角度控制
    gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle, User_yaw);
  }
  else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
  {
    // enconde模式下，电机编码角度控制
    gimbal_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle, User_yaw);
  }

  // pitch电机模式控制
  if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
  {
    // raw模式下，直接发送控制值
    set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
  }
  else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
  {
    // gyro模式下，陀螺仪角度控制
    gimbal_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle, User_pitch);
  }
  else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
  {
    // enconde模式下，电机编码角度控制
    gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle, User_pitch);
  }
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add, unsigned char set)
{
  static fp32 bias_angle;
  static fp32 angle_set;

  if (gimbal_motor == NULL)
  {
    return;
  }

  // 当前控制误差角度
  bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);

  /**角度限制*/
  if (set == User_pitch)
  {
    // 云台相对角度+ 误差角度 + 新增角度 如果大于 最大机械角度
    if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
    {
      // 如果是往最大机械角度控制方向
      if (add > 0.0f)
      {
        // 计算出一个最大的添加角度，
        add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
      }
    }
    else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle)
    {
      if (add < 0.0f)
      {
        add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
      }
    }
  }

  angle_set = gimbal_motor->absolute_angle_set;
  gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add, unsigned char set)
{
  if (gimbal_motor == NULL)
  {
    return;
  }
  gimbal_motor->relative_angle_set += add;

  // #if ANGLE_AREA == 1

  if (set == User_pitch)
  {
    // 是否超过最大 最小值
    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
    {
      gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
    }
    else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
    {
      gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
    }
  }
  // #endif
}

/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_control_loop(gimbal_control_t *control_loop)
{
  if (control_loop == NULL)
  {
    return;
  }
  // 判断 云台偏航角电机控制模式
  if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
  {
    gimbal_motor_raw_angle_control(&control_loop->gimbal_yaw_motor);
  }
  else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
  {
    gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor);
  }
  else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
  {
    gimbal_motor_relative_angle_control(&control_loop->gimbal_yaw_motor);
  }
  // 判断 云台俯仰角电机控制模式
  if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
  {
    gimbal_motor_raw_angle_control(&control_loop->gimbal_pitch_motor);
  }
  else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
  {
    gimbal_motor_absolute_angle_control(&control_loop->gimbal_pitch_motor);
  }
  else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
  {
    gimbal_motor_relative_angle_control(&control_loop->gimbal_pitch_motor);
  }
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
  if (gimbal_motor == NULL)
  {
    return;
  }
  // 角度环，速度环串级pid调试
  gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
  gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
  // 控制值赋值
  gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
  if (gimbal_motor == NULL)
  {
    return;
  }

  // 角度环，速度环串级pid调试
  gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro);
  gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
  // 控制值赋值
  gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_RAW，电流值直接发送到CAN总线.
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor)
{
  if (gimbal_motor == NULL)
  {
    return;
  }
  gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
  gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

#if GIMBAL_TEST_MODE
int32_t yaw_ins_int_1000, pitch_ins_int_1000;
int32_t yaw_ins_set_1000, pitch_ins_set_1000;
int32_t pitch_relative_set_1000, pitch_relative_angle_1000;
int32_t yaw_speed_int_1000, pitch_speed_int_1000;
int32_t yaw_speed_set_int_1000, pitch_speed_set_int_1000;
static void J_scope_gimbal_test(void)
{
  yaw_ins_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle * 1000);
  yaw_ins_set_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle_set * 1000);
  yaw_speed_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro * 1000);
  yaw_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro_set * 1000);

  pitch_ins_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle * 1000);
  pitch_ins_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle_set * 1000);
  pitch_speed_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro * 1000);
  pitch_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro_set * 1000);
  pitch_relative_angle_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle * 1000);
  pitch_relative_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle_set * 1000);
}

#endif

/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
 * @param[out]     gimbal_init:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
  if (pid == NULL)
  {
    return;
  }
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;

  pid->err = 0.0f;
  pid->get = 0.0f;

  pid->max_iout = max_iout;
  pid->max_out = maxout;
}
/**
 * @brief 云台PID计算
 *
 * @param pid pid参数
 * @param get 输入
 * @param set 输出
 * @param error_delta
 * @return fp32
 */
static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
  fp32 err;
  if (pid == NULL)
  {
    return 0.0f;
  }
  pid->get = get;
  pid->set = set;

  err = set - get;
  pid->err = rad_format(err);
  pid->Pout = pid->kp * pid->err;
  pid->Iout += pid->ki * pid->err;
  pid->Dout = pid->kd * error_delta;
  abs_limit(&pid->Iout, pid->max_iout);
  pid->out = pid->Pout + pid->Iout + pid->Dout;
  abs_limit(&pid->out, pid->max_out);
  return pid->out;
}

/**
 * @brief          云台PID清除，清除pid的out,iout
 * @param[out]     gimbal_pid_clear:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear)
{
  if (gimbal_pid_clear == NULL)
  {
    return;
  }
  gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
  gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}
