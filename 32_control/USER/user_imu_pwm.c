/**
 * @file user_imu_pwm.c
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief
 * @version 0.1
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "user_imu_pwm.h"

extern TIM_HandleTypeDef htim10;
/**
 * @brief imu pwm输出设置
 *
 * @param pwm
 */
void imu_pwm_set(uint16_t pwm)
{
  __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
}
