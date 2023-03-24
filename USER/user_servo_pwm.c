/**
 * @file user_servo_pwm.c
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief 舵机pwm输出
 * @version 0.1
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "user_servo_pwm.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
/**
 * @brief 舵机pwm输出
 *
 * @param pwm
 * @param i
 */
void servo_pwm_set(uint16_t pwm, uint8_t i)
{
  switch (i)
  {
  case 0:
  {
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm);
  }
  break;
  case 1:
  {
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm);
  }
  break;
  case 2:
  {
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm);
  }
  break;
  case 3:
  {
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm);
  }
  break;
  }
}
