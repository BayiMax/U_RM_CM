/**
 * @file user_laser.c
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief 激光初始化
 * @version 0.1
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "user_laser.h"

extern TIM_HandleTypeDef htim3;
/**
 * @brief   激光开
 *
 */
void laser_on(void)
{
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 8399);
}
/**
 * @brief 激光关
 *
 */
void laser_off(void)
{
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
}
