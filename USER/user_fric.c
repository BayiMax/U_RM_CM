/**
 * @file user_fric.c
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief 摩擦轮 控制
 * @version 0.1
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "user_fric.h"

extern TIM_HandleTypeDef htim1;
/**
 * @brief 摩擦轮关
 *
 */
void fric_off(void)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, FRIC_OFF);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, FRIC_OFF);
}
/**
 * @brief 摩擦轮 设置通道1频率
 *
 * @param cmd
 */
void fric1_on(uint16_t cmd)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, cmd);
}
/**
 * @brief 摩擦轮 设置通道2频率
 *
 * @param cmd
 */
void fric2_on(uint16_t cmd)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, cmd);
}
