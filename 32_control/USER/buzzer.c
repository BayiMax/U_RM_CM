#include "buzzer.h"

extern TIM_HandleTypeDef htim4;
/**
 * @brief 蜂鸣器开 设置频率控制音调
 *
 * @param psc
 * @param pwm
 */
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);
}
/**
 * @brief 蜂鸣器关
 *
 */
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}
