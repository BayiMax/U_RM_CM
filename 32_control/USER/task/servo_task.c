/**
 * @file servo_task.c
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief 弹丸发射电机控制 PWM方式
 * @version 0.1
 * @date 2023-02-25
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "servo_task.h"
#include "user_c.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
/**
 * @brief 弹丸发射电机PWM发送函数
 *
 * @param pwm
 * @param i
 */
static void servo_pwm_set(uint16_t pwm, uint8_t i);

#define SERVO_MIN_PWM 500
#define SERVO_MAX_PWM 2500

#define PWM_DETAL_VALUE 10

#define SERVO1_ADD_PWM_KEY KEY_PRESSED_OFFSET_Z
#define SERVO2_ADD_PWM_KEY KEY_PRESSED_OFFSET_X
#define SERVO3_ADD_PWM_KEY KEY_PRESSED_OFFSET_C
#define SERVO4_ADD_PWM_KEY KEY_PRESSED_OFFSET_V

#define SERVO_MINUS_PWM_KEY KEY_PRESSED_OFFSET_SHIFT

const RC_ctrl_t *servo_rc;
const static uint16_t servo_key[4] = {SERVO1_ADD_PWM_KEY, SERVO2_ADD_PWM_KEY, SERVO3_ADD_PWM_KEY, SERVO4_ADD_PWM_KEY};
uint16_t servo_pwm[4] = {SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM};
/**
 * @brief          servo_task
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
/**
 * @brief          舵机任务
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
void servo_task(void const *argument)
{
    servo_rc = get_remote_control_point();

    while (1)
    {
        for (uint8_t i = 0; i < 4; i++)
        {

            if ((servo_rc->key.v & SERVO_MINUS_PWM_KEY) && (servo_rc->key.v & servo_key[i]))
            {
                servo_pwm[i] -= PWM_DETAL_VALUE;
            }
            else if (servo_rc->key.v & servo_key[i])
            {
                servo_pwm[i] += PWM_DETAL_VALUE;
            }

            // 限制pwm
            if (servo_pwm[i] < SERVO_MIN_PWM)
            {
                servo_pwm[i] = SERVO_MIN_PWM;
            }
            else if (servo_pwm[i] > SERVO_MAX_PWM)
            {
                servo_pwm[i] = SERVO_MAX_PWM;
            }

            servo_pwm_set(servo_pwm[i], i);
        }
        osDelay(10);
    }
}
/**
 * @brief 弹丸发射电机PWM发送函数
 *
 * @param pwm
 * @param i
 */
static void servo_pwm_set(uint16_t pwm, uint8_t i)
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
