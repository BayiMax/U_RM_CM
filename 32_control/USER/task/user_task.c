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
#include "user_task.h"
#include "user_c.h"
/************************************************/

/************************************************/

/**
 * @brief 测试任务
 *
 * @param pvParameters
 */
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t user_high_water;
#endif

const RC_ctrl_t *user_rc_control;

int16_t test_value = 0;
//**********************//
void usere_test_task(void const *pvParameters)
{
  while (1)
  {
    user_rc_control = get_remote_control_point();
    if (user_rc_control->rc.s[1] == 2 || user_rc_control->rc.s[1] == 1)
    {
      laser_on();
//			CAN_cmd_chassis_reset_ID();
    }
    else if (user_rc_control->rc.s[1] == 3)
    {
      laser_off();
    }
    /*********/


    /*********/
    vTaskDelay(2);
#if INCLUDE_uxTaskGetStackHighWaterMark
    user_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
  }
}
