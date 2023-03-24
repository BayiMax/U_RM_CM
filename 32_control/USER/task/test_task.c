/**
 * @file test_task.c
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief 测试任务
 * @version 0.1
 * @date 2023-02-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "test_task.h"
#include "user_c.h"

static void buzzer_warn_error(uint8_t num);

const error_t *error_list_test_local;

/**
 * @brief          test任务
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
void test_task(void const *argument)
{
  static uint8_t error, last_error;
  static uint8_t error_num;
  error_list_test_local = get_error_list_point();

  while (1)
  {
    error = 0;

    // 发现错误
    for (error_num = 0; error_num < REFEREE_TOE; error_num++)
    {
      if (error_list_test_local[error_num].error_exist)
      {
        error = 1;
        break;
      }
    }

    // 没有错误, 停止蜂鸣器
    if (error == 0 && last_error != 0)
    {
      buzzer_off();
    }
    // 有错误
    if (error)
    {
      // 设备错误 蜂鸣器响报警
      buzzer_warn_error(error_num + 1);
    }

    last_error = error;
    osDelay(10);
  }
}

/**
 * @brief          使得蜂鸣器响
 * @param[in]      num:响声次数
 * @retval         none
 */
static void buzzer_warn_error(uint8_t num)
{
  static uint8_t show_num = 0;
  static uint8_t stop_num = 100;
  if (show_num == 0 && stop_num == 0)
  {
    show_num = num;
    stop_num = 100;
  }
  else if (show_num == 0)
  {
    stop_num--;
    buzzer_off();
  }
  else
  {
    static uint8_t tick = 0;
    tick++;
    if (tick < 50)
    {
      buzzer_off();
    }
    else if (tick < 100)
    {
      buzzer_on(1, 30000);
    }
    else
    {
      tick = 0;
      show_num--;
    }
  }
}
