/**
 * @file power_task.c
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief 电源检测任务
 * @version 0.1
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "power_task.h"

#define Formulas 0

#define FULL_BATTER_VOLTAGE 25.2f
#define LOW_BATTER_VOLTAGE 22.2f // about 20%

#define VOLTAGE_DROP 0.00f

static fp32 calc_battery_percentage(float voltage);

fp32 battery_voltage;
fp32 electricity_percentage;

/**
 * @brief          电源采样和计算电源百分比
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
void battery_voltage_task(void const *argument)
{
  osDelay(1000);
  // 使用内部1.2V校准
  init_vrefint_reciprocal();
  while (1)
  {
    battery_voltage = get_battery_voltage() + VOLTAGE_DROP;
    electricity_percentage = calc_battery_percentage(battery_voltage);
    osDelay(100);
  }
}

/**
 * @brief 计算电压百分比
 *
 * @param voltage 电压值
 * @return fp32
 */
static fp32 calc_battery_percentage(float voltage)
{
  fp32 percentage;
  fp32 voltage_2 = voltage * voltage;
  fp32 voltage_3 = voltage_2 * voltage;
#if Formulas == 0
  if (voltage < 19.5f)
  {
    percentage = 0.0f;
  }
  else if (voltage < 21.9f)
  {
    percentage = 0.005664f * voltage_3 - 0.3386f * voltage_2 + 6.765f * voltage - 45.17f;
  }
  else if (voltage < 25.5f)
  {
    percentage = 0.02269f * voltage_3 - 1.654f * voltage_2 + 40.34f * voltage - 328.4f;
  }
  else
  {
    percentage = 1.0f;
  }
  if (percentage < 0.0f)
  {
    percentage = 0.0f;
  }
  else if (percentage > 1.0f)
  {
    percentage = 1.0f;
  }

  // another formulas
  //     另一套公式
#elif Formulas == 1
  if (voltage < 19.5f)
  {
    percentage = 0.0f;
  }
  else if (voltage < 22.5f)
  {
    //        percentage = 0.05776f * (voltage - 22.5f) * (voltage_2 - 39.0f * voltage + 383.4f) + 0.5f;
    percentage = 0.05021f * voltage_3 - 3.075f * voltage_2 + 62.77f * voltage - 427.02953125f;
  }
  else if (voltage < 25.5f)
  {
    //        percentage = 0.01822f * (voltage - 22.5f) * (voltage_2 - 52.05f * voltage + 637.0f) + 0.5f;
    percentage = 0.0178f * voltage_3 - 1.292f * voltage_2 + 31.41f * voltage - 254.903125f;
  }
  else
  {
    percentage = 1.0f;
  }
#endif
  return percentage;
}

/**
 * @brief          获取电量
 * @param[in]      void
 * @retval         电量, 单位 1, 1 = 1%
 */
uint16_t get_battery_percentage(void)
{
  return (uint16_t)(electricity_percentage * 100.0f);
}