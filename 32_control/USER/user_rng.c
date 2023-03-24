/**
 * @file user_rng.c
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief 随机数生成
 * @version 0.1
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "user_rng.h"

extern RNG_HandleTypeDef hrng;
/**
 * @brief  随机数发生器
 *
 * @return uint32_t 返回随机数
 */
uint32_t RNG_get_random_num(void)
{
  static uint32_t rng;
  HAL_RNG_GenerateRandomNumber(&hrng, &rng);
  return rng;
}
/**
 * @brief 给定区间随机数发生
 *
 * @param min
 * @param max
 * @return int32_t 返回随机数
 */
int32_t RNG_get_random_rangle(int min, int max)
{
  static int32_t random;
  random = (RNG_get_random_num() % (max - min + 1)) + min;
  return random;
}
