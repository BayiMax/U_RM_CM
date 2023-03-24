/**
 * @file user_crc32.c
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief crc32 计算初始化
 *        crc32 计算 , 取值 ,设值
 * @version 0.1
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "user_crc32.h"

extern CRC_HandleTypeDef hcrc;
/**
 * @brief 获取CRC校验数据
 *
 * @param data
 * @param len
 * @return uint32_t
 */
uint32_t get_crc32_check_sum(uint32_t *data, uint32_t len)
{
  return HAL_CRC_Calculate(&hcrc, data, len);
}
/**
 * @brief 校验CRC数据
 *
 * @param data
 * @param len
 * @return bool_t
 */
bool_t verify_crc32_check_sum(uint32_t *data, uint32_t len)
{
  static uint32_t crc32;
  crc32 = get_crc32_check_sum(data, len - 1);
  return (crc32 == data[len - 1]);
}
/**
 * @brief 添加CRC校验数据
 *
 * @param data
 * @param len
 */
void append_crc32_check_sum(uint32_t *data, uint32_t len)
{
  uint32_t crc32;
  crc32 = get_crc32_check_sum(data, len - 1);
  data[len - 1] = crc32;
}
