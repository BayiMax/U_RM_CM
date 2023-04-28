/**
 * @file referee_task.h
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief 裁判系统数据获取任务
 * @version 0.1
 * @date 2023-04-25
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef REFEREE_TASK_H
#define REFEREE_TASK_H
#include "main.h"

#define USART_RX_BUF_LENGHT 512
#define REFEREE_FIFO_BUF_LENGTH 1024

/**
 * @brief          裁判系统
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
extern void referee_usart_task(void const *argument);
#endif // REFEREE_TASK_H
