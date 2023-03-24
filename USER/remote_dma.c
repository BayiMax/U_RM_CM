/**
 * @file remote_dma.c
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief 遥控器数据dma数据接收
 * @version 0.1
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "remote_dma.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
/**
 * @brief 遥控器 DMA 初始化
 *
 * @param rx1_buf 数据接收1 缓冲区
 * @param rx2_buf 数据接收2 缓冲区
 * @param dma_buf_num 数据长度
 */
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    // enable the DMA transfer for the receiver request
    // 使能DMA串口接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    // enalbe idle interrupt
    // 使能空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    // disable DMA
    // 失效DMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while (hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    // memory buffer 1
    // 内存缓冲区1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    // memory buffer 2
    // 内存缓冲区2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    // data length
    // 数据长度
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    // enable double memory buffer
    // 使能双缓冲区
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    // enable DMA
    // 使能DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
}
/**
 * @brief 遥控器数据接收关闭
 *
 */
void RC_unable(void)
{
    __HAL_UART_DISABLE(&huart3);
}
/**
 * @brief 遥控器数据接收 开始
 *
 * @param dma_buf_num
 */
void RC_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart3);
    __HAL_DMA_DISABLE(&hdma_usart3_rx);

    hdma_usart3_rx.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart3_rx);
    __HAL_UART_ENABLE(&huart3);
}
