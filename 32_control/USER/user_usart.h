#ifndef USER_USART_H
#define USER_USART_H
#include "user_c.h"

extern void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

extern void usart1_tx_dma_init(void);
extern void usart1_tx_dma_enable(uint8_t *data, uint16_t len);
/**
 * @brief UART+DMA 发送数据
 *
 * @param data
 * @param len
 */
void usart6_tx_dma_enable(uint8_t *data, uint16_t len);
#endif /*USER_USART_H*/
