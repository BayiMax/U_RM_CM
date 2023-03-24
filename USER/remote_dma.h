#ifndef __REMOTE_DMA_H
#define __REMOTE_DMA_H

#include "user_c.h"

extern void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void RC_unable(void);
extern void RC_restart(uint16_t dma_buf_num);

#endif /*__REMOTE_DMA_H*/
