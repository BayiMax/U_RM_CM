#include "delay.h"
#include "user_c.h"

static uint8_t fac_us = 0;
static uint32_t fac_ms = 0;

/**
 * @brief 延时初始化
 *
 */
void delay_init(void)
{
    fac_us = SystemCoreClock / 1000000;
    fac_ms = SystemCoreClock / 1000;
}
/**
 * @brief 微妙延时
 *
 * @param nus
 */
void delay_us(uint16_t nus)
{
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = nus * fac_us;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}
/**
 * @brief 毫秒延时
 *
 * @param nms
 */
void delay_ms(uint16_t nms)
{
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = nms * fac_ms;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}
