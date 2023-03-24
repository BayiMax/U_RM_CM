/**
 * @brief 遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
 *        资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
 *        的方式保证热插拔的稳定性。
 *
 */

#ifndef __REMOTE_H
#define __REMOTE_H

#include "struct_typedef.h"

/**
 * @brief
 * 遥控器数据解析
 *
 * RC_ctrl_t.rc.ch[0]=右-左右 右正
 * RC_ctrl_t.rc.ch[1]=右-前后 前正
 * RC_ctrl_t.rc.ch[2]=左-左右
 * RC_ctrl_t.rc.ch[3]=左-前后
 * RC_ctrl_t.rc.ch[4]=波轮 下为加
 * 
 * RC_ctrl_t.rc.s[1]=CL:3;HL:2;OFF:1
 * RC_ctrl_t.rc.s[0]=ATTI下:2;ATTI中:3;GPS:1
 *
 */
typedef __packed struct
{
  __packed struct
  {
    // 摇杆数据
    int16_t ch[5];
    // 开关数据
    char s[2];
  } rc;
  __packed struct
  {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t press_l;
    uint8_t press_r;
  } mouse;
  __packed struct
  {
    uint16_t v;
  } key;

} RC_ctrl_t;

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */

/* ----------------------- Internal Data ----------------------------------- */

extern void remote_control_init(void);
extern const RC_ctrl_t *get_remote_control_point(void);
extern uint8_t RC_data_is_error(void);
extern void slove_RC_lost(void);
extern void slove_data_error(void);
extern void sbus_to_usart1(uint8_t *sbus);
#endif
