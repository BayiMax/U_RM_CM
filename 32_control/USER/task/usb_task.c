/**
 * @file usb_task.c
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief usb任务
 * @version 0.1
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "usb_task.h"
#include "user_c.h"

#include "cmsis_os.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "detect_task.h"

// #include "voltage_task.h"

static uint8_t usb_buf[256];
static const char status[2][7] = {"OK", "ERROR!"};
const error_t *error_list_usb_local;

/**
 * @brief USB-task;usb数据传输 串口
 *
 * @param argument
 */
void usb_task(void const *argument)
{
  float Get_All_Value[3];

  MX_USB_DEVICE_Init();

  // error_list_usb_local = get_error_list_point();

  while (1)
  {
    osDelay(1000);
    Get_All_Value[0] = get_temprate();
    Get_All_Value[1] = (float)get_battery_percentage();
    usb_printf("ic temperature:%.4f;\r\n\
								battery level:%.2f\r\n 	\
								",
               Get_All_Value[0],
               Get_All_Value[1]);
  }
}
/**
 * @brief usb发送数据 (重定义printf)
 *
 * @param fmt
 * @param ...
 */
void usb_printf(const char *fmt, ...)
{
  static va_list ap;
  uint16_t len = 0;

  va_start(ap, fmt);

  len = vsprintf((char *)usb_buf, fmt, ap);

  va_end(ap);

  CDC_Transmit_FS(usb_buf, len);
}
