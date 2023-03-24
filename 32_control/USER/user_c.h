#ifndef __USER_C_H
#define __USER_C_H

#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "math.h"
#include "arm_math.h"
// sys
#include "cmsis_os.h"
#include "main.h"
#include "struct_typedef.h"
#include "can.h"
#include "user_lib.h"

// USER
#include "led.h"
#include "buzzer.h"
#include "delay.h"
#include "remote_dma.h"
#include "remote.h"

#include "user_adc.h"
#include "user_can.h"
#include "user_flash.h"
#include "user_fric.h"
#include "user_usart.h"
#include "user_spi.h"
#include "user_imu_pwm.h"
#include "user_i2c.h"
#include "user_laser.h"
#include "user_rng.h"
#include "CAN_receive.h"

//  task
#include "user_task.h"
#include "usb_task.h"
#include "power_task.h"
#include "rgb_task.h"
#include "INS_task.h"
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "chassis_power_control.h"
#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "calibrate_task.h"
#include "servo_task.h"
#include "shoot.h"

// APP
#include "BMI088_driver.h"
#include "BMI088_Middleware.h"
#include "ist8310_driver.h"
#include "ist8310_driver_middleware.h"

// algorithm
#include "AHRS_MiddleWare.h"
#include "crc8_crc16.h"
#include "fifo.h"
#include "mem_mang.h"
#include "pid.h"
#include "user_lib.h"
#include "AHRS.h"
//

#endif /*__USER_C_H*/
