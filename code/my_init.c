/*
 * my_init.c
 *
 *  Created on: 2026年2月3日
 *      Author: rue
 */

#include "my_init.h"

void all_init()
{
    //ips114_set_dir(3);
    ips114_init();
    //图像初始化
    mt9v03x_double_init(mt9v03x_1);
    //陀螺仪初始化
    gyro_init();
    //编码器初始化
    //encoder_init();
    //电机初始化
    //motor_init();
    //蓝牙初始化
    //HC_05_init();
    //遥控器初始化
//    Joystick_Init();


    //1ms中断初始化
    pit_ms_init(CCU60_CH0, 1);
}


