/*
 * car_control.c
 *
 *  Created on: 2026年2月3日
 *      Author: rue
 */

#include "car_control.h"

uint8 run_flag=0;

uint8 time_2ms = 0;
uint8 time_5ms = 0;

int base_speed = 1100;

float speed_diff = 0;

int16 left_pwm_output = 0;
int16 right_pwm_output = 0;

//遥控车变量
int gas=0;
int l_r_range=0;

void control()
{
    if(run_flag == 1)
    {
        if(++time_2ms >= 2)
        {
            time_2ms=0;
            speed_diff = Position_PID(&pid_turn,imu660ra_gyro_z,image_err);
        }

        if(++time_5ms >= 5)
        {
            time_5ms = 0;
            left_pwm_output = base_speed + speed_diff;
            right_pwm_output = base_speed - speed_diff;
        }
        Set_motor_pwm(left_pwm_output,right_pwm_output);
    }
    else
    {
        Set_motor_pwm(0,0);
    }

//    //遥控控制
//    // 1. 遥控归一化
//       float v = (128 - gas) / 128.0f;
//       float w = (l_r_range - 128) / 128.0f;
//
//       // 2. 死区处理
//       if(fabs(v) < 0.05f) v = 0;
//       if(fabs(w) < 0.05f) w = 0;
//
//       // 3. 最大 PWM 限制
//       float max_pwm = 1000;
//       float M_L = (v - w) * max_pwm;
//       float M_R = (v + w) * max_pwm;
//
//       // 4. 限幅
//       if(M_L > max_pwm) M_L = max_pwm;
//       if(M_L < -max_pwm) M_L = -max_pwm;
//       if(M_R > max_pwm) M_R = max_pwm;
//       if(M_R < -max_pwm) M_R = -max_pwm;
//
//       Set_motor_pwm(M_L,M_R);

}
