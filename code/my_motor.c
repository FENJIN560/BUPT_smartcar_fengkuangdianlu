/*
 * my_motor.c
 *
 *  Created on: 2026年2月3日
 *      Author: rue
 */

#include "my_motor.h"

void motor_init(void)
{
    //左电机
    gpio_init(P11_9, GPO, 1, GPO_PUSH_PULL);
    pwm_init(ATOM2_CH5_P11_10, 17000, 0);

    //右电机
    gpio_init(P11_11, GPO, 1, GPO_PUSH_PULL);
    pwm_init(ATOM3_CH7_P11_12, 17000, 0);

    //负压
    pwm_init(ATOM1_CH0_P21_2, 17000, 0);

}


void Set_motor_pwm(int16 left_duty,int16 right_duty)
{
    /*********左电机控制*******/
    //限幅
    if(left_duty > PWM_LIMIT_MAX) left_duty = PWM_LIMIT_MAX;
    else if(left_duty < -PWM_LIMIT_MAX) left_duty = -PWM_LIMIT_MAX;
    //控制能正反转
    if(left_duty >= 0)
    {
        gpio_set_level(P11_9,1);
        pwm_set_duty(ATOM2_CH5_P11_10,left_duty);
    }
    else
    {
        gpio_set_level(P11_9,0);
        pwm_set_duty(ATOM2_CH5_P11_10,-left_duty);
    }

    /***********右电机控制**********/
    if(right_duty > PWM_LIMIT_MAX) right_duty = PWM_LIMIT_MAX;
    else if(right_duty < -PWM_LIMIT_MAX) right_duty = -PWM_LIMIT_MAX;
    //控制能正反转
    if(right_duty >= 0)
    {
        gpio_set_level(P11_11,1);
        pwm_set_duty(ATOM3_CH7_P11_12,right_duty);
    }
    else
    {
        gpio_set_level(P11_11,0);
        pwm_set_duty(ATOM3_CH7_P11_12,-right_duty);
    }

}

uint8 fuya_flag=0;
void Set_fuya_pwm(int16 fuya_duty)
{
    if(fuya_flag == 1)
    {
        pwm_set_duty(ATOM1_CH0_P21_2,fuya_duty);
    }
    else
    {
        pwm_set_duty(ATOM1_CH0_P21_2,0);
    }
}

//void close_fuya()
//{
//    if(fuya_flag == 0)
//    {
//        pwm_set_duty(ATOM1_CH0_P21_2,0);
//    }
//}

