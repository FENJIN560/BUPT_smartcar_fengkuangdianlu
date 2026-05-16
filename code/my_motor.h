/*
 * my_motor.h
 *
 *  Created on: 2026Äê2ÔÂ3ÈÕ
 *      Author: rue
 */

#ifndef CODE_MY_MOTOR_H_
#define CODE_MY_MOTOR_H_

#include "zf_common_headfile.h"

#define   PWM_LIMIT_MAX     6666

void motor_init(void);
void Set_motor_pwm(int16 left_duty,int16 right_duty);
void Set_fuya_pwm(int16 fuya_duty);
void close_fuya();

extern uint8 fuya_flag;

#endif /* CODE_MY_MOTOR_H_ */
