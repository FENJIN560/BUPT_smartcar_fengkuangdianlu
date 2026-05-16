#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#include "zf_common_headfile.h"

#define MotorL_pwm           ATOM2_CH5_P11_10
#define MotorL_turn          P11_9

#define MotorR_pwm           ATOM3_CH7_P11_12
#define MotorR_turn          P11_11

// 负压电机 PWM 引脚
#define NegativeMotor_pwm    ATOM0_CH0_P21_2

void Motor_Init(void);
void MotorL_SetSpeed(int pwm);
void MotorR_SetSpeed(int pwm);

// 新增：负压电机
void NegativeMotor_Init(void);
void NegativeMotor_SetSpeed(uint16 pwm);

#endif /* CODE_MOTOR_H_ */
