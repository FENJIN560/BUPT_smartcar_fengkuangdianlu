#include "Motor.h"

static uint16 negative_motor_pwm = 0;

void Motor_Init()
{
    // 初始化左电机控制引脚
    gpio_init(MotorL_turn, GPO, 1, GPO_PUSH_PULL);
    pwm_init(MotorL_pwm, 5000, 0);

    // 初始化右电机控制引脚
    gpio_init(MotorR_turn, GPO, 1, GPO_PUSH_PULL);
    pwm_init(MotorR_pwm, 5000, 0);
}

void MotorL_SetSpeed(int pwm)
{
    if (pwm >= 0) {
        gpio_set_level(MotorL_turn, 1);
        pwm_set_duty(MotorL_pwm, pwm);
    }
    else {
        gpio_set_level(MotorL_turn, 0);
        pwm_set_duty(MotorL_pwm, -pwm);
    }
}

void MotorR_SetSpeed(int pwm)
{
    if (pwm >= 0) {
        gpio_set_level(MotorR_turn, 1);
        pwm_set_duty(MotorR_pwm, pwm);
    }
    else {
        gpio_set_level(MotorR_turn, 0);
        pwm_set_duty(MotorR_pwm, -pwm);
    }
}

// -------------------- 负压电机 --------------------
void NegativeMotor_Init(void)
{
    pwm_init(NegativeMotor_pwm, 10000, 0);   // 10kHz PWM，初始占空比 0
    negative_motor_pwm = 0;
}

void NegativeMotor_SetSpeed(uint16 pwm)
{
    if (pwm > 10000) pwm = 10000;   // 占空比限幅：0~10000
    negative_motor_pwm = pwm;
    pwm_set_duty(NegativeMotor_pwm, negative_motor_pwm);
}
