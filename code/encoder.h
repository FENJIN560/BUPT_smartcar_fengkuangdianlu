#ifndef ENCODER_H
#define ENCODER_H

#include "zf_common_headfile.h"

// -------------------- 编码器引脚定义 --------------------
#define ENCODER_1                   (TIM4_ENCODER)
#define ENCODER_1_A                 (TIM4_ENCODER_CH1_P02_8)
#define ENCODER_1_B                 (TIM4_ENCODER_CH2_P00_9)

#define ENCODER_3                   (TIM6_ENCODER)
#define ENCODER_3_A                 (TIM6_ENCODER_CH1_P20_3)
#define ENCODER_3_B                 (TIM6_ENCODER_CH2_P20_0)

// -------------------- PID 结构体定义 --------------------
typedef struct {
    float kp, ki, kd;    // PID 三参数
    float target;        // 目标转速 (RPM)
    float integral;      // 积分累计值
    float last_error;    // 上次误差
} PID_Pos;

// -------------------- 全局变量声明 --------------------
// 声明两个全局 PID 对象，供 main.c 修改 target
extern PID_Pos pid_L;
extern PID_Pos pid_R;

// 声明转速变量，用于调试显示
extern float encoder_rpm_1;
extern float encoder_rpm_3;

// -------------------- 函数声明 --------------------
void encoder_init_all(void);
int32 pid_position_compute(PID_Pos *p, float actual);

#endif // ENCODER_H
