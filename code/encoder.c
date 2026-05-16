#include "encoder.h"
#include "Motor.h"
#include "math.h"

// -------------------- 速度环参数（按你当前车的实测数据做了偏置补偿） --------------------
#define SPEED_LOOP_MS           (10.0f)
#define SPEED_FILTER_ALPHA      (0.5f)   // 新值占比，兼顾响应和抖动
#define MOTOR_OUTPUT_LIMIT      (10000)
#define PID_INTEGRAL_LIMIT      (5000.0f)
#define PID_INTEGRAL_ENABLE_ERR (500.0f)   // 误差过大时先靠P和前馈，避免积分暴冲

// 你静态标定时左轮需要额外补一点 PWM，这里直接放进速度环前馈里
#define MOTOR_L_TRIM            (40.0f)
#define MOTOR_R_TRIM            (0.0f)
#define MOTOR_KFF               (12.0f)   // 速度目标 -> PWM 的前馈系数

// --- 全局变量 ---
float encoder_rpm_1 = 0.0f;
float encoder_rpm_3 = 0.0f;
int32 motor_output_L = 0;
int32 motor_output_R = 0;

// PID 全局定义并初始化
// 现在改成"前馈 + PI + 少量D"的速度环，响应会比原来更稳更快
PID_Pos pid_L = { 3.2f, 0.2f, 0.05f, 0, 0, 0 };
PID_Pos pid_R = { 3.2f, 0.2f, 0.05f, 0, 0, 0 };

static inline float clamp_float(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static inline int32 clamp_int32(int32 value, int32 min_value, int32 max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static inline float low_pass_filter(float old_value, float new_value)
{
    return old_value * (1.0f - SPEED_FILTER_ALPHA) + new_value * SPEED_FILTER_ALPHA;
}

static inline float get_motor_trim(float target, float trim)
{
    if (fabsf(target) < 1.0f)
    {
        return 0.0f;
    }
    return (target > 0.0f) ? trim : -trim;
}

static inline float get_feedforward(float target, float trim)
{
    if (fabsf(target) < 1.0f)
    {
        return 0.0f;
    }
    return MOTOR_KFF * target + get_motor_trim(target, trim);
}

float get_rpm(int16 pulse_count)
{
    // 这个系数沿用你原来的换算，绝不改单位体系，避免把主控逻辑一起带之
    return (float)pulse_count * 0.7324f;
}

// --- PID 计算：这里只输出"修正量"，主输出由前馈负责 ---
int32 pid_position_compute(PID_Pos* p, float actual)
{
    float error = p->target - actual;
    float derivative = error - p->last_error;

    // 目标接近 0 时，直接清空积分并停转，防止低速抖动
    if (fabsf(p->target) < 1.0f)
    {
        p->integral = 0.0f;
        p->last_error = 0.0f;
        return 0;
    }

    // 小误差才积分，避免弯道/大加减速时积分堆死
    if (fabsf(error) < PID_INTEGRAL_ENABLE_ERR)
    {
        p->integral += error;
    }



    p->integral = clamp_float(p->integral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);

    p->last_error = error;

    return (int32)(p->kp * error + p->ki * p->integral + p->kd * derivative);
}


void encoder_init_all(void)
{
    encoder_dir_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);
    encoder_dir_init(ENCODER_3, ENCODER_3_A, ENCODER_3_B);

    encoder_rpm_1 = 0.0f;
    encoder_rpm_3 = 0.0f;
    motor_output_L = 0;
    motor_output_R = 0;

    pid_L.integral = 0.0f;
    pid_L.last_error = 0.0f;
    pid_L.target = 0.0f;

    pid_R.integral = 0.0f;
    pid_R.last_error = 0.0f;
    pid_R.target = 0.0f;

    // 10ms 速度环
    pit_ms_init(CCU60_CH0, (uint32)SPEED_LOOP_MS);
}

IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    pit_clear_flag(CCU60_CH0);

    // 1. 读取编码器并统一方向
    float raw_rpm_1 = -get_rpm(encoder_get_count(ENCODER_1));
    float raw_rpm_3 =  get_rpm(encoder_get_count(ENCODER_3));

    encoder_clear_count(ENCODER_1);
    encoder_clear_count(ENCODER_3);

    // 2. 一阶低通，减小编码器抖动对速度环的影响
    encoder_rpm_1 = low_pass_filter(encoder_rpm_1, raw_rpm_1);
    encoder_rpm_3 = low_pass_filter(encoder_rpm_3, raw_rpm_3);

    // 3. 前馈 + PID 修正
    motor_output_L = (int32)get_feedforward(pid_L.target, MOTOR_L_TRIM) + pid_position_compute(&pid_L, encoder_rpm_1);
    motor_output_R = (int32)get_feedforward(pid_R.target, MOTOR_R_TRIM) + pid_position_compute(&pid_R, encoder_rpm_3);

    // 4. 限幅
    motor_output_L = clamp_int32(motor_output_L, -MOTOR_OUTPUT_LIMIT, MOTOR_OUTPUT_LIMIT);
    motor_output_R = clamp_int32(motor_output_R, -MOTOR_OUTPUT_LIMIT, MOTOR_OUTPUT_LIMIT);

    // 5. 输出到电机
    MotorL_SetSpeed(motor_output_L);
    MotorR_SetSpeed(motor_output_R);
}
