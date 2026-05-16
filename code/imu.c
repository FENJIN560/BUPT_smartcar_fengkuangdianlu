#include "zf_common_headfile.h"
#include "zf_device_imu660rx.h"
#include "imu.h"

// zf_device_imu660rx.h 中声明名有笔误（写成了 imu660ra_transition_factor），
// 此处补充正确的外部声明，与 zf_device_imu660rx.c 中实际定义一致
extern float imu660rx_transition_factor[2];

// ================================================================
// 互补滤波器参数
// dt = PIT 周期（秒）；ALPHA 越大越信任加速度计（响应快但噪声大）
// ================================================================
#define IMU_PIT_MS          (5.0f)                  // PIT 周期 5ms → 200Hz
#define IMU_DT              (IMU_PIT_MS / 1000.0f)  // 单位：秒
#define IMU_CF_ALPHA        (0.02f)                 // 互补系数：0.02 信任加速度计，0.98 信任陀螺仪

// ================================================================
// 姿态角全局变量（单位：度）
// ================================================================
volatile float imu_pitch = 0.0f;
volatile float imu_roll  = 0.0f;
volatile float imu_yaw   = 0.0f;

// ================================================================
// 内部：由加速度计计算俯仰 / 横滚角（单位：度）
// ================================================================
static float acc_get_pitch(float ax, float ay, float az)
{
    return (float)(atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / 3.14159265f));
}

static float acc_get_roll(float ay, float az)
{
    return (float)(atan2f(ay, az) * (180.0f / 3.14159265f));
}

// ================================================================
// 姿态解算核心（在 PIT 中断或主循环中调用）
// ================================================================
void imu_update(void)
{
    // 1. 采集原始数据
    imu660rx_get_acc();
    imu660rx_get_gyro();

    // 2. 转换为物理单位
    float ax = imu660rx_acc_transition(imu660rx_acc_x);   // 单位 g
    float ay = imu660rx_acc_transition(imu660rx_acc_y);
    float az = imu660rx_acc_transition(imu660rx_acc_z);

    float gx = imu660rx_gyro_transition(imu660rx_gyro_x); // 单位 °/s
    float gy = imu660rx_gyro_transition(imu660rx_gyro_y);
    float gz = imu660rx_gyro_transition(imu660rx_gyro_z);

    // 3. 加速度计解算姿态角
    float acc_pitch = acc_get_pitch(ax, ay, az);
    float acc_roll  = acc_get_roll (ay, az);

    // 4. 互补滤波：陀螺仪积分 + 少量加速度计修正
    imu_pitch = (1.0f - IMU_CF_ALPHA) * (imu_pitch + gx * IMU_DT) + IMU_CF_ALPHA * acc_pitch;
    imu_roll  = (1.0f - IMU_CF_ALPHA) * (imu_roll  + gy * IMU_DT) + IMU_CF_ALPHA * acc_roll;
    imu_yaw  += gz * IMU_DT;

    // 偏航角保持在 [-180, 180] 范围内
    if (imu_yaw >  180.0f) imu_yaw -= 360.0f;
    if (imu_yaw < -180.0f) imu_yaw += 360.0f;
}

// ================================================================
// CCU60_CH1 PIT 中断（5ms，优先级31）—— 自动调用姿态解算
// encoder.c 已占用 CCU60_CH0（优先级30），这里使用 CCU60_CH1
// ================================================================
IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH1);
    imu_update();

    // 每 20 次中断（100ms）向上位机发送一次姿态数据（ASCII格式，普通串口助手可直接查看）
    static uint8 send_cnt = 0;
    if (++send_cnt >= 20)
    {
        send_cnt = 0;
        printf("Pitch:%.2f Roll:%.2f Yaw:%.2f\r\n",
               (float)imu_pitch, (float)imu_roll, (float)imu_yaw);
    }
}

// ================================================================
// 初始化 IMU660RX 并启动 PIT 定时姿态更新
// ================================================================
uint8 imu_init(void)
{
    uint8 ret = imu660rx_init();
    if (ret != 0)
    {
        return ret;   // 初始化失败
    }

    // 启动 PIT：CCU60_CH1，周期 5ms
    pit_ms_init(CCU60_CH1, (uint32)IMU_PIT_MS);

    return 0;
}
