#include "zf_common_headfile.h"
#include "Motor.h"
#include "encoder.h"
#include "turn.h"
#include "math.h"
#include <stdlib.h>
#include "image.h"
#include "zf_driver_uart.h"
#include "imu.h"

// -------------------- 宏定义 --------------------
#define LED2 (P20_8)
#define BASE_SPEED 100
#define KP 0.6f
#define KD 1.8f
#define CAMERA_W 188
#define CAMERA_H 120

// -------------------- 全局变量 --------------------
float center_error = 0;
float last_center_error = 0;
int current_step = 0;
uint16 junction_cooldown = 30;
uint16 turn_timer = 0;

// 外部变量声明
extern uint8 original_image[120][188];
extern uint8 bin_image[120][188];
extern uint8 image_threshold;
extern float encoder_rpm_1, encoder_rpm_3;
extern PID_Pos pid_L, pid_R;
extern uint8 center_line[120];

// -------------------- 外部电机输出变量 (警告 E272) --------------------
extern int32 motor_output_L; // 定义在 encoder.c 文件中
extern int32 motor_output_R;
extern int debug_left_cnt;   // 定义在 turn.c 文件中
extern int debug_right_cnt;
extern int debug_top_cnt;

// -------------------- 外部函数声明 (警告 W577) --------------------
// 这些函数在其他源文件中已定义，此处仅声明
extern void Motor_Init (void);
extern void encoder_init_all (void);
extern void image_process (void);

extern Junction detect_junction_by_border (uint8 bin[120][188]);

// -------------------- 小车状态枚举 --------------------
typedef enum
{
    STATUS_TRACKING = 0, STATUS_TURNING = 1, STATUS_STOP_CAR = 2
} CarStatus;
CarStatus current_status = STATUS_TRACKING;

// -------------------- 路线规划结构体 --------------------
typedef struct
{
    Turn turn;
    uint16 duration;
} RouteStep;

RouteStep route_plan[] = {{RIGHT, 15},  // 0
        {RIGHT, 15},  // 1
        {STRAIGHT, 15},  // 2
        {RIGHT, 15},  // 3
        {RIGHT, 15},  // 4
        {RIGHT, 15},  // 5
        {STRAIGHT, 15},  // 6
        {RIGHT, 15},  // 7
        {STRAIGHT, 20},  // 8
        {LEFT, 15},   // 9
        {LEFT, 15},
        {LEFT, 15},
        {LEFT, 15},
        {RIGHT, 15},
        {LEFT, 10},
        {RIGHT, 15},
        {RIGHT, 20},
        {STOP, 50}
};


// -------------------- 函数定义 --------------------

// 使用 VOFA+ JustFloat 协议发送调试数据
//void vofa_send_data(float f1, float f2)
//{
//    float data[2] = {f1, f2};
//
//    // 逐字节发送浮点数组: uart_write_buffer
//    // 参数1: 串口编号, 参数2: 数据缓冲区指针, 参数3: 数据字节数
//    uart_write_buffer(UART_0, (uint8 *)data, sizeof(data));
//
//    // JustFloat 协议尾帧: 0x00 0x00 0x80 0x7F 表示结束
//    uint8 tail[4] = {0x00, 0x00, 0x80, 0x7F};
//    uart_write_buffer(UART_0, tail, 4);
//}

int core0_main (void)
{
    clock_init();
    debug_init();
    gpio_init(LED2, GPO, 0, GPO_PUSH_PULL);
    ips114_init();
    system_delay_ms(100);

    Motor_Init();
    NegativeMotor_Init();
    encoder_init_all();
    imu_init();   // 初始化 IMU660RX，使用 CCU60_CH1 PIT 定时器 5ms 周期读取数据

    NegativeMotor_SetSpeed(5000);   // 负压风扇占空比50%，避免瞬间电流过大烧芯片

    // 摄像头双缓冲初始化
    while (mt9v03x_double_init(mt9v03x_1) != 0)
        system_delay_ms(1000);

    // 等待摄像头 100 帧稳定
    for (int i = 0; i < 100; i++)
    {
        while (!mt9v03x_finish_flag_1)
            ;
        mt9v03x_finish_flag_1 = 0;
    }

    static uint32 loop_count = 0;

    while (TRUE)

    {

        if (mt9v03x_finish_flag_1)
        {
            mt9v03x_finish_flag_1 = 0;
            loop_count++;

            // 1. 图像预处理
            image_process();

            // 2. 计算白色像素总数和加权中心误差
            int total_white_pixels = 0;
            for (int y = 40; y < 110; y += 4)
            {
                for (int x = 10; x < 178; x += 4)
                {
                    if (bin_image[y][x] == 255)
                        total_white_pixels++;
                }
            }

            float frame_error = 0;
            int valid_rows = 0;
            int rows[] = {110, 100, 90, 85, 80};
            float weights[] = {0.5f, 0.25f, 0.15f, 0.05f, 0.05f};

            for (int i = 0; i < 5; i++)
            {
                int r = rows[i];
                uint8 thr = image_threshold + 20;
                int32 sum_x = 0, cnt = 0;
                for (int x = 5; x < 183; x++)
                {
                    if (original_image[r][x] > thr)
                    {
                        sum_x += x;
                        cnt++;
                    }
                }
                if (cnt >= 2 && cnt < 150)
                {
                    frame_error += ((float) (sum_x / cnt) - 94.0f) * weights[i];
                    valid_rows++;
                }
            }

            if (valid_rows > 0)
            {
                center_error = frame_error / (valid_rows >= 4 ? 1.0f : 0.7f);
            }
            // -----------------------------------------------------------------------

            // 3. 越界保护_停止 (超过最大步骤数后强制进入 STOP 状态)
            if (current_step > 25)
            {
                current_status = STATUS_STOP_CAR;
            }

            // --- 状态机处理 ---
            switch (current_status)
            {
                case STATUS_TRACKING :
                {

                    if (valid_rows > 0)
                    {
                        float error_diff = center_error - last_center_error;
                        float pd_out = KP * center_error + KD * error_diff;

                        pid_L.target = (float) (BASE_SPEED + pd_out);
                        pid_R.target = (float) (BASE_SPEED - pd_out);

//                        if (loop_count % 10 == 0) // 每10帧发送一次调试数据
//                        {
//                            // 通过串口发送中心误差和PD输出
//                            vofa_send_data(center_error, d_out);
//                        }

                    }
                    else
                    {
                        // 丢线时沿上次误差方向盲跑
                        float blind_steer = (last_center_error > 0) ? 80.0f : -80.0f;
                        pid_L.target = (float) (BASE_SPEED + blind_steer);
                        pid_R.target = (float) (BASE_SPEED - blind_steer);
                    }
//                    if (loop_count % 20 == 0) {
                    // T: 目标转速, R: 实际转速, O: 电机输出占空比 PWM
//                        printf("T_L:%.0f R_L:%.0f O_L:%d | T_R:%.0f R_R:%.0f O_R:%d | S:%d\r\n",
//                               pid_L.target, encoder_rpm_1, motor_output_L,
//                               pid_R.target, encoder_rpm_3, motor_output_R,
//                               current_status);
//                    }

                    // --- 3. 路口检测 ---
                    if (junction_cooldown == 0)
                    {
                        if (valid_rows >= 2 && total_white_pixels < 200)
                        {
                            Junction junc = detect_junction_by_border(bin_image);
                            if (junc == ROAD_T || junc == ROAD_Y || junc == ROAD_L_LEFT || junc == ROAD_L_RIGHT)
                            {
                                if (route_plan[current_step].turn == STOP)
                                {
                                    current_status = STATUS_STOP_CAR;
                                    // 路线最后1步，进入停车状态，持续 150 个周期后完全停车
                                    turn_timer = route_plan[current_step].duration;
                                }
                                else
                                {
                                    // 根据当前步骤设置路口冷却时间
                                    switch (current_step)
                                    {
                                        case 0 :
                                            junction_cooldown = 50;
                                            break;
                                        case 1 :
                                            junction_cooldown = 50;
                                            break;
                                        case 3 :
                                            junction_cooldown = 50;
                                            break;
                                        case 4 :
                                            junction_cooldown = 80;
                                            break;
                                        case 5 :
                                            junction_cooldown = 50;
                                            break;
                                        case 6 :
                                            junction_cooldown = 10;
                                            break;
                                        case 7 :
                                            junction_cooldown = 10;
                                            break;
                                        case 13 :
                                            junction_cooldown = 100;
                                            break;
                                        default :
                                            junction_cooldown = 20;
                                            break;
                                    }
                                    turn_timer = route_plan[current_step].duration;
                                    current_step++;
                                    current_status = STATUS_TURNING;
                                    gpio_set_level(LED2, 0); // 点亮LED指示灯
                                }
                            }
                        }
                    }

                    last_center_error = center_error;
                    break;
                }
                case STATUS_TURNING :
                {
                    Turn action = route_plan[current_step - 1].turn;
                    if (turn_timer > 0)
                    {
                        if (action == RIGHT)
                        {
                            pid_L.target = 100;
                            pid_R.target = 50;
                        }
                        else if (action == LEFT)
                        {
                            pid_L.target = 50;
                            pid_R.target = 100;
                        }
                        else if (action == STRAIGHT)
                        {
                            pid_L.target = BASE_SPEED;
                            pid_R.target = BASE_SPEED;
                        }
                        turn_timer--;
                    }
                    else // 转弯计时结束
                    {
                        // --- 特殊处理：倒数第二步转弯后停车 (S=17) ---
                        if (current_step == 17)
                        {
                            current_status = STATUS_STOP_CAR;
                            turn_timer = 50; // 继续循迹减速停车，约2秒后完全停止
                            current_step++;
                        }
                        else
                        {
                            current_status = STATUS_TRACKING;

                            // 步骤 13 转弯后进入 S=14，需要特殊处理
                            if (current_step == 14)
                            {
                                junction_cooldown = 20;  // 冷却从100改为20，避免错过Y字路口
                                center_error = 0;         // 重置误差
                                last_center_error = 0;
                            }
                            else
                            {
                                // 其他步骤转弯后设置最小冷却时间
                                if (junction_cooldown < 20)
                                    junction_cooldown = 20;
                            }
                        }
                    } // 转弯计时结束 else 块
                    break;
                }
//                case STATUS_TURNING:
//                                {
//                                    Turn action = route_plan[current_step - 1].turn;
//                                    if (turn_timer > 0)
//                                    {
//                                        if (action == RIGHT) { pid_L.target = 100; pid_R.target = 50; }
//                                        else if (action == LEFT) { pid_L.target = 50; pid_R.target = 100; }
//                                        else if (action == STRAIGHT)
//                                        {
//                                            pid_L.target = BASE_SPEED;
//                                            pid_R.target = BASE_SPEED;
//                                        }
//                                        turn_timer--;
//                                    }
//                                    else // 转弯计时结束
//                                    {
//                                        current_status = STATUS_TRACKING;
//
//                                        // 步骤 13 转弯后进入 S=14，需要特殊处理
//                                        if (current_step == 14)
//                                        {
//                                            junction_cooldown = 20;  // 冷却从 100改为20 避免错过 Y 字路口
//                                            center_error = 0;         // 重置中心误差
//                                            last_center_error = 0;
//                                        }
//                                        else
//                                        {
//                                            // 其他步骤转弯后设置最小冷却时间
//                                            if (junction_cooldown < 20) junction_cooldown = 20;
//                                        }
//                                    } // 转弯计时结束 else 块
//                                    break;
//                                }

                case STATUS_STOP_CAR :
                {
                    if (turn_timer > 0)
                    {
                        // 减速停车期间仍循迹，利用 center_error 在 switch 外计算过的值避免逻辑偏差
                        float error_diff = center_error - last_center_error;
                        float pd_out = KP * center_error + KD * error_diff;

                        pid_L.target = (float) (BASE_SPEED + pd_out);
                        pid_R.target = (float) (BASE_SPEED - pd_out);

                        last_center_error = center_error;
                        turn_timer--;
                    }

                    else
                    {
                        // 完全停车，目标转速归零
                        pid_L.target = 0;
                        pid_R.target = 0;
                        // 强制输出 0 防止 PID 积分残留导致蠕动
                        motor_output_L = 0;
                        motor_output_R = 0;
                    }
                    break;
                }
            } // 状态机结束

            if (junction_cooldown > 0)
                junction_cooldown--;

            // --- 5. IPS 屏幕显示，每 20 帧刷新一次 ---
            if (loop_count % 20 == 0)
            {
                // 显示当前步骤和状态
                ips114_show_string(0, 0, "Step:");
                ips114_show_int(40, 0, current_step, 2);
                ips114_show_string(80, 0,
                        (current_status == STATUS_TRACKING) ? "TRACK" :
                        (current_status == STATUS_TURNING) ? "TURN " : "STOP ");

                // 显示冷却计时器和转弯计时器
                ips114_show_string(0, 15, "Cool:");
                ips114_show_int(40, 15, junction_cooldown, 3);
                ips114_show_string(80, 15, "Trm:");
                ips114_show_int(110, 15, turn_timer, 3);

                // 显示中心误差和白色像素数
                ips114_show_string(0, 30, "Err:");
                ips114_show_float(40, 30, center_error, 3, 2);
                ips114_show_string(100, 30, "W:");
                ips114_show_int(120, 30, total_white_pixels, 4);

                // 显示路口检测计数：左、上、右
                ips114_show_int(0, 45, debug_left_cnt, 3);
                ips114_show_int(60, 45, debug_top_cnt, 3);
                ips114_show_int(120, 45, debug_right_cnt, 3);

                // 显示下一步路线动作
                if (current_step < sizeof(route_plan) / sizeof(RouteStep))
                {
                    ips114_show_string(0, 60, "Next:");
                    Turn next_act = route_plan[current_step].turn;
                    ips114_show_string(40, 60, (next_act == RIGHT) ? "RIGHT" : (next_act == LEFT) ? "LEFT" : "STR");
                }

                // 显示IMU660RX 姿态数据
                ips114_show_string(0, 75, "P:");
                ips114_show_float(16, 75, (float)imu_pitch, 3, 1);
                ips114_show_string(70, 75, "R:");
                ips114_show_float(86, 75, (float)imu_roll, 3, 1);
                ips114_show_string(0, 90, "Y:");
                ips114_show_float(16, 90, (float)imu_yaw, 4, 1);
            }
        } // 结束 if (mt9v03x_finish_flag_1)
    } // 结束 while (TRUE)
}
