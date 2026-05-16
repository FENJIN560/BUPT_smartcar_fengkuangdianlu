#ifndef _TURN_H_
#define _TURN_H_

#include "zf_common_headfile.h"

// --- 宏定义 ---
#define MAX_PATH 100            // 最大路径记录节点数

extern int debug_left_cnt, debug_right_cnt, debug_top_cnt;

// 框的边界定义
#define BOX_L 15  // 稍微往里缩，避开摄像头边缘黑区
#define BOX_R 173
#define BOX_T 35   // 关键：决定了你能看多远的路口
#define BOX_B 110  // 靠近车头


typedef enum {
    ROAD_NONE = 0,
    ROAD_T,
    ROAD_Y,
    ROAD_L_LEFT,
    ROAD_L_RIGHT
} Junction;

// 2. 转向动作指令
typedef enum {
    STRAIGHT = 0,               // 直行
    LEFT,                       // 左转
    RIGHT,                      // 右转
    UTURN,                      // 掉头
    STOP                        // 停车
} Turn;

// --- 结构体定义 ---

// 路径节点信息
typedef struct {
    Junction type;              // 经过的路口类型 (此时 Junction 已定义，不会报错)
    Turn turn;                  // 执行的转向动作
} PathNode;

// --- 外部变量声明 ---
extern PathNode path[MAX_PATH];
extern int path_len;

// --- 函数原型声明 ---

/**
 * @brief 初始化转向控制模块
 */
void turn_init(void);

/**
 * @brief 路口检测函数
 * @param image 图像数组指针
 * @param scan_row 扫描行数
 * @return 检测到的路口类型
 */
Junction detect_junction(uint8 (*image)[188], int scan_row);

/**
 * @brief 根据角度变化判定实际转弯类型
 */
Turn record_turn(float angle_before, float angle_after);

/**
 * @brief 修改左右电机 PID 目标值以执行转向动作
 */
void execute_turn_pid(Turn t, int base_speed, float turn_ratio);

/**
 * @brief 向路径数组中存入一个新节点
 */
void add_path_node(Junction type, Turn t);

#endif /* _TURN_H_ */
