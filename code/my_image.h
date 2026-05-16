/*
 * my_image.h
 *
 *  Created on: 2026年2月3日
 *      Author: rue
 */

#ifndef CODE_MY_IMAGE_H_
#define CODE_MY_IMAGE_H_

#include "zf_common_headfile.h"

#define COL 94
#define ROW 60

#define BLACK 0
#define WHITE 255

#define BLOCK_FAR_START     0
#define BLOCK_FAR_END       20
#define BLOCK_MID_START     20
#define BLOCK_MID_END       40
#define BLOCK_NEAR_START    40
#define BLOCK_NEAR_END      60

#define THRESHOLD_MIN       70
#define THRESHOLD_MAX       150

#define DILATE_THRESHOLD    (255 * 5)
#define ERODE_THRESHOLD     (255 * 2)

// 节点状态
typedef enum
{
    NODE_NONE = 0,        // 无节点
    NODE_DETECTED,        // 检测到节点
    NODE_TURNING          // 正在转向
} NodeState;

// 转向方向
typedef enum
{
    TURN_NONE = 0,      // 无操作
    TURN_LEFT,          // 左转
    TURN_RIGHT,         // 右转
    GO_STRAIGHT         // 直行
} TurnDirection;

// 路径信息
typedef struct
{
    uint8 count;        // 路径数量
    uint8 top_exist;    // 上方有路
    uint8 bottom_exist; // 下方有路
    uint8 left_exist;   // 左侧有路
    uint8 right_exist;  // 右侧有路
} PathInfo;

// 矩形框坐标
#define RECT_LEFT   14   //14
#define RECT_RIGHT  79  //79
#define RECT_TOP    9    //9
#define RECT_BOTTOM 50   //50


uint8 otsu_threshold(uint8 *image, uint16 col, uint16 row_start, uint16 row_end);
void image_binarization(uint8 (*src_image)[COL]);
void image_filter(uint8 (*bin_image)[COL]);
void search_line(void);
void connect_line(uint8 x1, uint8 y1, uint8 x2, uint8 y2);
void find_down_point(uint8 start, uint8 end);
void find_up_point(uint8 start, uint8 end);
void node_init(void);
uint8 check_corner_detected(void);
uint8 count_paths_on_rect(uint8 (*bin_image)[COL]);
void draw_rect_box(void);
void node_proc(uint8 (*bin_image)[COL]);
void calc_image_error(void);
void drawkline(void);

TurnDirection decide_turn_for_corner(void);


extern uint8 bin_image[ROW][COL];
extern float image_err;
extern NodeState node_state;
extern uint8 threshold_mid;
extern int16 traveled_dist;

#endif /* CODE_MY_IMAGE_H_ */



