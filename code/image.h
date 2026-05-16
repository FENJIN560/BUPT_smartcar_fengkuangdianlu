#ifndef IMAGE_H
#define IMAGE_H

#include "zf_common_headfile.h"
#include "turn.h"  // 必须包含此头文件，否则编译器不知道 Junction 是什么类型

#define IMAGE_WIDTH  188
#define IMAGE_HEIGHT 120

#define white_pixel 255
#define black_pixel 0

#define border_max (IMAGE_WIDTH - 2)
#define border_min 1

// 变量外部引用
extern uint8 original_image[IMAGE_HEIGHT][IMAGE_WIDTH];
extern uint8 bin_image[IMAGE_HEIGHT][IMAGE_WIDTH];
extern uint8 l_border[IMAGE_HEIGHT];
extern uint8 r_border[IMAGE_HEIGHT];
extern uint8 center_line[IMAGE_HEIGHT];

// 函数原型
void IMAGE_Init(void);
void IMAGE_CaptureFrame(void);
void IMAGE_DisplayFrame(void);
void image_process(void);
void image_filter(uint8 (*bin_image)[IMAGE_WIDTH]);

// --- 关键补充：在这里声明 detect_junction ---
// 否则 main.c 调用它时会报 W577 (missing prototype)
Junction detect_junction(uint8 (*image)[IMAGE_WIDTH], int scan_row);

#endif
