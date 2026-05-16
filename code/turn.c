#include "zf_common_headfile.h"
#include "turn.h"

int debug_left_cnt = 0;
int debug_right_cnt = 0;
int debug_top_cnt = 0;

// 触框灵敏度：边框上至少有 6 个像素是白的才算触碰
#define TOUCH_THRESHOLD 10

Junction detect_junction_by_border (uint8 bin[120][188])
{
    int left_cnt = 0, right_cnt = 0, top_cnt = 0, bottom_cnt = 0;

    // 1. 扫描边框触碰像素数
    for (int y = BOX_T; y < BOX_B; y++) {
        if (bin[y][BOX_L] == 255) left_cnt++;
        if (bin[y][BOX_R] == 255) right_cnt++;
    }
    for (int x = BOX_L; x < BOX_R; x++) {
        if (bin[BOX_T][x] == 255) top_cnt++;
        if (bin[BOX_B][x] == 255) bottom_cnt++;
    }

    // 判定布尔值
    bool has_L = (left_cnt >= TOUCH_THRESHOLD);
    bool has_R = (right_cnt >= TOUCH_THRESHOLD);
    bool has_T = (top_cnt >= TOUCH_THRESHOLD);
    bool has_B = (bottom_cnt >= 2); // 底边保护

    debug_left_cnt  = left_cnt;
        debug_right_cnt = right_cnt;
        debug_top_cnt   = top_cnt;

    // --- 核心逻辑区分 ---

    // 【逻辑 1：T/Y 弯优先级最高】
    // 左右同时出线，且底边有线
    if (has_L && has_R) {
        // 如果上方也是封死的，或者上方有空隙（Y），判定为 T/Y
        return ROAD_T;
    }

    // 【逻辑 2：L 弯判定】
    // 核心要求：上侧不触线 (!has_T)
    if (!has_T)
    {
        // A. 左出线，右不触线
        if (has_L && !has_R) {
            return ROAD_L_LEFT;
        }
        // B. 右出线，左不触线
        if (has_R && !has_L) {
            return ROAD_L_RIGHT;
        }
    }

    // 【逻辑 3：兼容原有的前侧 T 字判定】
    // 如果上侧触线了，且只有单边触线，通常是斜着入 T 路口
    if (has_T && (has_L || has_R)) {
        return ROAD_T;
    }

    return ROAD_NONE;
}
