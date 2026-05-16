/*
 * my_image.c
 *
 *  Created on: 2026年2月3日
 *      Author: rue
 */

#include "my_image.h"

uint8 bin_image[ROW][COL];

uint8 otsu_threshold(uint8 *image, uint16 col, uint16 row_start, uint16 row_end)
{
    uint32 histogram[256] = {0}; // 灰度直方图
    uint32 pixel_count = 0;      // 像素总数
    uint32 pixel_sum = 0;        // 灰度总和

    // 统计灰度直方图
    for (uint16 row = row_start; row < row_end; row++)
    {
        for (uint16 col_idx = 0; col_idx < col; col_idx++)
        {
            uint8 gray = image[row * col + col_idx];
            histogram[gray]++;
            pixel_count++;
            pixel_sum += gray;
        }
    }

    // 找到有效灰度范围
    uint8 min_gray = 0, max_gray = 255;
    for (min_gray = 0; min_gray < 256 && histogram[min_gray] == 0; min_gray++)
        ;
    for (max_gray = 255; max_gray > min_gray && histogram[max_gray] == 0; max_gray--)
        ;

    // 边界情况处理
    if (max_gray == min_gray)
        return max_gray;
    if (min_gray + 1 == max_gray)
        return min_gray;

    // 计算最佳阈值
    float max_variance = 0.0f;
    uint8 best_threshold = min_gray;

    uint32 background_count = 0;
    uint32 background_sum = 0;

    for (uint8 threshold = min_gray; threshold < max_gray; threshold++)
    {
        // 累计背景像素
        background_count += histogram[threshold];
        background_sum += histogram[threshold] * threshold;

        // 计算前景像素
        uint32 foreground_count = pixel_count - background_count;
        if (foreground_count == 0)
            continue;

        uint32 foreground_sum = pixel_sum - background_sum;

        // 计算背景和前景的平均灰度
        float background_mean = (float)background_sum / background_count;
        float foreground_mean = (float)foreground_sum / foreground_count;

        // 计算类间方差
        float variance = (float)background_count * foreground_count *
                         (background_mean - foreground_mean) *
                         (background_mean - foreground_mean);

        // 记录最大方差对应的阈值
        if (variance > max_variance)
        {
            max_variance = variance;
            best_threshold = threshold;
        }
    }

    return best_threshold;
}

static uint8 threshold_update_cnt = 0;
static uint8 threshold_far = 128;
uint8 threshold_mid = 128;
static uint8 threshold_near = 128;
void image_binarization(uint8 (*src_image)[COL])
{
    // 每20帧更新一次阈值
    if (++threshold_update_cnt >= 20)
    {
        threshold_update_cnt = 0;

        // 分块计算阈值
        threshold_far = otsu_threshold(src_image[0], COL, BLOCK_FAR_START, BLOCK_FAR_END);
        threshold_mid = otsu_threshold(src_image[0], COL, BLOCK_MID_START, BLOCK_MID_END);
        threshold_near = otsu_threshold(src_image[0], COL, BLOCK_NEAR_START, BLOCK_NEAR_END);

        // 阈值限幅
        if (threshold_far < THRESHOLD_MIN)
            threshold_far = THRESHOLD_MIN;
        if (threshold_far > THRESHOLD_MAX)
            threshold_far = THRESHOLD_MAX;
        if (threshold_mid < THRESHOLD_MIN)
            threshold_mid = THRESHOLD_MIN;
        if (threshold_mid > THRESHOLD_MAX)
            threshold_mid = THRESHOLD_MAX;
        if (threshold_near < THRESHOLD_MIN)
            threshold_near = THRESHOLD_MIN;
        if (threshold_near > THRESHOLD_MAX)
            threshold_near = THRESHOLD_MAX;
    }

    // 分块二值化
    for (uint16 row = 0; row < ROW; row++)
    {
        uint8 threshold;

        // 根据行数选择阈值
        if (row < BLOCK_FAR_END)
        {
            threshold = threshold_far;
        }
        else if (row < BLOCK_MID_END)
        {
            threshold = threshold_mid;
        }
        else
        {
            threshold = threshold_near;
        }

        // 二值化处理
        for (uint16 col = 0; col < COL; col++)
        {
            if (src_image[row][col] > threshold)
            {
                bin_image[row][col] = WHITE;
            }
            else
            {
                bin_image[row][col] = BLACK;
            }
        }
    }
}

//腐蚀算法
void image_filter(uint8 (*bin_image)[COL])
{
    uint32 pixel_sum;

    // 遍历图像
    for (uint8 row = 1; row < ROW - 1; row++)
    {
        for (uint8 col = 1; col < COL - 1; col++)
        {
            // 统计周围8个像素的灰度和
            pixel_sum = bin_image[row - 1][col - 1] + bin_image[row - 1][col] + bin_image[row - 1][col + 1] +
                        bin_image[row][col - 1] + bin_image[row][col + 1] +
                        bin_image[row + 1][col - 1] + bin_image[row + 1][col] + bin_image[row + 1][col + 1];

            // 膨胀操作：当前为黑色，周围白色多 → 变白色
            if (pixel_sum >= DILATE_THRESHOLD && bin_image[row][col] == BLACK)
            {
                bin_image[row][col] = WHITE;
            }

            // 腐蚀操作：当前为白色，周围黑色多 → 变黑色
            if (pixel_sum <= ERODE_THRESHOLD && bin_image[row][col] == WHITE)
            {
                bin_image[row][col] = BLACK;
            }
        }
    }
}

uint8 l_border[ROW];
uint8 r_border[ROW];
uint8 mid_line[ROW];
void search_line(void)
{
    uint8 left_find_flag=0;
    uint8 right_find_flag=0;

    //初始化边界数组
    memset(l_border, -1, sizeof(l_border));
    memset(r_border, -1, sizeof(r_border));
    memset(mid_line, -1, sizeof(mid_line));

    for (int16 row = ROW - 1; row >= 0; row--)
    {
        left_find_flag=0;
        right_find_flag=0;

        //左边线搜索
        for (uint8 col = 0; col < COL - 2; col++)
        {
            if (bin_image[row][col] == BLACK && bin_image[row][col + 1] == WHITE)
            {
                left_find_flag = 1;
                l_border[row] = col;
                break;
            }
        }

        //如果左边线没有搜到，就置左边线为0
        if (!left_find_flag)
        {
            l_border[row] = 0;
        }

        // 右边线搜索
        for (uint8 col = COL - 1; col >= 2; col--)
        {
            if (bin_image[row][col] == BLACK && bin_image[row][col - 1] == WHITE)
            {
                right_find_flag = 1;
                r_border[row] = col;
                break;
            }
        }

        // 如果没有找到右边界，则将右边界置为 COL - 1
        if (!right_find_flag)
        {
            r_border[row] = COL - 1;
        }

        //中线计算
        mid_line[row] = (l_border[row] + r_border[row])/2;
    }
}

//补线函数
void connect_line(uint8 x1, uint8 y1, uint8 x2, uint8 y2)
{
    uint8 i, a1, a2, temp;
    uint8 hx;

    if (x1 > COL - 1)
        x1 = COL - 1;
    else if (x1 <= 0)
        x1 = 0;
    if (y1 > ROW - 1)
        y1 = ROW - 1;
    else if (y1 <= 0)
        y1 = 0;
    if (x2 > COL - 1)
        x2 = COL - 1;
    else if (x2 <= 0)
        x2 = 0;
    if (y2 > ROW - 1)
        y2 = ROW - 1;
    else if (y2 <= 0)
        y2 = 0;

    a1 = y1;
    a2 = y2;

    if (a1 > a2)
    {
        temp = a1;
        a1 = a2;
        a2 = temp;
    }

    for (i = a1; i <= a2; i++)
    {
        hx = (i - y1) * (x2 - x1) / (y2 - y1) + x1;
        if (hx >= COL)
            hx = COL - 1;
        else if (hx <= 0)
            hx = 0;
        mid_line[i] = hx;
//        l_border[i] = hx;
//        r_border[i] = hx;
    }
}

uint8 mid_left_flag = 0;
uint8 mid_right_flag = 0;
uint8 mid_rode_cont=0;
//防止二极管误判为节点
uint8 diode_tube_misjudgement(void)
{
        for(uint8 i=ROW-2;i>1;i--)
        {
            for(uint8 j=COL/2;j>COL/2-5;j--)
            {
                if(bin_image[i][j]==BLACK && bin_image[i-1][j]==WHITE && bin_image[i-2][j]==WHITE)
                {
                    mid_left_flag = 1;
                }
                else
                {
                    mid_left_flag = 0;
                }
            }
            for(uint8 j=COL/2;j<COL/2+5;j--)
            {
                if(bin_image[i][j]==BLACK && bin_image[i-1][j]==WHITE && bin_image[i-2][j]==WHITE)
                {
                    mid_right_flag = 1;
                }
                else
                {
                    mid_right_flag = 0;
                }
            }

            if(mid_left_flag && mid_right_flag) //如果中线左右两侧都有向上的跳变点，则判断中线丢线情况
            {
                if(mid_line[ROW] !=0)
                {
                    mid_rode_cont++;
                }
            }
        }
        if(mid_rode_cont > 80)
        {
            return 1;
        }
        else
        {
            return 0;
        }
}

uint8 l_down_jiaodian;
uint8 r_down_jiaodian;
void find_down_point(uint8 start, uint8 end)
{
    uint8 i, t;
    l_down_jiaodian = 0;
    r_down_jiaodian = 0;

    if (start < end)
    {
        t = start;
        start = end;
        end = t;
    }

    if (start >= MT9V03X_1_H - 1 - 5)
        start = MT9V03X_1_H - 1 - 5;
    if (end <= 5)
        end = 5;

    for (i = start; i >= end; i--)
    {
        // 检测左角点
        if (l_down_jiaodian == 0 && // 只找第一个符合条件的点
            abs(l_border[i] - l_border[i + 1]) <= 5 &&
            abs(l_border[i + 1] - l_border[i + 2]) <= 5 &&
            abs(l_border[i + 2] - l_border[i + 3]) <= 5 &&
            abs(l_border[i] - l_border[i - 2]) >= 35 &&
            abs(l_border[i] - l_border[i - 3]) >= 35 &&
            abs(l_border[i] - l_border[i - 4]) >= 35)
        {
            l_down_jiaodian = i;
        }

        // 检测右角点
        if (r_down_jiaodian == 0 && // 只找第一个符合条件的点
            abs(r_border[i] - r_border[i + 1]) <= 5 &&
            abs(r_border[i + 1] - r_border[i + 2]) <= 5 &&
            abs(r_border[i + 2] - r_border[i + 3]) <= 5 &&
            abs(r_border[i] - r_border[i - 2]) >= 35 &&
            abs(r_border[i] - r_border[i - 3]) >= 35 &&
            abs(r_border[i] - r_border[i - 4]) >= 35)
        {
            r_down_jiaodian = i;
        }

        if (l_down_jiaodian != 0 && r_down_jiaodian != 0)
        {
            break;
        }
    }
}

uint8 l_up_jiaodian;
uint8 r_up_jiaodian;
void find_up_point(uint8 start, uint8 end)
{
    uint8 i, t;
    l_up_jiaodian = 0;
    r_up_jiaodian = 0;

    if (start < end)
    {
        t = start;
        start = end;
        end = t;
    }

    if (end <= 5)
        end = 5;
    if (start >= MT9V03X_1_H - 1 - 5)
        start = MT9V03X_1_H - 1 - 5;

    for (i = start; i >= end; i--)
    {
        // 检测左角点
        if (l_up_jiaodian == 0 &&   // 只找第一个符合条件的点
            abs(l_border[i] - l_border[i - 1]) <= 5 &&
            abs(l_border[i - 1] - l_border[i - 2]) <= 5 &&
            abs(l_border[i - 2] - l_border[i - 3]) <= 5 &&
            (l_border[i] - l_border[i + 2]) >= 35 &&
            (l_border[i] - l_border[i + 3]) >= 35 &&
            (l_border[i] - l_border[i + 4]) >= 35 && diode_tube_misjudgement()==0)
        {
            l_up_jiaodian = i;
        }

        // 检测右角点
        if (r_up_jiaodian == 0 &&   // 只找第一个符合条件的点
            abs(r_border[i] - r_border[i - 1]) <= 5 &&
            abs(r_border[i - 1] - r_border[i - 2]) <= 5 &&
            abs(r_border[i - 2] - r_border[i - 3]) <= 5 &&
            (r_border[i] - r_border[i + 2]) <= -35 &&
            (r_border[i] - r_border[i + 3]) <= -35 &&
            (r_border[i] - r_border[i + 4]) <= -35 && diode_tube_misjudgement()==0)
        {
            r_up_jiaodian = i;
        }

        if (l_up_jiaodian != 0 && r_up_jiaodian != 0)
        {
            break;
        }
    }
}


NodeState node_state = NODE_NONE;
TurnDirection turn_direction = TURN_NONE;
PathInfo path_info = {0};
uint8 node_count = 0;

void node_init(void)
{
    node_state = NODE_NONE;
    turn_direction = TURN_NONE;
    memset(&path_info, 0, sizeof(PathInfo));
    node_count = 0;
}

//返回角点是否存在
uint8 check_corner_detected(void)
{
    // 如果左角点或右角点被检测到，返回1
    if (l_down_jiaodian != 0 || r_down_jiaodian != 0)
    {
        return 1;
    }

    return 0;
}

//检测矩形框内是否有路及个数
uint8 count_paths_on_rect(uint8 (*bin_image)[COL])
{
    // 清空路径信息
    memset(&path_info, 0, sizeof(PathInfo));

    // 检测上边（row=9，col从14到79）
    for (uint8 col = RECT_LEFT; col < RECT_RIGHT - 2; col++)
    {
        // 黑-白-白跳变，说明有白线
        if (bin_image[RECT_TOP][col] == BLACK &&
            bin_image[RECT_TOP][col + 1] == WHITE &&
            bin_image[RECT_TOP][col + 2] == WHITE)
        {
            path_info.top_exist = 1;
            path_info.count++;
            break; // 找到一条就够了
        }
    }

    // 检测下边（row=50，col从14到79）
    for (uint8 col = RECT_LEFT; col < RECT_RIGHT - 2; col++)
    {
        if (bin_image[RECT_BOTTOM][col] == BLACK &&
            bin_image[RECT_BOTTOM][col + 1] == WHITE &&
            bin_image[RECT_BOTTOM][col + 2] == WHITE)
        {
            path_info.bottom_exist = 1;
            path_info.count++;
            break;
        }
    }

    // 检测左边（col=14，row从9到50）
    for (uint8 row = RECT_TOP; row < RECT_BOTTOM - 2; row++)
    {
        if (bin_image[row][RECT_LEFT] == BLACK &&
            bin_image[row + 1][RECT_LEFT] == WHITE &&
            bin_image[row + 2][RECT_LEFT] == WHITE)
        {
            path_info.left_exist = 1;
            path_info.count++;
            break;
        }
    }

    // 检测右边（col=79，row从9到50）
    for (uint8 row = RECT_TOP; row < RECT_BOTTOM - 2; row++)
    {
        if (bin_image[row][RECT_RIGHT] == BLACK &&
            bin_image[row + 1][RECT_RIGHT] == WHITE &&
            bin_image[row + 2][RECT_RIGHT] == WHITE)
        {
            path_info.right_exist = 1;
            path_info.count++;
            break;
        }
    }

    return path_info.count;
}


void draw_rect_box(void)
{
    // 绘制固定矩形框
    ips200_draw_line(RECT_LEFT, RECT_TOP, RECT_RIGHT, RECT_TOP, RGB565_RED);        // 上边
    ips200_draw_line(RECT_LEFT, RECT_TOP, RECT_LEFT, RECT_BOTTOM, RGB565_RED);      // 左边
    ips200_draw_line(RECT_RIGHT, RECT_TOP, RECT_RIGHT, RECT_BOTTOM, RGB565_RED);    // 右边
    ips200_draw_line(RECT_LEFT, RECT_BOTTOM, RECT_RIGHT, RECT_BOTTOM, RGB565_RED);  // 下边
}


// ============ 多路径规划切换系统 ============
#define MAX_PATH_COUNT 7    // 最多支持7条路径
#define ARRAY_LEN(a) ((uint8)(sizeof(a) / sizeof((a)[0])))

typedef struct
{
    const TurnDirection *turns; // 指向路径数组
    uint8 length;               // 路径长度
} PathPlan;

// ============ 初赛路线 ============

// 初赛路线1
static const TurnDirection prelims_1[] =
{
    TURN_RIGHT,     // 路口1
    TURN_RIGHT,     // 路口2
    TURN_RIGHT,     // 路口3
    TURN_RIGHT,      // 路口4
    GO_STRAIGHT,
};

// 初赛路线2
static const TurnDirection prelims_2[] =
{
    TURN_RIGHT,     // 路口1
    TURN_RIGHT,     // 路口2
    TURN_LEFT,      // 路口3
    TURN_LEFT,      // 路口4
    TURN_RIGHT,     // 路口5
    TURN_LEFT,      // 路口6
    TURN_LEFT,      // 路口7
    TURN_LEFT,      // 路口8
    GO_STRAIGHT,    // 路口9
    TURN_RIGHT,     // 路口10
    GO_STRAIGHT,    // 路口11
    TURN_RIGHT,     // 路口12
};

// 初赛路线3
static const TurnDirection prelims_3[] =
{
    TURN_RIGHT,     // 路口1
    TURN_RIGHT,     // 路口2
    TURN_LEFT,      // 路口3
    TURN_LEFT,      // 路口4
    GO_STRAIGHT,    // 路口5
    TURN_RIGHT,     // 路口6
    TURN_RIGHT,     // 路口7
    TURN_RIGHT,     // 路口8
    TURN_RIGHT,     // 路口9
    TURN_LEFT,      // 路口10
    TURN_LEFT,      // 路口11
    TURN_RIGHT,     // 路口12
    TURN_RIGHT,     // 路口13
};

// ============ 决赛路线 ============

// 决赛路线1
static const TurnDirection final_1[] =
{
    TURN_RIGHT,     // 路口1
    GO_STRAIGHT,    // 路口2
    TURN_RIGHT,     // 路口3
    TURN_RIGHT,     // 路口4
    TURN_RIGHT,     // 路口5
};

// 决赛路线2
static const TurnDirection final_2[] =
{
    TURN_LEFT,      // 路口1
    GO_STRAIGHT,    // 路口2
    TURN_RIGHT,     // 路口3
};

// 决赛路线3
static const TurnDirection final_3[] =
{
    TURN_LEFT,     // 路口1
    TURN_LEFT,      // 路口2
    GO_STRAIGHT,    // 路口3
};

// ============ 备用路线 ============

static const TurnDirection backup[] =
{
        TURN_RIGHT,     // 路口1
        TURN_RIGHT,     // 路口2
        TURN_LEFT,      // 路口3
        TURN_LEFT,      // 路口4
    TURN_RIGHT,     // 路口5
    TURN_RIGHT,     // 路口6
    TURN_RIGHT,     // 路口7
    TURN_LEFT,      // 路口8
    TURN_LEFT,      // 路口9
    TURN_LEFT,      // 路口10
    GO_STRAIGHT,    // 路口11  /
    TURN_RIGHT,     // 路口12
    TURN_RIGHT,
    GO_STRAIGHT,     // 路口13
    TURN_RIGHT,     // 路口14
    GO_STRAIGHT,    // 路口15
    GO_STRAIGHT,    // 路口16
    TURN_RIGHT,     // 路口17
    GO_STRAIGHT,
};

// 统一路径表
static const PathPlan g_paths[MAX_PATH_COUNT] =
{
    { prelims_1, ARRAY_LEN(prelims_1) },  // 0 = 初赛路线1
    { prelims_2, ARRAY_LEN(prelims_2) },  // 1 = 初赛路线2
    { prelims_3, ARRAY_LEN(prelims_3) },  // 2 = 初赛路线3
    { final_1,   ARRAY_LEN(final_1)   },  // 3 = 决赛路线1
    { final_2,   ARRAY_LEN(final_2)   },  // 4 = 决赛路线2
    { final_3,   ARRAY_LEN(final_3)   },  // 5 = 决赛路线3
    { backup,    ARRAY_LEN(backup)    },  // 6 = 备用路线
};

// 当前使用的路径
uint8 menu_path_index = 6;
static const PathPlan *current_plan = &g_paths[6];
static uint8 current_path_index = 6;

void switch_path(uint8 path_index)
{
    if (path_index < MAX_PATH_COUNT &&
        g_paths[path_index].turns != NULL &&
        g_paths[path_index].length > 0)
    {
        current_plan = &g_paths[path_index];
        current_path_index = path_index;

        // 重置节点计数器，从头开始
        node_count = 0;
        node_state = NODE_NONE;
        turn_direction = TURN_NONE;
    }
}

//获取当前路径索引
uint8 get_current_path_index(void)
{
    return current_path_index;
}

//重置路径
void reset_path(void)
{
    node_count = 0;
    node_state = NODE_NONE;
    turn_direction = TURN_NONE;
}

/**
 * @brief 检查菜单路径索引是否改变，并执行切换
 * @note 在主循环或定时调用
 */
void check_path(void)
{
    static uint8 last_menu_path_index = 0xFF;  // 初始化为无效值

    // 检测路径索引是否改变
    if(menu_path_index != last_menu_path_index)
    {
        // 限制范围 0-6
        if(menu_path_index > 6)
            menu_path_index = 6;

        // 执行路径切换
        switch_path(menu_path_index);

        // 更新记录
        last_menu_path_index = menu_path_index;
    }
}

//返回转向路径
TurnDirection decide_turn_direction(void)
{
    if (current_plan != NULL &&
        current_plan->turns != NULL &&
        current_plan->length > 0)
    {
        // 使用取模运算实现循环
        uint8 index = node_count % current_plan->length;
        TurnDirection planned_turn = current_plan->turns[index];

        // 验证路径是否存在
        if (planned_turn == TURN_LEFT && path_info.left_exist)
            return TURN_LEFT;
        if (planned_turn == TURN_RIGHT && path_info.right_exist)
            return TURN_RIGHT;
        if (planned_turn == GO_STRAIGHT && path_info.top_exist)
            return GO_STRAIGHT;

        return planned_turn;
    }

    return TURN_NONE;
}

/**
 * @brief 直角转向决策（不使用路径规划）
 * @return 转向方向
 *
 * 直角只有一个出口，根据 path_info 判断：
 * - 左直角（只有左路）→ 左转
 * - 右直角（只有右路）→ 右转
 */
TurnDirection decide_turn_for_corner(void)
{
    // 如果检测到了左下角点，说明是左直角
    if (l_down_jiaodian != 0 && r_down_jiaodian == 0)
    {
        return TURN_LEFT;
    }

    // 如果检测到了右下角点，说明是右直角
    if (r_down_jiaodian != 0 && l_down_jiaodian == 0)
    {
        return TURN_RIGHT;
    }

    // 如果两边都检测到，使用 path_info 辅助判断
    if (path_info.right_exist && !path_info.left_exist)
        return TURN_RIGHT;
    if (path_info.left_exist && !path_info.right_exist)
        return TURN_LEFT;

    return TURN_NONE;
}

/**
 * @brief 在指定列上检测赛道的上边界
 * @param col 要检测的列
 * @param bin_image 二值图像
 * @return 赛道上边界的行号（0表示未找到）
 *
 * 检测逻辑：
 * 1. 从下往上扫描
 * 2. 找到 黑→白 跳变（进入赛道）
 * 3. 继续找 白→黑 跳变（离开赛道）
 * 4. 返回离开赛道的位置（上边界）
 */
uint8 find_track_top_at_col(uint8 col, uint8 (*bin_image)[COL])
{
    uint8 in_track = 0;     // 是否在赛道内
    uint8 track_top = 0;    // 赛道上边界

    if (bin_image[ROW - 1][col] == WHITE && bin_image[ROW - 2][col] == WHITE)
    {
        in_track = 1;  // 直接标记为在赛道内
    }

    // 从下往上扫描（从 ROW-1 到 2）
    for (int16 row = ROW - 1; row >= 2; row--)
    {
        // 检测赛道下边界（进入赛道）：黑→白→白
        if (!in_track)
        {
            if (bin_image[row][col] == BLACK &&
                bin_image[row - 1][col] == WHITE &&
                bin_image[row - 2][col] == WHITE)
            {
                in_track = 1;
            }
        }
        // 检测赛道上边界（离开赛道）：白→白→黑
        else
        {
            if (bin_image[row][col] == WHITE &&
                bin_image[row - 1][col] == WHITE &&
                bin_image[row - 2][col] == BLACK)
            {
                track_top = row - 1;
                break;
            }
        }
    }

    return track_top;
}

/**
 * @brief 在指定列上查找赛道下边界
 * @param col 列号（0-93）
 * @param bin_image 二值图像
 * @return 赛道下边界的行号，未找到返回0
 *
 * 扫描逻辑：
 * 1. 从下往上扫描
 * 2. 找到 黑→白 跳变（进入赛道）
 * 3. 返回进入赛道的位置（下边界）
 */
uint8 find_track_bottom_at_col(uint8 col, uint8 (*bin_image)[COL])
{
    uint8 track_bottom = 0;

    // 如果最底部已经是白色，直接返回 ROW-1
    if (bin_image[ROW - 1][col] == WHITE && bin_image[ROW - 2][col] == WHITE)
    {
        return ROW - 1;
    }

    // 从下往上扫描（从 ROW-1 到 2）
    for (uint8 row = ROW - 1; row >= 2; row--)
    {
        // 检测赛道下边界（进入赛道）：黑→白→白
        if (bin_image[row][col] == BLACK &&
            bin_image[row - 1][col] == WHITE &&
            bin_image[row - 2][col] == WHITE)
        {
            track_bottom = row;  // 返回下边界位置
            break;
        }
    }

    return track_bottom;
}


/**
 * @brief 节点处理函数
 * @param bin_image 二值图像
 *
 * 说明：
 * 这个函数整合了节点识别的完整流程
 */

void node_proc(uint8 (*bin_image)[COL])
{
    static float yaw_target = 0.0f;
    static uint8 angle_locked = 0;
    static uint8 current_path_count = 0;

    switch (node_state)
    {
        case NODE_NONE:
            find_down_point(ROW - 5, 5); //搜索角点 5

            if (check_corner_detected())   //检测到左角点或者右角点
            {
                uint8 path_count = count_paths_on_rect(bin_image); //找上下左右共有几条路

                // 角点存在就处理，不再依赖 path_count，转向方向由角点决定
                turn_direction = decide_turn_direction(); //转向方向

                if (turn_direction != TURN_NONE)
                {

                    // 保底设为2，确保后续处理正常
                    current_path_count = (path_count >= 2) ? path_count : 2;
                    node_state = NODE_DETECTED;
                    node_count++;
                }
            }
            break;

        case NODE_DETECTED:
            if (!angle_locked)
            {
                if (turn_direction == TURN_LEFT)
                {
                    yaw_target = yaw_total + 65.0f;
                }
                else if (turn_direction == TURN_RIGHT)
                {
                    yaw_target = yaw_total - 65.0f;
                }
                else if (turn_direction == GO_STRAIGHT)
                {
                    yaw_target = yaw_total;
                }
                angle_locked = 1;
                node_state = NODE_TURNING;
            }
            break;

        case NODE_TURNING:
            find_down_point(ROW - 5, 5);

            // ========== 直行优先处理（不管path_count是多少）==========
            if (turn_direction == GO_STRAIGHT)
            {
                // 直行中线动态偏置
                static uint8 corner_lost = 0;  // 角点是否已消失

                // 获取角点行（取左右角点中较大的那个）
                uint8 corner_row = 0;
                if (l_down_jiaodian != 0 && r_down_jiaodian != 0)
                    corner_row = (l_down_jiaodian > r_down_jiaodian) ? l_down_jiaodian : r_down_jiaodian;
                else if (l_down_jiaodian != 0)
                    corner_row = l_down_jiaodian;
                else if (r_down_jiaodian != 0)
                    corner_row = r_down_jiaodian;

                if (corner_row != 0 && !corner_lost)
                {
                    // 角点存在：角点上方偏置到中间，下方保持原始中线
                    connect_line(COL / 2, corner_row, COL / 2, 0);
                }
                else
                {
                    // 角点消失：全局偏置，整条中线锁在正中间
                    corner_lost = 1;
                    connect_line(COL / 2, ROW - 1, COL / 2, 0);
                }

                // 使用距离判断退出条件
                static int16 straight_start_dist = 0.0f;
                static int16 straight_started = 0;

                // 首次进入直行状态，记录起始距离
                if (!straight_started)
                {
                    straight_start_dist = total_dist;
                    straight_started = 1;
                    corner_lost = 0;  // 重置角点消失标志
                }

                // 计算已行驶距离
                float traveled_dist = total_dist - straight_start_dist;

                // 退出条件：行驶距离 > 阈值 且 路径数 < 3
                if (traveled_dist > 4000)
                {
                    uint8 path_count = count_paths_on_rect(bin_image);

                    if (path_count < 3)
                    {
                        //刷新数据
                        node_state = NODE_NONE;
                        turn_direction = TURN_NONE;
                        angle_locked = 0;
                        current_path_count = 0;
                        straight_started = 0;
                        corner_lost = 0;
                    }
                }
            }
            // ========== 直角处理（path_count == 2）==========
            else if (current_path_count == 2)
            {
                if (turn_direction == TURN_LEFT)
                {
                    uint8 corner_row = l_down_jiaodian;

                    if (corner_row != 0)
                    {
                        // 角点存在
                        // 段1：从角点到顶部（垂直偏置）
                        connect_line(1, corner_row, 1, 0);
                    }
                    else
                    {
                        // 角点消失：使用动态偏置（起点为赛道下边界）
                        uint8 track_bottom = find_track_bottom_at_col(0, bin_image);

                        if (track_bottom != 0)
                        {
                            // 从赛道下边界到顶部（垂直偏置）
                            connect_line(1, track_bottom, 1, 0);
                        }
                        else
                        {
                            // 未检测到赛道，全局偏置
                            connect_line(1, ROW - 1, 1, 0);
                        }
                    }

                    if (yaw_total >= yaw_target)
                    {
                        node_state = NODE_NONE;
                        turn_direction = TURN_NONE;
                        angle_locked = 0;
                        current_path_count = 0;
                    }
                }
                else if (turn_direction == TURN_RIGHT)
                {
                    uint8 corner_row = r_down_jiaodian;

                    if (corner_row != 0)
                    {
                        // 角点存在
                        connect_line(COL - 1, corner_row, COL - 1, 0);
                    }
                    else
                    {
                        uint8 track_bottom = find_track_bottom_at_col(COL - 2, bin_image);

                        if (track_bottom != 0)
                        {
                            connect_line(COL - 1, track_bottom, COL - 1, 0);
                        }
                        else
                        {
                            connect_line(COL - 1, ROW - 1, COL - 1, 0);
                        }
                    }

                    if (yaw_total <= yaw_target)
                    {
                        node_state = NODE_NONE;
                        turn_direction = TURN_NONE;
                        angle_locked = 0;
                        current_path_count = 0;
                    }
                }
            }
            // ========== 路口处理（path_count >= 3）==========
            else if (current_path_count >= 3)
            {
                if (turn_direction == TURN_LEFT)
                {
                    uint8 corner_row = l_down_jiaodian;

                    if (corner_row != 0)
                    {
                        connect_line(1, corner_row, 1, 0);
                    }
                    else
                    {
                        uint8 track_bottom = find_track_bottom_at_col(0, bin_image);
                        if (track_bottom != 0)
                            connect_line(1, track_bottom, 1, 0);
                        else
                            connect_line(1, ROW - 1, 1, 0);
                    }

                    if (yaw_total >= yaw_target)
                    {
                        node_state = NODE_NONE;
                        turn_direction = TURN_NONE;
                        angle_locked = 0;
                        current_path_count = 0;
                    }
                }
                else if (turn_direction == TURN_RIGHT)
                {
                    uint8 corner_row = r_down_jiaodian;

                    if (corner_row != 0)
                    {
                        connect_line(COL - 1, corner_row, COL - 1, 0);
                    }
                    else
                    {
                        uint8 track_bottom = find_track_bottom_at_col(COL - 2, bin_image);
                        if (track_bottom != 0)
                            connect_line(COL - 1, track_bottom, COL - 1, 0);
                        else
                            connect_line(COL - 1, ROW - 1, COL - 1, 0);
                    }

                    if (yaw_total <= yaw_target)
                    {
                        node_state = NODE_NONE;
                        turn_direction = TURN_NONE;
                        angle_locked = 0;
                        current_path_count = 0;
                    }
                }
            }
            else if (turn_direction == GO_STRAIGHT)
            {
                // 直行中线动态偏置
                static uint8 corner_lost = 0;  // 角点是否已消失

                // 获取角点行（取左右角点中较大的那个）
                uint8 corner_row = 0;
                if (l_down_jiaodian != 0 && r_down_jiaodian != 0)
                    corner_row = (l_down_jiaodian > r_down_jiaodian) ? l_down_jiaodian : r_down_jiaodian;
                else if (l_down_jiaodian != 0)
                    corner_row = l_down_jiaodian;
                else if (r_down_jiaodian != 0)
                    corner_row = r_down_jiaodian;

                if (corner_row != 0 && !corner_lost)
                {
                    // 角点存在：角点上方偏置到中间，下方保持原始中线
                    connect_line(COL / 2, corner_row, COL / 2, 0);
                }
                else
                {
                    // 角点消失：全局偏置，整条中线锁在正中间
                    corner_lost = 1;
                    connect_line(COL / 2, ROW - 1, COL / 2, 0);
                }

                // 使用距离判断退出条件
                static int16 straight_start_dist = 0.0f;
                static int16 straight_started = 0;

                // 首次进入直行状态，记录起始距离
                if (!straight_started)
                {
                    straight_start_dist = total_dist;
                    straight_started = 1;
                    corner_lost = 0;  // 重置角点消失标志
                }

                // 计算已行驶距离
                float traveled_dist = total_dist - straight_start_dist;

                // 退出条件：行驶距离 > 阈值 且 路径数 < 3
                if (traveled_dist > 4000)
                {
                    uint8 path_count = count_paths_on_rect(bin_image);

                    if (path_count < 3)
                    {
                        node_state = NODE_NONE;
                        turn_direction = TURN_NONE;
                        angle_locked = 0;
                        current_path_count = 0;
                        straight_started = 0;
                        corner_lost = 0;
                    }
                }
            }
            break;

        default:
            break;
    }
}


static const float image_err_weight[ROW] =
{
    // 远区（0-20行）：权重较小
    0.0, 0.0, 0.0, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1,
    1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.0,
    // 中区（20-40行）：权重最大
    2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
    2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
    // 近区（40-60行）：权重适中
    1.8, 1.7, 1.6, 1.5, 1.4, 1.3, 1.2, 1.1, 1.0, 0.9,
    0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.0, 0.0
};

float image_err = 0.0f;

void calc_image_error(void)
{
    float err = 0;
    float err_sum = 0;
    float weight_sum = 0;
    static float err_last;

    if(mid_line!=0)
    {
        for (uint8 i = 0; i < ROW; i++)
        {
            err_sum += ((mid_line[i]) - COL / 2) * image_err_weight[i];
            weight_sum += image_err_weight[i];
        }
    }


    err = err_sum / weight_sum;
    err = err * 0.9f + err_last * 0.1f;
    err_last = err;

    if (err >= 50)
        err = 50;
    if (err <= -50)
        err = -50;

    image_err = err;
}


void drawkline(void)
{
  for(uint8 i = MT9V03X_1_H-1 ; i > 1 ; i--)
  {
    if(l_border[i]>MT9V03X_1_W-1) l_border[i]= MT9V03X_1_W-1;
    else if(l_border[i]<0) l_border[i] = 0;
    ips114_draw_point((int16)l_border[i] , i , RGB565_YELLOW);

    if(r_border[i]>MT9V03X_1_W-1) r_border[i]= MT9V03X_1_W-1;
    else if(r_border[i]<0) r_border[i] = 0;
    ips114_draw_point((int16)r_border[i] , i , RGB565_RED);

    if(mid_line[i]>MT9V03X_1_W-1) mid_line[i]= MT9V03X_1_W-1;
    else if(mid_line[i]<0) mid_line[i] = 0;
    ips114_draw_point((int16)mid_line[i] , i , RGB565_GREEN);
  }
}
