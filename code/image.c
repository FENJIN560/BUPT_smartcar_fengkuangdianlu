#include "image.h"
#include"turn.h"
#include "zf_common_headfile.h"
uint8 original_image[IMAGE_HEIGHT][IMAGE_WIDTH];
uint8 bin_image[IMAGE_HEIGHT][IMAGE_WIDTH];

uint8 l_border[IMAGE_HEIGHT];
uint8 r_border[IMAGE_HEIGHT];
uint8 center_line[IMAGE_HEIGHT];

uint8 image_threshold = 0;

/* ---------- 摄像头 + LCD 初始化 ---------- */
void IMAGE_Init (void)
{
    ips114_init();
    ips114_clear();

    while (1)
    {
        if (mt9v03x_double_init(mt9v03x_1))
        {
            system_delay_ms(100);
        }
        else
        {
            break;
        }
    }

    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, mt9v03x_image_1,
    MT9V03X_1_W,
    MT9V03X_1_H);
}

/* ---------- 等待一帧图像 ---------- */
void IMAGE_CaptureFrame (void)
{
    while (!mt9v03x_finish_flag_1)
        ;
    mt9v03x_finish_flag_1 = 0;
}

/* ---------- LCD显示 ---------- */
void IMAGE_DisplayFrame (void)
{
    ips114_show_gray_image(0, 20, (uint8*) bin_image,
    IMAGE_WIDTH,
    IMAGE_HEIGHT, 240, 115, 64);
}

/* ---------- 灰度拷贝 ---------- */
static void Get_image (void)
{
    for (int y = 0; y < IMAGE_HEIGHT; y++)
        for (int x = 0; x < IMAGE_WIDTH; x++)
            original_image[y][x] = mt9v03x_image_1[y][x];
}

uint8 otsuThreshold (uint8 *image, uint16 col, uint16 row)
{
#define GrayScale 256

    uint16 width = col;
    uint16 height = row;

    uint32 histogram[GrayScale] = {0};

    uint32 pixelCount = width * height;

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            histogram[image[y * width + x]]++;
        }
    }

    float sum = 0;
    for (int i = 0; i < 256; i++)
        sum += i * histogram[i];

    float sumB = 0;
    uint32 wB = 0;
    uint32 wF = 0;

    float maxVar = 0;
    uint8 threshold = 0;

    for (int t = 0; t < 256; t++)
    {
        wB += histogram[t];
        if (wB == 0)
            continue;

        wF = pixelCount - wB;
        if (wF == 0)
            break;

        sumB += (float) (t * histogram[t]);

        float mB = sumB / wB;
        float mF = (sum - sumB) / wF;

        float varBetween = (float) wB * (float) wF * (mB - mF) * (mB - mF);

        if (varBetween > maxVar)
        {
            maxVar = varBetween;
            threshold = t;
        }
    }

    return threshold;
}

static void turn_to_bin (void)
{
    uint8 max_v = 0, min_v = 255;

    // 1. 快速扫描亮度区间
    for (int y = 0; y < IMAGE_HEIGHT; y += 5)
    {
        for (int x = 0; x < IMAGE_WIDTH; x += 5)
        {
            uint8 p = original_image[y][x];
            if (p > max_v)
                max_v = p;
            if (p < min_v)
                min_v = p;
        }
    }
    // 在 turn_to_bin 结尾增加一个标志位，或者让 image_threshold 异常化
    if (max_v < 60 || (max_v - min_v) < 40) {
        image_threshold = 255; // 强制阈值最大，代表全黑
    }

    // 2. 计算大津法阈值
    uint8 otsu_thr = otsuThreshold((uint8*) original_image, IMAGE_WIDTH, IMAGE_HEIGHT);

    // 3. 【关键修改】硬下限保护
    // 假设白线是 180+，反光通常在 130 以下。
    // 我们强制 image_threshold 不能太低。如果反光严重，就把这个值往上提（如 150）。
    image_threshold = otsu_thr;

    // 如果大津法算出来的阈值低于 145，说明它可能被反光带跑了
    if (image_threshold < 100)
    {
        image_threshold = 100;
    }

    // 3. 执行二值化
    for (int y = 0; y < IMAGE_HEIGHT; y++)
    {
        for (int x = 0; x < IMAGE_WIDTH; x++)
        {
            bin_image[y][x] = (original_image[y][x] > image_threshold) ? 255 : 0;
        }
    }
}

/* ---------- 简化边界提取 ---------- */
static void scan_border (void)
{
    for (int y = IMAGE_HEIGHT / 2; y < IMAGE_HEIGHT; y++)
    {
        l_border[y] = 0;
        r_border[y] = IMAGE_WIDTH - 1;

        for (int x = 1; x < IMAGE_WIDTH - 1; x++)
        {
            if (bin_image[y][x] == white_pixel && bin_image[y][x - 1] == black_pixel)
            {
                l_border[y] = x;
                break;
            }
        }

        for (int x = IMAGE_WIDTH - 2; x > 1; x--)
        {
            if (bin_image[y][x] == white_pixel && bin_image[y][x + 1] == black_pixel)
            {
                r_border[y] = x;
                break;
            }
        }

        center_line[y] = (l_border[y] + r_border[y]) >> 1;
    }
}
void image_filter (uint8 (*bin_image)[IMAGE_WIDTH])
{
    uint16 i, j;
    uint32 num = 0;

    for (i = 1; i < IMAGE_HEIGHT - 1; i++)
    {
        for (j = 1; j < IMAGE_WIDTH - 1; j++)
        {
            num = bin_image[i - 1][j - 1] + bin_image[i - 1][j] + bin_image[i - 1][j + 1] + bin_image[i][j - 1]
                    + bin_image[i][j + 1] + bin_image[i + 1][j - 1] + bin_image[i + 1][j] + bin_image[i + 1][j + 1];

            if (num >= 255 * 5 && bin_image[i][j] == 0)
            {
                bin_image[i][j] = 255;
            }

            if (num <= 255 * 2 && bin_image[i][j] == 255)
            {
                bin_image[i][j] = 0;
            }
        }
    }
}

void image_process (void)
{
    Get_image();
    turn_to_bin();
    image_filter(bin_image);

    // 关键修正：y 的上限必须小于 IMAGE_HEIGHT-1 (即 119)
    // 建议扫描区域设为 20 到 110，留出底部安全区
    for (int y = 20; y < 115; y++)
    {
        int left = -1;
        int right = -1;

        // 找左边界：从左往右，确保 x-1 不越界 (x从1开始)
        for (int x = 1; x < IMAGE_WIDTH / 2; x++)
        {
            if (bin_image[y][x] == 255 && bin_image[y][x - 1] == 0)
            {
                left = x;
                break;
            }
        }

        // 找右边界：从右往左，确保 x+1 不越界 (x最大宽度-2)
        for (int x = IMAGE_WIDTH - 2; x > IMAGE_WIDTH / 2; x--)
        {
            if (bin_image[y][x] == 255 && bin_image[y][x + 1] == 0)
            {
                right = x;
                break;
            }
        }

        if (left != -1 && right != -1)
        {
            l_border[y] = left;
            r_border[y] = right;
            center_line[y] = (left + right) / 2;
        }
        else if (left != -1)
        { // 丢右边，盲补右边
            center_line[y] = left + 40;
        }
        else if (right != -1)
        { // 丢左边，盲补左边
            center_line[y] = right - 40;
        }
    }
}

