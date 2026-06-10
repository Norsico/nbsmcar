#include "headfile.h"

/* 初始化 */
void image_apply_camera(void)
{
    uint8 i;
    short int config[MT9V03X_CONFIG_FINISH][2];

    if (Image.ready == 0)
    {
        return;
    }

    for (i = 0; i < MT9V03X_CONFIG_FINISH; i++)
    {
        config[i][0] = 0;
        config[i][1] = 0;
    }

    config[0][0] = MT9V03X_INIT;
    config[1][0] = MT9V03X_AUTO_EXP;
    config[1][1] = 0;
    config[2][0] = MT9V03X_EXP_TIME;
    config[2][1] = SmartCar.camera.exposure;
    config[3][0] = MT9V03X_FPS;
    config[3][1] = MT9V03X_FPS_DEF;
    config[4][0] = MT9V03X_SET_COL;
    config[4][1] = MT9V03X_W;
    config[5][0] = MT9V03X_SET_ROW;
    config[5][1] = MT9V03X_H;
    config[6][0] = MT9V03X_LR_OFFSET;
    config[6][1] = MT9V03X_LR_OFFSET_DEF;
    config[7][0] = MT9V03X_UD_OFFSET;
    config[7][1] = MT9V03X_UD_OFFSET_DEF;
    config[8][0] = MT9V03X_GAIN;
    config[8][1] = SmartCar.camera.gain;

    mt9v03x_sccb_set_config(config);
}

void image_init(void)
{
    uint8 retry;
    Image.ready = 0;
    Image.result_ready = 0;
    Image.result_ready = 0;
    Image.sequence = 0;
    Image.threshold = 0;
    Image.center = IMAGE_MID;
    Image.error = 0;
    Image.lost = 0;
    Image.ring = 0;
    Image.ring_step = 0;
    Image.zebra = 0;
    Image.zebra_count = 0;

    gpio_init(LED_DEBUG, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    retry = 0;
    while (retry < CAMERA_INIT_RETRY)
    {
        if (mt9v03x_init() == 0)
        {
            Image.ready = 1;
            break;
        }
        retry++;
        system_delay_ms(CAMERA_INIT_DELAY_MS);
    }

    image_apply_camera();
    mt9v03x_finish_flag = 0;
}

/* 图像处理 */

image_data Image;
uint8 ImageGray[IMAGE_H][IMAGE_W]; // 灰度数据
uint8 ImageBin[IMAGE_H][IMAGE_W];  // 二值数据

/* 图像压缩参数（参考 reference/image.c） */
#define IMAGE_COMPRESS_CUT_COL (1)         // 左右x列裁切
#define IMAGE_COMPRESS_CUT_ROW_TOP (0)     // 顶部裁切
#define IMAGE_COMPRESS_CUT_ROW_BOTTOM (10) // 底部裁切

/* OTSU 二值化阈值边界 */
#define IMAGE_THRESHOLD_DETACH (200) /* OTSU 扫描上限 */
#define IMAGE_THRESHOLD_STATIC (40)  /* 阈值下限 */

/* 丢失阈值 */
#define IMAGE_STOP_RAW_THRESHOLD (25)

#define IMAGE_COMPRESS_SRC_H MT9V03X_H - IMAGE_COMPRESS_CUT_ROW_TOP - IMAGE_COMPRESS_CUT_ROW_BOTTOM // 裁切行
#define IMAGE_COMPRESS_SRC_W MT9V03X_W - (IMAGE_COMPRESS_CUT_COL * 2)                               // 裁切列

/* 行/列采样映射表 + 灰度直方图 + 原始阈值缓存 */
static uint8 ImageRowMap[IMAGE_H];  // 压缩后IMAGE_H行分别对应原x行
static uint8 ImageColMap[IMAGE_W];  // 压缩后IMAGE_W列分别对应原x列
static uint8 ImageMapReady = 0;     // 映射表是否计算
static uint16 ImageHist[256];       // 记录255灰度出现次数
static uint8 ImageRawThreshold = 0; // 原始阈值

/* Compress MT9V03X raw frame to 80x60 grayscale buffer. */
static void image_compress(void)
{
    uint16 row;
    uint16 col;
    uint8 src_row;
    uint8 *dst;
    uint8 *src;

    // 分别计算行列映射
    if (ImageMapReady == 0)
    {
        for (row = 0; row < IMAGE_H; row++)
        {
            ImageRowMap[row] = (uint8)(IMAGE_COMPRESS_CUT_ROW_TOP +
                                       ((row * IMAGE_COMPRESS_SRC_H + (IMAGE_H / 2)) / IMAGE_H));
        }
        for (col = 0; col < IMAGE_W; col++)
        {
            ImageColMap[col] = (uint8)(IMAGE_COMPRESS_CUT_COL +
                                       ((col * IMAGE_COMPRESS_SRC_W + (IMAGE_W / 2)) / IMAGE_W));
        }
        ImageMapReady = 1;
    }

    for (row = 0; row < IMAGE_H; row++)
    {
        src_row = ImageRowMap[row];
        dst = ImageGray[row];         // 缓冲灰度行
        src = mt9v03x_image[src_row]; // 原始灰度行
        for (col = 0; col < IMAGE_W; col++)
        {
            dst[col] = src[ImageColMap[col]]; // 压缩复制列
        }
    }

    mt9v03x_finish_flag = 0;
}

/* Otsu threshold on ImageGray. */
static uint8 image_otsu(void)
{
    int width;
    int height;
    int pixelCount[IMAGE_GRAYSCALE];
    float pixelPro[IMAGE_GRAYSCALE];
    int i;
    int j;
    int pixel_sum;
    uint8 threshold;
    int threshold_j;
    uint8 *gray_ptr;
    uint32 gray_sum;
    float w0;
    float w1;
    float u0tmp;
    float u1tmp;
    float u0;
    float u1;
    float u;
    float delta_tmp;
    float delta_max;

    width = (int)col;
    height = (int)row;
    pixel_sum = width * height;
    threshold = 0;
    gray_ptr = image;

    for (i = 0; i < 256; i++)
    {
        ImageHist[i] = 0; // 重置数组
    }

    total = IMAGE_W * IMAGE_H;
    sum_all = 0;
    for (row = 0; row < IMAGE_H; row++)
    {
        for (col = 0; col < IMAGE_W; col++)
        {
            // 遍历全图记录灰度次数
            ImageHist[ImageGray[row][col]]++;
            sum_all += ImageGray[row][col];
        }
    }

    weight_back = 0;
    sum_back = 0;
    best_score = 0;
    threshold = 0;

    /* 以 i 为阈值，计算类间方差
    σ²(t) = wB(t) × wF(t) × [μB(t) - μF(t)]²
    wB 背景像素占比(灰度 < i 的像素数 / 总像素)
    wF 目标像素占比(灰度 ≥ i 的像素数 / 总像素)
    μB 背景平均灰度
    μF 目标平均灰度
     */
    for (i = 0; i < IMAGE_THRESHOLD_DETACH; i += 1) // 步进值为2
    {
        weight_back += ImageHist[i];
        if (weight_back == 0)
        {
            continue;
        }

        weight_front = total - weight_back;
        if (weight_front == 0)
        {
            break;
        }

        sum_back += (uint32)i * ImageHist[i];
        mean_back = (uint16)(sum_back / weight_back);
        mean_front = (uint16)((sum_all - sum_back) / weight_front);

        diff = (mean_front > mean_back) ? (mean_front - mean_back) : (mean_back - mean_front);
        score = (((uint32)weight_back * weight_front) >> 10) * (uint32)diff * diff; //
        if (score > best_score)
        {
            // 最大方差
            best_score = score;
            threshold = i;
        }
    }

    // 更新数据参数状态
    Image.threshold = threshold;
    // 判断全黑
    if (threshold < IMAGE_STOP_RAW_THRESHOLD)
    {
        Image.lost = 1; // 出界
    }
    Image.result_ready = Image.lost ? 0 : 1;

    return threshold;
}

/* Convert ImageGray to 0/1 ImageBin. */
static void image_binarize(uint8 threshold)
{
    uint8 row;
    uint8 col;
    uint8 thre;

    /* 参考代码使用 SmartCar.camera.threshold_offset 做动态偏置；
     * 当前项目暂未提供该结构体，此处保留变量名以便后续替换。 */
    ImageRawThreshold = threshold;
    if (threshold < IMAGE_THRESHOLD_STATIC)
    {
        threshold = IMAGE_THRESHOLD_STATIC;
    }

    // 为所有像素进行二值化，对边缘列进行阈值减10，底部已经裁剪
    for (row = 0; row < IMAGE_H; row++)
    {
        for (col = 0; col < IMAGE_W; col++)
        {
            if ((col <= 15) || ((col > 70) && (col <= 75)) || (col >= 65))
            {
                thre = (uint8)(threshold - 10);
            }
            else
            {
                thre = threshold;
            }

            if (ImageGray[row][col] > thre)
            {
                ImageBin[row][col] = IMAGE_WHITE;
            }
            else
            {
                ImageBin[row][col] = IMAGE_BLACK;
            }
        }
    }
}

/* 八邻域 */

static border_line Border;             // 八邻域直接爬出的原始边界点
static uint8 border_point[IMAGE_H][2]; // 逐行边界 0左1右
static uint8 row_lost_left[IMAGE_H];   // 左边界丢线行
static uint8 row_lost_right[IMAGE_H];  // 右边界丢线行
static uint8 row_valid[IMAGE_H];       // 每一行是否可信：1可信 0不可信
static uint8 both_valid_rows;          // 左右边界同时可信的总行数
static uint8 width_stable_score;       // 宽度变化平稳的行数
static uint8 border_balance_score;     // 左右边界点数量平衡性得分
static uint8 image_confidence;         // 本帧整体可信度 0~100

static void image_border_clear(void)
{
    uint16 i;
    for (i = 0; i < Border.left_data_num; i++)
    {
        Border.point_left[i][0] = 0;
        Border.point_left[i][1] = 0;
        Border.dir_left[i] = 0;
    }
    Border.left_data_num = 0;
    for (i = 0; i < Border.right_data_num; i++)
    {
        Border.point_right[i][0] = 0;
        Border.point_right[i][1] = 0;
        Border.dir_right[i] = 0;
    }
    Border.right_data_num = 0;
}
static void image_draw_black_box(void)
{
    // 绘制最边缘一圈黑框，防止八邻域爬出边界
    uint8 col;
    uint8 row;
    // 顶行
    for (col = 0; col < IMAGE_W; col++)
    {
        ImageBin[0][col] = IMAGE_BLACK;
    }
    // 左列
    for (row = 0; row < IMAGE_H; row++)
    {
        ImageBin[row][0] = IMAGE_BLACK;
    }
    // 右列
    for (row = 0; row < IMAGE_H; row++)
    {
        ImageBin[row][IMAGE_W - 1] = IMAGE_BLACK;
    }
}
static uint8 image_get_pixel(int16 row, int16 col)
{
    // 如果出界，判黑
    if (row < 0 || row >= IMAGE_H || col < 0 || col >= IMAGE_W)
    {
        return IMAGE_BLACK;
    }

    return ImageBin[row][col];
}
static uint8 image_get_start(uint8 scan_row)
{
    uint8 col;
    uint8 l_find = 0;
    uint8 r_find = 0;
    // 清空上一帧数据
    image_border_clear();
    image_draw_black_box(); // 绘制黑框，确保最边界为黑

    for (col = 0; col < IMAGE_W - 1; col++)
    {
        // 黑白跳变-左边界
        if (ImageBin[scan_row][col] == IMAGE_BLACK && ImageBin[scan_row][col + 1] == IMAGE_WHITE)
        {
            l_find = 1;
            Border.point_left[Border.left_data_num][0] = col;
            Border.point_left[Border.left_data_num][1] = scan_row;
            Border.left_data_num++;
            break;
        }
    }
    for (col = IMAGE_W - 1; col > 1; col--)
    {
        // 白黑跳变-右边界
        if (ImageBin[scan_row][col] == IMAGE_BLACK && ImageBin[scan_row][col - 1] == IMAGE_WHITE)
        {
            r_find = 1;
            Border.point_right[Border.right_data_num][0] = col;
            Border.point_right[Border.right_data_num][1] = scan_row;
            Border.right_data_num++;
            break;
        }
    }
    if (l_find && r_find)
        return 1; // 提前绘制边框，在弯道也能找到最边缘一行为边界
    return 0;
}
static void image_search_line(uint16 break_flag)
{
    uint8 i, j; // 循环变量
    uint8 index_l;
    uint8 index_r;
    uint8 selected_l;
    uint8 selected_r;
    int8 center_point_l[2];    // 左边界中心点
    int8 center_point_r[2];    // 右边界中心点
    int8 search_filds_l[8][2]; // 周围一圈在图像中的左边
    int8 search_filds_r[8][2];
    int8 temp_l[8][2];
    int8 temp_r[8][2];
    uint8 temp_dir_l[8];
    uint8 temp_dir_r[8];
    uint16 l_data_statics;
    uint16 r_data_statics;
    int8 next_x;
    int8 next_y;

    /* --->x
    | {-1,-1},{0,-1},{+1,-1}
    | {-1, 0},      ,{+1, 0}
    | {-1,+1},{0,+1},{+1,+1}
    y
    */

    // 顺时针
    static int8 search_l[8][2] = {{0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}, {1, 0}, {1, 1}};
    // 逆时针
    static int8 search_r[8][2] = {{0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};

    // 未找到种子
    if (Border.left_data_num == 0 || Border.right_data_num == 0)
    {
        return;
    }

    l_data_statics = Border.left_data_num;
    r_data_statics = Border.right_data_num;

    // 种子坐标
    center_point_l[0] = Border.point_left[0][0];
    center_point_l[1] = Border.point_left[0][1];
    center_point_r[0] = Border.point_right[0][0];
    center_point_r[1] = Border.point_right[0][1];

    while (break_flag--)
    {
        if (l_data_statics >= POINT_NUM || r_data_statics >= POINT_NUM)
        {
            break;
        }
        // 计算周围一圈的坐标
        for (i = 0; i < 8; i++)
        {
            next_x = center_point_l[0] + search_l[i][0];
            next_y = center_point_l[1] + search_l[i][1];
            search_filds_l[i][0] = next_x;
            search_filds_l[i][1] = next_y;
        }
        Border.point_left[l_data_statics][0] = center_point_l[0];
        Border.point_left[l_data_statics][1] = center_point_l[1];

        index_l = 0;
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0;
            temp_l[i][1] = 0;
            temp_dir_l[i] = 0;
        }
        for (i = 0; i < 8; i++)
        {
            // 取余成环
            if (image_get_pixel(search_filds_l[i][1], search_filds_l[i][0]) == IMAGE_BLACK && image_get_pixel(search_filds_l[(i + 1) & 7][1], search_filds_l[(i + 1) & 7][0]) == IMAGE_WHITE)
            {
                temp_l[index_l][0] = search_filds_l[i][0];
                temp_l[index_l][1] = search_filds_l[i][1];
                temp_dir_l[index_l] = i;
                index_l++;
            }
        }
        // 多个候选点
        if (index_l)
        {
            selected_l = 0;
            center_point_l[0] = temp_l[0][0];
            center_point_l[1] = temp_l[0][1];
            for (j = 1; j < index_l; j++)
            {
                if (center_point_l[1] > temp_l[j][1])
                {
                    center_point_l[0] = temp_l[j][0];
                    center_point_l[1] = temp_l[j][1];
                    selected_l = j;
                }
            }
            Border.dir_left[l_data_statics] = temp_dir_l[selected_l];
        }
        else
        {
            if (center_point_l[1] == 0)
            {
                break;
            }
            center_point_l[0] = IMAGE_MID;
            center_point_l[1]--;
        }
        l_data_statics++; // 左加

        for (i = 0; i < 8; i++)
        {
            next_x = center_point_r[0] + search_r[i][0];
            next_y = center_point_r[1] + search_r[i][1];
            search_filds_r[i][0] = next_x;
            search_filds_r[i][1] = next_y;
        }
        Border.point_right[r_data_statics][0] = center_point_r[0];
        Border.point_right[r_data_statics][1] = center_point_r[1];

        index_r = 0;
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0;
            temp_r[i][1] = 0;
            temp_dir_r[i] = 0;
        }
        for (i = 0; i < 8; i++)
        {
            if (image_get_pixel(search_filds_r[i][1], search_filds_r[i][0]) == IMAGE_BLACK && image_get_pixel(search_filds_r[(i + 1) & 7][1], search_filds_r[(i + 1) & 7][0]) == IMAGE_WHITE)
            {
                temp_r[index_r][0] = search_filds_r[i][0];
                temp_r[index_r][1] = search_filds_r[i][1];
                temp_dir_r[index_r] = i;
                index_r++;
            }
        }
        if (index_r)
        {
            selected_r = 0;
            center_point_r[0] = temp_r[0][0];
            center_point_r[1] = temp_r[0][1];
            for (j = 1; j < index_r; j++)
            {
                if (center_point_r[1] > temp_r[j][1])
                {
                    center_point_r[0] = temp_r[j][0];
                    center_point_r[1] = temp_r[j][1];
                    selected_r = j;
                }
            }
            Border.dir_right[r_data_statics] = temp_dir_r[selected_r];
        }
        else
        {
            if (center_point_r[1] == 0)
            {
                break;
            }
            center_point_r[0] = IMAGE_MID;
            center_point_r[1]--;
        }
        r_data_statics++; // 右加

        Border.left_data_num = l_data_statics;
        Border.right_data_num = r_data_statics;

        /* 结束条件判断 */
        if (r_data_statics >= 3 && Border.point_right[r_data_statics - 1][0] == Border.point_right[r_data_statics - 2][0] && Border.point_right[r_data_statics - 1][0] == Border.point_right[r_data_statics - 3][0] && Border.point_right[r_data_statics - 1][1] == Border.point_right[r_data_statics - 2][1] && Border.point_right[r_data_statics - 1][1] == Border.point_right[r_data_statics - 3][1])
        {
            break;
        }
        if (l_data_statics >= 3 && Border.point_left[l_data_statics - 1][0] == Border.point_left[l_data_statics - 2][0] && Border.point_left[l_data_statics - 1][0] == Border.point_left[l_data_statics - 3][0] && Border.point_left[l_data_statics - 1][1] == Border.point_left[l_data_statics - 2][1] && Border.point_left[l_data_statics - 1][1] == Border.point_left[l_data_statics - 3][1])
        {
            break;
        }
        if ((center_point_r[0] - center_point_l[0] < 2) && (center_point_r[0] - center_point_l[0] > -2) && (center_point_r[1] - center_point_l[1] < 2) && (center_point_r[1] - center_point_l[1] > -2))
        {
            break;
        }
        if (center_point_l[1] == 0 || center_point_r[1] == 0)
        {
            break;
        }
    }
}
static uint8 image_get_border(void)
{
    uint8 i;
    uint8 row; // 行
    uint16 j;
    uint8 width_now;
    uint8 width_prev;
    uint8 stable_rows;
    uint16 diff;
    uint16 balance_diff;
    uint16 balance_base;

    both_valid_rows = 0;
    width_stable_score = 0;
    border_balance_score = 0;

    // 第一步：把逐行边界和可信度数组清空，默认整行不可信
    for (i = 0; i < IMAGE_H; i++)
    {
        border_point[i][0] = 0;           // 左边界
        border_point[i][1] = IMAGE_W - 1; // 右边界
        row_valid[i] = 0;
    }
    row = IMAGE_H-2; // 底部向上 
    for (j = 0; j < Border.left_data_num; j++)
    {
        if (Border.point_left[j][1] == row)
        {
            border_point[row][0] = Border.point_left[j][0] + 1; // 左边界
        } else continue; // 每行只取一点
        row--; 
        if(row==0) break; // 顶部
    }
    row = IMAGE_H-2;
    for (j = 0; j < Border.right_data_num; j++)
    {
        if(Border.point_right[j][1] == row)
        {
            border_point[row][1] = Border.point_right[j][0] - 1; // 右边界
        } else continue;
        row--;
        if(row==0) break; // 顶部
    }

    // 判断稳定性和可信度
    stable_rows = 0;
    width_prev = 0;
    for (i = 0; i < IMAGE_H; i++)
    {
        // 丢线标志
        row_lost_left[i] = (border_point[i][0] <= 2) ? 1 : 0;
        row_lost_right[i] = (border_point[i][1] >= (IMAGE_W - 3)) ? 1 : 0;

        if (!row_lost_left[i] && !row_lost_right[i] && border_point[i][0] < border_point[i][1])
        {
            width_now = border_point[i][1] - border_point[i][0];
            if (width_now >= 8)
            {
                // 宽度足够
                row_valid[i] = 1;
                both_valid_rows++;
                // 宽度变化较小
                diff = (width_now > width_prev) ? (width_now - width_prev) : (width_prev - width_now);
                if (diff <= 6)
                {
                    stable_rows++;
                }
                width_prev = width_now;
            }
        }
    }

    width_stable_score = stable_rows; // 宽度稳定的行数
    // 左右爬线点相减，除较大值，计算balance平衡性，如果对称，平衡性较好，可信度较高，可以看得更远
    balance_diff = (Border.left_data_num > Border.right_data_num) ? (Border.left_data_num - Border.right_data_num) : (Border.right_data_num - Border.left_data_num);
    balance_base = (Border.left_data_num > Border.right_data_num) ? Border.left_data_num : Border.right_data_num;
    if (balance_base == 0)
    {
        // 完全不可信，判断出界
        border_balance_score = 0;
        Image.lost = 1;
    }
    else if ((balance_diff * 100U) / balance_base <= 10U)
    {
        border_balance_score = 100;
    }
    else if ((balance_diff * 100U) / balance_base <= 20U)
    {
        border_balance_score = 80;
    }
    else if ((balance_diff * 100U) / balance_base <= 30U)
    {
        border_balance_score = 60;
    }
    else if ((balance_diff * 100U) / balance_base <= 40U)
    {
        border_balance_score = 40;
    }
    else
    {
        border_balance_score = 20;
    }

    return 1;
}
/* 补线 */
static void image_find_corss()
{
    // 十字补线 - 新算法：
    // 检测方向"向上"(3,4,5)到"向内"(5,6,7)的转折
    // 转折行是补线终点（丢线区最底行）
    // 从该行向下找第一个非丢线行，用两点斜率向上回推补线
    uint16 i, j;
    uint8 enter_row,out_row;
    float k;
    uint8 dir_prev;
    uint8 dir_now;
    uint8 enter_prev,enter_now,enter;
    uint8 out_prew,out_now;

    // 左边
    enter = 0;
    for (i = 1; i < Border.left_data_num; i++)
    {
        dir_prev = Border.dir_left[i - 1];
        dir_now = Border.dir_left[i];

        enter_prev = (dir_prev == 1 || dir_prev == 2 || dir_prev == 3); // 起点
        enter_now  = (dir_now  == 3 || dir_now  == 4 || dir_now  == 5);
        out_prew = (dir_prev  == 3 || dir_prev  == 4 || dir_prev  == 5); // 终点
        out_now = (dir_now  == 5 || dir_now  == 6 || dir_now  == 7);
        if(enter_prev && enter_now)
        {
            enter = 1;
            enter_row = Border.point_left[i][1];
        } 
        if (enter && out_prew && out_now)
        {
            out_row = Border.point_left[i][1];
            if (out_row >= 2 && enter_row < IMAGE_H - 7)
            {
                if (row_lost_left[out_row] && row_lost_left[out_row + 1] && row_lost_left[out_row + 2])
                {
                    Image.cross |= 0x02;

                    // 用起点下方第2和第7行计算斜率
                    k = (float)(border_point[enter_row+2][0] - border_point[enter_row+7][0]) / (float)(7-2);
                    // 从转折行向下补线
                    j = 1;
                    while (j <= enter_row)
                    {
                        border_point[enter_row - j][0] = border_point[enter_row+2][0] + j * k;
                        j++;
                    }
                    break;
                }
            }
        }
        Image.cross = 0;
    }

    // 右边
    enter = 0;
    for (i = 1; i <Border.right_data_num; i++)
    {
        dir_prev = Border.dir_right[i - 1];
        dir_now  = Border.dir_right[i];

        enter_prev = (dir_prev == 1 || dir_prev == 2 || dir_prev == 3); // 起点
        enter_now  = (dir_now  == 3 || dir_now  == 4 || dir_now  == 5);
        out_prew = (dir_prev  == 3 || dir_prev  == 4 || dir_prev  == 5); // 终点
        out_now = (dir_now  == 5 || dir_now  == 6 || dir_now  == 7);
        if(enter_prev && enter_now)
        {
            enter = 1;
            enter_row = Border.point_right[i][1];
        } 
        if (enter && out_prew && out_now)
        {
            out_row = Border.point_right[i][1];
            if (out_row >= 2 && enter_row < IMAGE_H - 7)
            {
                // 终点前3行（行号+）
                if (row_lost_right[out_row] && row_lost_right[out_row + 1] && row_lost_right[out_row + 2])
                {
                    Image.cross |= 0x01;

                    k = (float)(border_point[enter_row+2][1] - border_point[enter_row+7][1]) / (float)(7-2);

                    j = 1;
                    while (j <= enter_row )
                    {
                        border_point[enter_row-j][1] = border_point[enter_row+2][1] + j * k;
                        j++;
                    }
                    break;
                }
            }
        }
        Image.cross = 0;
    }
}

/* 动态前瞻 */

#define CENTER_POINTS 7 // 计算中心加权行数（改动需要改动态前瞻相关表）

typedef struct
{
    uint8 base_row;    // 基础行
    uint8 offset_step; // 偏移步长
    uint8 max_row;     // 最大行
} tow_row_config;

static tow_row_config TowRowTable[CENTER_POINTS] = {
    {56, 0, 58},
    {45, 1, 50},
    {35, 1, 40},
    {27, 1, 33},
    {19, 2, 29},
    {13, 2, 23},
    {8, 3, 22}};
static const uint8 row_weight[CENTER_POINTS] = {1, 1, 1, 1, 1, 1, 1}; // 建议修改，先平均
static uint8 sample_center_point[CENTER_POINTS];                      // 中心采样点
static uint8 center_row[CENTER_POINTS];

static void image_calculate_confidence(void)
{
    uint16 confidence_sum; // 最终可信度
    uint16 valid_score;    // 左右同时可信得分
    uint16 stable_score;   // 稳定性可信得分

    // 映射 100 分
    valid_score = (both_valid_rows * 100U) / IMAGE_H;
    stable_score = (width_stable_score * 100U) / IMAGE_H;
    // 加权计算
    confidence_sum = valid_score * 4U;
    confidence_sum += stable_score * 3U;
    confidence_sum += border_balance_score * 3U;
    image_confidence = (uint8)(confidence_sum / 10U);
}

static void image_calculate_center(void)
{
    // 使用动态参数计算多行加权的中心
    uint8 i;
    uint8 row; // 前瞻行
    uint16 sum_center;
    uint16 sum_weight; // 权重分母
    uint8 center;
    sum_center = 0;
    sum_weight = 0; // 循环求和，占用时间不多，就不写只算一次的判断了
    for (i = 0; i < CENTER_POINTS; i++)
    {
        row = TowRowTable[i].base_row + TowRowTable[i].offset_step * (100 - image_confidence) / 20; // 最大行无需判断，步长最大乘5
        center_row[i] = row;                                                                        // 存储，方便画点
        sample_center_point[i] = (uint8)(((uint16)border_point[row][0] + (uint16)border_point[row][1]) >> 1);
        sum_weight += row_weight[i];
    }
    for (i = 0; i < CENTER_POINTS; i++)
    {
        sum_center += sample_center_point[i] * row_weight[i];
    }
    center = (uint8)(sum_center / sum_weight);

    // 更新参数
    Image.center = center;
    Image.error = (int16)(center - IMAGE_MID);
}

// 反向映射x轴y轴
static uint16 image_debug_x(uint16 x, uint16 w, int16 col)
{
    if (col < 0)
    {
        col = 0;
    }
    if (col >= IMAGE_W)
    {
        col = IMAGE_W - 1;
    }

    return (uint16)(x + (((uint16)col * w) + (IMAGE_W / 2)) / IMAGE_W);
}
static uint16 image_debug_y(uint16 y, uint16 h, int16 row)
{
    if (row < 0)
    {
        row = 0;
    }
    if (row >= IMAGE_H)
    {
        row = IMAGE_H - 1;
    }

    return (uint16)(y + (((uint16)row * h) + (IMAGE_H / 2)) / IMAGE_H);
}

static void image_process(void)
{
    gpio_set_level(LED_DEBUG, 1);
    image_compress();             // 压缩
    image_binarize(image_otsu()); // 二值化
    // 需要使用丢失阈值判断是否出界
    image_get_start(IMAGE_H - 2); // 底部起点
    image_search_line(100);       // 八邻域爬线
    image_get_border();           // 获取点边界
    image_calculate_confidence(); // 计算可信度
    image_find_corss();           // 补线
    image_calculate_center();     // 求中点和偏移
    gpio_set_level(LED_DEBUG, 0);
}

/* 更新和状态 */

#define IMAGE_LOST_STOP_COUNT (4)
#define IMAGE_RUN_START_IGNORE_FRAMES (3)

static uint8 ImageLostCount = 0;
static uint8 ImageRunFrameCount = 0;

void image_update(void)
{
    if (Image.ready == 0)
        return; // 未就绪
    if (mt9v03x_finish_flag == 0)
        return; // 无图像
    image_process();
    Image.sequence++;
    if (CarMode != CAR_MODE_RUN)
    {
        ImageLostCount = 0;
        ImageRunFrameCount = 0;
        return;
    }

    if (ImageRunFrameCount < IMAGE_RUN_START_IGNORE_FRAMES)
    {
        ImageRunFrameCount++;
        ImageLostCount = 0;
        return;
    }

    if (Image.lost)
    {
        if (ImageLostCount < IMAGE_LOST_STOP_COUNT)
        {
            ImageLostCount++;
        }
        if (ImageLostCount >= IMAGE_LOST_STOP_COUNT)
        {
            CarMode = CAR_MODE_STOP;
        }
    }
    else
    {
        ImageLostCount = 0;
    }
}
// 画点
void image_show_debug_overlay(uint16 x, uint16 y, uint16 w, uint16 h)
{
    uint16 i;
    uint16 draw_x;
    uint16 draw_y;

    if (Image.ready == 0)
    {
        return;
    }
    // 搜线
    for (i = 0; i < Border.left_data_num; i++)
    {
        draw_x = image_debug_x(x, w, Border.point_left[i][0]);
        draw_y = image_debug_y(y, h, Border.point_left[i][1]);
        ips200_draw_point(draw_x, draw_y, RGB565_GREEN);
    }

    for (i = 0; i < Border.right_data_num; i++)
    {
        draw_x = image_debug_x(x, w, Border.point_right[i][0]);
        draw_y = image_debug_y(y, h, Border.point_right[i][1]);
        ips200_draw_point(draw_x, draw_y, RGB565_CYAN);
    }
    // 行边界
    for (i = 0; i < IMAGE_H; i++)
    {
        draw_x = image_debug_x(x, w, border_point[i][0]); // 左
        draw_y = image_debug_y(y, h, i);
        ips200_draw_point(draw_x, draw_y, RGB565_BLUE);
    }
    for (i = 0; i < IMAGE_H; i++)
    {
        draw_x = image_debug_x(x, w, border_point[i][1]); // 右
        draw_y = image_debug_y(y, h, i);
        ips200_draw_point(draw_x, draw_y, RGB565_PURPLE);
    }
    // 中点
    for (i = 0; i < CENTER_POINTS; i++)
    {
        draw_x = image_debug_x(x, w, sample_center_point[i]);
        draw_y = image_debug_y(y, h, center_row[i]);
        ips200_draw_point(draw_x, draw_y, RGB565_RED);
    }
}
