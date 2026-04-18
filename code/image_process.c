#include "image_process.h"
#include "zf_common_headfile.h"

unsigned char erzhihua[IMG_H][IMG_W] = {0}; // 二值化图像
int path_center_x[IMG_H] = {0};
int path_left_x[IMG_H] = {0};
int path_right_x[IMG_H] = {0};
int branch_valid_row_cnt = 0;
uint8_t cam_capture_done = 0;
Path_TypeDef current_path_type = PATH_TYPE_UNKNOWN;

// 静态缓冲区，避免栈上分配大数组（原版 temp_img 在栈上约 22KB，存在溢出风险）
static unsigned char morph_temp[IMG_H][IMG_W];

// OTSU 缓存策略：减少每帧计算开销，亮度变化时才重新计算
#define OTSU_UPDATE_PERIOD  5    // 每 N 帧强制更新一次
#define OTSU_MEAN_DELTA     8    // ROI 平均亮度变化超过此值时触发更新
// OTSU 统计范围：只统计赛道所在区域（FAR_ROI_START ~ ROI_ROW_END）
// 反光区域如果不在此范围内不会影响阈值
#define OTSU_ROW_START  FAR_ROI_START
#define OTSU_ROW_END    ROI_ROW_END

static uint8_t cached_threshold  = 128;
static int     otsu_frame_cnt    = 0;
static int     last_roi_mean     = -1;

/**
 * @brief ROI 区域 OTSU 大津法（仅统计赛道区域，抗反光）
 */
static uint8_t otsu_threshold_roi(void)
{
    uint32_t histogram[256] = {0};
    uint32_t total_pixels = (OTSU_ROW_END - OTSU_ROW_START + 1) * IMG_W;

    for (int i = OTSU_ROW_START; i <= OTSU_ROW_END; i++)
        for (int j = 0; j < IMG_W; j++)
            histogram[mt9v03x_image_1[i][j]]++;

    uint32_t sum_all = 0;
    for (int t = 0; t < 256; t++)
        sum_all += t * histogram[t];

    uint32_t sum_bg = 0;
    uint32_t cnt_bg = 0;
    float max_variance = 0;
    uint8_t best_threshold = 128;

    for (int t = 0; t < 256; t++)
    {
        cnt_bg += histogram[t];
        if (cnt_bg == 0) continue;

        uint32_t cnt_fg = total_pixels - cnt_bg;
        if (cnt_fg == 0) break;

        sum_bg += t * histogram[t];

        float mean_bg = (float)sum_bg / cnt_bg;
        float mean_fg = (float)(sum_all - sum_bg) / cnt_fg;
        float diff = mean_bg - mean_fg;

        float variance = (float)cnt_bg * cnt_fg * diff * diff;
        if (variance > max_variance)
        {
            max_variance = variance;
            best_threshold = (uint8_t)t;
        }
    }

    return best_threshold;
}

/**
 * @brief 带缓存的 OTSU 阈值获取（减少计算量，提升帧率）
 *
 * 策略：
 *   1. 每帧计算 ROI 平均亮度（很快，只需累加和除法）
 *   2. 亮度变化 > OTSU_MEAN_DELTA 或达到强制更新周期 → 重新计算 OTSU
 *   3. 否则直接返回缓存值
 */
static uint8_t otsu_threshold(void)
{
    // 快速计算 ROI 平均亮度
    uint32_t roi_sum = 0;
    int roi_pixels = (OTSU_ROW_END - OTSU_ROW_START + 1) * IMG_W;
    for (int i = OTSU_ROW_START; i <= OTSU_ROW_END; i++)
        for (int j = 0; j < IMG_W; j++)
            roi_sum += mt9v03x_image_1[i][j];
    int roi_mean = roi_sum / roi_pixels;

    otsu_frame_cnt++;

    // 判断是否需要更新
    int need_update = 0;
    if (last_roi_mean < 0)                                    need_update = 1;  // 首次
    if (otsu_frame_cnt >= OTSU_UPDATE_PERIOD)                 need_update = 1;  // 周期到
    if (abs(roi_mean - last_roi_mean) >= OTSU_MEAN_DELTA)     need_update = 1;  // 亮度变化

    if (need_update)
    {
        cached_threshold = otsu_threshold_roi();
        otsu_frame_cnt = 0;
        last_roi_mean = roi_mean;
    }

    return cached_threshold;
}

/**
 * @brief 多次形态学闭运算（膨胀+腐蚀）
 *
 * 原版只做1次3x3闭运算，等效作用域约3像素。
 * 疯狂电路赛道上的SMD元器件（电阻、芯片）在20cm拍摄高度下
 * 可达5-10像素宽，3次3x3闭运算可覆盖约7像素宽的黑色孤立块。
 *
 * @param binary_img  待处理的二值化图像（原地修改）
 * @param passes      闭运算次数（建议3次）
 */
static void morphological_close_multipass(unsigned char binary_img[IMG_H][IMG_W], int passes)
{
    for (int p = 0; p < passes; p++)
    {
        // === 膨胀（Dilation）：取3x3邻域最大值 ===
        for (int i = 1; i < IMG_H - 1; i++)
        {
            for (int j = 1; j < IMG_W - 1; j++)
            {
                uint8_t mv = 0;
                for (int di = -1; di <= 1; di++)
                    for (int dj = -1; dj <= 1; dj++)
                        if (binary_img[i + di][j + dj] > mv)
                            mv = binary_img[i + di][j + dj];
                morph_temp[i][j] = mv;
            }
        }
        // 边界直接复制，不参与形态学计算
        for (int i = 0; i < IMG_H; i++)
        {
            morph_temp[i][0]        = binary_img[i][0];
            morph_temp[i][IMG_W-1]  = binary_img[i][IMG_W-1];
        }
        for (int j = 0; j < IMG_W; j++)
        {
            morph_temp[0][j]        = binary_img[0][j];
            morph_temp[IMG_H-1][j]  = binary_img[IMG_H-1][j];
        }

        // === 腐蚀（Erosion）：取3x3邻域最小值 ===
        for (int i = 1; i < IMG_H - 1; i++)
        {
            for (int j = 1; j < IMG_W - 1; j++)
            {
                uint8_t mv = 255;
                for (int di = -1; di <= 1; di++)
                    for (int dj = -1; dj <= 1; dj++)
                        if (morph_temp[i + di][j + dj] < mv)
                            mv = morph_temp[i + di][j + dj];
                binary_img[i][j] = mv;
            }
        }
    }
}

/**
 * @brief 填充赛道行内部像素为白色（元器件二次清除）
 *
 * 在边线追踪成功后，将 [left_x, right_x] 区间内的所有像素置为白色。
 * 这样即使形态学闭运算未完全消除的残余元器件黑点，也会在逐行追踪过程中
 * 被清除，避免下一行追踪时起始点落入黑色噪点。
 *
 * @param binary_img  二值化图像（原地修改）
 * @param row         当前处理行号
 * @param left_x      已确认的左边线坐标
 * @param right_x     已确认的右边线坐标
 */
static void fill_row_interior(unsigned char binary_img[IMG_H][IMG_W],
                               int row, int left_x, int right_x)
{
    if (left_x < 1 || right_x >= IMG_W - 1 || left_x >= right_x) return;
    for (int x = left_x + 1; x < right_x; x++)
        binary_img[row][x] = 255;
}

/**
 * @brief 对路径数组进行线性插值，填补追踪失败（值为-1）的行
 *
 * 当元器件导致某几行追踪失败时，用相邻有效行的值做线性插值，
 * 确保偏差计算不受短暂丢线影响，整体路径仍被识别为直线。
 *
 * @param arr        路径坐标数组（-1 表示失败行）
 * @param row_start  插值范围起始行（含）
 * @param row_end    插值范围结束行（含）
 */
static void interpolate_failed_rows(int* arr, int row_start, int row_end)
{
    int y = row_start;
    while (y <= row_end)
    {
        if (arr[y] == -1)
        {
            // 向前找最近的有效行
            int prev_y = y - 1;
            while (prev_y >= row_start && arr[prev_y] == -1) prev_y--;

            // 向后找最近的有效行
            int next_y = y + 1;
            while (next_y <= row_end && arr[next_y] == -1) next_y++;

            if (prev_y >= row_start && next_y <= row_end)
            {
                // 两端都有有效值：线性插值
                int pv = arr[prev_y];
                int nv = arr[next_y];
                int span = next_y - prev_y;
                for (int iy = y; iy < next_y; iy++)
                    arr[iy] = pv + (nv - pv) * (iy - prev_y) / span;
            }
            else if (prev_y >= row_start)
            {
                // 只有前方有效：外推（保持末尾值）
                for (int iy = y; iy <= row_end; iy++)
                    arr[iy] = arr[prev_y];
            }
            else if (next_y <= row_end)
            {
                // 只有后方有效：外推（保持首端值）
                for (int iy = y; iy < next_y; iy++)
                    arr[iy] = arr[next_y];
            }
            // 两端都无有效值：保持 -1，不处理
            y = next_y + 1;
        }
        else
        {
            y++;
        }
    }
}

/**
 * @brief 图像处理及获取中线
 *
 * 修改说明：
 * 1. 将 temp_img 改为静态全局（原版在栈上分配 ~22KB，存在溢出风险）
 * 2. 改用 morphological_close_multipass() 执行3次闭运算（原版仅1次）
 *    - 1次闭运算覆盖约3像素的黑色孤立块
 *    - 3次闭运算覆盖约7像素的黑色孤立块，可有效消除SMD元器件干扰
 *
 * @return 1=处理了新图像，0=无新图像
 */
uint8_t Image_Processing_And_Get_Center(void)
{
    if (!mt9v03x_finish_flag_1)
        return 0;

    mt9v03x_finish_flag_1 = 0;

    // OTSU 动态计算阈值
    uint8_t threshold = otsu_threshold();

    // 全图二值化
    for (int i = 0; i < IMG_H; i++)
        for (int j = 0; j < IMG_W; j++)
            erzhihua[i][j] = (mt9v03x_image_1[i][j] > threshold) ? 255 : 0;

    // 多次形态学闭运算（消除元器件黑色孤立块）
    // 3次 = 有效消除约7px宽的黑色块，覆盖常见SMD元器件在此摄像头高度下的投影大小
    morphological_close_multipass(erzhihua, 3);

    // 识别路径类型并提取中线
    current_path_type = recognize_path_type(erzhihua);

    return 1;
}

volatile float near_image_error = 0;
volatile float far_image_error  = 0;

/**
 * @brief 获取最终用于 PID 的偏差值
 */
void Calculate_All_Errors(void)
{
    float near_total = 0, far_total = 0;
    int near_cnt = 0, far_cnt = 0;
    int image_center = IMG_W / 2;

    for (int i = 0; i < IMG_H; i++)
    {
        if (path_center_x[i] > 0 && path_center_x[i] < IMG_W)
        {
            if (i >= ROI_ROW_START && i <= ROI_ROW_END)
            {
                near_total += (float)(path_center_x[i] - image_center);
                near_cnt++;
            }
            else if (i >= FAR_ROI_START && i <= FAR_ROI_END)
            {
                far_total += (float)(path_center_x[i] - image_center);
                far_cnt++;
            }
        }
    }

    float near_temp = (near_cnt > 5) ? (near_total / near_cnt) : 0;
    float far_temp  = (far_cnt  > 5) ? (far_total  / far_cnt)  : 0;

    uint32 cpu_status = interrupt_global_disable();
    near_image_error = near_temp;
    far_image_error  = far_temp;
    interrupt_global_enable(cpu_status);
}

/**
 * @brief 中心扩散与边缘跟踪算法（区域生长版）
 *
 * 原版逻辑保留，通过上一行的边线坐标作为起始点缩小搜索范围，
 * 配合允许 ≤2 个噪点像素的容错机制过滤细微黑点。
 */
static uint8_t scan_row_tracking(uint16_t row, unsigned char binary_img[IMG_H][IMG_W],
                                 int* left_x, int* right_x, int last_left, int last_right)
{
    *left_x = -1;
    *right_x = -1;

    int start_l = (last_left  != -1) ? last_left  : (IMG_W / 2);
    int start_r = (last_right != -1) ? last_right : (IMG_W / 2);

    // 搜索左边界
    int black_noise_cnt = 0;
    for (int x = start_l; x >= 0; x--)
    {
        if (binary_img[row][x] <= PATH_THRESHOLD)
        {
            black_noise_cnt++;
            if (black_noise_cnt > 2)
            {
                *left_x = x + 2;
                break;
            }
        }
        else
        {
            black_noise_cnt = 0;
        }
    }

    // 搜索右边界
    black_noise_cnt = 0;
    for (int x = start_r; x < IMG_W; x++)
    {
        if (binary_img[row][x] <= PATH_THRESHOLD)
        {
            black_noise_cnt++;
            if (black_noise_cnt > 2)
            {
                *right_x = x - 2;
                break;
            }
        }
        else
        {
            black_noise_cnt = 0;
        }
    }

    return (*left_x != -1 && *right_x != -1 && (*right_x - *left_x) >= PATH_WIDTH_MIN) ? 1 : 0;
}

/**
 * @brief 环形边框交点统计（用于路口类型判定）
 */
static uint8_t count_box_intersections_fixed(unsigned char bin_img[IMG_H][IMG_W],
                                              int cx, int cy, int size)
{
    int half = size / 2;
    int l = cx - half, r = cx + half, t = cy - half, b = cy + half;

    if (l < 0) l = 0; if (r >= IMG_W) r = IMG_W - 1;
    if (t < 0) t = 0; if (b >= IMG_H) b = IMG_H - 1;

    uint8_t jumps = 0;
    uint8_t last = (bin_img[t][l] == 255);
    uint8_t first = last;

    for (int x = l + 1; x <= r; x++) { uint8_t c = (bin_img[t][x] == 255); if (c && !last) jumps++; last = c; }
    for (int y = t + 1; y <= b; y++) { uint8_t c = (bin_img[y][r] == 255); if (c && !last) jumps++; last = c; }
    for (int x = r - 1; x >= l; x--) { uint8_t c = (bin_img[b][x] == 255); if (c && !last) jumps++; last = c; }
    for (int y = b - 1; y >  t; y--) { uint8_t c = (bin_img[y][l] == 255); if (c && !last) jumps++; last = c; }

    if (first && !last) jumps++;

    return jumps;
}

/**
 * @brief 单行多边界扫描（用于分支检测）
 */
uint8_t scan_row_multi_boundary(uint16_t row, unsigned char binary_img[IMG_H][IMG_W],
                                int* left1, int* right1, int* left2, int* right2)
{
    *left1 = *right1 = *left2 = *right2 = -1;
    int edge_cnt = 0;

    int* left_arr[2]  = {left1, left2};
    int* right_arr[2] = {right1, right2};

    int seg_start = -1;
    int white_run = 0;

    for (int x = 0; x < IMG_W; x++)
    {
        if (binary_img[row][x] > PATH_THRESHOLD)
        {
            white_run++;
            if (white_run == EDGE_CONTINUE_CNT && seg_start == -1)
                seg_start = x - EDGE_CONTINUE_CNT + 1;
        }
        else
        {
            if (seg_start != -1)
            {
                int seg_end   = x - 1;
                int seg_width = seg_end - seg_start + 1;
                if (seg_width >= PATH_WIDTH_MIN && edge_cnt < 2)
                {
                    *left_arr[edge_cnt]  = seg_start;
                    *right_arr[edge_cnt] = seg_end;
                    edge_cnt++;
                }
                seg_start = -1;
            }
            white_run = 0;
        }
    }

    if (seg_start != -1)
    {
        int seg_end   = IMG_W - 1;
        int seg_width = seg_end - seg_start + 1;
        if (seg_width >= PATH_WIDTH_MIN && edge_cnt < 2)
        {
            *left_arr[edge_cnt]  = seg_start;
            *right_arr[edge_cnt] = seg_end;
            edge_cnt++;
        }
    }

    return edge_cnt;
}

/**
 * @brief 最小二乘法计算中线斜率
 */
float calculate_center_slope(int* center_x, int* row_y, int cnt)
{
    if (cnt < 2) return 0.0f;

    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
    for (int i = 0; i < cnt; i++)
    {
        float x = (float)row_y[i];
        float y = (float)center_x[i];
        sum_x  += x;
        sum_y  += y;
        sum_xy += x * y;
        sum_xx += x * x;
    }

    float denom = cnt * sum_xx - sum_x * sum_x;
    if (fabs(denom) < 1e-6f) return 0.0f;

    return (cnt * sum_xy - sum_x * sum_y) / denom;
}

/**
 * @brief 贝塞尔曲线补线：检测到直角弯时覆写 path_center_x，生成通过弯道的虚拟中线
 *
 * 二次贝塞尔曲线：B(t) = (1-t)²×P0 + 2(1-t)t×P1 + t²×P2,  t ∈ [0,1]
 *
 * P0 = (最后有效中线位置, ROI_ROW_END)    — 近端起点（车当前位置）
 * P1 = (P0.x,            中间行)          — 控制点（先直走再弯，曲线自然）
 * P2 = (P0.x + dir*SHIFT, FAR_ROI_START)  — 远端终点（偏向弯道方向）
 *
 * PID 跟踪这条曲线 → 自然产生差速 → 闭环转弯，不需要开环固定内外轮速
 *
 * @param dir  +1=右转（中线向右偏移），-1=左转（中线向左偏移）
 */
static void bezier_fill_corner(int dir)
{
    int y_start = ROI_ROW_END;
    int y_end   = FAR_ROI_START;
    int total   = y_start - y_end;
    if (total <= 0) return;

    // P0：近端起点 — 找最近的有效中线位置
    int x0 = IMG_W / 2;
    for (int y = y_start; y >= y_start - 10; y--)
    {
        if (path_center_x[y] > 0 && path_center_x[y] < IMG_W)
        {
            x0 = path_center_x[y];
            break;
        }
    }

    // P2：远端终点 — 偏向弯道方向
    int x2 = x0 + dir * TURN_BEZIER_SHIFT;
    if (x2 < 0)      x2 = 0;
    if (x2 >= IMG_W)  x2 = IMG_W - 1;

    // P1：控制点 — 偏移到 P0 和 P2 的中间偏向 P2 侧，让曲线从近端就开始弯
    // 若 P1=P0 则曲线前半段几乎是直线，近端 ROI 看不到有效偏差
    // 若 P1=P2 则曲线像四分之一圆，从一开始就大幅弯曲
    int x1 = x0 + dir * TURN_BEZIER_SHIFT * 1 ;

    // 用贝塞尔公式覆写 path_center_x
    for (int y = y_start; y >= y_end; y--)
    {
        float t = (float)(y_start - y) / total;
        float u = 1.0f - t;
        int bx = (int)(u * u * x0 + 2.0f * u * t * x1 + t * t * x2 + 0.5f);
        if (bx < 0)      bx = 0;
        if (bx >= IMG_W)  bx = IMG_W - 1;
        path_center_x[y] = bx;
    }
}

/**
 * @brief 直角弯检测：直接扫二值图像最右侧白色像素（不依赖边线追踪）
 *
 * 【原拐点法失效的根本原因】
 *   scan_row_tracking 在拐弯区域因 T 形路口几何形状改变而返回 -1，
 *   path_right_x[y] 在拐点行全部为 -1，基线建立和跳变检测完全没有数据。
 *
 * 【新方法：直接扫二值化图像】
 *   逐行从右往左扫，找每行最右侧白色像素的 x 坐标（rightmost_x）。
 *   直角弯衍生线出现时，该行白色像素会延伸到图像右边界（rightmost_x ≈ IMG_W-1）。
 *   统计触边行数：达到 CORNER_BOUND_ROWS 行即确认右转。
 *
 *   元器件过滤：元器件是孤立小块，不会延伸到图像边界，触边行数为 0。
 *   赛道偏右过滤：赛道本身偏右时，rightmost_x 是赛道右边线，不是图像边界，不触发。
 *                 （赛道宽约 20px，居中时右边线 x ≈ 104，偏右最多到 x ≈ 150，
 *                  离 IMG_W-1=187 仍有距离，CORNER_EDGE_TOL 容差够用）
 *
 * 扫描范围：CORNER_SCAN_START 向上至 FAR_ROI_START，覆盖整个 ROI。
 */
static Path_TypeDef detect_corner_by_inflection(unsigned char binary_img[IMG_H][IMG_W])
{
    int right_bound_cnt = 0;  // 右边界触边行数
    int left_bound_cnt  = 0;  // 左边界触边行数

    for (int y = CORNER_SCAN_START; y >= FAR_ROI_START; y--)
    {
        // --- 找本行最右侧白色像素 ---
        int rightmost = -1;
        for (int x = IMG_W - 1; x >= 0; x--)
        {
            if (binary_img[y][x] > PATH_THRESHOLD) { rightmost = x; break; }
        }
        if (rightmost >= IMG_W - 1 - CORNER_EDGE_TOL)
            right_bound_cnt++;

        // --- 找本行最左侧白色像素 ---
        int leftmost = -1;
        for (int x = 0; x < IMG_W; x++)
        {
            if (binary_img[y][x] > PATH_THRESHOLD) { leftmost = x; break; }
        }
        if (leftmost != -1 && leftmost <= CORNER_EDGE_TOL)
            left_bound_cnt++;
    }

    if (right_bound_cnt >= CORNER_BOUND_ROWS) return PATH_TYPE_TURN_RIGHT;
    if (left_bound_cnt  >= CORNER_BOUND_ROWS) return PATH_TYPE_TURN_LEFT;

    return PATH_TYPE_STRAIGHT;
}

/**
 * @brief 路径类型识别与中线提取（完整修改版）
 *
 * === 与原版的关键差异 ===
 *
 * [Bug修复] 原版在此函数内部对 ROI 区域重新从原始摄像头数据二值化（第382-384行），
 *           完全覆盖了 Image_Processing_And_Get_Center 中已完成的形态学闭运算结果，
 *           导致元器件干扰抑制完全失效。
 *           → 修复：删除 ROI 重二值化，直接使用调用方传入的已清洁 binary_img。
 *
 * [新增] 逐行追踪成功后，对追踪到的赛道内部执行 fill_row_interior()：
 *        将 [left_x, right_x] 范围内的像素全部置白，消除形态学未能完全去除的
 *        残余元器件黑点，防止干扰上方行的边界搜索起始坐标。
 *
 * [新增] 追踪失败行（元器件完全遮挡导致丢线）通过 interpolate_failed_rows()
 *        线性插值恢复 path_center_x，使偏差计算连续、斜率判断稳定，
 *        避免短暂丢线被错误识别为转弯。
 *
 * [新增] 记录 last_actual_left / last_actual_right（实际最近有效左右边线），
 *        在连续失败行数 ≤ 3 时对该行做保守性预填充（±5px 余量），
 *        帮助追踪算法越过元器件遮挡区域继续工作。
 *
 * @param binary_img  已经过 OTSU + 多次形态学闭运算处理的二值化图像
 * @return            识别到的路径类型
 */
Path_TypeDef recognize_path_type(unsigned char binary_img[IMG_H][IMG_W])
{
    // ---------------------------------------------------------------
    // 【关键修改】：此处不再重新二值化 ROI 区域。
    // 原版代码在此处对每一行执行：
    //   erzhihua[y][x] = (mt9v03x_image_1[y][x] > th) ? 255 : 0;
    // 这会覆盖 Image_Processing_And_Get_Center 中多次闭运算的结果，
    // 使元器件干扰重新出现。现直接使用传入的 binary_img。
    // ---------------------------------------------------------------

    int valid_cnt = 0;
    int buf_x[ROI_ROW_NUM], buf_y[ROI_ROW_NUM];

    // 初始化种子坐标（从图像底部中心出发）
    int seed_x = IMG_W / 2;
    if (binary_img[ROI_ROW_END][seed_x] <= PATH_THRESHOLD)
    {
        for (int i = 1; i < IMG_W / 2; i++)
        {
            if (binary_img[ROI_ROW_END][seed_x + i] > PATH_THRESHOLD) { seed_x += i; break; }
            if (binary_img[ROI_ROW_END][seed_x - i] > PATH_THRESHOLD) { seed_x -= i; break; }
        }
    }

    int last_l = seed_x;
    int last_r = seed_x;

    // 记录最近一次实际追踪成功的左右边线（用于预填充失败行）
    int last_actual_left  = -1;
    int last_actual_right = -1;

    // 连续失败行计数（超过阈值才认为是路口/丢线，而非元器件遮挡）
    int consecutive_fail = 0;

    // ---------------------------------------------------------------
    // 阶段1：从近处往远处逐行追踪，同时执行行内填充
    // ---------------------------------------------------------------
    for (int y = ROI_ROW_END; y >= FAR_ROI_START; y--)
    {
        if (scan_row_tracking(y, binary_img, &path_left_x[y], &path_right_x[y], last_l, last_r))
        {
            // --- 追踪成功 ---

            int width = path_right_x[y] - path_left_x[y];

            // ---------------- 行级异常宽度检测 ----------------
            // 目标：把“大块元器件/多条细线”对应的异常行直接视为失败行，
            //      交给后续插值去补，避免被误判为分支/有效赛道。
            bool row_abnormal = false;

            // 1) 绝对宽度过大：几乎整行都被覆盖，基本不可能是真实赛道宽
            if (width > IMG_W * 3 / 4)
            {
                row_abnormal = true;
            }

            // 2) 相对宽度与最近一次真实赛道宽度相差过大
            if (!row_abnormal && last_actual_left != -1)
            {
                int ref_width = last_actual_right - last_actual_left;
                if (ref_width > 8) // 过滤极端近距离噪声
                {
                    // 允许 [0.5x, 2.0x] 的变化，超出则视为异常
                    if (width < ref_width / 2 || width > ref_width * 2)
                    {
                        row_abnormal = true;
                    }
                }
            }

            if (row_abnormal)
            {
                // 当作失败行处理，交给插值逻辑；同时计入连续失败行
                path_left_x[y]   = -1;
                path_right_x[y]  = -1;
                path_center_x[y] = -1;
                consecutive_fail++;
            }
            else
            {
                // 【正常行】行内填充：将赛道内部像素置白
                // 消除形态学闭运算未能完全去除的残余元器件黑点，
                // 避免当前行内部噪点成为上方行追踪时的虚假边界
                fill_row_interior(binary_img, y, path_left_x[y], path_right_x[y]);

                path_center_x[y] = (path_left_x[y] + path_right_x[y]) / 2;

                // 更新下一行的追踪种子（使用当前行中心，维持区域生长连续性）
                last_l = path_center_x[y];
                last_r = path_center_x[y];

                // 记录真实边线坐标（供失败行预填充使用）
                last_actual_left  = path_left_x[y];
                last_actual_right = path_right_x[y];
                consecutive_fail  = 0;
            }

            if (y >= ROI_ROW_START && y <= ROI_ROW_END)
            {
                buf_x[valid_cnt] = path_center_x[y];
                buf_y[valid_cnt] = y;
                valid_cnt++;
            }
        }
        else
        {
            // --- 追踪失败（可能是元器件遮挡） ---
            path_left_x[y]   = -1;
            path_right_x[y]  = -1;
            path_center_x[y] = -1;
            consecutive_fail++;

            // 【新增】保守性预填充：
            // 若失败行数 ≤ 3（判断为短暂元器件遮挡而非真实路口），
            // 基于最近有效边线做 ±5px 余量的行内填充，
            // 帮助追踪算法在遮挡结束后快速恢复
            if (consecutive_fail <= 3 && last_actual_left != -1)
            {
                int fill_l = last_actual_left  - 5;
                int fill_r = last_actual_right + 5;
                if (fill_l < 0)      fill_l = 0;
                if (fill_r >= IMG_W) fill_r = IMG_W - 1;
                fill_row_interior(binary_img, y, fill_l, fill_r);
            }
        }
    }

    // ---------------------------------------------------------------
    // 阶段2：对 path_center_x 中的失败行（-1）做线性插值恢复
    //
    // 目的：当元器件导致连续几行 center_x == -1 时，
    //       插值确保偏差计算连续，斜率计算不被"空洞"拉偏，
    //       整体路径仍被正确识别为直线。
    //       left_x / right_x 保持 -1，不影响路口检测逻辑。
    // ---------------------------------------------------------------
    interpolate_failed_rows(path_center_x, FAR_ROI_START, ROI_ROW_END);

    // ---------------------------------------------------------------
    // 阶段3：路口（拐点+矩形边框）检测
    // 注：此处使用原始 path_left_x（未插值），保留路口检测精度
    // ---------------------------------------------------------------
    int inf_x = -1, inf_y = -1;
    for (int y = ROI_ROW_END; y > ROI_ROW_START + 5; y--)
    {
        if (path_left_x[y] != -1 && path_left_x[y - 3] != -1)
        {
            if (abs(path_left_x[y - 3] - path_left_x[y]) > 15)
            {
                inf_x = path_left_x[y];
                inf_y = y;
                break;
            }
        }
    }

    if (inf_x != -1)
    {
        uint8_t branches = count_box_intersections_fixed(binary_img, inf_x, inf_y, 20);
        if (branches >= 4) return PATH_TYPE_CROSSROAD;
        if (branches == 3) return PATH_TYPE_BRANCH;
    }

    // ---------------------------------------------------------------
    // 阶段4：计算偏差（使用插值后的 path_center_x，覆盖范围更完整）
    // ---------------------------------------------------------------
    if (valid_cnt < 4) return PATH_TYPE_UNKNOWN;

    float n_sum = 0, f_sum = 0;
    int   n_cnt = 0, f_cnt = 0;

    for (int y = FAR_ROI_START; y <= ROI_ROW_END; y++)
    {
        // 插值后 center_x 应在合法范围内才纳入统计
        if (path_center_x[y] > 0 && path_center_x[y] < IMG_W)
        {
            float err = (float)path_center_x[y] - (IMG_W / 2.0f);
            if (y >= ROI_ROW_START && y <= ROI_ROW_END) { n_sum += err; n_cnt++; }
            if (y >= FAR_ROI_START && y <= FAR_ROI_END) { f_sum += err; f_cnt++; }
        }
    }

    if (n_cnt > 0) near_image_error = n_sum / (float)n_cnt;
    if (f_cnt > 0) far_image_error  = f_sum / (float)f_cnt;

    // 调试显示
    ips200_show_int  (0,   160, valid_cnt,          3);
    ips200_show_int  (120, 200, current_path_type,  1);
    ips200_show_float(120, 220, near_image_error, 3, 1);
    ips200_show_float(120, 240, far_image_error,  3, 1);

    // ---------------------------------------------------------------
    // 阶段5：直角弯识别
    // 偏置在 motor.c 的 Motor_SteeringLayer 中完成（TURN_BIAS_PX）
    // ---------------------------------------------------------------
    return detect_corner_by_inflection(binary_img);
}

/**
 * @brief 使用逐飞助手协议发送二值化图像和识别到的三条边线
 */
void Send_Image_To_ZF_Assistant(void)
{
    static int     path_y_coords[ROI_ROW_NUM];
    static uint8_t is_initialized = 0;

    if (!is_initialized)
    {
        for (int i = 0; i < ROI_ROW_NUM; i++)
            path_y_coords[i] = ROI_ROW_START + i;
        is_initialized = 1;
    }

    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        &erzhihua[0][0],
        IMG_W,
        IMG_H
    );

    seekfree_assistant_camera_boundary_config(
        XY_BOUNDARY,
        ROI_ROW_NUM,
        &path_center_x[ROI_ROW_START],
        &path_left_x[ROI_ROW_START],
        &path_right_x[ROI_ROW_START],
        path_y_coords,
        path_y_coords,
        path_y_coords
    );

    seekfree_assistant_camera_send();
    seekfree_assistant_data_analysis();
}

void Show_Image_On_IPS200(void)
{
    ips200_show_gray_image(0, 0, &erzhihua[0][0], IMG_W, IMG_H, IMG_W, IMG_H, 0);

    for (int row = 0; row < IMG_H; row++)
    {
        if (path_left_x[row]   != -1) ips200_draw_point((uint16)path_left_x[row],   (uint16)row, RGB565_GREEN);
        if (path_right_x[row]  != -1) ips200_draw_point((uint16)path_right_x[row],  (uint16)row, RGB565_BLUE);
        if (path_center_x[row] != -1) ips200_draw_point((uint16)path_center_x[row], (uint16)row, RGB565_RED);
    }

    // 实时统计触边行数并显示，调参时观察直行/弯道时的数值差异
    // 黄色点 = 本帧中右侧触边的行（最右白色像素 >= IMG_W-1-CORNER_EDGE_TOL）
    int right_cnt = 0;
    for (int y = CORNER_SCAN_START; y >= FAR_ROI_START; y--)
    {
        for (int x = IMG_W - 1; x >= 0; x--)
        {
            if (erzhihua[y][x] > PATH_THRESHOLD)
            {
                if (x >= IMG_W - 1 - CORNER_EDGE_TOL)
                {
                    ips200_draw_point((uint16)(IMG_W - 1), (uint16)y, RGB565_YELLOW);
                    right_cnt++;
                }
                break;
            }
        }
    }
    // 左上角显示触边行数，直行时应为 0，弯道时应 >= CORNER_BOUND_ROWS
    ips200_show_int(0, 140, right_cnt, 3);
}
