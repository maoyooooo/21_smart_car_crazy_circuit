#include "image_process.h"
#include "zf_common_headfile.h"

unsigned char erzhihua[IMG_H][IMG_W] = {0}; // 二值化图像
int path_center_x[IMG_H] = {0};
int path_left_x[IMG_H] = {0};
int path_right_x[IMG_H] = {0};
int branch_valid_row_cnt = 0;
uint8_t cam_capture_done = 0;
Path_TypeDef current_path_type = PATH_TYPE_UNKNOWN;

/**
 * @brief OTSU 大津法计算动态二值化阈值
 * @return 最佳二值化阈值 (0-255)
 */
static uint8_t otsu_threshold(void)
{
    uint32_t histogram[256] = {0};
    uint32_t total_pixels = IMG_H * IMG_W;

    // 统计灰度直方图
    for (int i = 0; i < IMG_H; i++)
        for (int j = 0; j < IMG_W; j++)
            histogram[mt9v03x_image_1[i][j]]++;

    uint32_t sum_all = 0;  // 所有像素灰度总和
    for (int t = 0; t < 256; t++)
        sum_all += t * histogram[t];

    uint32_t sum_bg = 0;   // 背景像素灰度总和
    uint32_t cnt_bg = 0;   // 背景像素数量
    float max_variance = 0;
    uint8_t best_threshold = 100; // 默认阈值

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
 * @brief 图像处理及获取中线
 * @return 1=处理了新图像，0=无新图像
 */
uint8_t Image_Processing_And_Get_Center(void)
{
    if (!mt9v03x_finish_flag_1)
        return 0;

    mt9v03x_finish_flag_1 = 0;

    // OTSU 动态计算阈值
    uint8_t threshold = otsu_threshold();

    // 二值化处理
    for (int i = 0; i < IMG_H; i++)
        for (int j = 0; j < IMG_W; j++)
            erzhihua[i][j] = (mt9v03x_image_1[i][j] > threshold) * 255;

    // 识别路径类型并提取中线
    current_path_type = recognize_path_type(erzhihua);

    return 1;
}

volatile float near_image_error = 0; // 近距离图像偏差（FAR_ROI区域）
volatile float far_image_error = 0;  // 远距离图像偏差（FAR_ROI区域）

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

    // 更新全局变量
    // 1. 先在局部变量中计算好结果，不要直接写全局变量
    float near_temp = (near_cnt > 5) ? (near_total / near_cnt) : 0;
    float far_temp  = (far_cnt > 5)  ? (far_total / far_cnt)   : 0;

    // 2. 进入临界区：关闭中断
    // 获取当前中断状态并关闭中断，防止在此期间被 ISR 打断
    uint32 cpu_status = interrupt_global_disable();

    // 3. 原子更新全局变量
    near_image_error = near_temp;
    far_image_error  = far_temp;

    // 4. 退出临界区：恢复中断
    interrupt_global_enable(cpu_status);
}

/**
 * 辅助函数：单行边界扫描（已优化边界扫描逻辑）
 */
static uint8_t scan_row_boundary(uint16_t row, unsigned char binary_img[IMG_H][IMG_W],
                                 int* left_x, int* right_x)       
{
    *left_x = -1;
    *right_x = -1;

    // 左边界：连续EDGE_CONTINUE_CNT个目标像素
    for (int x = 0; x < IMG_W - EDGE_CONTINUE_CNT; )
    {
        int i;
        for (i = 0; i < EDGE_CONTINUE_CNT; i++)
        {
            if (binary_img[row][x + i] <= PATH_THRESHOLD) break;  
        }
        if (i >= EDGE_CONTINUE_CNT)
        {
            *left_x = x + EDGE_CONTINUE_CNT / 2;
            break;
        }
        x += (i + 1);  // 加速跳过已扫描的非路径像素
    }

    // 右边界
    for (int x = IMG_W - 1; x >= EDGE_CONTINUE_CNT; )
    {
        int i;
        for (i = 0; i < EDGE_CONTINUE_CNT; i++)
        {
            if (binary_img[row][x - i] <= PATH_THRESHOLD) break;  
        }
        if (i >= EDGE_CONTINUE_CNT)
        {
            *right_x = x - EDGE_CONTINUE_CNT / 2;
            break;
        }
        x -= (i + 1);
    }

    return (*left_x != -1 && *right_x != -1 && (*right_x - *left_x) >= PATH_WIDTH_MIN) ? 1 : 0;
}

/**
 * 辅助函数：单行多边界扫描（基于白像素连续段提取）
 */
uint8_t scan_row_multi_boundary(uint16_t row, unsigned char binary_img[IMG_H][IMG_W],
                                int* left1, int* right1, int* left2, int* right2)
{
    *left1 = *right1 = *left2 = *right2 = -1;
    int edge_cnt = 0;

    int* left_arr[2]  = {left1, left2};
    int* right_arr[2] = {right1, right2};

    int seg_start = -1;  // 记录当前连续白块的起始位置
    int white_run = 0;   // 记录连续白块的长度

    for (int x = 0; x < IMG_W; x++)
    {
        if (binary_img[row][x] > PATH_THRESHOLD)
        {
            white_run++;
            if (white_run == EDGE_CONTINUE_CNT && seg_start == -1)
            {
                // 确认有效白块起始点
                seg_start = x - EDGE_CONTINUE_CNT + 1;
            }
        }
        else
        {
            // 遇到黑点，结算上一个白块
            if (seg_start != -1)
            {
                int seg_end = x - 1;
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

    // 边界结算处理，防止白块连到最右侧边缘没有结算
    if (seg_start != -1)
    {
        int seg_end = IMG_W - 1;
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
 * 最小二乘法计算中线斜率（线性回归拟合）
 * slope = (N*sum(xy) - sum(x)*sum(y)) / (N*sum(x^2) - (sum(x))^2)
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
 * 核心函数：识别路径类型并提取中线
 */
Path_TypeDef recognize_path_type(unsigned char binary_img[IMG_H][IMG_W])
{
    int valid_row_cnt = 0;
    int center_x_buf[ROI_ROW_NUM] = {0};
    int row_y_buf[ROI_ROW_NUM] = {0};
    branch_valid_row_cnt = 0;

    for(int i = 0; i < IMG_H; i++) {
        path_center_x[i] = -1;
        path_left_x[i] = -1;
        path_right_x[i] = -1;
    }

    for (int row = FAR_ROI_START; row <= ROI_ROW_END; row++)      
    {
        int left1, right1, left2, right2;
        uint8_t edge_cnt = scan_row_multi_boundary(row, binary_img, &left1, &right1, &left2, &right2);

        if (row >= ROI_ROW_START && row <= ROI_ROW_END)
        {
            if (edge_cnt >= BRANCH_EDGE_CNT)
            {
                branch_valid_row_cnt++;
            }

            if (edge_cnt >= 1)
            {
                center_x_buf[valid_row_cnt] = (left1 + right1) / 2;
                row_y_buf[valid_row_cnt] = row;
                valid_row_cnt++;
            }
        }

        if (edge_cnt >= 1)
        {
            path_center_x[row] = (left1 + right1) / 2;
            path_left_x[row] = left1;
            path_right_x[row] = right1;
        }
    }

    if (branch_valid_row_cnt >= BRANCH_VALID_ROW)
    {
        return PATH_TYPE_BRANCH;
    }

    if (valid_row_cnt < ROI_ROW_NUM / 2)
    {
        return PATH_TYPE_UNKNOWN;
    }

    int filtered_cnt = 0;
    int filtered_x[ROI_ROW_NUM] = {0};
    int filtered_y[ROI_ROW_NUM] = {0};
    int center_mean = 0;
    for (int i = 0; i < valid_row_cnt; i++)
    {
        center_mean += center_x_buf[i];
    }
    center_mean /= valid_row_cnt;
    
    for (int i = 0; i < valid_row_cnt; i++)
    {
        if (abs(center_x_buf[i] - center_mean) <= OUTLIER_THRESHOLD)
        {
            filtered_x[filtered_cnt] = center_x_buf[i];
            filtered_y[filtered_cnt] = row_y_buf[i];
            filtered_cnt++;
        }
    }
    if (filtered_cnt < valid_row_cnt / 2)
    {
        return PATH_TYPE_UNKNOWN;
    }

    float slope = calculate_center_slope(filtered_x, filtered_y, filtered_cnt);

    if (fabs(slope) < TURN_SLOPE_LOW)
    {
        return PATH_TYPE_STRAIGHT;
    }
    else if (slope < -TURN_SLOPE_LOW)
    {
        return PATH_TYPE_TURN_LEFT;
    }
    else if (slope > TURN_SLOPE_LOW)
    {
        return PATH_TYPE_TURN_RIGHT;
    }
    else
    {
        return PATH_TYPE_UNKNOWN;
    }
}

/**
 * @brief 使用逐飞助手协议发送二值化图像和识别到的三条边线
 */
void Send_Image_To_ZF_Assistant(void)
{
    static int path_y_coords[ROI_ROW_NUM];
    static uint8_t is_initialized = 0;

    if (!is_initialized) 
    {
        for (int i = 0; i < ROI_ROW_NUM; i++) 
        {
            path_y_coords[i] = ROI_ROW_START + i; 
        }
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

/**
 * @brief 在IPS200屏幕上显示二值化图像并画出中线、边线
 */
void Show_Image_On_IPS200(void)
{
    ips200_show_gray_image(0, 0, &erzhihua[0][0], IMG_W, IMG_H, IMG_W, IMG_H, 0);

    for (int row = 0; row < IMG_H; row++)
    {
        if (path_left_x[row] != -1)
        {
            ips200_draw_point((uint16)path_left_x[row], (uint16)row, RGB565_GREEN);
        }
        
        if (path_right_x[row] != -1)
        {
            ips200_draw_point((uint16)path_right_x[row], (uint16)row, RGB565_BLUE);
        }

        if (path_center_x[row] != -1)
        {
            ips200_draw_point((uint16)path_center_x[row], (uint16)row, RGB565_RED);
        }
    }
}
