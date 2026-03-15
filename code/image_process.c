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
 * @brief 连通域标记与小面积黑洞填充
 * 目的：填充赛道内小于阈值的黑色元器件区域
 */
static void fill_small_black_holes(unsigned char binary_img[IMG_H][IMG_W], int min_area)
{
    unsigned char visited[IMG_H][IMG_W] = {0};
    
    // 遍历每个黑色像素，进行洪泛填充
    for (int i = 1; i < IMG_H - 1; i++) 
    {
        for (int j = 1; j < IMG_W - 1; j++) 
        {
            if (binary_img[i][j] <= PATH_THRESHOLD && !visited[i][j]) 
            {
                // 计算连通域面积
                int area = 0;
                int stack_ptr = 0;
                int stack_i[IMG_H * IMG_W], stack_j[IMG_H * IMG_W];
                
                stack_i[stack_ptr] = i;
                stack_j[stack_ptr] = j;
                stack_ptr++;
                visited[i][j] = 1;
                
                // 洪泛填充，统计连通域大小
                while (stack_ptr > 0) 
                {
                    stack_ptr--;
                    int cy = stack_i[stack_ptr];
                    int cx = stack_j[stack_ptr];
                    area++;
                    
                    // 检查4邻域（或8邻域）
                    for (int di = -1; di <= 1; di++) 
                    {
                        for (int dj = -1; dj <= 1; dj++) 
                        {
                            if (di == 0 && dj == 0) continue;
                            if (di != 0 && dj != 0) continue;  // 仅4邻域
                            
                            int ny = cy + di;
                            int nx = cx + dj;
                            if (ny >= 0 && ny < IMG_H && nx >= 0 && nx < IMG_W &&
                                !visited[ny][nx] && binary_img[ny][nx] <= PATH_THRESHOLD) 
                            {
                                visited[ny][nx] = 1;
                                stack_i[stack_ptr] = ny;
                                stack_j[stack_ptr] = nx;
                                stack_ptr++;
                            }
                        }
                    }
                }
                
                // 如果面积小于阈值，填充为白色（赛道）
                if (area < min_area) 
                {
                    for (int ii = 0; ii < IMG_H; ii++) 
                    {
                        for (int jj = 0; jj < IMG_W; jj++) 
                        {
                            if (visited[ii][jj]) 
                            {
                                binary_img[ii][jj] = 255;  // 填充为白色
                            }
                        }
                    }
                }
            }
        }
    }
}

/**
 * @brief 改进的图像处理与中线获取
 */
uint8_t Image_Processing_And_Get_Center(void)
{
    if (!mt9v03x_finish_flag_1)
        return 0;

    mt9v03x_finish_flag_1 = 0;

    uint8_t threshold = otsu_threshold();

    // 二值化处理
    for (int i = 0; i < IMG_H; i++)
        for (int j = 0; j < IMG_W; j++)
            erzhihua[i][j] = (mt9v03x_image_1[i][j] > threshold) * 255;

    //  关键优化：填充赛道内小黑洞（元器件）
    // min_area 可根据实际调整，建议 20-50 像素
    fill_small_black_holes(erzhihua, 30);

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



// 优化的中心扩散与边缘跟踪算法
static uint8_t scan_row_tracking(uint16_t row, unsigned char binary_img[IMG_H][IMG_W],
                                 int* left_x, int* right_x, int last_left, int last_right)       
{
    *left_x = -1;
    *right_x = -1;
    
    int start_l = (last_left != -1) ? last_left : (IMG_W / 2);
    int start_r = (last_right != -1) ? last_right : (IMG_W / 2);

    // 搜索左边界 (允许最多跨越 2 像素的噪点，针对20cm高度)
    int black_noise_cnt = 0;
    for (int x = start_l; x >= 0; x--)
    {
        if (binary_img[row][x] <= PATH_THRESHOLD) 
        {
            black_noise_cnt++;
            if (black_noise_cnt > 2)  // 从 3 改为 2
            {
                *left_x = x + 2;  // 从 +3 改为 +2
                break;
            }
        } else {
            black_noise_cnt = 0;
        }
    }

    // 搜索右边界同理
    black_noise_cnt = 0;
    for (int x = start_r; x < IMG_W; x++)
    {
        if (binary_img[row][x] <= PATH_THRESHOLD) 
        {
            black_noise_cnt++;
            if (black_noise_cnt > 2)  // 从 3 改为 2
            {
                *right_x = x - 2;  // 从 -3 改为 -2
                break;
            }
        } else {
            black_noise_cnt = 0; 
        }
    }

    return (*left_x != -1 && *right_x != -1 && (*right_x - *left_x) >= PATH_WIDTH_MIN) ? 1 : 0;
}
/**
 * @brief 环形边框交点统计函数定义
 * 请确保该函数名与 recognize_path_type 中调用的名称完全一致
 */
static uint8_t count_box_intersections_fixed(unsigned char bin_img[IMG_H][IMG_W], int cx, int cy, int size)
{
    int half = size / 2;
    int l = cx - half, r = cx + half, t = cy - half, b = cy + half;
    
    // 边界限幅保护，防止坐标超出数组范围导致硬件异常
    if (l < 0) l = 0; if (r >= IMG_W) r = IMG_W - 1;
    if (t < 0) t = 0; if (b >= IMG_H) b = IMG_H - 1;

    uint8_t jumps = 0;
    // 判定逻辑：由于是二值化图，直接判断是否等于255（白色）
    uint8_t last = (bin_img[t][l] == 255);
    uint8_t first = last;

    // 1. 顶边扫描 (左->右)
    for (int x = l + 1; x <= r; x++) { 
        uint8_t c = (bin_img[t][x] == 255); 
        if (c && !last) jumps++; 
        last = c; 
    }
    // 2. 右边扫描 (上->下)
    for (int y = t + 1; y <= b; y++) { 
        uint8_t c = (bin_img[y][r] == 255); 
        if (c && !last) jumps++; 
        last = c; 
    }
    // 3. 底边扫描 (右->左)
    for (int x = r - 1; x >= l; x--) { 
        uint8_t c = (bin_img[b][x] == 255); 
        if (c && !last) jumps++; 
        last = c; 
    }
    // 4. 左边扫描 (下->上)
    for (int y = b - 1; y > t; y--)  { 
        uint8_t c = (bin_img[y][l] == 255); 
        if (c && !last) jumps++; 
        last = c; 
    }
    
    // 环形闭合检测
    if (first && !last) jumps++;
    
    return jumps;
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
/**
 * @brief 最终优化版识别函数
 */
Path_TypeDef recognize_path_type(unsigned char binary_img[IMG_H][IMG_W])
{
    uint8_t th = otsu_threshold();
    int valid_cnt = 0;
    int buf_x[ROI_ROW_NUM], buf_y[ROI_ROW_NUM];
    
    int seed_x = IMG_W / 2;
        // 如果底行中心是黑色的（车子偏了），向两边扫描找白色的赛道
        if (binary_img[ROI_ROW_END][seed_x] <= PATH_THRESHOLD) {
            for (int i = 1; i < IMG_W / 2; i++) {
                if (binary_img[ROI_ROW_END][seed_x + i] > PATH_THRESHOLD) { seed_x = seed_x + i; break; }
                if (binary_img[ROI_ROW_END][seed_x - i] > PATH_THRESHOLD) { seed_x = seed_x - i; break; }
            }
        }
        // 将找到的白点作为追踪的初始起点
        int last_l = seed_x;
        int last_r = seed_x;

    // 1. 局部二值化与带噪点容错的边界追踪 (从近处往远处扫)
    for (int y = ROI_ROW_END; y >= FAR_ROI_START; y--) 
    {
        // 仅对 ROI 涉及的行进行二值化，提升 TC264 效率
        for (int x = 0; x < IMG_W; x++) {
            erzhihua[y][x] = (mt9v03x_image_1[y][x] > th) ? 255 : 0;
        }

        // 调用升级方案中的 scan_row_tracking，跳过元器件噪点
        if (scan_row_tracking(y, erzhihua, &path_left_x[y], &path_right_x[y], last_l, last_r)) 
        {
            path_center_x[y] = (path_left_x[y] + path_right_x[y]) / 2;
            last_l = path_center_x[y];
            last_r = path_center_x[y];
            if (y >= ROI_ROW_START && y <= ROI_ROW_END) {
                buf_x[valid_cnt] = path_center_x[y];
                buf_y[valid_cnt] = y;
                valid_cnt++;
            }
        } else {
            path_left_x[y] = -1; path_right_x[y] = -1; path_center_x[y] = -1;
        }
    }

    // 2. 增强型特殊路口识别
    int inf_x = -1, inf_y = -1;
    // 在近处寻找明显的拐点（突变阈值增加到 15，防止因元器件边缘产生细微抖动误判）
    for (int y = ROI_ROW_END; y > ROI_ROW_START + 5; y--) {
        if (path_left_x[y] != -1 && path_left_x[y-3] != -1) {
            if (abs(path_left_x[y-3] - path_left_x[y]) > 15) { 
                inf_x = path_left_x[y]; inf_y = y; break; 
            }
        }
    }

    if (inf_x != -1) {
        // 使用修正后的环形扫描
        uint8_t branches = count_box_intersections_fixed(erzhihua, inf_x, inf_y, 20);
        if (branches >= 4) return PATH_TYPE_CROSSROAD;
        if (branches == 3) return PATH_TYPE_BRANCH;
    }

    // 3. 常规路径判定
    if (valid_cnt < 4) return PATH_TYPE_UNKNOWN; // 增加有效行要求，防止误判
    float slope = calculate_center_slope(buf_x, buf_y, valid_cnt);

        // --- 偏差计算逻辑开始 ---
    float n_sum = 0, f_sum = 0;
    int n_cnt = 0, f_cnt = 0;

    // 遍历从远端起始到近端结束的所有行
    for (int y = FAR_ROI_START; y <= ROI_ROW_END; y++) 
    {
        // 只有当这一行成功识别到了中线时才参与计算
        if (path_center_x[y] != -1) 
        {
            // 计算当前行偏差：(实际中心 - 图像理论中心)
            // 图像宽度为 188，所以中心是 94。
            // 结果 > 0 说明线在右边，小车应向右修正；< 0 说明线在左边。
            float current_row_err = (float)path_center_x[y] - (IMG_W / 2.0f);

            // 归入近端误差 (ROI: 80-110行)
            if (y >= ROI_ROW_START && y <= ROI_ROW_END) {
                n_sum += current_row_err;
                n_cnt++;
            }
            // 归入远端误差 (FAR_ROI: 40-60行)
            if (y >= FAR_ROI_START && y <= FAR_ROI_END) {
                f_sum += current_row_err;
                f_cnt++;
            }
        }
    }

    // 计算平均值，防止除以0
    // 如果某区域完全没识别到线（cnt==0），建议保持 0 或上一次的值
    if (n_cnt > 0) near_image_error = n_sum / (float)n_cnt;
     

    if (f_cnt > 0) far_image_error = f_sum / (float)f_cnt;
    
    // --- 偏差计算逻辑结束 ---
    ips200_show_int(0,160, valid_cnt, 3);
    ips200_show_int(120, 200, current_path_type,1);
    ips200_show_float(120, 220, near_image_error, 3, 1);
    ips200_show_float(120, 240, far_image_error, 3, 1);

    if (fabs(slope) < TURN_SLOPE_LOW) return PATH_TYPE_STRAIGHT;
    return (slope < 0) ? PATH_TYPE_TURN_RIGHT : PATH_TYPE_TURN_LEFT;
    

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
