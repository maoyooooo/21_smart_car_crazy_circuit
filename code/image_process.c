#include "image_process.h"
#include "zf_common_headfile.h"

unsigned char huidu[IMG_H][IMG_W] = {0};    
unsigned char erzhihua[IMG_H][IMG_W] = {0}; // 二值化图像
int path_center_x[ROI_ROW_NUM] = {0};
int path_left_x[ROI_ROW_NUM] = {0};
int path_right_x[ROI_ROW_NUM] = {0};
int branch_valid_row_cnt = 0;
uint8_t cam_capture_done = 0;

static uint8_t CAM_Read_Pixel(void);                    //读取单个像素值（8位灰度）


void Image_Processing_And_Get_Center(void)
{
    // 1. 获取总钻风图像（底层库直接把DMA数据传给数组）

    // 2. 二值化处理（参考你队友原来的逻辑，但适配新数组）

    if (mt9v03x_finish_flag_1) // 确保图像采集完成
    {
        mt9v03x_finish_flag_1 = 0; // 重置标志位，准备下一帧采集
            for(int i = 0; i < IMG_H; i++) 
            {
                for(int j = 0; j < IMG_W; j++) 
                {
                    if(mt9v03x_image_1[i][j] > 100) // 100是阈值
                        {erzhihua[i][j] = 255;} // 白
                    else
                        {erzhihua[i][j] = 0;}   // 黑
                }
            }

    
    
    }
    

    // 3. 调用你队友写的核心算法计算中线
    recognize_path_type(erzhihua);
}


/**
 * @brief 获取最终用于 PID 的偏差值
 * @return float: 0代表正中心，正数代表偏右，负数代表偏左
 */
volatile float near_image_error = 0; // 近距离图像偏差（FAR_ROI区域）
volatile float far_image_error = 0;  // 远距离图像偏差（FAR_ROI区域）

void Calculate_All_Errors(void)
{
    float near_total = 0, far_total = 0;
    int near_cnt = 0, far_cnt = 0;
    int image_center = IMG_W / 2;

    // 假设 path_center_x 数组已经存储了全图 120 行的中线（你需要确保 recognize_path_type 处理了全图）
    for (int i = 0; i < IMG_H; i++)
    {
        if (path_center_x[i] > 0 && path_center_x[i] < IMG_W)
        {
            // 计算近端偏差 (ROI: 80-110)
            if (i >= ROI_ROW_START && i <= ROI_ROW_END)
            {
                near_total += (float)(path_center_x[i] - image_center);
                near_cnt++;
            }
            // 计算远端偏差 (ROI: 40-60)
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
    // 因为中断已关，这两个赋值操作绝对不会被拆开
    near_image_error = near_temp;
    far_image_error  = far_temp;

    // 4. 退出临界区：恢复中断
    interrupt_global_enable(cpu_status);
}





/**
 * 基础函数：单行单边界扫描（保留，用于直/转弯识别）
 */
static uint8_t scan_row_boundary(uint16_t row, unsigned char binary_img[IMG_H][IMG_W], 
                                 int* left_x, int* right_x)
{
    *left_x = -1;
    *right_x = -1;
    int continue_cnt = 0;

    // 左边界：连续EDGE_CONTINUE_CNT个目标像素
    for (int x = 0; x < IMG_W - EDGE_CONTINUE_CNT; x++)
    {
        continue_cnt = 0;
        for (int i = 0; i < EDGE_CONTINUE_CNT; i++)
        {
            if (binary_img[row][x+i] > PATH_THRESHOLD) continue_cnt++;
            else break;
        }
        if (continue_cnt >= EDGE_CONTINUE_CNT)
        {
            *left_x = x + EDGE_CONTINUE_CNT/2;
            break;
        }
    }

    // 右边界：连续EDGE_CONTINUE_CNT个目标像素
    for (int x = IMG_W - 1; x >= EDGE_CONTINUE_CNT; x--)
    {
        continue_cnt = 0;
        for (int i = 0; i < EDGE_CONTINUE_CNT; i++)
        {
            if (binary_img[row][x-i] > PATH_THRESHOLD) continue_cnt++;
            else break;
        }
        if (continue_cnt >= EDGE_CONTINUE_CNT)
        {
            *right_x = x - EDGE_CONTINUE_CNT/2;
            break;
        }
    }

    return (*left_x != -1 && *right_x != -1 && (*right_x - *left_x) >= PATH_WIDTH_MIN) ? 1 : 0;
}

/**
 * 单行多边界扫描（用于分支道路识别）
 * @return 检测到的有效边界组数（1=单路径，2=分支，0=无效）
 */
uint8_t scan_row_multi_boundary(uint16_t row, unsigned char binary_img[IMG_H][IMG_W],
                                int* left1, int* right1, int* left2, int* right2)
{
    // 初始化
    *left1 = *right1 = *left2 = *right2 = -1;
    int edge_cnt = 0;
    int current_x = 0;

    // 扫描第一组边界
    if (scan_row_boundary(row, binary_img, left1, right1))
    {
        edge_cnt++;
        current_x = *right1 + PATH_WIDTH_MIN; // 跳过当前路径，扫描下一组
    }

    // 扫描第二组边界（需与第一组间距≥最小路径宽度）
    if (current_x < IMG_W - PATH_WIDTH_MIN)
    {
        // 从current_x开始扫描，避免重叠
        for (int x = current_x; x < IMG_W - EDGE_CONTINUE_CNT; x++)
        {
            int continue_cnt = 0;
            for (int i = 0; i < EDGE_CONTINUE_CNT; i++)
            {
                if (binary_img[row][x+i] > PATH_THRESHOLD) continue_cnt++;
                else break;
            }
            if (continue_cnt >= EDGE_CONTINUE_CNT)
            {
                *left2 = x + EDGE_CONTINUE_CNT/2;
                break;
            }
        }
        // 找第二组右边界
        if (*left2 != -1)
        {
            for (int x = IMG_W - 1; x >= *left2 + EDGE_CONTINUE_CNT; x--)
            {
                int continue_cnt = 0;
                for (int i = 0; i < EDGE_CONTINUE_CNT; i++)
                {
                    if (binary_img[row][x-i] > PATH_THRESHOLD) continue_cnt++;
                    else break;
                }
                if (continue_cnt >= EDGE_CONTINUE_CNT)
                {
                    *right2 = x - EDGE_CONTINUE_CNT/2;
                    break;
                }
            }
        }
        // 验证第二组边界有效性
        if (*left2 != -1 && *right2 != -1 && (*right2 - *left2) >= PATH_WIDTH_MIN)
        {
            edge_cnt++;
        }
    }

    return edge_cnt;
}

/**
 * 辅助函数：计算中心点序列的斜率（判断转弯方向）
 */
float calculate_center_slope(int* center_x, int* row_y, int cnt)
{
    if (cnt < 2) return 0.0f;

    // 最小二乘法计算斜率（简化版，只算首尾两点斜率，降低算力）
    float k = (center_x[cnt-1] - center_x[0]) / (float)(row_y[cnt-1] - row_y[0]);
    return k;
}

/**
 * 核心函数：识别路径类型（直/左拐/右拐/分支/未知）
 */
Path_TypeDef recognize_path_type(unsigned char binary_img[IMG_H][IMG_W])
{
    int valid_row_cnt = 0;
    int center_x_buf[ROI_ROW_NUM] = {0};
    int row_y_buf[ROI_ROW_NUM] = {0};
    branch_valid_row_cnt = 0;

    // 第一步：扫描ROI行，同时检测单边界（转弯）和多边界（分支）
    for (int i = 0; i < ROI_ROW_NUM; i++)
    {
        int row = ROI_ROW_START + i;
        int left1, right1, left2, right2;
        
        // 先检测分支（多边界）
        uint8_t edge_cnt = scan_row_multi_boundary(row, binary_img, &left1, &right1, &left2, &right2);
        if (edge_cnt >= BRANCH_EDGE_CNT)
        {
            branch_valid_row_cnt++; // 累计分支有效行
        }

        // 单边界数据（用于直/转弯识别）
        if (edge_cnt >= 1) // 至少有一组有效边界
        {
            center_x_buf[valid_row_cnt] = (left1 + right1) / 2;
            row_y_buf[valid_row_cnt] = row;
            path_center_x[i] = center_x_buf[valid_row_cnt];
            path_left_x[i] = left1;
            path_right_x[i] = right1;
            valid_row_cnt++;
        }
        else
        {
            path_center_x[i] = -1;
        }
    }

    // 第二步：优先判断分支道路（连续多行检测到多边界）
    if (branch_valid_row_cnt >= BRANCH_VALID_ROW)
    {
        return PATH_TYPE_BRANCH;
    }

    // 有效行过少，返回未知
    if (valid_row_cnt < ROI_ROW_NUM / 2)
    {
        return PATH_TYPE_UNKNOWN;
    }

    // 第三步：剔除异常中心点（抗干扰）
    int filtered_cnt = 0;
    int filtered_x[ROI_ROW_NUM] = {0};
    int filtered_y[ROI_ROW_NUM] = {0};
    int center_mean = 0;
    for (int i = 0; i < valid_row_cnt; i++)
    {
        center_mean += center_x_buf[i];
    }
    center_mean /= valid_row_cnt;
    // 过滤异常点
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

    // 第四步：计算斜率，判断直/转弯
    float slope = calculate_center_slope(filtered_x, filtered_y, filtered_cnt);
    
    // 直线路径：斜率绝对值<最小转弯斜率
    if (fabs(slope) < TURN_SLOPE_LOW)
    {
        return PATH_TYPE_STRAIGHT;
    }
    // 左转弯：斜率为负，且绝对值≥最小转弯斜率
    else if (slope < -TURN_SLOPE_LOW)
    {
        return PATH_TYPE_TURN_LEFT;
    }
    // 右转弯：斜率为正，且绝对值≥最小转弯斜率
    else if (slope > TURN_SLOPE_LOW)
    {
        return PATH_TYPE_TURN_RIGHT;
    }
    // 其他情况返回未知
    else
    {
        return PATH_TYPE_UNKNOWN;
    }
}

/**
 * @brief 使用逐飞助手协议发送二值化图像和识别到的三条边线
 * @note 建议在 CPU1 的 while(1) 循环中低频调用（例如每隔 50ms 发送一次），
 * 或者通过按键触发单帧发送，防止串口占用过高导致控制失常。
 */
void Send_Image_To_ZF_Assistant(void)
{
    // 由于你的边线数组长度是 ROI_ROW_NUM (31)，而图像高度是 120
    // 我们需要构造三个 Y 坐标数组，告诉上位机这些点对应的是哪些行
    static int path_y_coords[ROI_ROW_NUM];
    static uint8_t is_initialized = 0;

    // 只初始化一次 Y 坐标数组 (80 到 110 行)
    if (!is_initialized) 
    {
        for (int i = 0; i < ROI_ROW_NUM; i++) 
        {
            path_y_coords[i] = ROI_ROW_START + i; 
        }
        is_initialized = 1;
    }

    // 1. 配置图像基础信息
    // 告诉上位机：发的是 MT9V03X 类型的灰度图（二值图在上位机选灰度模式显示即可）
    // 数据源是 erzhihua，宽高是 188x120
    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X, 
        &erzhihua[0][0], 
        IMG_W, 
        IMG_H
    );

    // 2. 配置边线信息 (XY_BOUNDARY 表示你要同时发送 X 和 Y 坐标)
    // 我们发送三条线：中线(dot_x1), 左线(dot_x2), 右线(dot_x3)
    // 它们的 Y 坐标全是 path_y_coords
    seekfree_assistant_camera_boundary_config(
        XY_BOUNDARY,              // 发送带有 XY 坐标的点集
        ROI_ROW_NUM,              // 你的数组长度 (31个点)
        path_center_x,            // 中线 X 数组
        path_left_x,              // 左线 X 数组
        path_right_x,             // 右线 X 数组
        path_y_coords,            // 中线 Y 数组
        path_y_coords,            // 左线 Y 数组
        path_y_coords             // 右线 Y 数组
    );

    // 3. 执行物理发送 (底层会通过串口把上面配置的所有数据发出去)
    seekfree_assistant_camera_send();
}