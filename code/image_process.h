#ifndef IMAGE_PROCESS_H
#define IMAGE_PROCESS_H

#include "zf_common_headfile.h"

//---适配总钻风使用的MT9V03X摄像头参数定义---
#define IMG_W MT9V03X_1_W //188
#define IMG_H MT9V03X_1_H //120

//---适配120行ROI定义---
#define ROI_ROW_START 80
#define ROI_ROW_END   110
#define ROI_ROW_NUM   (ROI_ROW_END - ROI_ROW_START + 1)
#define FAR_ROI_START 40
#define FAR_ROI_END   60
#define FAR_ROI_NUM   (FAR_ROI_END - FAR_ROI_START + 1)

#define PATH_THRESHOLD  100
#define PATH_WIDTH_MIN  20
#define EDGE_CONTINUE_CNT 3
#define OUTLIER_THRESHOLD 10

// 转弯/分支识别阈值
#define TURN_SLOPE_LOW    0.1f   // 转弯判定最小斜率（>0.1则为转弯）
#define TURN_SLOPE_HIGH   0.5f   // 大角度转弯判定阈值
#define BRANCH_EDGE_CNT   2      // 分支判定：单行检测到≥2组边界
#define BRANCH_VALID_ROW  5      // 分支判定：连续≥5行检测到多边界

// 路径类型枚举
typedef enum {
    PATH_TYPE_UNKNOWN = 0,  // 未知路径
    PATH_TYPE_STRAIGHT,     // 直线路径
    PATH_TYPE_TURN_LEFT,    // 左转弯
    PATH_TYPE_TURN_RIGHT,   // 右转弯
    PATH_TYPE_BRANCH        // 分支道路
} Path_TypeDef;

// 全局变量（新增分支检测缓存）
extern unsigned char erzhihua[IMG_H][IMG_W]; // 二值化图像
extern int path_center_x[IMG_H];


extern int path_left_x[IMG_H];
extern int path_right_x[IMG_H];
extern int branch_valid_row_cnt;  // 分支有效行计数

extern volatile float near_image_error; // 近距离图像偏差（FAR_ROI区域）
extern volatile float far_image_error;  // 远距离图像偏差（FAR_ROI区域）

// 函数声明

void Image_Processing_And_Get_Center(void);
void Calculate_All_Errors(void);        // 计算近远距离图像偏差，更新 near_image_error 和 far_image_error 全局变量
Path_TypeDef recognize_path_type(unsigned char binary_img[IMG_H][IMG_W]);
void Send_Image_To_ZF_Assistant(void); // 通过逐飞助手协议发送二值化图像和识别到的三条边线（调试用）
void Show_Image_On_IPS200(void);


// 摄像头采集标志
extern uint8_t cam_capture_done;
// 摄像头初始化&采集函数声明
uint8_t TC264_CAM_Init(void);
void TC264_CAM_Capture_Frame(void);

#endif // IMAGE_PROCESS_H
