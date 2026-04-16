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
#define PATH_WIDTH_MIN  8
#define EDGE_CONTINUE_CNT 2
#define OUTLIER_THRESHOLD 10

// 分支识别阈值
#define BRANCH_EDGE_CNT   2      // 分支判定：单行检测到≥2组边界
#define BRANCH_VALID_ROW  5      // 分支判定：连续≥5行检测到多边界

// 直角弯识别阈值（直接扫二值图像，不依赖边线追踪结果）
//
// 原拐点检测依赖 path_right_x[y]，但 scan_row_tracking 在拐弯区域
// 因几何形状改变（T形路口）返回 -1，导致整个检测链路数据为空。
//
// 新方法：直接逐行扫二值化图像，找每行最右侧白色像素位置。
// 直角弯衍生线出现时，某些行的最右侧白色像素会到达图像右边界（x≈187）。
// 统计这样的行数，达到 CORNER_BOUND_ROWS 即确认右转。
//
// 元器件过滤：元器件不会延伸到图像边界，最右侧白色像素不会触边。
//
// 调参说明：
//   CORNER_BOUND_ROWS   触边行数阈值（建议 2-5）
//                       衍生线约有 3-8 行触边，元器件 0 行触边
//   CORNER_EDGE_TOL     触边容差（允许最右侧几列为黑，建议 2-4）
//   CORNER_SCAN_START   开始检测的行（从下往上，建议 ROI_ROW_END 或略小）
//
#define CORNER_BOUND_ROWS    3   // 触边行数阈值
#define CORNER_EDGE_TOL      3   // 触边容差（px）
#define CORNER_SCAN_START  ROI_ROW_END  // 检测起始行（最靠近车）

// 贝塞尔曲线补线参数（检测到直角弯时，用曲线覆写中线，PID 跟线自然转弯）
#define TURN_BEZIER_SHIFT   130   // 贝塞尔终点偏移量（px，越大转弯越急，建议 40-80）

// 路径类型枚举
typedef enum {
    PATH_TYPE_UNKNOWN = 0,  // 未知路径
    PATH_TYPE_STRAIGHT,     // 直线路径
    PATH_TYPE_TURN_LEFT,    // 左转弯
    PATH_TYPE_TURN_RIGHT,   // 右转弯
    PATH_TYPE_BRANCH,       // T型路口 / 分支
    PATH_TYPE_CROSSROAD     // 十字路口 [新增]
} Path_TypeDef;

// 全局变量
extern unsigned char erzhihua[IMG_H][IMG_W]; // 二值化图像
extern int path_center_x[IMG_H];
extern int path_left_x[IMG_H];
extern int path_right_x[IMG_H];
extern int branch_valid_row_cnt;  // 分支有效行计数

extern volatile float near_image_error; // 近距离图像偏差（FAR_ROI区域）
extern volatile float far_image_error;  // 远距离图像偏差（FAR_ROI区域）

// 新增：当前识别到的路径类型
extern Path_TypeDef current_path_type;

// 函数声明

uint8_t Image_Processing_And_Get_Center(void);  // 返回1=处理了一帧，0=未处理
void Calculate_All_Errors(void);        // 计算近远距离图像偏差，更新 near_image_error 和 far_image_error 全局变量
Path_TypeDef recognize_path_type(unsigned char binary_img[IMG_H][IMG_W]);
void Send_Image_To_ZF_Assistant(void); // 通过逐飞助手协议发送二值化图像和识别到的三条边线（调试用）
void Show_Image_On_IPS200(void);


// 摄像头采集标志
extern uint8_t cam_capture_done;
// 摄像头初始化&采集函数声明

#endif // IMAGE_PROCESS_H
