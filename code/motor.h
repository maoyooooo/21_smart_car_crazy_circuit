#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#include "zf_common_headfile.h"

// --- 硬件引脚宏定义 (基于你提供的示例) ---
#define PWM_R1      ATOM2_CH7_P11_12
#define DIR_R1      P11_11
#define PWM_L1      ATOM3_CH5_P11_10
#define DIR_L1      P11_9
#define DUTY        10000      // PWM占空比基准值 (对应 20%)

// --- 控制参数 ---
#define CONTROL_PERIOD_MS    10        // 控制周期 10ms
#define ENCODER_LINES        1024.0f    // 编码器线数 (请根据实际修改)

// --- PID 结构体 ---
typedef struct
{
    float kp, ki, kd;
    float target;           // 目标值
    float current;          // 当前值
    float error;            // 当前误差
    float last_error;       // 上次误差
    float integral;         // 积分项
    float i_limit;          // 积分限幅
    float out_limit;        // 输出限幅 (通常设为 100)
} 
PID_Controller; 


// --- 电机当前转速（在 motor.c 中定义） ---
extern float current_L_rpm;
extern float current_R_rpm;

// --- 外部变量声明 ---
extern PID_Controller MotorL_PID;
extern PID_Controller MotorR_PID;

extern volatile float image_error; // 从图像处理模块获取的当前图像偏差值

// --- 直角转弯参数（贝塞尔曲线补线 + PID 闭环转弯）---
//
// 三阶段状态机：IDLE → TURNING → RECOVER
//
// 策略：图像处理层检测到直角 → 贝塞尔曲线覆写 path_center_x
//       → PID 跟踪弯道中线自然产生差速 → 陀螺仪判定完成 → RECOVER
//
// 优势：全程 PID 闭环，不存在开环甩固定内外轮速的问题。
//       转弯激进程度由 TURN_BIAS_PX 控制。
//
#define TURN_BIAS_PX        350.0f   // 中线偏置量（px），饱和 steer_delta_limit 产生最大差速
#define TURN_SPEED_RPM      400.0f   // 转弯期间 base_speed RPM
#define TURN_TARGET_DEG      80.0f   // 目标旋转角度
#define TURN_RECOVER_TICKS    20     // 转弯完成后恢复帧数

typedef enum {
    TURN_IDLE    = 0,   // 空闲
    TURN_ACTIVE  = 1,   // 中线偏置转弯中（PID 全程闭环）
    TURN_RECOVER = 2,   // 恢复直行
} TurnState;

extern TurnState g_turn_state;
extern float     g_angle_z_snapshot; // 入弯时的偏航角快照（由 motor.c 维护）

// --- 函数原型声明 ---
void Motor_Init(void);
void Motor_Control_Loop(float target_L, float target_R);
float PID_Calculate(PID_Controller *pid, float target, float current);
void Set_Motor_PWM(float out_L, float out_R);
float Get_Steering_Delta(float near_err, float far_err);
void Motor_Steering_Control(float base_speed_rpm);
void Motor_ControlTick(float base_speed_rpm);
void Encoder_test(void);

#endif 