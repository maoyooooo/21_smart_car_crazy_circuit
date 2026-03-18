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

extern float Steer_Kp;
extern float Steer_Kd;
extern float Steer_Gkd;
extern float Steer_Preview_Kp;      // 预瞄比例增益

// --- 直角右转状态机参数 ---
//
// 四阶段状态机：IDLE → BRAKING → LOCKING → RECOVER
//
// 【根本原因分析】
//   即使 base_speed=200 RPM，摄像头检测到弯道时车头已到达转角正前方。
//   状态机触发 → 电机指令下发 → 车速实际降低，这段机械响应延迟约 50-100ms，
//   车辆在此期间已行进 10-25mm，足够冲出转角入口。
//   解决方案：在 LOCKING 前插入 BRAKING 阶段，先把速度降到极低，
//   确认车速已实际降下来后再开始转弯动作。
//
// 各阶段说明：
//   BRAKING  : 检测到弯道后立即将双轮速度降到 TURN_BRAKE_SPEED，
//              持续 TURN_BRAKE_TICKS 帧（=50ms），等车速实际降下来
//   LOCKING  : 执行 IMU 差速转弯，外轮 TURN_OUTER_SPEED，内轮 TURN_INNER_SPEED
//   RECOVER  : 转弯完成，恢复正常跟线
//
#define TURN_BRAKE_SPEED    200.0f   // 制动阶段目标速度 RPM（接近停车，越小越稳）
#define TURN_BRAKE_TICKS      5     // 制动帧数（×10ms=50ms），让电机实际减速
#define TURN_TARGET_DEG     75.0f   // 目标旋转角度（略小于90°，留出惯性余量）
#define TURN_OUTER_SPEED   350.0f   // 转弯外轮（左轮）目标转速 RPM
#define TURN_INNER_SPEED     0.0f   // 转弯内轮（右轮）目标转速 RPM（0=原地转）
#define TURN_GKD_BRAKE       0.5f   // 陀螺仪制动增益（抑制外轮角速度过冲）
#define TURN_RECOVER_TICKS    5    // 转弯完成后恢复帧数（×10ms = 150ms 直行稳定期）

typedef enum {
    TURN_RIGHT_IDLE    = 0,   // 空闲，等待摄像头信号
    TURN_RIGHT_BRAKING = 1,   // 【新增】预制动：降速等待车速实际降低
    TURN_RIGHT_LOCKING = 2,   // IMU 差速转弯中
    TURN_RIGHT_RECOVER = 3,   // 转弯完成，短暂恢复直行
} TurnRightState;

extern TurnRightState g_turn_right_state;
extern float          g_angle_z_snapshot; // 入弯时的偏航角快照（由 motor.c 维护）

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