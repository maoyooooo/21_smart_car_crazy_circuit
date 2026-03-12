#include "motor.h"
#include "zf_common_headfile.h"

PID_Controller MotorL_PID, MotorR_PID;      

float current_L_rpm = 0.0f;
float current_R_rpm = 0.0f;
volatile float image_error = 0; // 全局变量，存储当前的图像偏差值
#define RPM_FILTER_ALPHA 0.2f // 低通滤波系数 (0.0 - 1.0)，值越小平滑效果越强，但响应越慢



//----转向控制参数--------------------------------------------------
float Steer_Kp  = 0.00f;   // 基础比例（提供基础回正力）
float Steer_Kp2 = 0.00f;  // 非线性比例（偏差越大，转向力呈指数级增强）
float Steer_Kd  = 0.00f;   // 微分（预测弯道，防止过冲）
float Steer_Gkd = 0.00f;   // 陀螺仪微分（专门用于抑制车体震荡）

float last_image_error = 0; // 记录上一次的图像偏差
//-------------------------------------------------------------------




/**
 * @brief 电机硬件初始化
 */
void Motor_Init(void)
{
    // 1. 初始化方向引脚
    gpio_init(DIR_R1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(DIR_L1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    
    // 2. 初始化 PWM (频率 17kHz)
    pwm_init(PWM_R1, 17000, 0);
    pwm_init(PWM_L1, 17000, 0);

    // 3. 赋初始 PID 参数 (保守值测试，防止烧毁)
    MotorL_PID.kp = 0.015f;             // 比例增益 (加大，提供强劲起步动力)
    MotorL_PID.ki = 0.012f;             // 积分增益 (加大，快速消除稳态误差)
    MotorL_PID.kd = 0.110f;             // 微分增益 (继续保持关闭，避免噪声放大)
    MotorL_PID.i_limit = 5000.0f;         // 积分限幅，防止积分风暴。
    MotorL_PID.out_limit = 100.0f;       // 极重要：输出限幅，确保安全

    MotorR_PID = MotorL_PID;            // 右轮暂用相同参数
}


/**
 * @brief PID 核心算法
 * 公式: $$Output = K_p \cdot e(t) + K_i \cdot \int e(t)dt + K_d \cdot \frac{de(t)}{dt}$$
 */
float PID_Calculate(PID_Controller *pid, float target, float current)
{
    pid->error = target - current;
    
    // 1. 正常累加积分
    pid->integral += pid->error;    
    
    // 2. 计算纯积分项的输出
    float integral_out = pid->ki * pid->integral;

    // 3. 动态积分限幅 (抗饱和) 适当放宽到 30.0f 防止正常爬升阶段积分受限导致起步肉
    float max_i_out = 15.0f; 
    if (integral_out > max_i_out) {
        integral_out = max_i_out;
        pid->integral = max_i_out / pid->ki; 
    }
    else if (integral_out < -max_i_out) {
        integral_out = -max_i_out;
        pid->integral = -max_i_out / pid->ki;
    }

    float output = (pid->kp * pid->error) + 
                   integral_out +
                   (pid->kd * (pid->error - pid->last_error));

    // 更新状态
    pid->last_error = pid->error;

    // 5. 总输出限幅 (限制在物理允许的占空比最大值)
    if (output > pid->out_limit)  output = pid->out_limit;
    if (output < -pid->out_limit) output = -pid->out_limit;

    return output;
}

/**
 * @brief 执行电机输出 (PWM + 方向)
 * @param out_L/R 范围 -100 到 100
 */
void Set_Motor_PWM(float out_L, float out_R)
{
    // 左轮处理
    if (out_L >= 0) {
        gpio_set_level(DIR_L1, GPIO_LOW);
        pwm_set_duty(PWM_L1, (uint32)(out_L * (DUTY / 100)));
    } else {
        gpio_set_level(DIR_L1, GPIO_HIGH);
        pwm_set_duty(PWM_L1, (uint32)(-out_L * (DUTY / 100)));
    }

    // 右轮处理
    if (out_R >= 0) {
        gpio_set_level(DIR_R1, GPIO_LOW);
        pwm_set_duty(PWM_R1, (uint32)(out_R * (DUTY / 100)));
    } else {
        gpio_set_level(DIR_R1, GPIO_HIGH);
        pwm_set_duty(PWM_R1, (uint32)(-out_R * (DUTY / 100)));
    }
}


void Encoder_test(void)
{
    Encoder();
    float raw_L = (Mf.Left_encoder * 6000.0f) / (ENCODER_LINES * 4.1f);
    float raw_R = (Mf.Right_encoder * 6000.0f) / (ENCODER_LINES * 4.1f);
    // 齿数比 41:10，所以传动比为 4.1
    // 车轮 RPM = (脉冲 * 6000) / (编码器线数 * 传动比)
    current_L_rpm = current_L_rpm * (1.0f - RPM_FILTER_ALPHA) + raw_L * RPM_FILTER_ALPHA;
    current_R_rpm = current_R_rpm * (1.0f - RPM_FILTER_ALPHA) + raw_R * RPM_FILTER_ALPHA;
}


/**
 * @brief 闭环控制主任务 (建议每 10ms 调用一次)     放在中断里面
 */
//轮毂半径12.15 mm 周长 = 76.34 mm  1000 rpm = 1272.3 mm/s = 127.2 3 cm/s
void Motor_Control_Loop(float target_L, float target_R)
{
    // 1. 获取原始数据
    Encoder();

    float raw_L = (Mf.Left_encoder * 6000.0f) / (ENCODER_LINES * 4.1f);
    float raw_R = (Mf.Right_encoder * 6000.0f) / (ENCODER_LINES * 4.1f);
    // 齿数比 41:10，所以传动比为 4.1
    // 车轮 RPM = (脉冲 * 6000) / (编码器线数 * 传动比)
    current_L_rpm = current_L_rpm * (1.0f - RPM_FILTER_ALPHA) + raw_L * RPM_FILTER_ALPHA;
    current_R_rpm = current_R_rpm * (1.0f - RPM_FILTER_ALPHA) + raw_R * RPM_FILTER_ALPHA;

    // 4. 计算 PID (建议尝试 Kp=0.05, Ki=0.01, Kd=0)
    float out_l = PID_Calculate(&MotorL_PID, target_L, current_L_rpm);
    float out_r = PID_Calculate(&MotorR_PID, target_R, current_R_rpm);

    Set_Motor_PWM(out_l , out_r);
}


/****************************************************************************************************************
*@brief 获取转向调整值
*@param image_error 图像偏差值（例如：车道线中心与图像中心的距离）
*@return 转向调整值 （正值表示向右转，负值表示向左转）【暂定！！！！！！！】
*@note 该函数结合了双PD控制和陀螺仪数据来计算转向调整值，能够在弯道中提供更稳定的转向控制。
*公式说明：
*1. 基础比例项：Steer_Kp * image_error
*   - 这是最基本的比例控制，根据当前的图像偏差直接提供一个转向调整值。
****************************************************************************************************************** 
*/
float Steer_Preview_Kp = 0.05f;      // 初始预瞄比例增益 (需要根据实车调试调整)过高可能导致过度转向，过低可能反应不足

float Get_Steering_Delta(float near_err, float far_err)
{
    float turn_value = 0;

    // 获取 Z 轴角速度 (Yaw轴旋转速度)
    // 注意：如果后面发现陀螺仪加进去反而震荡更厉害，说明极性反了，给 gyro_z 前面加个负号即可
    float gyro_z = icm_data.gyro_z; 

    /* 预瞄航向角逻辑：
       1. 基于远端偏差 far_err 计算出“期望的角速度” (Target Yaw Rate)
       2. 如果当前 gyro_z 小于期望值，说明转弯不够快，加大输出
    */
   
    // 基础比例项
    float base_term = (near_err * Steer_Kp) + ((near_err - last_image_error) * Steer_Kd);

    // 预瞄项 (基于远端偏差和陀螺仪数据):(期望角速度 - 当前角速度) * 微分增益
    // far_err * Steer_Preview_Kp 相当于把远端偏差映射成一个期望的角速度。
    float preview_term = (far_err * Steer_Preview_Kp - gyro_z) * Steer_Gkd; // 预瞄项
    
    turn_value = base_term + preview_term;

    // 更新历史偏差
    last_image_error = near_err;

    return turn_value;
}




/*
*@brief 转向控制主函数
*@param base_speed_rpm 基础速度（单位：RPM），表示车辆在直行时的目标速度
*@return 无返回值，函数内部调用 Motor_Control_Loop 来调整左右轮速度以实现转向控制
*@note 该函数首先调用 Encoder() 更新编码器数据，然后根据图像偏差计算转向调整值，并将其应用于左右轮的目标速度。通过调整 target_L 和 target_R 的值，车辆能够在保持基础速度的同时实现转向。
*@note 该函数不能在主循环中调用！！！！！过于频繁，建议每 10ms 调用一次，以保持控制的稳定性和响应速度。
*@note 放到pit中断里面每10ms调用一次
*/
void Motor_Steering_Control(float base_speed_rpm)
{

//[暂定，准备分开两个核搞，有没有懂的]    Image_Processing_And_Get_Center(); // 获取图像并处理，更新 path_center_x 数组


    float steering_rpm_delta = Get_Steering_Delta(near_image_error, far_image_error); // 获取转向调整值（单位：RPM）

    // 根据转向调整值修改左右轮速度
    float target_L = base_speed_rpm + steering_rpm_delta;
    float target_R = base_speed_rpm - steering_rpm_delta;

    // 调用主控制循环
    Motor_Control_Loop(target_L, target_R);
}