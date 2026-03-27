#include "motor.h"
#include "zf_common_headfile.h"
#include "image_process.h"

PID_Controller MotorL_PID, MotorR_PID;      

float current_L_rpm = 0.0f;
float current_R_rpm = 0.0f;
volatile float image_error = 0;
#define RPM_FILTER_ALPHA 0.2f

typedef struct
{
    float near_err;
    float far_err;
    Path_TypeDef path_type;

    float gyro_z_raw;
    float gyro_z_filt;
    float angle_z;

    float rpm_l;
    float rpm_r;
} Motor_SensorSnapshot;

typedef struct
{
    float kp;
    float kd;
    float gkd;
    float preview_kp;

    float gyro_lpf_alpha;      
    float err_lpf_alpha;       
    float gyro_sign;           

    float steer_delta_limit;   
    float steer_slew_per_tick; 
} Motor_SteerConfig;

static inline float Motor_ClampF(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static const Motor_SteerConfig* Motor_GetSteerConfig(Path_TypeDef path_type)    
{
    static const Motor_SteerConfig cfg_default = {
        .kp                 = 0.42f ,
        .kd                 = 0.01f ,
        .gkd                = 0.12f ,
        .preview_kp         = 0.10f ,
        .gyro_lpf_alpha     = 0.25f ,
        .err_lpf_alpha      = 0.35f ,
        .gyro_sign          = 1.0f  ,
        .steer_delta_limit  = 260.0f,
        .steer_slew_per_tick= 60.0f ,
    };

    static const Motor_SteerConfig cfg_cross = {
        .kp                = 0.40f,
        .kd                = 0.3f,
        .gkd               = 0.3f,
        .preview_kp         = 0.3f,
        .gyro_lpf_alpha     = 0.25f,
        .err_lpf_alpha      = 0.35f,
        .gyro_sign          = 1.0f,
        .steer_delta_limit  = 220.0f,
        .steer_slew_per_tick= 50.0f,
    };

    switch (path_type)
    {
        case PATH_TYPE_CROSSROAD: return &cfg_cross;
        case PATH_TYPE_UNKNOWN:
        case PATH_TYPE_STRAIGHT:
        case PATH_TYPE_TURN_LEFT:
        case PATH_TYPE_TURN_RIGHT:
        case PATH_TYPE_BRANCH:
        default: return &cfg_default;
    }
}

static void Motor_UpdateRpmFromEncoder(void)
{
    Encoder();

    // 齿数比 41:10，所以传动比为 4.1
    // 车轮 RPM = (脉冲 * 6000) / (编码器线数 * 传动比)
    float raw_L = (Mf.Left_encoder  * 6000.0f) / (ENCODER_LINES * 4.1f);
    float raw_R = (Mf.Right_encoder * 6000.0f) / (ENCODER_LINES * 4.1f);

    current_L_rpm = current_L_rpm * (1.0f - RPM_FILTER_ALPHA) + raw_L * RPM_FILTER_ALPHA;
    current_R_rpm = current_R_rpm * (1.0f - RPM_FILTER_ALPHA) + raw_R * RPM_FILTER_ALPHA;
}

static void Motor_SpeedLoop(float target_L, float target_R)
{
    float out_l = PID_Calculate(&MotorL_PID, target_L, current_L_rpm);
    float out_r = PID_Calculate(&MotorR_PID, target_R, current_R_rpm);
    Set_Motor_PWM(out_l, out_r);
}


float last_image_error = 0;
//-------------------------------------------------------------------

// --- 直角右转状态机 ---
TurnRightState g_turn_right_state = TURN_RIGHT_IDLE;
float          g_angle_z_snapshot = 0.0f;
static int     s_brake_tick       = 0;  // 制动阶段帧计数器
static int     s_recover_tick     = 0;  // 恢复阶段帧计数器

static float   s_gyro_z_filt      = 0.0f;
static float   s_near_err_filt    = 0.0f;
static float   s_last_near_err_filt = 0.0f;
static float   s_last_steer_delta = 0.0f;
static float   s_last_target_L    = 0.0f;
static float   s_last_target_R    = 0.0f;
static float   s_turn_accum_deg   = 0.0f;   // 仅在 LOCKING 阶段累计，避免全局航向角长期漂移影响



/**
 * @brief 电机硬件初始化
 */
void Motor_Init(void)
{
    gpio_init(DIR_R1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(DIR_L1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    
    pwm_init(PWM_R1, 17000, 0);
    pwm_init(PWM_L1, 17000, 0);

    MotorL_PID.kp = 0.011f;             
    MotorL_PID.ki = 0.007f;             
    MotorL_PID.kd = 0.110f;             
    MotorL_PID.i_limit = 5000.0f;       
    MotorL_PID.out_limit = 100.0f;      

    MotorR_PID = MotorL_PID;            
}


/**
 * @brief 速度环PID计算
 */
float PID_Calculate(PID_Controller *pid, float target, float current)
{
    pid->error = target - current;
    
    pid->integral += pid->error;    
    
    
    float integral_out = pid->ki * pid->integral;

    float max_i_out = 20.0f; 
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

    pid->last_error = pid->error;

    if (output > pid->out_limit)  output = pid->out_limit;
    if (output < -pid->out_limit) output = -pid->out_limit;

    return output;
}

void Set_Motor_PWM(float out_L, float out_R)
{
    if (out_L >= 0) {
        gpio_set_level(DIR_L1, GPIO_LOW);
        pwm_set_duty(PWM_L1, (uint32)(out_L * (DUTY / 100)));
    } else {
        gpio_set_level(DIR_L1, GPIO_HIGH);
        pwm_set_duty(PWM_L1, (uint32)(-out_L * (DUTY / 100)));
    }

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
    Motor_UpdateRpmFromEncoder();
}


/**
 * @brief 闭环控制主任务 (建议每 10ms 调用一次)     放在中断里面
 */
//轮毂半径12.15 mm 周长 = 76.34 mm  1000 rpm = 1272.3 mm/s = 127.2 3 cm/s
void Motor_Control_Loop(float target_L, float target_R)
{
    Motor_UpdateRpmFromEncoder();
    Motor_SpeedLoop(target_L, target_R);
}



float Get_Steering_Delta(float near_err, float far_err)
{
    const Motor_SteerConfig* cfg = Motor_GetSteerConfig(current_path_type);

    float gyro_z_raw = icm_data.gyro_z * cfg->gyro_sign;
    s_gyro_z_filt = s_gyro_z_filt * (1.0f - cfg->gyro_lpf_alpha) + gyro_z_raw * cfg->gyro_lpf_alpha;

    s_near_err_filt = s_near_err_filt * (1.0f - cfg->err_lpf_alpha) + near_err * cfg->err_lpf_alpha;
    float derr = s_near_err_filt - s_last_near_err_filt;
    s_last_near_err_filt = s_near_err_filt;

    float base_term = (near_err * cfg->kp) + (derr * cfg->kd);
    float preview_term = (far_err * cfg->preview_kp - s_gyro_z_filt) * cfg->gkd;

    float steer = base_term + preview_term;

    steer = Motor_ClampF(steer, -cfg->steer_delta_limit, cfg->steer_delta_limit);
    float step = Motor_ClampF(steer - s_last_steer_delta, -cfg->steer_slew_per_tick, cfg->steer_slew_per_tick);
    s_last_steer_delta += step;

    last_image_error = near_err;
    return s_last_steer_delta;
}

static Motor_SensorSnapshot Motor_BuildSnapshot(void)
{
    Motor_SensorSnapshot s;
    s.near_err = (float)near_image_error;
    s.far_err  = (float)far_image_error;
    s.path_type= current_path_type;

    const Motor_SteerConfig* cfg = Motor_GetSteerConfig(s.path_type);
    s.gyro_z_raw = icm_data.gyro_z * cfg->gyro_sign;
    s_gyro_z_filt = s_gyro_z_filt * (1.0f - cfg->gyro_lpf_alpha) + s.gyro_z_raw * cfg->gyro_lpf_alpha;
    s.gyro_z_filt = s_gyro_z_filt;

    s.angle_z = angle_Z;
    s.rpm_l = current_L_rpm;
    s.rpm_r = current_R_rpm;
    return s;
}

static void Motor_SteeringLayer(const Motor_SensorSnapshot* snap, float base_speed_rpm, float* out_target_L, float* out_target_R)
{
    if (snap->path_type == PATH_TYPE_TURN_RIGHT &&
        g_turn_right_state == TURN_RIGHT_IDLE)
    {
        g_turn_right_state = TURN_RIGHT_BRAKING;
        s_brake_tick       = 0;
        s_recover_tick     = 0;
    }

    if (g_turn_right_state == TURN_RIGHT_BRAKING)
    {
        s_brake_tick++;
        if (s_brake_tick >= TURN_BRAKE_TICKS)
        {
            g_turn_right_state = TURN_RIGHT_LOCKING;
            g_angle_z_snapshot = snap->angle_z;  // 快照，不清零，避免与其它中断竞态
            s_turn_accum_deg   = 0.0f;           // 每次入弯重置局部累计角
        }
        else
        {
            *out_target_L = TURN_BRAKE_SPEED;
            *out_target_R = TURN_BRAKE_SPEED;
            return;
        }
    }

    if (g_turn_right_state == TURN_RIGHT_LOCKING)
    {
        // 用“局部积分角”做主判据，规避 angle_Z 长时间运行的漂移累积问题
        // 假设 gyro_z 单位为 deg/s，控制周期 10ms
        const float dt_s = (float)CONTROL_PERIOD_MS * 0.001f;
        float yaw_rate = snap->gyro_z_filt;
        if (yaw_rate < 0.0f) yaw_rate = -yaw_rate; // 取旋转角速度幅值，弱化极性误配影响
        if (yaw_rate < 2.0f) yaw_rate = 0.0f;      // 小抖动死区，避免静止时积累
        s_turn_accum_deg += yaw_rate * dt_s;

        // 保留 angle_Z 增量作为兜底（若 IMU瞬时异常，至少不至于完全失效）
        float turned_by_angle = snap->angle_z - g_angle_z_snapshot;
        if (turned_by_angle < -180.0f) turned_by_angle += 360.0f;
        if (turned_by_angle >  180.0f) turned_by_angle -= 360.0f;
        if (turned_by_angle < 0.0f) turned_by_angle = 0.0f;

        float turned_deg = s_turn_accum_deg;
        if (turned_by_angle > turned_deg) turned_deg = turned_by_angle;

        if (turned_deg >= TURN_TARGET_DEG)
        {
            g_turn_right_state = TURN_RIGHT_RECOVER;
            s_recover_tick     = 0;
        }
        else
        {
            // 临近目标角度自动减速，减少 >90° 过冲：剩余角度越小，外轮目标越低
            float remain_deg = TURN_TARGET_DEG - turned_deg;
            const float slow_start_deg   = 20.0f;   // 剩余角度 <= 该值开始降速（可调）
            const float min_outer_speed  = 200.0f;  // 末端最低外轮速度（太低可能卡住）
            float scale = remain_deg / slow_start_deg;
            if (scale > 1.0f) scale = 1.0f;
            if (scale < 0.25f) scale = 0.25f;

            float outer_speed = (TURN_OUTER_SPEED * scale) - TURN_GKD_BRAKE * snap->gyro_z_filt;
            if (outer_speed < min_outer_speed) outer_speed = min_outer_speed;
            *out_target_L = outer_speed;
            *out_target_R = TURN_INNER_SPEED;
            return;
        }
    }

    if (g_turn_right_state == TURN_RIGHT_RECOVER)
    {
        s_recover_tick++;
        if (s_recover_tick >= TURN_RECOVER_TICKS)
        {
            g_turn_right_state = TURN_RIGHT_IDLE;
        }
        // RECOVER 期间仍按正常循迹输出，但不会修改 angle_Z
    }

    // ==================================================================
    // 常规循迹（只读快照）
    // ==================================================================
    const Motor_SteerConfig* cfg = Motor_GetSteerConfig(snap->path_type);
    (void)cfg; // cfg 已在 Get_Steering_Delta 内使用（包含滤波/限幅/速率）

    // 保持与你现有“near + far + gyro”的结构，但输出做限幅+速率限制+滤波
    float steer_delta = Get_Steering_Delta(snap->near_err, snap->far_err);
    *out_target_L = base_speed_rpm + steer_delta;
    *out_target_R = base_speed_rpm - steer_delta;
}

void Motor_ControlTick(float base_speed_rpm)
{
    // 1) SensorSnapshot：周期开始只做一次采样与RPM更新
    Motor_UpdateRpmFromEncoder();
    Motor_SensorSnapshot snap = Motor_BuildSnapshot();

    // 2) Steering：快照 -> 目标转速（含右转状态机）
    float target_L = base_speed_rpm;
    float target_R = base_speed_rpm;
    Motor_SteeringLayer(&snap, base_speed_rpm, &target_L, &target_R);

    // 2.5) Target ramp：限制目标转速变化速度，避免直角弯切换过猛导致点头/前翻
    // 每 10ms 最大允许变化的 rpm（可按车重心/抓地调：越小越丝滑，但响应更慢）
    const float target_slew_rpm_per_tick = 80.0f;
    float dl = Motor_ClampF(target_L - s_last_target_L, -target_slew_rpm_per_tick, target_slew_rpm_per_tick);
    float dr = Motor_ClampF(target_R - s_last_target_R, -target_slew_rpm_per_tick, target_slew_rpm_per_tick);
    s_last_target_L += dl;
    s_last_target_R += dr;
    target_L = s_last_target_L;
    target_R = s_last_target_R;

    // 3) Speed：速度 PID -> PWM（本周期只执行一次）
    Motor_SpeedLoop(target_L, target_R);
}




/*
*@brief 转向控制主函数
*@param base_speed_rpm 基础速度（单位：RPM），表示车辆在直行时的目标速度
*@return 无返回值，函数内部调用 Motor_Control_Loop 来调整左右轮速度以实现转向控制
*@note 该函数首先调用 Encoder() 更新编码器数据，然后根据图像偏差计算转向调整值，并将其应用于左右轮的目标速度。通过调整 target_L 和 target_R 的值，车辆能够在保持基础速度的同时实现转向。
*@note 该函数不能在主循环中调用！！！！！过于频繁，建议每 10ms 调用一次，以保持控制的稳定性和响应速度。
*@note 放到pit中断里面每10ms调用一次
*/
// 在 motor.c 顶部定义转向参数


void Motor_Steering_Control(float base_speed_rpm)
{
    // 兼容旧入口：外部调用仍然有效，但内部已统一到“快照/转向/速度”单入口
    Motor_ControlTick(base_speed_rpm);
}