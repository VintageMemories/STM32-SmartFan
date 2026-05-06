#include "pid.h"

/**
 * @brief  初始化PID控制器
 * @param  pid         控制器指针
 * @param  kp          比例系数
 * @param  ki          积分系数
 * @param  kd          微分系数
 * @param  out_max     输出上限
 * @param  out_min     输出下限
 * @param  int_max     积分限幅(对称)
 * @param  dead_zone   死区范围
 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float out_max, float out_min, float int_max, float dead_zone)
{
    pid->kp           = kp;
    pid->ki           = ki;
    pid->kd           = kd;
    pid->output_max   = out_max;
    pid->output_min   = out_min;
    pid->integral_max = int_max;
    pid->integral_min = -int_max;
    pid->dead_zone    = dead_zone;
    
    pid->setpoint    = 0.0f;
    pid->last_error  = 0.0f;
    pid->integral    = 0.0f;
    pid->output      = 0.0f;
}

/**
 * @brief  设置目标值
 * @param  setpoint  目标值
 */
void PID_SetSetpoint(PID_Controller *pid, float setpoint)
{
    pid->setpoint = setpoint;
}

/**
 * @brief  重置PID状态（切换模式时调用，清除积分累积）
 */
void PID_Reset(PID_Controller *pid)
{
    pid->last_error = 0.0f;
    pid->integral   = 0.0f;
    pid->output     = 0.0f;
}

/**
 * @brief  PID增量式更新
 * @param  measurement  当前测量值
 * @return 输出值
 */
float PID_Update(PID_Controller *pid, float measurement)
{
    float error = measurement - pid->setpoint;
    
    /* 死区处理：误差在死区内，保持上次输出 */
    if (error > -pid->dead_zone && error < pid->dead_zone) {
        return pid->output;
    }
    
    /* 积分累加并限幅 */
    pid->integral += error;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < pid->integral_min) pid->integral = pid->integral_min;
    
    /* 微分计算 */
    float derivative = error - pid->last_error;
    
    /* PID公式 */
    pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    
    /* 输出限幅 */
    if (pid->output > pid->output_max) pid->output = pid->output_max;
    if (pid->output < pid->output_min) pid->output = pid->output_min;
    
    pid->last_error = error;
    return pid->output;
}

/**
 * @brief  获取当前输出值
 * @return 输出值
 */
float PID_GetOutput(const PID_Controller *pid)
{
    return pid->output;
}