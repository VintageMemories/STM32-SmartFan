#ifndef __PID_H
#define __PID_H

#include "sys.h"

/* PID控制器结构体 */
typedef struct {
    float kp;               /* 比例系数 */
    float ki;               /* 积分系数 */
    float kd;               /* 微分系数 */
    float output_max;       /* 输出上限 */
    float output_min;       /* 输出下限 */
    float integral_max;     /* 积分上限 */
    float integral_min;     /* 积分下限 */
    float dead_zone;        /* 死区范围(±) */
    
    float setpoint;         /* 目标值 */
    float last_error;       /* 上次误差 */
    float integral;         /* 积分累加 */
    float output;           /* 当前输出 */
} PID_Controller;

void  PID_Init(PID_Controller *pid, float kp, float ki, float kd,
               float out_max, float out_min, float int_max, float dead_zone);
void  PID_SetSetpoint(PID_Controller *pid, float setpoint);
void  PID_Reset(PID_Controller *pid);
float PID_Update(PID_Controller *pid, float measurement);
float PID_GetOutput(const PID_Controller *pid);

#endif