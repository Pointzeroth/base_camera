/*
 * pid.h
 *
 *  Created on: 2024年7月6日
 *      Author: Lenovo
 */

#ifndef PID_H_
#define PID_H_

#include "headfile.h"
#include "isr.h"

// PID控制器结构体
typedef struct
{
    float Setpoint;     // 设定目标值
    float Input;        // 当前反馈值
    float Output;       // 控制输出
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float LastError;     // 上一次的误差
    float Integral;     // 误差积分
} PID_Controller;

extern PID_Controller positon;

void PID_Init(PID_Controller *pid, float kp, float ki, float kd);
float PID_Compute(PID_Controller *pid);
void PID_SetInput(PID_Controller *pid, float input);
void PID_SetSetpoint(PID_Controller *pid, float setpoint);


#endif /* PID_H_ */
