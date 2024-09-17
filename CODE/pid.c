/*
 * pid.c
 *
 *  Created on: 2024��7��6��
 *      Author: Lenovo
 */

#include "pid.h"

// PID��������ʼ������
void PID_Init(PID_Controller *pid, float kp, float ki, float kd)
{
    pid->Setpoint = 0.0f;
    pid->Input = 0.0f;
    pid->Output = 0.0f;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->LastError = 0.0f;
    pid->Integral = 0.0f;
}

// PID���㺯��
float PID_Compute(PID_Controller *pid)
{
    float error = err_sum();
    pid->Integral += error;
    float derivative = error - pid->LastError;
    pid->Output = pid->Kp * error + pid->Ki * pid->Integral + pid->Kd * derivative;
    pid->LastError = error;
    return pid->Output;
}

// ����PID����ĺ���
void PID_SetInput(PID_Controller *pid, float input)
{
    pid->Input = input;
}

// ����PID�趨ֵ�ĺ���
void PID_SetSetpoint(PID_Controller *pid, float setpoint)
{
    pid->Setpoint = setpoint;
}

