/*
 * pid.h
 *
 *  Created on: 2024��7��6��
 *      Author: Lenovo
 */

#ifndef PID_H_
#define PID_H_

#include "headfile.h"
#include "isr.h"

// PID�������ṹ��
typedef struct
{
    float Setpoint;     // �趨Ŀ��ֵ
    float Input;        // ��ǰ����ֵ
    float Output;       // �������
    float Kp;           // ����ϵ��
    float Ki;           // ����ϵ��
    float Kd;           // ΢��ϵ��
    float LastError;     // ��һ�ε����
    float Integral;     // ������
} PID_Controller;

extern PID_Controller positon;

void PID_Init(PID_Controller *pid, float kp, float ki, float kd);
float PID_Compute(PID_Controller *pid);
void PID_SetInput(PID_Controller *pid, float input);
void PID_SetSetpoint(PID_Controller *pid, float setpoint);


#endif /* PID_H_ */
