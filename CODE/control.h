/*
 * control.h
 *
 *  Created on: 2024Äê7ÔÂ6ÈÕ
 *      Author: Lenovo
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#define PWM_MAX     10000
#define PWM_MIN     -10000
#define stop_line   50

#include "headfile.h"
#include "isr.h"

float err_sum(void);
int my_abs(int p);
void limit(int *motoA,int *motoB);
void motor_load(int moto1, int moto2);
float motor_value(void);
void motor_barrier(void);
void flag_control(uint8_t flag);
void barn_in_control(void);
void element_handler(void);
void motor_control(void);

#endif /* CONTROL_H_ */
