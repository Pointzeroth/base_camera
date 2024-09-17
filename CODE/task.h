/*
 * task.h
 *
 *  Created on: 2024年7月13日
 *      Author: Lenovo
 */

#ifndef TASK_H_
#define TASK_H_

#include "headfile.h"

typedef struct
{
  uint8_t task1;
  uint8_t task2;
  uint8_t task3;
  uint8_t task4;
  uint8_t fork_rl;
  uint8_t task1_1;
} Tasktypedef;       //标志位结构体


Tasktypedef RoadTask;

extern volatile uint32_t time_cnt;

uint16_t time_count(void);
float TOFX(void);
uint16_t tof_test();
void Task1(void);
void task1_handle(void);
void task2_handle(void);

#endif /* TASK_H_ */
