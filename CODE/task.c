/*
 * task.c
 *
 *  Created on: 2024年7月13日
 *      Author: Lenovo
 */

#include "task.h"

volatile uint32_t time_cnt = 0;

Tasktypedef RoadTask;

/***************************************************************
* 函数名称：Daimxa_Pc_Display188x120(void)
* 函数输入：无
* 函数输出：无
* 功能说明：下位机协议
***************************************************************/
void Daimxa_Pc_Display188x120(void)
{   //unsigned char y;
    int i=0;
    int j=0;
    uart_putchar(UART_3,0x00); //发送帧头，函数、UART改成自己的
    uart_putchar(UART_3,0xff);//发送帧头
    uart_putchar(UART_3,0x01);//发送帧头
    uart_putchar(UART_3,0x01);//发送帧头
    for(i=0;i<120;i++)//发送图像，像素为188X120
    {
        for(j=0;j<188;j++)
            uart_putchar(UART_3,mt9v03x_image_dvp[i][j]);//灰度，二值化均可
    }
}

float TOFX(void)
{
    uint16_t a;
    a = Get_Distance_TOF400C();
    uint16_t SysTickCount = 0;

    while(a<= 500)
    {
        SysTickCount++;
        return  SysTickCount;
    }



    return  SysTickCount;
}

uint16_t tof_test()
{
    static uint16_t mm = 0;
    uint16_t distance = 0;

    distance = Get_Distance_TOF400C();

if(distance < 600)
{
    timer_pit_interrupt_ms(TIM_5, 100);
    mm++;
}

else {
    timer_pit_close(TIM_5);;
}
    return mm;
}

//使用时在需要计数的函数上开启定时器中断
uint16_t time_count(void)
{
    uint16_t ms = 0;
    TIM_Cmd(TIM5, DISABLE);  //失能TIMx
    ms = time_cnt;
    TIM_Cmd(TIM5, ENABLE);
    return ms;
}

//void task_execution(uint32_t duration_ms)
//{
//    uint32_t start_time = time_count();
//
//    while (time_count() - start_time < duration_ms)
//    {
//        // 执行任务
//        // Your task code here...
//    }
//}

void task1_handle(void)
{
    static uint8_t n = 0;
    if(RoadTask.task1)
    {
        n++;
        if(n > 5)
            n = 7;

        if(n == 1) {
            car_both_rgb_on(red);
            Delay_Ms(1000);
        }
        if(n == 4)
        {

            car_both_rgb_on(green);
            Delay_Ms(1000);
            RoadTask.task1_1 = 1;
        }
        if(n == 5)
        {
            car_both_rgb_off();
        }
     }
}

void task2_handle(void)
{

    if(RoadTask.task2 == 17)
    {
        static uint8_t m = 0;
        m ++;
        if(m > 120)
            m = 120;

        if(m <= 20)
        {
            if(RoadTask.fork_rl == 1)
            {
                ws2812b_write(rgb_yellow);
                ws2812b_write(rgb_off);
            }
            if(RoadTask.fork_rl == 0)
            {
                ws2812b_write(rgb_off);
                ws2812b_write(rgb_yellow);
            }
        }
         if(m > 40 && m <= 60)
        {
            if(RoadTask.fork_rl == 1)
            {
                ws2812b_write(rgb_yellow);
                ws2812b_write(rgb_off);
            }
            if(RoadTask.fork_rl == 0)
            {
                ws2812b_write(rgb_off);
                ws2812b_write(rgb_yellow);
            }
        }
         if(m > 80 && m <= 100)
        {
            if(RoadTask.fork_rl == 1)
            {
                ws2812b_write(rgb_yellow);
                ws2812b_write(rgb_off);
            }
            if(RoadTask.fork_rl == 0)
            {
                ws2812b_write(rgb_off);
                ws2812b_write(rgb_yellow);
            }
        }

        else {
            car_both_rgb_off();
        }

    }

}
