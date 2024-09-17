/*
 * task.c
 *
 *  Created on: 2024��7��13��
 *      Author: Lenovo
 */

#include "task.h"

volatile uint32_t time_cnt = 0;

Tasktypedef RoadTask;

/***************************************************************
* �������ƣ�Daimxa_Pc_Display188x120(void)
* �������룺��
* �����������
* ����˵������λ��Э��
***************************************************************/
void Daimxa_Pc_Display188x120(void)
{   //unsigned char y;
    int i=0;
    int j=0;
    uart_putchar(UART_3,0x00); //����֡ͷ��������UART�ĳ��Լ���
    uart_putchar(UART_3,0xff);//����֡ͷ
    uart_putchar(UART_3,0x01);//����֡ͷ
    uart_putchar(UART_3,0x01);//����֡ͷ
    for(i=0;i<120;i++)//����ͼ������Ϊ188X120
    {
        for(j=0;j<188;j++)
            uart_putchar(UART_3,mt9v03x_image_dvp[i][j]);//�Ҷȣ���ֵ������
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

//ʹ��ʱ����Ҫ�����ĺ����Ͽ�����ʱ���ж�
uint16_t time_count(void)
{
    uint16_t ms = 0;
    TIM_Cmd(TIM5, DISABLE);  //ʧ��TIMx
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
//        // ִ������
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
