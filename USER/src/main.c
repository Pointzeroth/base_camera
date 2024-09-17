/*********************************************************************************************************************
* @file            main.c
* @brief           ����ͷͼ����Ļ��ʾ����
* @author          lemon
* @version         1.0
* @Target core     CH32V307VCT6
* @date            2022-3-2
*
********************************************************************************************************************/

/* �û�ͷ�ļ� */
#include "main.h"
#include "headfile.h"

/* ȫ�ֱ������� */
PID_Controller positon;
float time = 0, TIME = 0;
float a = 0, b = 0;

/* ������ */
int main(void)
{
    /* �˴�������Ҫ�õ��ľֲ����� */
//    uint8 thre;
    RoadTask.task2 = 0;
    /* ���ܳ���ʼ�� */
    car_init();
    /* �˴���д�������еĴ���(���磺��ʼ�������) */
    ips114_init();     //��ʼ����Ļ
    board_led_init(1);
    ws2812b_init();
    mt9v03x_init_dvp();
    Init_TOF400C();
    PID_Init(&positon,116, 0, 45);//155    161 0 -31 -12
    Delay_Ms(100);
    car_both_rgb_off();

    ips114_show_string(0,48,"lenth:");
    ips114_show_string(98,48,"mm");

    while(1)
    {
        /* �ɼ�һ֡ͼ���� */
        if(mt9v03x_finish_flag_dvp)
        {
            mt9v03x_finish_flag_dvp = 0;
            /* ͼ������ */
            image_process();
//            thre = my_adapt_threshold(Image_Use[0], LCDH, LCDW);
            /*Ԫ�ش���*/
            element_handler();
            /* ͼ����ʾ����Ļ�� */
//            isp_use_image (80,10, Image_Use[0],LCDW,LCDH,LCDW,LCDH);
//            ips114_show_gray_image (80, 75, Image_Use[0], LCDW, LCDH, LCDW, LCDH, thre);

            if(RoadBarrier.flag != 1
            && RoadFlag.barn_in_flag2 !=1
            && RoadFlag.barn_in_flag != 1
            && RoadFlag.bridge_flag2 != 1
            && RoadFlag.zebra_flag != 1
            && RoadFlag.Lroad_flag != 1
            && RoadFlag.Lroad_flag2 != 1
            && RoadTask.task1_1 == 1
            )
                motor_control();

            /* ��λ��ִ�д��� */
//            Daimxa_Pc_Display188x120();

            /*����ֱ����ʱʹ��*/
//            ips114_show_string(0,80,"RMiss:");
//            ips114_show_string(0,96,"LMiss:");
//            ips114_show_string(0,112,"While:");

//            ips114_show_uint(48,80,ImageStatus.RWLine,4);
//            ips114_show_uint(48,96,ImageStatus.LWLine,4);
//            ips114_show_uint(48,112,ImageStatus.WhiteLine,4);

            if(RoadFlag.fork_flag != 1
            && RoadFlag.Lroad_flag != 1
            && RoadFlag.Lroad_flag2 != 1
            && (ImageStatus.WhiteLine <= 10)
            && ImageStatus.OFFLine < 20
            && RoadTask.task1_1 == 1
            )
            {
                time = tof_test();
                a = time_cnt * 7.76;
            }
            ips114_show_uint(48,48,a,5);
//            ips114_show_float(48,64,a,5,5);

            task2_handle();
        }

        task1_handle();


    }
}

