/*********************************************************************************************************************
* @file            main.c
* @brief           摄像头图像屏幕显示例程
* @author          lemon
* @version         1.0
* @Target core     CH32V307VCT6
* @date            2022-3-2
*
********************************************************************************************************************/

/* 用户头文件 */
#include "main.h"
#include "headfile.h"

/* 全局变量声明 */
PID_Controller positon;
float time = 0, TIME = 0;
float a = 0, b = 0;

/* 主函数 */
int main(void)
{
    /* 此处声明需要用到的局部变量 */
//    uint8 thre;
    RoadTask.task2 = 0;
    /* 智能车初始化 */
    car_init();
    /* 此处编写单次运行的代码(例如：初始化代码等) */
    ips114_init();     //初始化屏幕
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
        /* 采集一帧图像处理 */
        if(mt9v03x_finish_flag_dvp)
        {
            mt9v03x_finish_flag_dvp = 0;
            /* 图像处理函数 */
            image_process();
//            thre = my_adapt_threshold(Image_Use[0], LCDH, LCDW);
            /*元素处理*/
            element_handler();
            /* 图像显示在屏幕上 */
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

            /* 下位机执行代码 */
//            Daimxa_Pc_Display188x120();

            /*调试直角弯时使用*/
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

