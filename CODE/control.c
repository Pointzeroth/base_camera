/*
 * control.c
 *
 *  Created on: 2024��7��6��
 *      Author: Lenovo
 */

#ifndef CONTROL_C_
#define CONTROL_C_

#include "control.h"

////��Ȩ����
//const uint8 Weight[LCDH]=
//{
//    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //ͼ����Զ��00 ����09 ��Ȩ��
//    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //ͼ����Զ��10 ����19 ��Ȩ��
//    1, 1, 1, 1, 1, 1, 1, 3, 4, 5,              //ͼ����Զ��20 ����29 ��Ȩ��
//    6, 7, 9,11,13,15,17,19,20,20,              //ͼ����Զ��30 ����39 ��Ȩ��
//   19,17,15,13,11, 9, 7, 5, 3, 1,              //ͼ����Զ��40 ����49 ��Ȩ��
//    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //ͼ����Զ��50 ����59 ��Ȩ��
//};//Ȩ��ֻ��ѡȡĳһ��Χ����Ϊ��Ҫ�����У����Ҿ�������������ͼ�������
//  //����û�й̶���Χ����Դ��������ʵ�ʲ��Ժ��þͿ���

//��Ȩ����
const uint8 Weight[LCDH]=
{
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //ͼ����Զ��00 ����09 ��Ȩ��
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //ͼ����Զ��10 ����19 ��Ȩ��
        1, 1, 1, 1, 1, 1, 6, 13, 14, 16,              //ͼ����Զ��20 ����29 ��Ȩ��
        6, 7, 9,11,13,15,17,19,20,20,              //ͼ����Զ��30 ����39 ��Ȩ��
       19,17,15,13,11, 9, 7, 15, 4, 4,              //ͼ����Զ��40 ����49 ��Ȩ��
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //ͼ����Զ��50 ����59 ��Ȩ��
};//Ȩ��ֻ��ѡȡĳһ��Χ����Ϊ��Ҫ�����У����Ҿ�������������ͼ�������
  //����û�й̶���Χ����Դ��������ʵ�ʲ��Ժ��þͿ���

////��Ȩ����
//const uint8 Weight[LCDH]=
//{
//        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //ͼ����Զ��00 ����09 ��Ȩ��
//        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //ͼ����Զ��10 ����19 ��Ȩ��
//        1, 1, 1, 1, 1, 1, 6, 13, 14, 16,              //ͼ����Զ��20 ����29 ��Ȩ��
//        6, 7, 9,11,13,15,17,19,20,20,              //ͼ����Զ��30 ����39 ��Ȩ��
//       19,17,15,13,11, 9, 7, 15, 4, 4,              //ͼ����Զ��40 ����49 ��Ȩ��
//        13, 12, 1, 5, 1, 1, 1, 1, 1, 1,              //ͼ����Զ��50 ����59 ��Ȩ��
//};//Ȩ��ֻ��ѡȡĳһ��Χ����Ϊ��Ҫ�����У����Ҿ�������������ͼ�������
//  //����û�й̶���Χ����Դ��������ʵ�ʲ��Ժ��þͿ���



float err_sum(void)
{
    int i;
    float err=0;
    float weight_count=0;
    //�������
    for(i=LCDH-1;i>=LCDH-ImageStatus.OFFLine-1;i--)//����������
    {
        err+=(40-(ImageDeal[i].Center))*Weight[i];//����1λ����Ч��2
        weight_count+=Weight[i];
    }

    err=err/weight_count;

    return err;
}

//����ֵ����
int my_abs(int p)
{
    int q;
    q=p>0?p:(-p);
    return q;
}

//�޷�����
void limit(int *motoA,int *motoB)
{
    if(*motoA>PWM_MAX)*motoA=PWM_MAX;
    if(*motoA<PWM_MIN)*motoA=PWM_MIN;

    if(*motoB>PWM_MAX)*motoB=PWM_MAX;
    if(*motoB<PWM_MIN)*motoB=PWM_MIN;
}

//���װ�غ���
void motor_load(int moto1, int moto2)   //moto1������         moto2���ҵ��
{
    if(moto1 > 0)
    {
        if(moto1 > 10000)
        {
            moto1 = 10000;
        }
        motor_forward(left, moto1);
    }
    else
    {
        if(moto1 < -10000)
        {
            moto1 = -10000;
        }
        motor_back(left, my_abs(moto1));
    }

    if(moto2 > 0)
    {
        if(moto2 > 10000)
        {
            moto2 = 10000;
        }
        motor_forward(right, moto2);
    }
    else
    {
        if(moto2 < -10000)
        {
            moto2 = -10000;
        }
        motor_back(right, my_abs(moto2));
    }
}

float motor_value(void)
{
    float moto_value;
    moto_value = PID_Compute(&positon);
    return moto_value;
}

//ȷ���ٶȷ�����д

//���Ͽ��ƺ���
void motor_barrier(void)
{
//    static int barrier_num = 0;
    /*ֱ��д���ǶȲ����ã�����һ��һ��ʶ���׼�������ٶ�Ҫ������*/
    if(RoadBarrier.flag == 1)
    {
        motor_forward(left,3000);//4000
        motor_forward(right,5500);//6000
        Delay_Ms(480);
        motor_forward(left, 7000);
        motor_forward(right, 3300);
        Delay_Ms(450);
    }

    if(RoadFlag.block_flag)
    {
        motor_forward(right,3000);//4000
        motor_forward(left,5500);//6000
        Delay_Ms(480);
        motor_forward(right, 7000);
        motor_forward(left, 3300);
        Delay_Ms(450);
    }
}

/*ֱ�Ǵ���*/
void zhijiao_handle(void)
{
    if(RoadFlag.Lroad_flag == 1)
    {
        motor_forward(left,5000);//4000
        motor_forward(right,4500);//6000
        Delay_Ms(150);
        motor_forward(left,4500);//4000
        motor_forward(right,0);//6000
        Delay_Ms(300);
    }

    if(RoadFlag.Lroad_flag2 == 1)//��
    {
        motor_forward(left,5000);//4000
        motor_forward(right,4500);//6000
        Delay_Ms(150);
        motor_forward(right,4500);//4000
        motor_forward(left,0);//6000
        Delay_Ms(300);
    }

}

void barn_in_control(void)
{
//    uint8_t net;
    if(RoadFlag.barn_in_flag)
    {
        RoadFlag.barn_in_flag2 = 1;
        motor_forward(left,4500);
        motor_forward(right,4500);
        Delay_Ms(100);
        motor_forward(left,2700);
        motor_forward(right,6000);
        Delay_Ms(600);
        motor_forward(left,4500);
        motor_forward(right,4500);
        Delay_Ms(200);
        car_stop();
        while(RoadFlag.barn_in_flag2);
    }
}

void qianche_zebra_handle(void)
{
    if(RoadFlag.zebra_flag)
    {
        RoadFlag.zebra_flag2 = 1;
        car_stop();
        Delay_Ms(1000);
        uart_putchar(CH9143_UART,'A');
        Delay_Ms(1500);
        barn_in_control();
    }
}

void houche_zebra_handle(void)
{
    if(RoadFlag.zebra_flag)
    {
        //ǰ��һ��ʱ���ͣ��
        car_stop();
    }
}

void barn_out_control(void)
{
    static uint8_t a = 0;
    if(RoadFlag.barn_out_flag)
    {
        a ++ ;
        ips114_show_uint(48,48,a,3);
        if(a > 10)
            a = 10;
        if(a == 2)
        {
            motor_forward(left,5000);
            motor_forward(right,5000);
            Delay_Ms(400);
            motor_forward(left,3000);
            motor_forward(right,5000);
            Delay_Ms(400);
        }

        if(a == 10)
        {
            RoadFlag.barn_out_flag2 = 1;
        }
    }

}

void bridge_handle(void)//Ӧ�û����Լ����Ż�
{
    static int bt = 0;
    if(RoadFlag.bridge_flag == 1)
    {
        bt ++ ;
        if(RoadFlag.bridge_flag2 == 1)
       {
//            RoadFlag.bridge_flag2 = 0;

       motor_forward(left,5000);
       motor_forward(right,5000);
       Delay_Ms(500);
       motor_forward(left,4000);
       motor_forward(right,5000);
       Delay_Ms(200);

       }
        if(bt == 3)//5
        {
            RoadFlag.bridge_flag2 = 1;
        }
        else {
            RoadFlag.bridge_flag2 = 0;
        }
    }
}

void fork_handle(void)
{
    if(RoadFlag.fork_flag == 1)
    {
        motor_forward(left,5000);//4000
        motor_forward(right,5000);//6000
        Delay_Ms(400);
        motor_forward(left,5000);//4000
        motor_forward(right,2000);//6000
        Delay_Ms(200);
    }
}

//Ԫ�ش�����
void element_handler(void)
{

    motor_barrier();
    Fork_Handle();
    zhijiao_handle();
    qianche_zebra_handle();
}

//������ƺ���
void motor_control(void)
{
    float moto_value;
    float speed1, speed2;
    moto_value = PID_Compute(&positon);
    speed1 = 4500 - moto_value;//4850
    speed2 = 4500 + moto_value;
    motor_load(speed1, speed2);
}

#endif /* CONTROL_C_ */
