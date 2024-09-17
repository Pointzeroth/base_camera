/*********************************************************************************************************************
* @file            car_light.h
* @author          Andreas HF
* @Target core     CH32V307VCT6
* @revisions       -2022.10.22, V1.0
* @modify          none
 ********************************************************************************************************************/

#ifndef __CAR_LIGHT_H
#define __CAR_LIGHT_H

#include "headfile.h"

//ȫ�ֱ�������
extern  unsigned char rgb_off[3];
extern  unsigned char rgb_blue[3];
extern  unsigned char rgb_green[3];
extern  unsigned char rgb_cyan[3];
extern  unsigned char rgb_red[3];
extern  unsigned char rgb_purple[3];
extern  unsigned char rgb_yellow[3];
extern  unsigned char rgb_white[3];

/* ���Ͷ��� */
typedef enum
{
    red,
    yellow,
    white,
    green,
    blue,
    cyan,
    purple
}rgb_color;

//��������(�ⲿ�������û������е���)
/* ���ܳ�β��ͬʱ���� */
void car_both_rgb_on(rgb_color color);
/* ���ܳ�β��Ϩ�� */
void car_both_rgb_off();
/* ���ܳ�β�������˸ */
void car_left_rgb_flash(rgb_color color, uint16_t flash_count, uint16_t time);
/* ���ܳ�β���ҵ���˸ */
void car_right_rgb_flash(rgb_color color, uint16_t flash_count, uint16_t time);
/* ���ܳ�β��ͬʱ��˸ */
void car_both_rgb_flash(rgb_color color, uint16_t flash_count, uint16_t time);

#endif
