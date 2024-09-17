/*********************************************************************************************************************
* @file            main.h
* @version         1.0
* @Target core     CH32V307VCT6
* @date            2022-3-2
********************************************************************************************************************/
#ifndef MAIN_H_
#define MAIN_H_

#include "headfile.h"
#include "isr.h"

/* �궨�� */

// CH9143����ģ�鴮�ں�
#define CH9143_UART             UART_6
// CH9143����ģ�鲨����
#define CH9143_UART_BAUD        115200
// CH9143����ģ��TXD����
#define CH9143_UART_TX_PIN      UART6_TX_C0
// CH9143����ģ��RXD����
#define CH9143_UART_RX_PIN      UART6_RX_C1


/* �������� */

//extern int32 key;

/* �������� */

#endif /* MAIN_H_ */
