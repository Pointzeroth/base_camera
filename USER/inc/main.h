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

/* 宏定义 */

// CH9143蓝牙模块串口号
#define CH9143_UART             UART_6
// CH9143蓝牙模块波特率
#define CH9143_UART_BAUD        115200
// CH9143蓝牙模块TXD引脚
#define CH9143_UART_TX_PIN      UART6_TX_C0
// CH9143蓝牙模块RXD引脚
#define CH9143_UART_RX_PIN      UART6_RX_C1


/* 变量定义 */

//extern int32 key;

/* 函数声明 */

#endif /* MAIN_H_ */
