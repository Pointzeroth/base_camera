/*********************************************************************************************************************
* @file            car_init.c
* @author          Andreas HF
* @Target core     CH32V307VCT6
* @revisions       2022.09.07, V1.0
* @modify          none
********************************************************************************************************************/

#include "car_init.h"


//-------------------------------------------------------------------------------------------------------------------
// @brief        flash初始化
// @param        void
// @return       void
// Sample usage:        Config_Flash_SRAM(FLASH_256_SRAM_64);
                        //flash初始化
//-------------------------------------------------------------------------------------------------------------------
void Config_Flash_SRAM(FLASH_SRAM_DEFIN SetFlashSRAM)
{
    uint8_t UserByte = FLASH_GetUserOptionByte() & 0xff;//get user option byte

    switch(SetFlashSRAM)
    {
    case 0:
        UserByte &= ~(0xc0); // SRAM_CODE_MODE = 00
        break;
    case 1:
        UserByte &= ~(0xc0); // SRAM_CODE_MODE = 00
        UserByte |= 0x7f;    // SRAM_CODE_MODE = 01
        break;
    case 2:
        UserByte &= ~(0xc0); // SRAM_CODE_MODE = 00
        UserByte |= 0xbf;    // SRAM_CODE_MODE = 10
        break;
    case 3:
        UserByte |= 0xff;    // SRAM_CODE_MODE = 11
        break;
    default:
        break;
    }

    FLASH_Unlock();
    FLASH_ProgramOptionByteData(0x1ffff802, UserByte);
    FLASH_Lock();
}


//-------------------------------------------------------------------------------------------------------------------
// @brief        初始化
// @param        void
// @return       void
// Sample usage:        car_init();
                        //智能车初始化
//-------------------------------------------------------------------------------------------------------------------

void car_init()
{
//    DisableGlobalIRQ();                                         //默认关所有中断
    interrupt_global_disable();
    board_init();
    Delay_Init();

    /* 初始化电机 */
    motor_init(left);
    motor_init(right);

    /* 初始化按键 */
    key_init(K1);
    key_init(K2);
    key_init(K3);
    key_init(K4);
    key_init(K5);
    key_init(K6);

    adc_init(ADC_IN2_A2);
    adc_init(ADC_IN3_A3);
    adc_init(ADC_IN4_A4);
    adc_init(ADC_IN5_A5);
    adc_init(ADC_IN6_A6);

    /* 初始化板载LED */
    board_led_init(LED1);
    board_led_init(LED2);

    exti_enable(EXTI_Line3, C3, BOTH);
    exti_enable(EXTI_Line4, C4, BOTH);
    exti_enable(EXTI_Line5, C5, BOTH);


    /* 初始化干簧管 */
    reed_init();

    /* 初始化中断 */
    int_init();
    interrupt_global_enable();
    /* 我的初始化 */
    Config_Flash_SRAM(FLASH_256_SRAM_64);

    // 初始化CH9143蓝牙模块
    uart_init(CH9143_UART,CH9143_UART_BAUD,CH9143_UART_TX_PIN,CH9143_UART_RX_PIN);

    uart_init(UART_3, 115200, UART3_TX_B10, UART3_RX_B11);
    // 开启串口接收中断
    uart_interrupt_init(CH9143_UART,ENABLE,UARTINT_RX);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief        中断初始化
// @param        void
// @return       void
// Sample usage:        int_init();//中断初始化
//-------------------------------------------------------------------------------------------------------------------
void int_init(void)
{
//    timer_pit_interrupt_ms(TIM_4, 10);
//    timer_pit_interrupt_ms(TIM_5, 10);
}



