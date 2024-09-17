/*
 * flash.c
 *
 *  Created on: 2024年7月20日
 *      Author: Lenovo
 */

#include "flash.h"

// 修改配置项的函数
void modify_config(int index, int16_t new_value) {
    if (index >= 0 && index < CONFIG_FINISH) {
        mt9v03x_set_confing_buffer_dvp[index][1] = new_value;
    } else {
        ips114_show_string(0,80,"无效的值");
    }
}

