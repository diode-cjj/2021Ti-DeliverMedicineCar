#ifndef __RGB_LED_H
#define __RGB_LED_H

#include "main.h"

#define RGB_LED_RED   0x01U
#define RGB_LED_GREEN 0x02U
#define RGB_LED_BLUE  0x04U

/* 初始化三色 LED 使用的 PE4/PE5/PE6 引脚 */
void RGB_LED_Init(void);

/* 设置三色 LED 的红绿蓝状态，参数为 RGB_LED_RED/GREEN/BLUE 组合 */
void RGB_LED_Set(uint8_t color);

/* 关闭三色 LED */
void RGB_LED_Off(void);

/* 切换指定颜色通道状态 */
void RGB_LED_Toggle(uint8_t color);

#endif
