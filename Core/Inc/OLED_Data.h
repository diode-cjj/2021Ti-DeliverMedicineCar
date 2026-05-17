#ifndef __OLED_DATA_H
#define __OLED_DATA_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

/* 中文字模结构体：Index 保存 UTF-8 字符，Data 保存 16x16 点阵 */
typedef struct
{
    char Index[5];
    uint8_t Data[32];
} ChineseCell_t;

/* 宽 8 像素、高 16 像素的 ASCII 字模表 */
extern const uint8_t OLED_F8x16[][16];

/* 宽 6 像素、高 8 像素的 ASCII 字模表 */
extern const uint8_t OLED_F6x8[][6];

/* 16x16 中文字模表，OLED_ShowString 会在这里查找中文字符 */
extern const ChineseCell_t OLED_CF16x16[];

#endif
