#ifndef __OLED_H
#define __OLED_H

#include "stm32h7xx_hal.h"

/* OLED 字符串按 UTF-8 解析，中文显示依赖 OLED_Data.c 中的字模表 */
#define OLED_CHARSET_UTF8

/* 字体大小：8x16 用于较大字符，6x8 用于调试状态页 */
#define OLED_8X16 8
#define OLED_6X8  6

/* 图形填充参数：0 不填充，1 填充 */
#define OLED_UNFILLED 0
#define OLED_FILLED   1


void OLED_Init(void);
void OLED_Clear(void);
void OLED_Update(void);
void OLED_ShowChar(int16_t X, int16_t Y, char Char, uint8_t FontSize);
void OLED_ShowString(int16_t X, int16_t Y, char *String, uint8_t FontSize);
void OLED_ShowNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);
void OLED_ShowSignedNum(int16_t X, int16_t Y, int32_t Number, uint8_t Length, uint8_t FontSize);
void OLED_ShowHexNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);
void OLED_ShowBinNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);
void OLED_ShowFloatNum(int16_t X, int16_t Y, double Number, uint8_t IntLength, uint8_t FraLength, uint8_t FontSize); 
void OLED_ShowImage(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, const uint8_t *Image);     /* 在显存中绘制一张点阵图片 */
void OLED_Printf(int16_t X, int16_t Y, uint8_t FontSize, char *format, ...);    /* 按 printf 格式在 OLED 上显示调试字符串 */

#endif
