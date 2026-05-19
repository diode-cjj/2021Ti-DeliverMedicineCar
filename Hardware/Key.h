#ifndef __KEY_H
#define __KEY_H

#include "main.h"

#define KEY_NONE 0U
#define KEY_1    1U
#define KEY_2    2U
#define KEY_3    3U

/* 初始化按键引脚，K1/K2/K3 使用 PE0/PE1/PE2，上拉输入，按下为低电平 */
void Key_Init(void);

/* 扫描按键状态，返回 KEY_NONE、KEY_1、KEY_2 或 KEY_3 */
uint8_t Key_GetNum(void);

/* 10ms 周期调用，更新按键下降沿事件 */
void Key_Scan10ms(void);

/* 读取并清除 K1/K2/K3 按下事件 */
uint8_t Key_TakeK1Press(void);
uint8_t Key_TakeK2Press(void);
uint8_t Key_TakeK3Press(void);

#endif
