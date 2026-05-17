#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

/* 初始化左右轮编码器定时器 */
void Encoder_Init(void);

/* 读取左轮本周期编码器增量，读取后计数器清零 */
int16_t Encoder_GetLeftSpeed(void);

/* 读取右轮本周期编码器增量，读取后计数器清零 */
int16_t Encoder_GetRightSpeed(void);

#endif
