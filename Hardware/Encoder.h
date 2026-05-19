#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

/* 左右轮累计位置，由编码器速度不断累加得到 */
extern long long current_location_L;
extern long long current_location_R;

/* 初始化左右轮编码器定时器 */
void Encoder_Init(void);

/* 读取左轮本周期编码器增量，读取后计数器清零 */
int16_t Encoder_GetLeftSpeed(void);

/* 读取右轮本周期编码器增量，读取后计数器清零 */
int16_t Encoder_GetRightSpeed(void);

/* 清零左右轮累计位置 */
void Encoder_ClearLocation(void);

#endif
