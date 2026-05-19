#ifndef __GRAY_SENSOR_H
#define __GRAY_SENSOR_H

#include "main.h"

#define GRAY_SENSOR_NUM 8

/* 8 路灰度传感器的原始 ADC 值，索引 0~7 对应 8 个通道 */
extern uint16_t Gray_Value[GRAY_SENSOR_NUM];

/* 初始化灰度传感器使用的 ADC3、DMA 和三位通道选择脚 */
void Gray_Sensor_Init(void);

/* 使用 ADC DMA 读取 8 路灰度原始值，结果保存到 Gray_Value 数组 */
void Gray_Sensor_Read(void);

/* 使用 ADC DMA 读取指定一路灰度原始值，channel 范围 0~7 */
uint16_t Gray_Sensor_ReadChannel(uint8_t channel);

#endif
