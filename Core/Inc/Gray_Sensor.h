#ifndef __GRAY_SENSOR_H
#define __GRAY_SENSOR_H

#include "main.h"

/* 8 路灰度传感器的原始 ADC 值，蓝牙调试会直接打印这个数组 */
extern uint16_t Gray_Value[8];

/* 初始化灰度传感器使用的 ADC3 校准 */
void Gray_Sensor_Init(void);

/* 读取 8 路灰度原始值，结果保存到 Gray_Value 数组 */
void Gray_Sensor_Read(void);

/* 获取巡线误差：正常返回偏差值，888 表示路口，999 表示脱线 */
float Get_Grayscale_Error(void);

#endif
