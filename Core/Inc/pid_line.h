#ifndef __PID_LINE_H
#define __PID_LINE_H

#include "pid_core.h"

/* 巡线 PID 参数：输入是灰度误差，输出是左右轮差速修正量 */
#define LINE_KP 1.2f
#define LINE_KI 0.0f
#define LINE_KD 5.0f

/* 巡线修正最大输出，防止转向修正过猛 */
#define LINE_MAX_OUT 40.0f

/* 初始化巡线 PID */
void PID_Line_Init(void);

/* 根据灰度误差计算转向修正量 */
float PID_Line_GetTurn(float line_error);

#endif
