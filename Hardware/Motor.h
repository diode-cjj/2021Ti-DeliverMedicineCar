#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

/* 电机 PWM 最大占空比，对应 TIM1 的计数范围 */
#define MOTOR_MAX_PULSE 1000

/* 初始化电机 PWM 输出通道 */
void Motor_Init(void);

/* 设置左轮速度，正数前进，负数后退 */
void Motor_SetLeft(int speed);

/* 设置右轮速度，正数前进，负数后退 */
void Motor_SetRight(int speed);

/* 同时设置左右轮速度，供上层直接差速控制 */
void Motor_Drive(int left_speed, int right_speed);

/* 立即停止左右两个电机 */
void Motor_Stop(void);

#endif
