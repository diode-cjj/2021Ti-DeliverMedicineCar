#ifndef __PID_SPEED_H
#define __PID_SPEED_H

#include "pid_core.h"

/* 速度环比例系数：决定速度误差变化时 PWM 调整的力度 */
#define SPEED_KP 1.5f

/* 速度环积分系数：负责维持恒定速度时的持续输出 */
#define SPEED_KI 0.5f

/* 当前编码器速度离散性较强，微分项容易放大抖动，所以保持为 0 */
#define SPEED_KD 0.0f

/* PWM 最大输出限幅，防止下发给电机的速度过大 */
#define SPEED_MAX_OUT 1000.0f

/* 通用 PID 结构体保留积分限幅参数，速度环当前主要使用增量式 PI 逻辑 */
#define SPEED_MAX_INT 1000.0f

/* 初始化左右轮速度环 */
void PID_Speed_Init(void);

/* 设置左右轮目标速度，目标为 0 时会清空油门记忆 */
void PID_Speed_SetTarget(float left_target, float right_target);

/* 执行一次速度闭环，根据编码器反馈更新电机 PWM */
void PID_Speed_Execute(void);

#endif
