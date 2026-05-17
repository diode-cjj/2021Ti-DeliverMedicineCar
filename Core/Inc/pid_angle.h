#ifndef __PID_ANGLE_H
#define __PID_ANGLE_H

#include <stdint.h>

/* 角度环比例系数，决定转向时纠正角度误差的力度 */
extern float angle_kp;

/* 角度环微分系数，主要用于抑制转向过冲 */
extern float angle_kd;

/* 最小输出补偿，用来克服电机静摩擦，避免最后几度不动 */
extern float angle_min_out;

/* 最大输出限幅，用来限制原地转向速度，防止打滑 */
extern float angle_max_out;

/* 初始化角度 PID 的目标角、历史误差和稳定计数 */
void PID_Angle_Init(void);

/* 设置新的目标航向角，开始一次原地转向 */
void PID_Angle_StartTurn(float target_yaw);

/* 根据当前航向角计算左右轮差速转向输出 */
float PID_Angle_GetTurn(float current_yaw);

/* 判断角度是否已经连续稳定一段时间 */
int PID_Angle_IsSettled(void);

#endif
