#ifndef __PID_CORE_H
#define __PID_CORE_H

/* PID 通用结构体：速度环、巡线环等都可以共用这一套参数和中间量 */
typedef struct {
    float target_val;  /* 目标值，例如目标速度或目标误差 */
    float actual_val;  /* 实际值，例如编码器速度或当前灰度误差 */
    float err;         /* 本次误差，等于目标值减实际值 */
    float err_last;    /* 上一次误差，用于计算微分项 */
    float integral;    /* 积分累计值，用于消除长期静差 */
    float Kp, Ki, Kd;  /* 比例、积分、微分三个 PID 参数 */
    float out_max;     /* 输出限幅，防止 PID 输出过大 */
    float int_max;     /* 积分限幅，防止积分长期累加到失控 */
} PID_TypeDef;

/* 初始化 PID 参数和限幅，并清空运行中的误差数据 */
void PID_Init_Core(PID_TypeDef *pid, float p, float i, float d, float max_out, float max_int);

/* 清空目标值、实际值、误差和积分，用于重新开始控制 */
void PID_Clear(PID_TypeDef *pid);

/* 位置式 PID 计算函数，输入目标值和实际值，返回限幅后的控制输出 */
float PID_Calc(PID_TypeDef *pid, float target, float actual);

#endif
