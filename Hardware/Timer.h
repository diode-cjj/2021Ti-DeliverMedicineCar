#ifndef __TIMER_H
#define __TIMER_H

#include "main.h"

/* 初始化 TIM6 定时中断，当前 CubeMX 配置为 10ms 进入一次中断 */
void Timer_Init(void);

/* 读取并清除 10ms 定时标志 */
uint8_t Timer_Take10msFlag(void);

/* 获取 TIM6 已经产生的 10ms 节拍数量 */
uint32_t Timer_Get10msTick(void);

/* TIM6 每次更新中断进入后调用一次 */
void Timer_OnTim6Elapsed(void);

#endif
