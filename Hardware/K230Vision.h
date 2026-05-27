#ifndef __K230VISION_H
#define __K230VISION_H

#include "main.h"

typedef struct {
    uint8_t found;
    int16_t cx;
    int16_t cy;
    int16_t width;
    int16_t height;
    uint16_t distance_cm;
    uint32_t tick;
} K230Vision_Target_t;

/* 初始化 K230 视觉串口接收，K230 默认接在 USART3 */
void K230Vision_Init(void);

/* 清空 K230 目标数据 */
void K230Vision_Clear(void);

/* K230 串口每收到一个字节时调用，用于拼接一帧视觉数据 */
void K230Vision_OnRxByte(uint8_t byte);

/* 在主循环中解析 K230 最新一帧数据 */
void K230Vision_Update(void);

/* 读取当前小球数据，返回 1 表示当前帧识别到小球 */
uint8_t K230Vision_GetTarget(K230Vision_Target_t *target);

/* 判断 K230 是否仍在持续发送数据 */
uint8_t K230Vision_IsOnline(uint32_t timeout_ms);

#endif
