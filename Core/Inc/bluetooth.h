#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "main.h"

/* 初始化蓝牙串口接收，同时启动 K230 串口接收 */
void Bluetooth_Init(void);

/* 蓝牙串口每收到一个字节时调用，用于拼接一条完整命令 */
void Bluetooth_OnRxByte(uint8_t byte);

/* 主循环中调用，解析已经接收完成的蓝牙命令 */
void Bluetooth_Loop(void);

/* 通过蓝牙 USART2 打印调试信息 */
void Bluetooth_Printf(char *format, ...);

/* 通过 USART1 打印 VOFA 调试信息 */
void VOFA_Printf(char *format, ...);

/* 发送状态、航向角和灰度误差遥测数据 */
void Bluetooth_SendTelemetry(uint8_t state, float yaw, float err);

/* 发送 8 路灰度 ADC 原始数据 */
void Bluetooth_SendGrayData(uint16_t *gray);

#endif
