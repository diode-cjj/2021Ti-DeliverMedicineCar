#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "main.h"

/* 初始化蓝牙串口接收，蓝牙默认接在 USART2 */
void Bluetooth_Init(void);

/* 蓝牙串口每收到一个字节时调用，用于拼接一条完整命令 */
void Bluetooth_OnRxByte(uint8_t byte);

/* 判断是否收到完整蓝牙命令 */
uint8_t Bluetooth_HasCommand(void);

/* 复制一条完整蓝牙命令，读取后清除接收标志 */
uint8_t Bluetooth_GetCommand(char *buffer, uint16_t buffer_size);

/* 通过蓝牙 USART2 打印调试信息 */
void Bluetooth_Printf(char *format, ...);

/* 发送 8 路灰度 ADC 原始数据 */
void Bluetooth_SendGrayData(uint16_t *gray);

#endif
