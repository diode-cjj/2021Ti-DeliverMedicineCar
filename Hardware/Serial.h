#ifndef __SERIAL_H
#define __SERIAL_H

#include "main.h"

typedef void (*Serial_RxCallback)(uint8_t byte);

/* 初始化串口接收中断，USART1/2/3 都启动单字节接收 */
void Serial_Init(void);

/* 注册指定串口收到字节后的回调函数 */
void Serial_SetRxCallback(UART_HandleTypeDef *huart, Serial_RxCallback callback);

/* 通过指定串口发送原始数据 */
void Serial_SendData(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length);

/* 通过指定串口发送字符串 */
void Serial_SendString(UART_HandleTypeDef *huart, char *string);

/* 通过 USART1 打印调试信息 */
void Serial_Printf(char *format, ...);

#endif
