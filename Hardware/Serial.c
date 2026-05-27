#include "Serial.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static uint8_t usart1_rx_data;
static uint8_t usart2_rx_data;
static uint8_t usart3_rx_data;

static Serial_RxCallback usart1_callback;
static Serial_RxCallback usart2_callback;
static Serial_RxCallback usart3_callback;

void Serial_Init(void)
{
    /* 启动 USART1/2 单字节中断接收 */
    HAL_UART_Receive_IT(&huart1, &usart1_rx_data, 1);
    HAL_UART_Receive_IT(&huart2, &usart2_rx_data, 1);
    HAL_UART_Receive_IT(&huart3, &usart3_rx_data, 1);
}

void Serial_SetRxCallback(UART_HandleTypeDef *huart, Serial_RxCallback callback)
{
    /* 根据串口句柄保存对应的接收回调 */
    if (huart->Instance == USART1) {
        usart1_callback = callback;
    } else if (huart->Instance == USART2) {
        usart2_callback = callback;
    } else if (huart->Instance == USART3) {
        usart3_callback = callback;
    }
}

void Serial_SendData(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length)
{
    /* 通过指定串口阻塞发送一段数据 */
    HAL_UART_Transmit(huart, data, length, 100);
}

void Serial_SendString(UART_HandleTypeDef *huart, char *string)
{
    /* 通过指定串口发送字符串 */
    HAL_UART_Transmit(huart, (uint8_t *)string, strlen(string), 100);
}

void Serial_Printf(char *format, ...)
{
    char str[128];
    va_list arg;

    /* 按 printf 格式拼接调试字符串 */
    va_start(arg, format);
    vsprintf(str, format, arg);
    va_end(arg);

    /* 默认通过 USART1 输出给电脑调试工具 */
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        /* USART1 收到字节后调用用户回调，并重新打开接收中断 */
        if (usart1_callback != 0) usart1_callback(usart1_rx_data);
        HAL_UART_Receive_IT(&huart1, &usart1_rx_data, 1);
    } else if (huart->Instance == USART2) {
        /* USART2 收到字节后调用用户回调，并重新打开接收中断 */
        if (usart2_callback != 0) usart2_callback(usart2_rx_data);
        HAL_UART_Receive_IT(&huart2, &usart2_rx_data, 1);
    } else if (huart->Instance == USART3) {
        /* USART3 鏀跺埌瀛楄妭鍚庤皟鐢ㄧ敤鎴峰洖璋冿紝骞堕噸鏂版墦寮€鎺ユ敹涓柇 */
        if (usart3_callback != 0) usart3_callback(usart3_rx_data);
        HAL_UART_Receive_IT(&huart3, &usart3_rx_data, 1);
    }
}
