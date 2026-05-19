#include "Bluetooth.h"
#include "Serial.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* 蓝牙命令缓存，命令格式为 [tag,param,param] */
static char bluetooth_rx_buffer[100];
static uint8_t bluetooth_rx_state;
static uint8_t bluetooth_rx_index;
static uint8_t bluetooth_rx_flag;

void Bluetooth_Init(void)
{
    /* USART2 用于蓝牙，接收到字节后交给 Bluetooth_OnRxByte 解析 */
    bluetooth_rx_state = 0U;
    bluetooth_rx_index = 0U;
    bluetooth_rx_flag = 0U;
    Serial_SetRxCallback(&huart2, Bluetooth_OnRxByte);
}

void Bluetooth_OnRxByte(uint8_t byte)
{
    /* 等待命令起始符 '[' */
    if (bluetooth_rx_state == 0U) {
        if (byte == '[' && bluetooth_rx_flag == 0U) {
            bluetooth_rx_state = 1U;
            bluetooth_rx_index = 0U;
        }
    } else if (bluetooth_rx_state == 1U) {
        /* 收到 ']' 表示一条蓝牙命令接收完成 */
        if (byte == ']') {
            bluetooth_rx_state = 0U;
            bluetooth_rx_buffer[bluetooth_rx_index] = '\0';
            bluetooth_rx_flag = 1U;
        } else if (bluetooth_rx_index < sizeof(bluetooth_rx_buffer) - 1U) {
            /* 命令内容写入缓存，最多保存 99 个字符 */
            bluetooth_rx_buffer[bluetooth_rx_index++] = byte;
        }
    }
}

uint8_t Bluetooth_HasCommand(void)
{
    return bluetooth_rx_flag;
}

uint8_t Bluetooth_GetCommand(char *buffer, uint16_t buffer_size)
{
    if (bluetooth_rx_flag == 0U || buffer_size == 0U) {
        return 0U;
    }

    /* 复制完整命令给上层，随后清除标志允许接收下一条命令 */
    strncpy(buffer, bluetooth_rx_buffer, buffer_size - 1U);
    buffer[buffer_size - 1U] = '\0';
    bluetooth_rx_flag = 0U;
    return 1U;
}

void Bluetooth_Printf(char *format, ...)
{
    char str[128];
    va_list arg;

    /* 按 printf 格式拼接字符串 */
    va_start(arg, format);
    vsprintf(str, format, arg);
    va_end(arg);

    /* 通过蓝牙 USART2 发送给手机 */
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
}

void Bluetooth_SendGrayData(uint16_t *gray)
{
    char send_buffer[160];

    /* 打印 8 路灰度 ADC 原始值，用于现场判断红线和白底阈值 */
    sprintf(send_buffer,
            "ADC|0:%04u 1:%04u 2:%04u 3:%04u 4:%04u 5:%04u 6:%04u 7:%04u\r\n",
            gray[0], gray[1], gray[2], gray[3], gray[4], gray[5], gray[6], gray[7]);

    /* 通过蓝牙发送灰度原始数据 */
    HAL_UART_Transmit(&huart2, (uint8_t *)send_buffer, strlen(send_buffer), 100);
}
