#include "bluetooth.h"
#include "vision.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>

#include "pid_speed.h"
#include "pid_angle.h"
#include "state_machine.h"

/* USART1 用于 VOFA，USART2 用于蓝牙，USART3 用于 K230 */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/* 速度环 PID 参数通过蓝牙 slider 命令在线调整 */
extern PID_TypeDef pid_speed_L;
extern PID_TypeDef pid_speed_R;

/* 蓝牙单字节接收缓存 */
uint8_t rx_data;

/* K230 单字节接收缓存 */
uint8_t vision_rx_data;

/* 蓝牙命令缓存，命令格式为 [tag,param,param] */
char rx_buffer[100];
uint8_t rx_state = 0;
uint8_t rx_index = 0;
uint8_t rx_flag = 0;

void Bluetooth_Init(void)
{
    /* 启动蓝牙 USART2 中断接收 */
    HAL_UART_Receive_IT(&huart2, &rx_data, 1);

    /* 启动 K230 USART3 中断接收 */
    HAL_UART_Receive_IT(&huart3, &vision_rx_data, 1);
}

void Bluetooth_OnRxByte(uint8_t byte)
{
    /* 等待命令起始符 '[' */
    if (rx_state == 0) {
        if (byte == '[' && rx_flag == 0) {
            rx_state = 1;
            rx_index = 0;
        }
    } else if (rx_state == 1) {
        /* 收到 ']' 表示一条蓝牙命令接收完成 */
        if (byte == ']') {
            rx_state = 0;
            rx_buffer[rx_index] = '\0';
            rx_flag = 1;
        } else if (rx_index < 99) {
            /* 命令内容写入缓存，最多保存 99 个字符 */
            rx_buffer[rx_index++] = byte;
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        /* USART2 收到的是蓝牙命令字节 */
        Bluetooth_OnRxByte(rx_data);
        HAL_UART_Receive_IT(&huart2, &rx_data, 1);
    } else if (huart->Instance == USART3) {
        /* USART3 收到的是 K230 视觉协议字节 */
        Vision_OnRxByte(vision_rx_data);
        HAL_UART_Receive_IT(&huart3, &vision_rx_data, 1);
    }
}

void Bluetooth_Loop(void)
{
    /* rx_flag 为 1 表示已经收到一条完整蓝牙命令 */
    if (rx_flag == 1) {
        char *tag = strtok(rx_buffer, ",");
        if (tag != NULL) {
            if (strcmp(tag, "slider") == 0) {
                /* slider 命令用于手机端在线调速度环参数 */
                char *name = strtok(NULL, ",");
                char *value_str = strtok(NULL, ",");
                if (name != NULL && value_str != NULL) {
                    float value = atof(value_str);
                    if (strcmp(name, "SpeedKp") == 0) {
                        pid_speed_L.Kp = value;
                        pid_speed_R.Kp = value;
                    } else if (strcmp(name, "SpeedKi") == 0) {
                        pid_speed_L.Ki = value;
                        pid_speed_R.Ki = value;
                    } else if (strcmp(name, "SpeedKd") == 0) {
                        pid_speed_L.Kd = value;
                        pid_speed_R.Kd = value;
                    }
                }
            }
            else if (strcmp(tag, "joystick") == 0) {
                /* joystick 命令用于手机摇杆遥控左右轮速度 */
                strtok(NULL, ",");
                char *lv_str = strtok(NULL, ",");
                char *rh_str = strtok(NULL, ",");

                if (lv_str != NULL && rh_str != NULL) {
                    float lv = atof(lv_str);
                    float rh = atof(rh_str);

                    /* 小于 5 的摇杆值当作死区，防止手松开后小车抖动 */
                    if (lv < 5.0f && lv > -5.0f) lv = 0.0f;
                    if (rh < 5.0f && rh > -5.0f) rh = 0.0f;

                    /* 把百分比摇杆量换算成实际目标速度 */
                    float MAX_REAL_SPEED = 120.0f;
                    float set_L = (lv + rh) / 100.0f * MAX_REAL_SPEED;
                    float set_R = (lv - rh) / 100.0f * MAX_REAL_SPEED;
                    PID_Speed_SetTarget(set_L, set_R);
                }
            }
            else if (strcmp(tag, "turn") == 0) {
                /* turn 命令用于调试原地转向角度 */
                char *val_str = strtok(NULL, ",");
                if (val_str != NULL) {
                    float angle = atof(val_str);
                    Exec_Debug_Turn(angle);
                }
            }
            else if (strcmp(tag, "Kp") == 0) {
                /* Kp 命令用于在线调整角度环比例系数 */
                char *val_str = strtok(NULL, ",");
                if (val_str != NULL) angle_kp = atof(val_str);
            }
            else if (strcmp(tag, "Kd") == 0) {
                /* Kd 命令用于在线调整角度环微分系数 */
                char *val_str = strtok(NULL, ",");
                if (val_str != NULL) angle_kd = atof(val_str);
            }
            else if (strcmp(tag, "Min") == 0) {
                /* Min 命令用于在线调整角度环最小输出补偿 */
                char *val_str = strtok(NULL, ",");
                if (val_str != NULL) angle_min_out = atof(val_str);
            }
        }

        /* 当前命令处理完成，允许接收下一条命令 */
        rx_flag = 0;
    }
}

void Bluetooth_Printf(char *format, ...)
{
    char str[100];
    va_list arg;

    /* 按 printf 格式拼接字符串 */
    va_start(arg, format);
    vsprintf(str, format, arg);
    va_end(arg);

    /* 通过蓝牙 USART2 发送给手机 */
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
}

void VOFA_Printf(char *format, ...)
{
    char str[100];
    va_list arg;

    /* 按 printf 格式拼接字符串 */
    va_start(arg, format);
    vsprintf(str, format, arg);
    va_end(arg);

    /* 通过 USART1 发送给电脑 VOFA */
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
}

void Bluetooth_SendTelemetry(uint8_t state, float yaw, float err)
{
    char SendBuffer[128];

    /* 手动拆分浮点数，避免 printf 浮点支持未开启时打印异常 */
    char yaw_sign = (yaw < 0) ? '-' : ' ';
    char err_sign = (err < 0) ? '-' : ' ';

    /* 提取航向角整数部分和 1 位小数 */
    int yaw_i = abs((int)yaw);
    int yaw_d = (int)(fabs(yaw) * 10.0f) % 10;

    /* 提取灰度误差整数部分和 1 位小数 */
    int err_i = abs((int)err);
    int err_d = (int)(fabs(err) * 10.0f) % 10;

    /* 保持原来的蓝牙遥测格式，方便手机端直接观察 */
    sprintf(SendBuffer, "State:%d | YAW:%c%d.%d | ERR:%c%d.%d\r\n",
            state,
            yaw_sign, yaw_i, yaw_d,
            err_sign, err_i, err_d);

    /* 通过蓝牙发送遥测数据 */
    HAL_UART_Transmit(&huart2, (uint8_t *)SendBuffer, strlen(SendBuffer), 100);
}

void Bluetooth_SendGrayData(uint16_t *gray)
{
    char SendBuffer[160];

    /* 打印 8 路灰度 ADC 原始值，用于现场判断红线和白底阈值 */
    sprintf(SendBuffer,
            "ADC|0:%04u 1:%04u 2:%04u 3:%04u 4:%04u 5:%04u 6:%04u 7:%04u\r\n",
            gray[0], gray[1], gray[2], gray[3], gray[4], gray[5], gray[6], gray[7]);

    /* 通过蓝牙发送灰度原始数据 */
    HAL_UART_Transmit(&huart2, (uint8_t *)SendBuffer, strlen(SendBuffer), 100);
}
