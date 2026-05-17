#ifndef __VISION_H
#define __VISION_H

#include "main.h"

/* K230 返回的目标左右位置：无、左、右 */
#define VISION_LOR_NONE  0U
#define VISION_LOR_LEFT  1U
#define VISION_LOR_RIGHT 2U

/* 初始化视觉接收状态机和稳定数字缓存 */
void Vision_Init(void);

/* 串口每收到 1 个字节，就调用一次这个函数解析视觉帧 */
void Vision_OnRxByte(uint8_t byte);

/* 10ms 周期任务，用于定时向 K230 发送当前识别任务 */
void Vision_Loop10ms(void);

/* 立即向 K230 发送一次任务编号和目标数字 */
void Vision_SendTask(uint8_t task, uint8_t target_num);

/* 设置后续要周期发送给 K230 的任务编号和目标数字 */
void Vision_SetTask(uint8_t task, uint8_t target_num);

/* 清空起点稳定识别到的目标数字 */
void Vision_ResetStableDigit(void);

/* 判断是否已经得到稳定目标数字 */
uint8_t Vision_HasStableDigit(void);

/* 获取稳定目标数字 */
uint8_t Vision_GetStableDigit(void);

/* 获取最近一帧视觉识别到的数字 */
uint8_t Vision_GetDigit(void);

/* 获取最近一帧视觉识别到的左右位置 */
uint8_t Vision_GetLoR(void);

/* 获取最近一帧视觉识别有效标志 */
uint8_t Vision_GetFindedFlag(void);

/* 获取最近一帧视觉任务号 */
uint8_t Vision_GetFindTask(void);

/* 获取最近一帧 K230 置信度 */
uint8_t Vision_GetConfidence(void);

/* 获取最近收到的原始字节，方便调试通信 */
uint8_t Vision_GetLastByte(void);

/* 判断最近一帧是否匹配当前目标数字 */
uint8_t Vision_TargetMatched(void);

/* 获取视觉串口收到的总字节数 */
uint32_t Vision_GetRxByteCount(void);

/* 获取成功解析的有效视觉帧数量 */
uint32_t Vision_GetPacketCount(void);

/* 获取 K230 AA55 协议校验失败次数 */
uint32_t Vision_GetChecksumErrorCount(void);

#endif
