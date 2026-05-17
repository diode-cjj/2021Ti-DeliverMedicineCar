#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32h7xx_hal.h"

/* Z 轴陀螺仪零漂值，main.c 会用它修正航向角积分 */
extern float gz_offset;

/* 向 MPU6050 指定寄存器写入 1 个字节 */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);

/* 从 MPU6050 指定寄存器读取 1 个字节 */
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

/* 初始化 MPU6050 的电源、采样率、量程等寄存器 */
void MPU6050_Init(void);

/* 读取 MPU6050 设备 ID，用于检查 I2C 通信是否正常 */
uint8_t MPU6050_GetID(void);

/* 一次性读取三轴加速度和三轴陀螺仪原始数据 */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

/* 启动时静止校准 Z 轴陀螺仪零漂 */
void MPU6050_Calibration(void);

#endif
