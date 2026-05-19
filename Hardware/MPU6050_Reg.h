#ifndef __MPU6050_REG_H
#define __MPU6050_REG_H

/* 采样率、低通滤波、陀螺仪量程、加速度量程配置寄存器 */
#define MPU6050_SMPLRT_DIV     0x19
#define MPU6050_CONFIG         0x1A
#define MPU6050_GYRO_CONFIG    0x1B
#define MPU6050_ACCEL_CONFIG   0x1C

/* 三轴加速度原始数据寄存器，高字节在前，低字节在后 */
#define MPU6050_ACCEL_XOUT_H   0x3B
#define MPU6050_ACCEL_XOUT_L   0x3C
#define MPU6050_ACCEL_YOUT_H   0x3D
#define MPU6050_ACCEL_YOUT_L   0x3E
#define MPU6050_ACCEL_ZOUT_H   0x3F
#define MPU6050_ACCEL_ZOUT_L   0x40

/* 温度原始数据寄存器，本工程暂时不使用 */
#define MPU6050_TEMP_OUT_H     0x41
#define MPU6050_TEMP_OUT_L     0x42

/* 三轴陀螺仪原始数据寄存器，高字节在前，低字节在后 */
#define MPU6050_GYRO_XOUT_H    0x43
#define MPU6050_GYRO_XOUT_L    0x44
#define MPU6050_GYRO_YOUT_H    0x45
#define MPU6050_GYRO_YOUT_L    0x46
#define MPU6050_GYRO_ZOUT_H    0x47
#define MPU6050_GYRO_ZOUT_L    0x48

/* 电源管理寄存器，用于解除休眠和开启各轴 */
#define MPU6050_PWR_MGMT_1     0x6B
#define MPU6050_PWR_MGMT_2     0x6C

/* 设备 ID 寄存器，用于检查 I2C 通信是否正常 */
#define MPU6050_WHO_AM_I       0x75

#endif
