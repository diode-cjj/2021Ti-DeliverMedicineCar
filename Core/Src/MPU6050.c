#include "MPU6050.h"
#include "i2c.h"
#include "MPU6050_Reg.h"

/* HAL I2C 读写地址需要包含读写位，所以 0x68 左移 1 位后是 0xD0 */
#define MPU6050_ADDRESS 0xD0

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    /* 通过硬件 I2C1 向指定寄存器写入 1 个字节 */
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, RegAddress, I2C_MEMADD_SIZE_8BIT, &Data, 1, 100);
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;

    /* 通过硬件 I2C1 从指定寄存器读取 1 个字节 */
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, RegAddress, I2C_MEMADD_SIZE_8BIT, &Data, 1, 100);

    /* 返回读到的寄存器数据 */
    return Data;
}

void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count)
{
    /* 从指定起始寄存器开始连续读取多个字节，提高六轴数据读取效率 */
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, RegAddress, I2C_MEMADD_SIZE_8BIT, DataArray, Count, 100);
}

void MPU6050_Init(void)
{
    /* 硬件 I2C1 已经由 MX_I2C1_Init 初始化，这里只等待 MPU6050 上电稳定 */
    HAL_Delay(50);

    /* 解除休眠，并选择陀螺仪 X 轴 PLL 作为时钟源 */
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);

    /* 开启加速度计和陀螺仪所有轴 */
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);

    /* 设置采样率分频 */
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x07);

    /* 设置数字低通滤波器 */
    MPU6050_WriteReg(MPU6050_CONFIG, 0x00);

    /* 陀螺仪量程设置为 ±2000°/s */
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);

    /* 加速度计量程设置为 ±16g */
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
}

uint8_t MPU6050_GetID(void)
{
    /* WHO_AM_I 正常应返回 MPU6050 的设备地址相关值 */
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t Data[14];

    /* 从 ACCEL_XOUT_H 开始连续读取 14 字节：加速度、温度、陀螺仪 */
    MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, Data, 14);

    /* 高 8 位和低 8 位合成 16 位有符号加速度原始值 */
    *AccX = (Data[0] << 8) | Data[1];
    *AccY = (Data[2] << 8) | Data[3];
    *AccZ = (Data[4] << 8) | Data[5];

    /* 跳过 Data[6] 和 Data[7] 的温度值，合成三轴陀螺仪原始值 */
    *GyroX = (Data[8] << 8) | Data[9];
    *GyroY = (Data[10] << 8) | Data[11];
    *GyroZ = (Data[12] << 8) | Data[13];
}

/* Z 轴陀螺仪静态零漂，main.c 积分航向角时会减去它 */
float gz_offset = 0.0f;

void MPU6050_Calibration(void)
{
    int32_t sum_gz = 0;
    int16_t ax, ay, az, gx, gy, gz;

    /* 启动时保持小车静止，连续采样 100 次 Z 轴角速度 */
    for (int i = 0; i < 100; i++) {
        MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);
        sum_gz += gz;
        HAL_Delay(5);
    }

    /* 平均值就是当前陀螺仪 Z 轴零漂 */
    gz_offset = (float)sum_gz / 100.0f;
}
