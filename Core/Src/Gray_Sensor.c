#include "Gray_Sensor.h"

/* 8 路灰度原始 ADC 数值，索引 0~7 对应外部多路选择器的 8 个通道 */
uint16_t Gray_Value[8] = {0};

/* 灰度传感器实际接在 ADC3 上，8 路通道通过 PC5/PC6/PC7 外部选择 */
extern ADC_HandleTypeDef hadc3;

void Gray_Sensor_Init(void)
{
    /* 启动 ADC3 校准，提高后续灰度采样稳定性 */
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
}

void Gray_Sensor_Read(void)
{
    uint8_t i, j;
    uint32_t sum;
    uint8_t valid_count;

    /* 依次读取外部多路选择器的 8 个灰度通道 */
    for (i = 0; i < 8; i++)
    {
        /* PC5/PC6/PC7 组成三位地址，选择当前要读取的灰度通道 */
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, (i & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, (i & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, (i & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        /* 通道切换后等待一小段时间，让模拟电压稳定 */
        for (volatile int delay = 0; delay < 50; delay++);

        /* 每一路采样 3 次后求平均，兼顾速度和稳定性 */
        sum = 0;
        valid_count = 0;
        for (j = 0; j < 3; j++)
        {
            /* 启动一次 ADC 软件转换 */
            HAL_ADC_Start(&hadc3);

            /* 转换成功就累加本次 ADC 数值 */
            if (HAL_ADC_PollForConversion(&hadc3, 1) == HAL_OK)
            {
                sum += HAL_ADC_GetValue(&hadc3);
                valid_count++;
            }
            else
            {
                /* 如果本次转换超时，就停止 ADC 并放弃当前通道后续采样 */
                HAL_ADC_Stop(&hadc3);
                break;
            }
        }

        /* 有有效采样就保存平均值，否则用 9999 表示本通道读取失败 */
        if (valid_count > 0) Gray_Value[i] = sum / valid_count;
        else Gray_Value[i] = 9999;
    }
}

/* 巡线只使用中间 6 路：索引 1~6，最外侧索引 0 和 7 不参与误差计算 */
/* 左侧传感器权重为正，右侧传感器权重为负，中心传感器权重为 0 */
static const float Weight[8] = {0.0f, 2.0f, 1.0f, 0.0f, 0.0f, -1.0f, -2.0f, 0.0f};

float Get_Grayscale_Error(void)
{
    /* 每次计算误差前先刷新 8 路灰度 ADC 数值 */
    Gray_Sensor_Read();

    /* Sum_Val 是红线强度总和，Sum_Weight 是带方向的加权总和 */
    float Sum_Val = 0.0f;
    float Sum_Weight = 0.0f;

    /* line_count 记录有多少个探头明确压到红线，用于判断路口 */
    int line_count = 0;

    /* 只遍历中间 6 路灰度，避免两侧边缘探头干扰巡线 */
    for (int i = 1; i <= 6; i++)
    {
        /* raw 是当前通道原始 ADC 值 */
        float raw = (float)Gray_Value[i];

        /* norm 是归一化后的红线强度，0 表示白底，1 表示红线 */
        float norm = 0.0f;

        /* ADC 大于 1200 认为是白底，不参与巡线 */
        if (raw >= 1200.0f) {
            norm = 0.0f;
        }
        /* ADC 小于 500 认为明显压到红线 */
        else if (raw <= 500.0f) {
            norm = 1.0f;
        }
        else {
            /* 500~1200 之间认为是红线边缘，按比例转换成强度 */
            norm = (1200.0f - raw) / (1200.0f - 500.0f);
        }

        /* 红线强度大于 0.5，认为这个探头压线 */
        if (norm > 0.5f) {
            line_count++;
        }

        /* 不是纯白底时才参与方向误差计算 */
        if (norm > 0.01f) {
            Sum_Val += norm;
            Sum_Weight += norm * Weight[i];
        }
    }

    /* 多个探头同时压线，认为到达路口或大面积红线 */
    if (line_count >= 4) {
        return 888.0f;
    }

    /* 几乎没有探头看到红线，认为脱线或到达终点 */
    if (Sum_Val < 0.1f) {
        return 999.0f;
    }

    /* 正常巡线误差，正负号由权重数组决定 */
    return Sum_Weight / Sum_Val;
}
