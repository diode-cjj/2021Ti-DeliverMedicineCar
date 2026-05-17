#include "encoder.h"

/* TIM2 和 TIM3 分别工作在编码器模式，用来读取左右轮脉冲 */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/* 左右轮累计位置，由编码器速度不断累加得到 */
extern long long current_location_L;
extern long long current_location_R;

void Encoder_Init(void)
{
    /* 启动左轮编码器接口 */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    /* 启动右轮编码器接口 */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

int16_t Encoder_GetLeftSpeed(void)
{
    /* 读取左轮从上次清零到现在的计数增量 */
    int16_t speed = 0;
    speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);

    /* 读取后立即清零，让下一次读到的是新的 10ms 增量 */
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    /* 累加左轮里程，供调试或距离判断使用 */
    current_location_L += speed;

    /* 返回本周期左轮编码器速度 */
    return speed;
}

int16_t Encoder_GetRightSpeed(void)
{
    /* 读取右轮从上次清零到现在的计数增量 */
    int16_t speed = 0;
    speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

    /* 读取后立即清零，让下一次读到的是新的 10ms 增量 */
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    /* 累加右轮里程，供调试或距离判断使用 */
    current_location_R += speed;

    /* 返回本周期右轮编码器速度 */
    return speed;
}
