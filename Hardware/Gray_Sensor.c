#include "Gray_Sensor.h"
#include "adc.h"

#define GRAY_SELECT_PORT GPIOC
#define GRAY_SELECT_A0   GPIO_PIN_5
#define GRAY_SELECT_A1   GPIO_PIN_6
#define GRAY_SELECT_A2   GPIO_PIN_7
#define GRAY_SAMPLE_NUM  1

/* 8 路灰度原始 ADC 数值，索引 0~7 对应外部多路选择器的 8 个通道 */
uint16_t Gray_Value[GRAY_SENSOR_NUM] = {0};

static uint16_t gray_dma_buffer[GRAY_SAMPLE_NUM] __attribute__((section(".dma_buffer"), aligned(32)));
static volatile uint8_t gray_dma_finish;

static void Gray_Sensor_SelectChannel(uint8_t channel)
{
    /* PC5/PC6/PC7 组成三位地址，选择当前要读取的灰度通道 */
    HAL_GPIO_WritePin(GRAY_SELECT_PORT, GRAY_SELECT_A0, (channel & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GRAY_SELECT_PORT, GRAY_SELECT_A1, (channel & 0x02U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GRAY_SELECT_PORT, GRAY_SELECT_A2, (channel & 0x04U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Gray_Sensor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 灰度模块使用 PC5/PC6/PC7 作为外部多路选择器地址线 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct.Pin = GRAY_SELECT_A0 | GRAY_SELECT_A1 | GRAY_SELECT_A2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GRAY_SELECT_PORT, &GPIO_InitStruct);

    /* 启动 ADC3 校准，提高后续灰度采样稳定性 */
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
}

uint16_t Gray_Sensor_ReadChannel(uint8_t channel)
{
    uint32_t sum = 0;

    /* 通道范围限制在 0~7，避免选择脚出现无效组合 */
    channel &= 0x07U;
    Gray_Sensor_SelectChannel(channel);

    /* 通道切换后等待一小段时间，让模拟电压稳定 */
    for (volatile uint16_t delay = 0; delay < 80; delay++);

    gray_dma_finish = 0U;
    if (HAL_ADC_Start_DMA(&hadc3, (uint32_t *)gray_dma_buffer, GRAY_SAMPLE_NUM) != HAL_OK) {
        HAL_ADC_Stop_DMA(&hadc3);
        return 9999U;
    }

    /* 等待本路 DMA 采样完成，超时则返回 9999 表示读取失败 */
    uint32_t tick = HAL_GetTick();
    while (gray_dma_finish == 0U) {
        if (HAL_GetTick() - tick > 5U) {
            HAL_ADC_Stop_DMA(&hadc3);
            return 9999U;
        }
    }
    HAL_ADC_Stop_DMA(&hadc3);

    /* 每一路 DMA 采样完成后求平均，兼顾速度和稳定性 */
    for (uint8_t i = 0; i < GRAY_SAMPLE_NUM; i++) {
        sum += gray_dma_buffer[i];
    }

    return (uint16_t)(sum / GRAY_SAMPLE_NUM);
}

void Gray_Sensor_Read(void)
{
    /* 依次读取外部多路选择器的 8 个灰度通道 */
    for (uint8_t i = 0; i < GRAY_SENSOR_NUM; i++) {
        Gray_Value[i] = Gray_Sensor_ReadChannel(i);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC3) {
        gray_dma_finish = 1U;
    }
}
