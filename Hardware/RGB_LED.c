#include "RGB_LED.h"

#define RGB_LED_BLUE_PORT  GPIOE
#define RGB_LED_BLUE_PIN   GPIO_PIN_4
#define RGB_LED_GREEN_PORT GPIOE
#define RGB_LED_GREEN_PIN  GPIO_PIN_5
#define RGB_LED_RED_PORT   GPIOE
#define RGB_LED_RED_PIN    GPIO_PIN_6

void RGB_LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 三色 LED 使用 PE4/PE5/PE6，所以先打开 GPIOE 时钟 */
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitStruct.Pin = RGB_LED_BLUE_PIN | RGB_LED_GREEN_PIN | RGB_LED_RED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* 初始化后默认关闭三色灯 */
    RGB_LED_Off();
}

void RGB_LED_Set(uint8_t color)
{
    /* 根据颜色掩码分别控制红绿蓝三路输出 */
    HAL_GPIO_WritePin(RGB_LED_RED_PORT, RGB_LED_RED_PIN, (color & RGB_LED_RED) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RGB_LED_GREEN_PORT, RGB_LED_GREEN_PIN, (color & RGB_LED_GREEN) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RGB_LED_BLUE_PORT, RGB_LED_BLUE_PIN, (color & RGB_LED_BLUE) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void RGB_LED_Off(void)
{
    /* 三个颜色通道全部关闭 */
    HAL_GPIO_WritePin(RGB_LED_RED_PORT, RGB_LED_RED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RGB_LED_GREEN_PORT, RGB_LED_GREEN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RGB_LED_BLUE_PORT, RGB_LED_BLUE_PIN, GPIO_PIN_RESET);
}

void RGB_LED_Toggle(uint8_t color)
{
    /* 按颜色掩码切换指定颜色通道 */
    if (color & RGB_LED_RED) HAL_GPIO_TogglePin(RGB_LED_RED_PORT, RGB_LED_RED_PIN);
    if (color & RGB_LED_GREEN) HAL_GPIO_TogglePin(RGB_LED_GREEN_PORT, RGB_LED_GREEN_PIN);
    if (color & RGB_LED_BLUE) HAL_GPIO_TogglePin(RGB_LED_BLUE_PORT, RGB_LED_BLUE_PIN);
}
