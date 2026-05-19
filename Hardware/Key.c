#include "Key.h"

#define KEY1_PORT GPIOE
#define KEY1_PIN  GPIO_PIN_0
#define KEY2_PORT GPIOE
#define KEY2_PIN  GPIO_PIN_1
#define KEY3_PORT GPIOE
#define KEY3_PIN  GPIO_PIN_2

static uint8_t key1_event;
static uint8_t key2_event;
static uint8_t key3_event;

void Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* K1/K2/K3 使用 PE0/PE1/PE2，上拉输入，按下时为低电平 */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitStruct.Pin = KEY1_PIN | KEY2_PIN | KEY3_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

uint8_t Key_GetNum(void)
{
    /* 阻塞式按键读取，适合简单菜单或测试程序 */
    if (HAL_GPIO_ReadPin(KEY1_PORT, KEY1_PIN) == GPIO_PIN_RESET) {
        HAL_Delay(20);
        while (HAL_GPIO_ReadPin(KEY1_PORT, KEY1_PIN) == GPIO_PIN_RESET);
        HAL_Delay(20);
        return KEY_1;
    }
    if (HAL_GPIO_ReadPin(KEY2_PORT, KEY2_PIN) == GPIO_PIN_RESET) {
        HAL_Delay(20);
        while (HAL_GPIO_ReadPin(KEY2_PORT, KEY2_PIN) == GPIO_PIN_RESET);
        HAL_Delay(20);
        return KEY_2;
    }
    if (HAL_GPIO_ReadPin(KEY3_PORT, KEY3_PIN) == GPIO_PIN_RESET) {
        HAL_Delay(20);
        while (HAL_GPIO_ReadPin(KEY3_PORT, KEY3_PIN) == GPIO_PIN_RESET);
        HAL_Delay(20);
        return KEY_3;
    }
    return KEY_NONE;
}

void Key_Scan10ms(void)
{
    static uint8_t key1_last = 1U;
    static uint8_t key2_last = 1U;
    static uint8_t key3_last = 1U;

    uint8_t key1_now = HAL_GPIO_ReadPin(KEY1_PORT, KEY1_PIN);
    uint8_t key2_now = HAL_GPIO_ReadPin(KEY2_PORT, KEY2_PIN);
    uint8_t key3_now = HAL_GPIO_ReadPin(KEY3_PORT, KEY3_PIN);

    /* 从高电平变成低电平，说明按键刚刚被按下 */
    if (key1_last == 1U && key1_now == 0U) key1_event = 1U;
    if (key2_last == 1U && key2_now == 0U) key2_event = 1U;
    if (key3_last == 1U && key3_now == 0U) key3_event = 1U;

    key1_last = key1_now;
    key2_last = key2_now;
    key3_last = key3_now;
}

uint8_t Key_TakeK1Press(void)
{
    uint8_t event = key1_event;
    key1_event = 0U;
    return event;
}

uint8_t Key_TakeK2Press(void)
{
    uint8_t event = key2_event;
    key2_event = 0U;
    return event;
}

uint8_t Key_TakeK3Press(void)
{
    uint8_t event = key3_event;
    key3_event = 0U;
    return event;
}
