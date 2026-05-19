#include "Timer.h"
#include "tim.h"

static volatile uint8_t Timer_10msFlag;
static volatile uint32_t Timer_10msTick;

void Timer_Init(void)
{
    /* 启动 TIM6 基本定时器中断，TIM6 的分频和周期由 CubeMX 生成代码配置 */
    Timer_10msFlag = 0U;
    Timer_10msTick = 0U;
    HAL_TIM_Base_Start_IT(&htim6);
}

uint8_t Timer_Take10msFlag(void)
{
    uint8_t flag = Timer_10msFlag;

    /* 主循环读取后清除标志，下一次 TIM6 中断会重新置位 */
    Timer_10msFlag = 0U;
    return flag;
}

uint32_t Timer_Get10msTick(void)
{
    /* 返回 TIM6 产生过的 10ms 节拍数量 */
    return Timer_10msTick;
}

/* 定时器中断函数，可以复制到使用它的地方 */
void Timer_OnTim6Elapsed(void)
{
    /* TIM6 当前配置为 10ms 周期，在中断回调里置位定时标志 */
    Timer_10msFlag = 1U;
    Timer_10msTick++;
}
