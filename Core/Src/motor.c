#include "motor.h"
#include "main.h"

/* TIM1 用来输出左右电机 PWM */
extern TIM_HandleTypeDef htim1;

void Motor_Init(void)
{
    /* 启动左轮 PWM 输出通道 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    /* 启动右轮 PWM 输出通道 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void Motor_SetLeft(int speed)
{
    /* 对输入速度限幅，防止 PWM 超出定时器范围 */
    if (speed > MOTOR_MAX_PULSE) speed = MOTOR_MAX_PULSE;
    if (speed < -MOTOR_MAX_PULSE) speed = -MOTOR_MAX_PULSE;

    if (speed >= 0) {
        /* 左轮正转：AIN1=0，AIN2=1 */
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);

        /* PWM 占空比越大，左轮速度越快 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
    } else {
        /* 左轮反转：AIN1=1，AIN2=0 */
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);

        /* 负速度只表示方向，PWM 仍然使用正数占空比 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -speed);
    }
}

void Motor_SetRight(int speed)
{
    /* 对输入速度限幅，防止 PWM 超出定时器范围 */
    if (speed > MOTOR_MAX_PULSE) speed = MOTOR_MAX_PULSE;
    if (speed < -MOTOR_MAX_PULSE) speed = -MOTOR_MAX_PULSE;

    if (speed >= 0) {
        /* 右轮正转：BIN1=0，BIN2=1 */
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);

        /* PWM 占空比越大，右轮速度越快 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed);
    } else {
        /* 右轮反转：BIN1=1，BIN2=0 */
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);

        /* 负速度只表示方向，PWM 仍然使用正数占空比 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -speed);
    }
}

void Motor_Drive(int left_speed, int right_speed)
{
    /* 差速控制入口：分别设置左右轮速度 */
    Motor_SetLeft(left_speed);
    Motor_SetRight(right_speed);
}

void Motor_Stop(void)
{
    /* 四个方向控制脚全部拉低，关闭电机驱动方向 */
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);

    /* PWM 占空比清零，确保左右轮都停止输出 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
}
