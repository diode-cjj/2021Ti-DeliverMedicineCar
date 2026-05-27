#include "StepperMotor.h"

#define STEPPERMOTOR_BASE_PUL_PORT GPIOE
#define STEPPERMOTOR_BASE_PUL_PIN  GPIO_PIN_6
#define STEPPERMOTOR_BASE_DIR_PORT GPIOE
#define STEPPERMOTOR_BASE_DIR_PIN  GPIO_PIN_7

#define STEPPERMOTOR_TOP_PUL_PORT  GPIOE
#define STEPPERMOTOR_TOP_PUL_PIN   GPIO_PIN_4
#define STEPPERMOTOR_TOP_DIR_PORT  GPIOE
#define STEPPERMOTOR_TOP_DIR_PIN   GPIO_PIN_5

#define STEPPERMOTOR_PULSE_HIGH_MS 1U

static void StepperMotor_WritePul(uint8_t motor, GPIO_PinState pin_state)
{
    if (motor == STEPPERMOTOR_BASE) {
        HAL_GPIO_WritePin(STEPPERMOTOR_BASE_PUL_PORT, STEPPERMOTOR_BASE_PUL_PIN, pin_state);
    } else if (motor == STEPPERMOTOR_TOP) {
        HAL_GPIO_WritePin(STEPPERMOTOR_TOP_PUL_PORT, STEPPERMOTOR_TOP_PUL_PIN, pin_state);
    }
}

void StepperMotor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 底座电机使用 PE6/PE7，上方横放电机使用 PE4/PE5，均为普通脉冲方向信号 */
    __HAL_RCC_GPIOE_CLK_ENABLE();

    HAL_GPIO_WritePin(STEPPERMOTOR_BASE_PUL_PORT, STEPPERMOTOR_BASE_PUL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STEPPERMOTOR_BASE_DIR_PORT, STEPPERMOTOR_BASE_DIR_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STEPPERMOTOR_TOP_PUL_PORT, STEPPERMOTOR_TOP_PUL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STEPPERMOTOR_TOP_DIR_PORT, STEPPERMOTOR_TOP_DIR_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = STEPPERMOTOR_BASE_PUL_PIN | STEPPERMOTOR_BASE_DIR_PIN
                          | STEPPERMOTOR_TOP_PUL_PIN | STEPPERMOTOR_TOP_DIR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void StepperMotor_SetDir(uint8_t motor, uint8_t direction)
{
    GPIO_PinState pin_state;

    pin_state = (direction == STEPPERMOTOR_DIR_FORWARD) ? GPIO_PIN_SET : GPIO_PIN_RESET;

    if (motor == STEPPERMOTOR_BASE) {
        HAL_GPIO_WritePin(STEPPERMOTOR_BASE_DIR_PORT, STEPPERMOTOR_BASE_DIR_PIN, pin_state);
    } else if (motor == STEPPERMOTOR_TOP) {
        HAL_GPIO_WritePin(STEPPERMOTOR_TOP_DIR_PORT, STEPPERMOTOR_TOP_DIR_PIN, pin_state);
    }
}

void StepperMotor_OutputPulse(uint8_t motor)
{
    StepperMotor_WritePul(motor, GPIO_PIN_SET);
    HAL_Delay(STEPPERMOTOR_PULSE_HIGH_MS);
    StepperMotor_WritePul(motor, GPIO_PIN_RESET);
}

void StepperMotor_Stop(void)
{
    StepperMotor_WritePul(STEPPERMOTOR_BASE, GPIO_PIN_RESET);
    StepperMotor_WritePul(STEPPERMOTOR_TOP, GPIO_PIN_RESET);
}
