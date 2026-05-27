#ifndef __STEPPERMOTOR_H
#define __STEPPERMOTOR_H

#include "main.h"

#define STEPPERMOTOR_BASE 0U
#define STEPPERMOTOR_TOP  1U

#define STEPPERMOTOR_DIR_FORWARD 0U
#define STEPPERMOTOR_DIR_BACK    1U

/* 初始化两个步进电机驱动器的 PUL 和 DIR 控制引脚 */
void StepperMotor_Init(void);

/* 设置指定步进电机方向，direction 为 0 正转、1 反转 */
void StepperMotor_SetDir(uint8_t motor, uint8_t direction);

/* 输出一个步进脉冲，上升沿触发驱动器走一步 */
void StepperMotor_OutputPulse(uint8_t motor);

/* 停止两个步进电机的脉冲输出 */
void StepperMotor_Stop(void);

#endif
