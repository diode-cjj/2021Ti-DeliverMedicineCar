#ifndef __CARCONTROL_H
#define __CARCONTROL_H

#include "main.h"

void Car_PID_Init(void);
void Car_ClearControl(void);
void Car_SetSpeedTarget(float left_target, float right_target);
void Car_SpeedControl(int16_t left_speed, int16_t right_speed);
void Car_StartBrake(uint8_t finish_mode);
void Car_BrakeControl(int16_t left_speed, int16_t right_speed);
void Car_FinishBrake(void);
void Car_UpdateYaw(float dt);
float Angle_Limit(float angle);
float Angle_GetError(float target, float actual);

#endif
