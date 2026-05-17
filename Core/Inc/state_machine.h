#ifndef __STATE_MACHINE_H
#define __STATE_MACHINE_H

#include "main.h"

void State_Machine_Init(void);
void State_Machine_ScanKeys10ms(void);
void State_Machine_Run(void);
void State_Emergency_Stop(void);
void Exec_Debug_Turn(float angle_offset);

uint8_t State_GetMode(void);
uint8_t State_GetTargetDigit(void);
uint8_t State_GetIntersectionCount(void);
uint8_t State_GetMedicineState(void);
uint8_t State_GetStartConfirmed(void);
uint8_t State_GetAppState(void);
uint8_t State_IsIdle(void);
char State_GetRouteCode(void);

#endif
