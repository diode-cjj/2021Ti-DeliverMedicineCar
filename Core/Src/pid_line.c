#include "pid_line.h"

/* 巡线 PID 结构体，专门把灰度误差转换成转向修正量 */
PID_TypeDef pid_line;

void PID_Line_Init(void)
{
    /* 巡线目标是误差为 0，也就是红线位于小车正中间 */
    PID_Init_Core(&pid_line, LINE_KP, LINE_KI, LINE_KD, LINE_MAX_OUT, 0);
}

float PID_Line_GetTurn(float line_error)
{
    /* 目标误差永远是 0，实际值是当前灰度误差 */
    return PID_Calc(&pid_line, 0.0f, line_error);
}
