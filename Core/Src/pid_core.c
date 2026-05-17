#include "pid_core.h"

void PID_Init_Core(PID_TypeDef *pid, float p, float i, float d, float max_out, float max_int)
{
    /* 保存比例、积分、微分参数 */
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;

    /* 保存输出限幅和积分限幅 */
    pid->out_max = max_out;
    pid->int_max = max_int;

    /* 初始化完成后清空历史误差，保证 PID 从干净状态开始 */
    PID_Clear(pid);
}

void PID_Clear(PID_TypeDef *pid)
{
    /* 清空目标值和实际值 */
    pid->target_val = 0;
    pid->actual_val = 0;

    /* 清空本次误差、上次误差和积分累计 */
    pid->err = 0;
    pid->err_last = 0;
    pid->integral = 0;
}

float PID_Calc(PID_TypeDef *pid, float target, float actual)
{
    /* 保存目标值和实际值，方便调试时观察 PID 状态 */
    pid->target_val = target;
    pid->actual_val = actual;

    /* 误差等于目标值减实际值 */
    pid->err = pid->target_val - pid->actual_val;

    /* 积分项累加误差，用来消除长期偏差 */
    pid->integral += pid->err;

    /* 对积分项限幅，防止长时间偏差导致积分过大 */
    if (pid->integral > pid->int_max) pid->integral = pid->int_max;
    if (pid->integral < -pid->int_max) pid->integral = -pid->int_max;

    /* 位置式 PID：比例项 + 积分项 + 微分项 */
    float out = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * (pid->err - pid->err_last);

    /* 保存本次误差，下一次计算微分项时使用 */
    pid->err_last = pid->err;

    /* 对最终输出限幅，保护电机和控制量 */
    if (out > pid->out_max) out = pid->out_max;
    if (out < -pid->out_max) out = -pid->out_max;

    /* 返回限幅后的 PID 输出 */
    return out;
}
