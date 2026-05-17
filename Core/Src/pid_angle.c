#include "pid_angle.h"
#include <math.h>

/* 角度环比例系数，数值越大转向越有力 */
float angle_kp = 1.0f;

/* 角度环微分系数，用来提前刹车，减少过冲 */
float angle_kd = 6.0f;

/* 最小输出补偿，解决电机静摩擦导致的小角度转不动 */
float angle_min_out = 8.0f;

/* 最大输出限幅，限制原地转向速度 */
float angle_max_out = 25.0f;

/* 本次转向的目标航向角 */
static float target_yaw = 0.0f;

/* 上一次角度误差，用于计算微分项 */
static float err_last = 0.0f;

/* 稳定计数，连续多次误差很小才认为转向完成 */
static uint16_t settle_count = 0;

void PID_Angle_Init(void)
{
    /* 清空目标角、历史误差和稳定计数 */
    target_yaw = 0.0f;
    err_last = 0.0f;
    settle_count = 0;
}

void PID_Angle_StartTurn(float new_target_yaw)
{
    /* 保存新的目标航向角 */
    target_yaw = new_target_yaw;

    /* 每次开始新转向，都要清空历史误差和稳定计数 */
    err_last = 0.0f;
    settle_count = 0;
}

float PID_Angle_GetTurn(float current_yaw)
{
    /* 角度误差等于目标航向角减当前航向角 */
    float err = target_yaw - current_yaw;

    /* 把误差限制在 -180° 到 +180°，保证走最短转向方向 */
    while (err > 180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;

    /* 误差进入 2° 以内时开始计数，连续稳定后才算转到位 */
    if (fabs(err) <= 2.0f) {
        settle_count++;
    } else {
        settle_count = 0;
    }

    /* 微分项等于本次误差减上次误差 */
    float d_err = err - err_last;
    err_last = err;

    /* 角度环使用 PD 控制，不使用积分，避免转向时积分拖尾 */
    float turn_out = (angle_kp * err) + (angle_kd * d_err);

    /* 误差还比较大时，给一个最小输出，防止电机因为静摩擦转不动 */
    if (fabs(err) > 2.0f) {
        if (turn_out > 0 && turn_out < angle_min_out) turn_out = angle_min_out;
        if (turn_out < 0 && turn_out > -angle_min_out) turn_out = -angle_min_out;
    }

    /* 限制最大转向输出，避免原地转向过猛 */
    if (turn_out > angle_max_out) turn_out = angle_max_out;
    if (turn_out < -angle_max_out) turn_out = -angle_max_out;

    /* 返回差速转向量，状态机会转换成左右轮速度 */
    return turn_out;
}

int PID_Angle_IsSettled(void)
{
    /* 连续 10 个控制周期都在误差范围内，才认为角度稳定 */
    return (settle_count >= 10);
}
