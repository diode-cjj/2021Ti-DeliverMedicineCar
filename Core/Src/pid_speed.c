#include "pid_speed.h"
#include "motor.h"
#include "encoder.h"

/* 左右轮速度 PID 结构体，保存参数和历史误差 */
PID_TypeDef pid_speed_L, pid_speed_R;

/* 左右轮目标速度，由状态机或蓝牙遥控设置 */
float target_L = 0, target_R = 0;

/* 增量式速度环的 PWM 记忆值，每次只在原来的基础上加减一点 */
static float current_pwm_L = 0.0f;
static float current_pwm_R = 0.0f;

void PID_Speed_Init(void)
{
    /* 初始化左轮速度环参数 */
    PID_Init_Core(&pid_speed_L, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_MAX_OUT, SPEED_MAX_INT);

    /* 初始化右轮速度环参数 */
    PID_Init_Core(&pid_speed_R, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_MAX_OUT, SPEED_MAX_INT);

    /* 清空左右轮 PWM 记忆值 */
    current_pwm_L = 0.0f;
    current_pwm_R = 0.0f;
}

void PID_Speed_SetTarget(float left_target, float right_target)
{
    /* 如果目标速度为 0，立即清空 PWM 记忆，避免停车后还残留输出 */
    if (left_target == 0.0f && right_target == 0.0f) {
        current_pwm_L = 0.0f;
        current_pwm_R = 0.0f;
    }

    /* 保存新的左右轮目标速度 */
    target_L = left_target;
    target_R = right_target;
}

void PID_Speed_Execute(void)
{
    /* 读取左右轮编码器速度，左轮方向取反是为了统一前进为正 */
    int16_t speed_L = -Encoder_GetLeftSpeed();
    int16_t speed_R = Encoder_GetRightSpeed();

    /* 左右轮累计位置由编码器速度积分得到，给调试和里程判断使用 */
    extern long long current_location_L;
    extern long long current_location_R;
    current_location_L += speed_L;
    current_location_R += speed_R;

    /* 如果状态机要求停车，就直接关闭电机，不再进行速度 PID 计算 */
    if (target_L == 0.0f && target_R == 0.0f) {
        Motor_Stop();
        return;
    }

    /* 计算左右轮当前速度误差 */
    pid_speed_L.err = target_L - speed_L;
    pid_speed_R.err = target_R - speed_R;

    /* 增量式 PI：根据误差变化量和当前误差，计算这次 PWM 需要增加多少 */
    float delta_L = pid_speed_L.Kp * (pid_speed_L.err - pid_speed_L.err_last) + pid_speed_L.Ki * pid_speed_L.err;
    float delta_R = pid_speed_R.Kp * (pid_speed_R.err - pid_speed_R.err_last) + pid_speed_R.Ki * pid_speed_R.err;

    /* 保存本次误差，下一轮用于计算误差变化量 */
    pid_speed_L.err_last = pid_speed_L.err;
    pid_speed_R.err_last = pid_speed_R.err;

    /* 在当前 PWM 基础上叠加增量，速度稳定时增量接近 0，PWM 会保持住 */
    current_pwm_L += delta_L;
    current_pwm_R += delta_R;

    /* PWM 输出限幅，防止速度环给电机过大的控制量 */
    if (current_pwm_L > SPEED_MAX_OUT) current_pwm_L = SPEED_MAX_OUT;
    if (current_pwm_L < -SPEED_MAX_OUT) current_pwm_L = -SPEED_MAX_OUT;
    if (current_pwm_R > SPEED_MAX_OUT) current_pwm_R = SPEED_MAX_OUT;
    if (current_pwm_R < -SPEED_MAX_OUT) current_pwm_R = -SPEED_MAX_OUT;

    /* 把计算出来的 PWM 输出下发到底层电机驱动 */
    Motor_SetLeft((int)current_pwm_L);
    Motor_SetRight((int)current_pwm_R);
}
