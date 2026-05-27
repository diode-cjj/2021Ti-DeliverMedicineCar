#include "CarControl.h"
#include "AppProcess.h"
#include "BallTrackProcess.h"
#include "CarConfig.h"
#include "CarState.h"
#include "Bluetooth.h"
#include "Encoder.h"
#include "Motor.h"
#include "MPU6050.h"
#include "OLED.h"
#include "PID.h"
#include "StepperMotor.h"

PID_t SpeedPID_L;
PID_t SpeedPID_R;
PID_t AnglePID;

float TargetSpeed_L;
float TargetSpeed_R;
float CurrentYaw;
float CurrentGz;
float TurnStartYaw;
float TargetYaw;
uint8_t TestMode;
uint8_t SelectMode;
uint8_t ComboStep;
uint8_t BrakeFinishMode;
uint16_t TestCount;
uint16_t BuzzerCount;
uint16_t TrackSetTime;
uint16_t TrackStraight1Time;
uint16_t TrackStraight2Time;
uint16_t TrackTurnMaxTime;
float TrackTargetSpeed;
float TrackTurnCenterSpeed;
uint8_t BallStep;
int16_t BallBaseStep;
int16_t BallTopStep;
int8_t BallSearchDir;
int8_t BallTiltSearchDir;
uint8_t BallTiltCount;
uint8_t BallSearchEdgeCount;

/* 功能：初始化速度环、角度环PID和车辆运行状态；参数：无；返回：无。 */
void Car_PID_Init(void)
{
  PID_Init(&SpeedPID_L);
  SpeedPID_L.Kp = SPEED_KP;
  SpeedPID_L.Ki = SPEED_KI;
  SpeedPID_L.Kd = SPEED_KD;
  SpeedPID_L.OutMax = SPEED_MAX_OUT;
  SpeedPID_L.OutMin = -SPEED_MAX_OUT;
  SpeedPID_L.ErrorIntMax = SPEED_MAX_INT;
  SpeedPID_L.ErrorIntMin = -SPEED_MAX_INT;
  SpeedPID_L.OutOffset = 0.0f;

  PID_Init(&SpeedPID_R);
  SpeedPID_R.Kp = SPEED_KP;
  SpeedPID_R.Ki = SPEED_KI;
  SpeedPID_R.Kd = SPEED_KD;
  SpeedPID_R.OutMax = SPEED_MAX_OUT;
  SpeedPID_R.OutMin = -SPEED_MAX_OUT;
  SpeedPID_R.ErrorIntMax = SPEED_MAX_INT;
  SpeedPID_R.ErrorIntMin = -SPEED_MAX_INT;
  SpeedPID_R.OutOffset = 0.0f;

  PID_Init(&AnglePID);
  AnglePID.Kp = ANGLE_KP;
  AnglePID.Ki = ANGLE_KI;
  AnglePID.Kd = ANGLE_KD;
  AnglePID.OutMax = ANGLE_MAX_OUT;
  AnglePID.OutMin = -ANGLE_MAX_OUT;
  AnglePID.ErrorIntMax = 0.0f;
  AnglePID.ErrorIntMin = 0.0f;
  AnglePID.OutOffset = 0.0f;

  TargetSpeed_L = 0.0f;
  TargetSpeed_R = 0.0f;
  CurrentYaw = 0.0f;
  CurrentGz = 0.0f;
  TurnStartYaw = 0.0f;
  TargetYaw = 0.0f;
  TestMode = TEST_IDLE;
  SelectMode = MODE_COMBO;
  ComboStep = COMBO_STEP_IDLE;
  BrakeFinishMode = 0U;
  TestCount = 0U;
  BuzzerCount = 0U;
  TrackSetTime = TRACK_TIME_MIN;
  TrackStraight1Time = 0U;
  TrackStraight2Time = 0U;
  TrackTurnMaxTime = 0U;
  TrackTargetSpeed = 0.0f;
  TrackTurnCenterSpeed = 0.0f;
  BallStep = BALL_STEP_SEARCH;
  BallTrack_Reset();
}

/* 功能：把角度归一化到-180~180度；参数angle为待限制角度；返回归一化后的角度。 */
float Angle_Limit(float angle)
{
  /* 把角度限制在 -180 到 180 度之间，避免跨 0 度时误差突变 */
  while (angle > 180.0f) {
    angle -= 360.0f;
  }
  while (angle < -180.0f) {
    angle += 360.0f;
  }
  return angle;
}

/* 功能：计算跨±180度时连续的角度误差；参数target为目标角，actual为当前角；返回target-actual的归一化误差。 */
float Angle_GetError(float target, float actual)
{
  return Angle_Limit(target - actual);
}

/* 功能：清空控制状态并停止电机/云台；参数：无；返回：无。 */
void Car_ClearControl(void)
{
  /* 停车或重新启动前清掉 PID 历史量，避免上一次积分影响本次起步 */
  PID_Init(&SpeedPID_L);
  PID_Init(&SpeedPID_R);
  PID_Init(&AnglePID);
  TargetSpeed_L = 0.0f;
  TargetSpeed_R = 0.0f;
  TestMode = TEST_IDLE;
  ComboStep = COMBO_STEP_IDLE;
  BrakeFinishMode = 0U;
  TestCount = 0U;
  TrackTurnMaxTime = 0U;
  TrackTargetSpeed = 0.0f;
  TrackTurnCenterSpeed = 0.0f;
  Encoder_ClearLocation();
  StepperMotor_Stop();
  Motor_Stop();
}

/* 功能：设置左右轮目标速度；参数left_target为左轮目标，right_target为右轮目标；返回：无。 */
void Car_SetSpeedTarget(float left_target, float right_target)
{
  TargetSpeed_L = left_target * SPEED_LEFT_RATIO;
  TargetSpeed_R = right_target;

  /* 目标为 0 时直接清空速度环，保证停车干净 */
  if (left_target == 0.0f && right_target == 0.0f) {
    PID_Init(&SpeedPID_L);
    PID_Init(&SpeedPID_R);
  }
}

/* 功能：用速度PID闭环驱动左右轮；参数left_speed/right_speed为编码器实际速度；返回：无。 */
void Car_SpeedControl(int16_t left_speed, int16_t right_speed)
{
  if (TargetSpeed_L == 0.0f && TargetSpeed_R == 0.0f) {
    Motor_Stop();
    return;
  }

  SpeedPID_L.Target = TargetSpeed_L;
  SpeedPID_L.Actual = (float)left_speed;
  PID_Update(&SpeedPID_L);

  SpeedPID_R.Target = TargetSpeed_R;
  SpeedPID_R.Actual = (float)right_speed;
  PID_Update(&SpeedPID_R);

  Motor_Drive((int)SpeedPID_L.Out, (int)SpeedPID_R.Out);
}

/* 功能：进入刹车状态并记录刹车结束后所属模式；参数finish_mode为完成提示用的模式编号；返回：无。 */
void Car_StartBrake(uint8_t finish_mode)
{
  PID_Init(&SpeedPID_L);
  PID_Init(&SpeedPID_R);
  TargetSpeed_L = 0.0f;
  TargetSpeed_R = 0.0f;
  TestMode = TEST_BRAKE;
  ComboStep = COMBO_STEP_IDLE;
  BrakeFinishMode = finish_mode;
  TestCount = 0U;

  if (finish_mode == MODE_COMBO || finish_mode == MODE_TIMED_TRACK || finish_mode == MODE_BALL_TRACK) {
    Buzzer_Beep();
  }
}

/* 功能：按当前速度反向制动直到车辆停止；参数left_speed/right_speed为编码器实际速度；返回：无。 */
void Car_BrakeControl(int16_t left_speed, int16_t right_speed)
{
  SpeedPID_L.Target = 0.0f;
  SpeedPID_L.Actual = (float)left_speed;
  PID_Update(&SpeedPID_L);
  if (left_speed > BRAKE_STOP_SPEED && SpeedPID_L.Out > -BRAKE_MIN_OUT) {
    SpeedPID_L.Out = -BRAKE_MIN_OUT;
  } else if (left_speed < -BRAKE_STOP_SPEED && SpeedPID_L.Out < BRAKE_MIN_OUT) {
    SpeedPID_L.Out = BRAKE_MIN_OUT;
  } else if (left_speed <= BRAKE_STOP_SPEED && left_speed >= -BRAKE_STOP_SPEED) {
    SpeedPID_L.Out = 0.0f;
  }

  SpeedPID_R.Target = 0.0f;
  SpeedPID_R.Actual = (float)right_speed;
  PID_Update(&SpeedPID_R);
  if (right_speed > BRAKE_STOP_SPEED && SpeedPID_R.Out > -BRAKE_MIN_OUT) {
    SpeedPID_R.Out = -BRAKE_MIN_OUT;
  } else if (right_speed < -BRAKE_STOP_SPEED && SpeedPID_R.Out < BRAKE_MIN_OUT) {
    SpeedPID_R.Out = BRAKE_MIN_OUT;
  } else if (right_speed <= BRAKE_STOP_SPEED && right_speed >= -BRAKE_STOP_SPEED) {
    SpeedPID_R.Out = 0.0f;
  }

  Motor_Drive((int)SpeedPID_L.Out, (int)SpeedPID_R.Out);
}

/* 功能：刹车完成后清状态、回菜单并输出停止信息；参数：无；返回：无。 */
void Car_FinishBrake(void)
{
  uint8_t finish_mode;

  finish_mode = BrakeFinishMode;
  Car_ClearControl();
  OLED_ShowMenu();

  if (finish_mode == MODE_BALL_TRACK) {
    Bluetooth_Printf("Ball Track Stop\r\n");
  } else if (finish_mode == MODE_TIMED_TRACK) {
    Bluetooth_Printf("Track Test Stop\r\n");
  } else if (finish_mode == MODE_COMBO) {
    Bluetooth_Printf("Combo Test Stop\r\n");
  }
}

/* 功能：读取MPU6050陀螺仪并积分更新当前航向角；参数dt为本次更新间隔秒数；返回：无。 */
void Car_UpdateYaw(float dt)
{
  int16_t ax, ay, az, gx, gy, gz;
  float real_gz;

  MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);
  real_gz = (float)gz - gz_offset;

  /* 静止时很小的抖动不参与积分，减小航向角漂移 */
  if (real_gz < 2.0f && real_gz > -2.0f) {
    real_gz = 0.0f;
  }

  CurrentGz = real_gz / 16.4f;
  CurrentYaw += CurrentGz * dt;
  CurrentYaw = Angle_Limit(CurrentYaw);

  /* 待机且陀螺仪接近静止时，缓慢修正Z轴零漂 */
  if (TestMode == TEST_IDLE) {
    if ((float)gz > gz_offset - 15.0f && (float)gz < gz_offset + 15.0f) {
      gz_offset = gz_offset * 0.995f + (float)gz * 0.005f;
    }
  }
}
