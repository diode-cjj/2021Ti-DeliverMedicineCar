#include "BallTrackProcess.h"
#include "CarConfig.h"
#include "CarControl.h"
#include "CarState.h"
#include "Bluetooth.h"
#include "K230Vision.h"
#include "PID.h"
#include "StepperMotor.h"

static float BallTrack_Limit(float value, float limit);
static void BallTrack_MoveBase(int8_t direction, uint8_t pulse_count);
static void BallTrack_MoveTop(int8_t direction, uint8_t pulse_count);
static uint8_t BallTrack_Search(void);
static uint8_t BallTrack_UpdateGimbal(K230Vision_Target_t *target);
static uint8_t BallTrack_CenterBase(void);
static void BallTrack_StartForward(void);
static void BallTrack_Stop(void);

/* 功能：限制追球转向修正量幅值；参数value为输入值，limit为正负限幅；返回限幅后的值。 */
static float BallTrack_Limit(float value, float limit)
{
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

/* 功能：复位追球流程和云台步数记录；参数：无；返回：无。 */
void BallTrack_Reset(void)
{
  BallStep = BALL_STEP_SEARCH;
  BallBaseStep = 0;
  BallTopStep = 0;
  BallSearchDir = BALL_SEARCH_RIGHT;
  BallTiltSearchDir = BALL_SEARCH_UP;
  BallTiltCount = 0U;
  BallSearchEdgeCount = 0U;
}

/* 功能：按限位移动底座水平云台；参数direction为方向，pulse_count为移动脉冲数；返回：无。 */
static void BallTrack_MoveBase(int8_t direction, uint8_t pulse_count)
{
  uint8_t i;

  if (direction > 0) {
    StepperMotor_SetDir(STEPPERMOTOR_BASE, BALL_BASE_RIGHT_DIR);
  } else {
    StepperMotor_SetDir(STEPPERMOTOR_BASE, BALL_BASE_LEFT_DIR);
  }

  for (i = 0U; i < pulse_count; i++) {
    if (direction > 0) {
      if (BallBaseStep >= (int16_t)BALL_BASE_HALF_RANGE_PULSE) {
        break;
      }
      StepperMotor_OutputPulse(STEPPERMOTOR_BASE);
      BallBaseStep++;
    } else {
      if (BallBaseStep <= -(int16_t)BALL_BASE_HALF_RANGE_PULSE) {
        break;
      }
      StepperMotor_OutputPulse(STEPPERMOTOR_BASE);
      BallBaseStep--;
    }
  }
}

/* 功能：按限位移动俯仰云台；参数direction为方向，pulse_count为移动脉冲数；返回：无。 */
static void BallTrack_MoveTop(int8_t direction, uint8_t pulse_count)
{
  uint8_t i;

  if (direction > 0) {
    StepperMotor_SetDir(STEPPERMOTOR_TOP, BALL_TOP_DOWN_DIR);
  } else {
    StepperMotor_SetDir(STEPPERMOTOR_TOP, BALL_TOP_UP_DIR);
  }

  for (i = 0U; i < pulse_count; i++) {
    if (direction > 0) {
      if (BallTopStep >= (int16_t)BALL_TOP_DOWN_RANGE_PULSE) {
        break;
      }
      StepperMotor_OutputPulse(STEPPERMOTOR_TOP);
      BallTopStep++;
    } else {
      if (BallTopStep <= -(int16_t)BALL_TOP_UP_RANGE_PULSE) {
        break;
      }
      StepperMotor_OutputPulse(STEPPERMOTOR_TOP);
      BallTopStep--;
    }
  }
}

/* 功能：让云台按水平和俯仰范围扫描搜索目标；参数：无；返回1表示水平扫到边界。 */
static uint8_t BallTrack_Search(void)
{
  uint8_t edge_flag;

  edge_flag = 0U;

  /* 水平云台扫到右侧限位后，改为向左搜索，并告诉上层已经扫到边界。 */
  if (BallBaseStep >= (int16_t)BALL_BASE_HALF_RANGE_PULSE) {
    BallSearchDir = BALL_SEARCH_LEFT;
    edge_flag = 1U;
  } else if (BallBaseStep <= -(int16_t)BALL_BASE_HALF_RANGE_PULSE) {
    /* 水平云台扫到左侧限位后，改为向右搜索。 */
    BallSearchDir = BALL_SEARCH_RIGHT;
    edge_flag = 1U;
  }

  /* 每次调用都让水平云台按当前方向移动一点，形成左右扫描。 */
  BallTrack_MoveBase(BallSearchDir, BALL_SEARCH_PULSE);

  /* 俯仰云台不用每次都动，通过计数降低上下扫描速度。 */
  if (++BallTiltCount >= BALL_TOP_SEARCH_PERIOD) {
    BallTiltCount = 0U;
    if (BallTopStep >= (int16_t)BALL_TOP_DOWN_RANGE_PULSE) {
      BallTiltSearchDir = BALL_SEARCH_UP;
    } else if (BallTopStep <= -(int16_t)BALL_TOP_UP_RANGE_PULSE) {
      BallTiltSearchDir = BALL_SEARCH_DOWN;
    }
    BallTrack_MoveTop(BallTiltSearchDir, BALL_SEARCH_PULSE);
  }

  /* 返回是否扫到水平边界，外层可据此判断是否需要让车体旋转继续找球。 */
  return edge_flag;
}

/* 功能：根据K230目标中心调整云台对准目标；参数target为视觉目标信息指针；返回1表示水平误差已锁定。 */
static uint8_t BallTrack_UpdateGimbal(K230Vision_Target_t *target)
{
  int16_t x_error;
  int16_t y_error;
  int16_t abs_x_error;

  x_error = target->cx - BALL_IMAGE_CENTER_X;
  y_error = target->cy - BALL_IMAGE_CENTER_Y;
  /* abs_x_error 表示目标中心与图像中心的水平误差绝对值，用来判断是否已对准。 */
  abs_x_error = x_error;
  if (abs_x_error < 0) {
    abs_x_error = -abs_x_error;
  }

  if (x_error > BALL_X_DEADZONE) {
    BallTrack_MoveBase(1, BALL_TRACK_BASE_PULSE);
  } else if (x_error < -BALL_X_DEADZONE) {
    BallTrack_MoveBase(-1, BALL_TRACK_BASE_PULSE);
  }

  if (y_error > BALL_Y_DEADZONE) {
    BallTrack_MoveTop(1, BALL_TRACK_TOP_PULSE);
  } else if (y_error < -BALL_Y_DEADZONE) {
    BallTrack_MoveTop(-1, BALL_TRACK_TOP_PULSE);
  }

  return (abs_x_error <= BALL_LOCK_X_ERROR) ? 1U : 0U;
}

/* 功能：把水平云台逐步回中；参数：无；返回1表示已回到中心附近。 */
static uint8_t BallTrack_CenterBase(void)
{
  if (BallBaseStep > 0) {
    BallTrack_MoveBase(-1, BALL_SEARCH_PULSE);
  } else if (BallBaseStep < 0) {
    BallTrack_MoveBase(1, BALL_SEARCH_PULSE);
  }

  if (BallBaseStep >= -(int16_t)BALL_CENTER_BASE_STEP &&
      BallBaseStep <= (int16_t)BALL_CENTER_BASE_STEP) {
    BallBaseStep = 0;
    return 1U;
  }

  return 0U;
}

/* 功能：追球锁定后切换到前进阶段并记录当前航向；参数：无；返回：无。 */
static void BallTrack_StartForward(void)
{
  TestCount = 0U;
  BallStep = BALL_STEP_FORWARD;
  TargetYaw = CurrentYaw;
  PID_Init(&SpeedPID_L);
  PID_Init(&SpeedPID_R);
  Bluetooth_Printf("Ball Forward Start\r\n");
}

/* 功能：追球到达停车距离后启动刹车；参数：无；返回：无。 */
static void BallTrack_Stop(void)
{
  Car_StartBrake(MODE_BALL_TRACK);
  Bluetooth_Printf("Ball Found Stop\r\n");
}

/* 功能：追球状态机控制云台搜索、车体旋转、前进和停车；参数left_speed/right_speed为编码器实际速度；返回：无。 */
void BallTrack_Control(int16_t left_speed, int16_t right_speed)
{
  K230Vision_Target_t target;
  uint8_t target_valid;
  uint8_t search_edge;
  float forward_speed;
  float turn_speed;
  float angle_error;
  float angle_abs_error;

  target_valid = 0U;
  if (K230Vision_IsOnline(BALL_LOST_TIMEOUT_MS) != 0U &&
      K230Vision_GetTarget(&target) != 0U &&
      HAL_GetTick() - target.tick <= BALL_LOST_TIMEOUT_MS) {
    target_valid = 1U;
  }

  if (target_valid != 0U &&
      target.distance_cm <= BALL_STOP_DISTANCE_CM &&
      target.width >= BALL_STOP_WIDTH_MIN) {
    BallTrack_Stop();
    Car_BrakeControl(left_speed, right_speed);
    return;
  }

  if (BallStep == BALL_STEP_SEARCH || BallStep == BALL_STEP_ROTATE_SEARCH) {
    if (target_valid != 0U) {
      BallTrack_UpdateGimbal(&target);
      BallTrack_StartForward();
      return;
    }

    search_edge = BallTrack_Search();
    if (BallStep == BALL_STEP_SEARCH) {
      Car_SetSpeedTarget(0.0f, 0.0f);
      Car_SpeedControl(left_speed, right_speed);

      if (search_edge != 0U) {
        if (++BallSearchEdgeCount >= BALL_SEARCH_EDGE_COUNT) {
          BallStep = BALL_STEP_ROTATE_SEARCH;
          PID_Init(&SpeedPID_L);
          PID_Init(&SpeedPID_R);
          Bluetooth_Printf("Ball Rotate Search\r\n");
        }
      }
    } else {
      Car_SetSpeedTarget(BALL_ROTATE_SEARCH_SPEED, -BALL_ROTATE_SEARCH_SPEED);
      Car_SpeedControl(left_speed, right_speed);
    }
    return;
  }

  if (BallStep == BALL_STEP_TURN) {
    if (target_valid != 0U && target.distance_cm <= BALL_SLOW_DISTANCE_CM) {
      BallTrack_StartForward();
      return;
    }

    BallTrack_CenterBase();

    angle_error = Angle_GetError(TargetYaw, CurrentYaw);
    angle_abs_error = angle_error;
    if (angle_abs_error < 0.0f) {
      angle_abs_error = -angle_abs_error;
    }

    turn_speed = BALL_BODY_TURN_SPEED;
    if (angle_error > 0.0f) {
      Car_SetSpeedTarget(-turn_speed, turn_speed);
    } else {
      Car_SetSpeedTarget(turn_speed, -turn_speed);
    }
    Car_SpeedControl(left_speed, right_speed);

    if (angle_abs_error <= BALL_TURN_ALLOW_ERROR || ++TestCount >= BALL_TURN_MAX_TIME) {
      BallTrack_CenterBase();
      BallTrack_StartForward();
    }
    return;
  }

  if (BallStep == BALL_STEP_FORWARD) {
    if (target_valid == 0U) {
      BallStep = BALL_STEP_SEARCH;
      BallSearchEdgeCount = 0U;
      Car_SetSpeedTarget(0.0f, 0.0f);
      Car_SpeedControl(left_speed, right_speed);
      return;
    }

    BallTrack_UpdateGimbal(&target);

    forward_speed = (target.distance_cm > BALL_SLOW_DISTANCE_CM) ? BALL_FORWARD_SPEED : BALL_SLOW_SPEED;
    turn_speed = BallTrack_Limit((float)(target.cx - BALL_IMAGE_CENTER_X) * BALL_FORWARD_TURN_KP, BALL_ALIGN_SPEED);
    Car_SetSpeedTarget(forward_speed + turn_speed, forward_speed - turn_speed);
    Car_SpeedControl(left_speed, right_speed);
    return;
  }

  BallStep = BALL_STEP_SEARCH;
}

/* 功能：清状态并启动追球模式；参数：无；返回：无。 */
void Car_StartBallTrack(void)
{
  Car_ClearControl();
  BallTrack_Reset();
  K230Vision_Clear();
  TestMode = TEST_BALL_TRACK;
  Bluetooth_Printf("Ball Track Start\r\n");
}
