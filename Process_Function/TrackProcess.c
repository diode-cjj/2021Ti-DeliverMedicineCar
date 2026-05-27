#include "TrackProcess.h"
#include "CarConfig.h"
#include "CarControl.h"
#include "CarState.h"
#include "Bluetooth.h"
#include "Gray_Sensor.h"
#include "PID.h"

static uint8_t Gray_IsStopMark(void);

/* 功能：根据设定时间计算定时循迹各段时间和速度；参数：无；返回：无。 */
void Track_CalcParam(void)
{
  uint16_t track_stop_advance_time;

  /* 模式2按设定时间整体缩放模式1，轨迹结构仍为直行1、右转、直行2 */
  TrackStraight1Time = (uint16_t)((uint32_t)(COMBO_STRAIGHT1_TIME - TRACK_STRAIGHT1_REDUCE_TIME) * TrackSetTime / TRACK_BASE_TIME);
  if (TrackStraight1Time < 1U) {
    TrackStraight1Time = 1U;
  }

  TrackStraight2Time = (uint16_t)((uint32_t)(COMBO_STRAIGHT2_TIME + TRACK_STRAIGHT1_REDUCE_TIME) * TrackSetTime / TRACK_BASE_TIME);
  if (TrackStraight2Time < 1U) {
    TrackStraight2Time = 1U;
  }
  track_stop_advance_time = (uint16_t)((uint32_t)TRACK_STOP_ADVANCE_TIME * TrackSetTime / TRACK_TIME_MAX);
  if (track_stop_advance_time < 1U) {
    track_stop_advance_time = 1U;
  }
  if (TrackStraight2Time > track_stop_advance_time) {
    TrackStraight2Time -= track_stop_advance_time;
  } else {
    TrackStraight2Time = 1U;
  }

  /* 速度和时间成反比，保证时间变长时，每一段走过的距离基本不变 */
  TrackTurnMaxTime = (uint16_t)((uint32_t)COMBO_TURN_MAX_TIME * TrackSetTime / TRACK_BASE_TIME);
  if (TrackTurnMaxTime < COMBO_TURN_MAX_TIME) {
    TrackTurnMaxTime = COMBO_TURN_MAX_TIME;
  }

  TrackTargetSpeed = STRAIGHT_SPEED * (float)TRACK_BASE_TIME / (float)TrackSetTime;
  TrackTurnCenterSpeed = COMBO_TURN_CENTER_SPEED * (float)TRACK_BASE_TIME / (float)TrackSetTime;
}

/* 功能：检测灰度传感器是否识别到停车黑线标志；参数：无；返回1表示检测到，0表示未检测到。 */
static uint8_t Gray_IsStopMark(void)
{
  uint8_t i;

  Gray_Sensor_Read();

  /* 任意相连三路同时识别到黑线时，认为到达停车标志位 */
  for (i = 0U; i + 2U < GRAY_SENSOR_NUM; i++) {
    if (Gray_Value[i] <= GRAY_BLACK_VALUE &&
        Gray_Value[i + 1U] <= GRAY_BLACK_VALUE &&
        Gray_Value[i + 2U] <= GRAY_BLACK_VALUE) {
      return 1U;
    }
  }

  return 0U;
}

void TrackProcess_RunTimed(int16_t left_speed, int16_t right_speed)
{
  float angle_error;
  float angle_abs_error;
  float combo_left_speed;
  float combo_right_speed;

  if (ComboStep == COMBO_STEP_STRAIGHT1) {
    /* 模式2直行段和模式1一样，只用速度环，不再用角度环修正，避免低速左右摇摆 */
    Car_SetSpeedTarget(TrackTargetSpeed, TrackTargetSpeed);
    Car_SpeedControl(left_speed, right_speed);

    if (++TestCount >= TrackStraight1Time) {
      TestCount = 0U;
      ComboStep = COMBO_STEP_TURN;
      TurnStartYaw = CurrentYaw;
      TargetYaw = Angle_Limit(TurnStartYaw - TURN_TARGET_ANGLE);
      PID_Init(&SpeedPID_L);
      PID_Init(&SpeedPID_R);
      PID_Init(&AnglePID);
      Bluetooth_Printf("Track Turn Start\r\n");
    }
  } else if (ComboStep == COMBO_STEP_TURN) {
    angle_error = Angle_GetError(TargetYaw, CurrentYaw);
    angle_abs_error = angle_error;
    if (angle_abs_error < 0.0f) {
      angle_abs_error = -angle_abs_error;
    }

    /* 模式2转弯段同步降低中心速度，半径和角度仍和模式1一致 */
    combo_left_speed = TrackTurnCenterSpeed * (COMBO_TURN_RADIUS_CM + CAR_TRACK_WIDTH_CM / 2.0f) / COMBO_TURN_RADIUS_CM;
    combo_right_speed = TrackTurnCenterSpeed * (COMBO_TURN_RADIUS_CM - CAR_TRACK_WIDTH_CM / 2.0f) / COMBO_TURN_RADIUS_CM;
    Car_SetSpeedTarget(combo_left_speed, combo_right_speed);
    Car_SpeedControl(left_speed, right_speed);

    if (angle_abs_error <= TURN_ALLOW_ERROR || ++TestCount >= TrackTurnMaxTime) {
      TestCount = 0U;
      ComboStep = COMBO_STEP_STRAIGHT2;
      PID_Init(&SpeedPID_L);
      PID_Init(&SpeedPID_R);
      PID_Init(&AnglePID);
      Bluetooth_Printf("Track Straight2 Start\r\n");
    }
  } else if (ComboStep == COMBO_STEP_STRAIGHT2) {
    /* 模式2第二段直行也按模式1方式控制，只用速度环，减少出弯后的左右摆动 */
    Car_SetSpeedTarget(TrackTargetSpeed, TrackTargetSpeed);
    Car_SpeedControl(left_speed, right_speed);

    TestCount++;
    if (Gray_IsStopMark() || TestCount >= TrackStraight2Time) {
      Car_StartBrake(MODE_TIMED_TRACK);
      Car_BrakeControl(left_speed, right_speed);
    }
  } else {
    Car_ClearControl();
  }
}

void TrackProcess_RunCombo(int16_t left_speed, int16_t right_speed)
{
  float angle_error;
  float angle_abs_error;
  float combo_left_speed;
  float combo_right_speed;

  if (ComboStep == COMBO_STEP_STRAIGHT1) {
    Car_SetSpeedTarget(STRAIGHT_SPEED, STRAIGHT_SPEED);
    Car_SpeedControl(left_speed, right_speed);

    if (++TestCount >= COMBO_STRAIGHT1_TIME) {
      TestCount = 0U;
      ComboStep = COMBO_STEP_TURN;
      TurnStartYaw = CurrentYaw;
      TargetYaw = Angle_Limit(TurnStartYaw - TURN_TARGET_ANGLE);
      PID_Init(&SpeedPID_L);
      PID_Init(&SpeedPID_R);
      Bluetooth_Printf("Combo Turn Start\r\n");
    }
  } else if (ComboStep == COMBO_STEP_TURN) {
    angle_error = Angle_GetError(TargetYaw, CurrentYaw);
    angle_abs_error = angle_error;
    if (angle_abs_error < 0.0f) {
      angle_abs_error = -angle_abs_error;
    }

    /* 半径20cm、轮距20cm时，左轮外圈和右轮内圈速度比为3:1 */
    combo_left_speed = COMBO_TURN_CENTER_SPEED * (COMBO_TURN_RADIUS_CM + CAR_TRACK_WIDTH_CM / 2.0f) / COMBO_TURN_RADIUS_CM;
    combo_right_speed = COMBO_TURN_CENTER_SPEED * (COMBO_TURN_RADIUS_CM - CAR_TRACK_WIDTH_CM / 2.0f) / COMBO_TURN_RADIUS_CM;
    Car_SetSpeedTarget(combo_left_speed, combo_right_speed);
    Car_SpeedControl(left_speed, right_speed);

    if (angle_abs_error <= TURN_ALLOW_ERROR || ++TestCount >= COMBO_TURN_MAX_TIME) {
      TestCount = 0U;
      ComboStep = COMBO_STEP_STRAIGHT2;
      PID_Init(&SpeedPID_L);
      PID_Init(&SpeedPID_R);
      Bluetooth_Printf("Combo Straight2 Start\r\n");
    }
  } else if (ComboStep == COMBO_STEP_STRAIGHT2) {
    Car_SetSpeedTarget(STRAIGHT_SPEED, STRAIGHT_SPEED);
    Car_SpeedControl(left_speed, right_speed);

    TestCount++;
    if (Gray_IsStopMark() || TestCount >= COMBO_STRAIGHT2_TIME) {
      Car_StartBrake(MODE_COMBO);
      Car_BrakeControl(left_speed, right_speed);
    }
  } else {
    Car_ClearControl();
  }
}

/* 功能：清状态并启动定时循迹模式；参数：无；返回：无。 */
void Car_StartTimedTrack(void)
{
  Car_ClearControl();
  Track_CalcParam();
  TestMode = TEST_TIMED_TRACK;
  ComboStep = COMBO_STEP_STRAIGHT1;
  TurnStartYaw = CurrentYaw;
  TargetYaw = CurrentYaw;
  Car_SetSpeedTarget(TrackTargetSpeed, TrackTargetSpeed);
  Bluetooth_Printf("Track Test Start %ds\r\n", TrackSetTime / 100U);
}

/* 功能：清状态并启动组合轨迹模式；参数：无；返回：无。 */
void Car_StartCombo(void)
{
  Car_ClearControl();
  TestMode = TEST_COMBO;
  ComboStep = COMBO_STEP_STRAIGHT1;
  Car_SetSpeedTarget(STRAIGHT_SPEED, STRAIGHT_SPEED);
  Bluetooth_Printf("Combo Test Start\r\n");
}
