#include "AppProcess.h"
#include "BallTrackProcess.h"
#include "CarConfig.h"
#include "CarControl.h"
#include "CarState.h"
#include "TrackProcess.h"
#include "Bluetooth.h"
#include "Encoder.h"
#include "Key.h"
#include "K230Vision.h"
#include "Motor.h"
#include "OLED.h"
#include "Timer.h"

static void Buzzer_Init(void);
static void Buzzer_Update(void);
static void OLED_ShowRun(char *mode_name);
static void Mode_SelectNext(void);
static void Mode_StartSelected(void);
static void Mode_TimeNext(void);

void AppProcess_Init(void)
{
  Buzzer_Init();
  Car_PID_Init();
  OLED_ShowMenu();
  Bluetooth_Printf("Speed Angle Track Ready\r\n");
}

/* 功能：初始化蜂鸣器GPIO并关闭蜂鸣器；参数：无；返回：无。 */
static void Buzzer_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* 蜂鸣器高电平触发，初始化时先输出低电平，防止上电误响 */
  HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_PIN, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = BUZZER_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_PORT, &GPIO_InitStruct);

  BuzzerCount = 0U;
}

/* 功能：启动一次短鸣提示；参数：无；返回：无。 */
void Buzzer_Beep(void)
{
  BuzzerCount = BUZZER_TIME;
  HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_PIN, GPIO_PIN_SET);
}

/* 功能：在10ms任务中递减蜂鸣器计时并到时关闭；参数：无；返回：无。 */
static void Buzzer_Update(void)
{
  if (BuzzerCount > 0U) {
    BuzzerCount--;
    if (BuzzerCount == 0U) {
      HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    }
  }
}

/* 功能：刷新OLED模式选择菜单；参数：无；返回：无。 */
void OLED_ShowMenu(void)
{
  OLED_Clear();

  OLED_ShowString(0, 0, "Mode Select", OLED_8X16);

  if (SelectMode == MODE_COMBO) {
    OLED_ShowString(0, 18, ">M1 Combo", OLED_6X8);
  } else {
    OLED_ShowString(0, 18, " M1 Combo", OLED_6X8);
  }

  if (SelectMode == MODE_TIMED_TRACK) {
    OLED_Printf(0, 28, OLED_6X8, ">M2 Track %ds", TrackSetTime / 100U);
  } else {
    OLED_Printf(0, 28, OLED_6X8, " M2 Track %ds", TrackSetTime / 100U);
  }

  if (SelectMode == MODE_BALL_TRACK) {
    OLED_ShowString(0, 38, ">M3 Ball", OLED_6X8);
  } else {
    OLED_ShowString(0, 38, " M3 Ball", OLED_6X8);
  }

  OLED_ShowString(0, 56, "K1Sel K2Run K3T", OLED_6X8);
  OLED_Update();
}

/* 功能：显示当前正在运行的模式；参数mode_name为模式名称字符串；返回：无。 */
static void OLED_ShowRun(char *mode_name)
{
  OLED_Clear();
  OLED_ShowString(0, 0, "Running", OLED_8X16);
  OLED_ShowString(0, 24, mode_name, OLED_8X16);
  OLED_ShowString(0, 50, "Please wait...", OLED_6X8);
  OLED_Update();
}

/* 功能：按键切换到下一个可选模式；参数：无；返回：无。 */
static void Mode_SelectNext(void)
{
  SelectMode++;
  if (SelectMode > MODE_BALL_TRACK) {
    SelectMode = MODE_COMBO;
  }

  OLED_ShowMenu();
}

/* 功能：启动当前菜单选中的模式；参数：无；返回：无。 */
static void Mode_StartSelected(void)
{
  if (SelectMode == MODE_COMBO) {
    OLED_ShowRun("M1 Combo");
    Car_StartCombo();
  } else if (SelectMode == MODE_TIMED_TRACK) {
    OLED_ShowRun("M2 Track");
    Car_StartTimedTrack();
  } else {
    OLED_ShowRun("M3 Ball");
    Car_StartBallTrack();
  }
}

/* 功能：循环调整定时循迹模式的目标时间；参数：无；返回：无。 */
static void Mode_TimeNext(void)
{
  TrackSetTime += TRACK_TIME_STEP;
  if (TrackSetTime > TRACK_TIME_MAX) {
    TrackSetTime = TRACK_TIME_MIN;
  }

  OLED_ShowMenu();
  Bluetooth_Printf("Track Time:%ds\r\n", TrackSetTime / 100U);
}

/* 功能：10ms主任务，处理按键、传感器、模式状态机和调试输出；参数：无；返回：无。 */
void Car_TestRun(void)
{
  static uint8_t DebugCount = 0U;
  static uint32_t LastTick = 0U;
  uint32_t CurrentTick;
  int16_t LeftSpeed;
  int16_t RightSpeed;
  float turn_angle;
  float dt;

  Key_Scan10ms();

  if (Key_TakeK1Press()) {    //切换模式
    if (TestMode == TEST_IDLE) {
      Mode_SelectNext();
    }
  }
  if (Key_TakeK2Press()) {    //确认模式
    if (TestMode == TEST_IDLE) {
      Mode_StartSelected();
    }
  }
  if (Key_TakeK3Press()) {    //修改时间
    if (TestMode == TEST_IDLE && SelectMode == MODE_TIMED_TRACK) {
      Mode_TimeNext();
    }
  }

  if (Timer_Take10msFlag() == 0U) {
    return;
  }

  Buzzer_Update();
  K230Vision_Update();  //空的

  CurrentTick = HAL_GetTick();    //单位ms，计算与上次更新的时间差dt，单位s
  if (LastTick == 0U) {
    dt = 0.01f;
  } else {
    dt = (float)(CurrentTick - LastTick) / 1000.0f;
  }
  LastTick = CurrentTick;

  /* 异常间隔仍按10ms处理，避免调试暂停后角度跳变 */
  if (dt <= 0.0f || dt > 0.1f) {
    dt = 0.01f;
  }

  Car_UpdateYaw(dt);
  turn_angle = Angle_GetError(CurrentYaw, TurnStartYaw);
  LeftSpeed = ENCODER_LEFT_DIR * Encoder_GetLeftSpeed();
  RightSpeed = ENCODER_RIGHT_DIR * Encoder_GetRightSpeed();

  if (TestMode == TEST_BALL_TRACK) {
    BallTrack_Control(LeftSpeed, RightSpeed);
  } else if (TestMode == TEST_TIMED_TRACK) {
    TrackProcess_RunTimed(LeftSpeed, RightSpeed);
  } else if (TestMode == TEST_COMBO) {
    TrackProcess_RunCombo(LeftSpeed, RightSpeed);
  } else if (TestMode == TEST_BRAKE) {
    Car_BrakeControl(LeftSpeed, RightSpeed);

    if (++TestCount >= BRAKE_TIME) {
      Car_FinishBrake();
    }
  } else {
    Motor_Stop();
  }

  if (++DebugCount >= DEBUG_PERIOD) {
    DebugCount = 0U;
    Bluetooth_Printf("M:%d L:%d R:%d Y:%d T:%d A:%d\r\n",
                     TestMode,
                     LeftSpeed,
                     RightSpeed,
                     (int)(CurrentYaw * 10.0f),
                     (int)(TargetYaw * 10.0f),
                     (int)(turn_angle * 10.0f));
  }
}
