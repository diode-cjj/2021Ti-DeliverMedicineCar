#ifndef __CARSTATE_H
#define __CARSTATE_H

#include "main.h"
#include "PID.h"

extern PID_t SpeedPID_L;
extern PID_t SpeedPID_R;
extern PID_t AnglePID;

extern float TargetSpeed_L;
extern float TargetSpeed_R;
extern float CurrentYaw;
extern float CurrentGz;
extern float TurnStartYaw;
extern float TargetYaw;
extern uint8_t TestMode;
extern uint8_t SelectMode;
extern uint8_t ComboStep;
extern uint8_t BrakeFinishMode;
extern uint16_t TestCount;
extern uint16_t BuzzerCount;
extern uint16_t TrackSetTime;
extern uint16_t TrackStraight1Time;
extern uint16_t TrackStraight2Time;
extern uint16_t TrackTurnMaxTime;
extern float TrackTargetSpeed;
extern float TrackTurnCenterSpeed;
extern uint8_t BallStep;
extern int16_t BallBaseStep;
extern int16_t BallTopStep;
extern int8_t BallSearchDir;
extern int8_t BallTiltSearchDir;
extern uint8_t BallTiltCount;
extern uint8_t BallSearchEdgeCount;

#endif
