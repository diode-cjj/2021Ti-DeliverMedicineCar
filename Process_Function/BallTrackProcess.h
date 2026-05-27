#ifndef __BALLTRACKPROCESS_H
#define __BALLTRACKPROCESS_H

#include "main.h"

void BallTrack_Reset(void);
void BallTrack_Control(int16_t left_speed, int16_t right_speed);
void Car_StartBallTrack(void);

#endif
