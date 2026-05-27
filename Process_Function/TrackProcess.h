#ifndef __TRACKPROCESS_H
#define __TRACKPROCESS_H

#include "main.h"

void Track_CalcParam(void);
void TrackProcess_RunTimed(int16_t left_speed, int16_t right_speed);
void TrackProcess_RunCombo(int16_t left_speed, int16_t right_speed);
void Car_StartTimedTrack(void);
void Car_StartCombo(void);

#endif
