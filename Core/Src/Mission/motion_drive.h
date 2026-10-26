#ifndef MOTION_DRIVE_H
#define MOTION_DRIVE_H
#include "mission_segment.h"
/* Foreground-only actuator adapter; contains no GPIO/HAL dependencies. */
void MotionDrive_Init(void);
void MotionDrive_Command(MissionMotionMode mode, float left, float right, uint32_t now_ms);
void MotionDrive_Stop(void);
MissionMotionMode MotionDrive_Mode(void);
#endif
