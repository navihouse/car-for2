#ifndef MISSION_PORT_H
#define MISSION_PORT_H
#include "mission.h"

/* Single hardware boundary. Tests replace these functions, not the scheduler. */
void MissionPort_Init(uint32_t now_ms);
void MissionPort_Service(uint32_t now_ms);
MissionInput MissionPort_Capture(uint32_t now_ms);
uint8_t MissionPort_ButtonPressed(void);
void MissionPort_Command(MissionMotionMode mode, float left_cps, float right_cps, uint32_t now_ms);
uint32_t MissionPort_UpdateDrive(uint32_t now_ms);
void MissionPort_StopDrive(void);
void MissionPort_CalibrationStart(uint32_t now_ms);
void MissionPort_CalibrationCancel(void);
/* Nonblocking arrival hook. Sound hardware is deliberately unassigned. */
void MissionPort_NotifyPoint(MissionPoint point, uint32_t now_ms);
void MissionPort_Draw(const MissionDiagnostics *diag);
#endif
