#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H
#include "mission_segment.h"
#include "motion_tuning.h"

void MotionDistance_Begin(MissionDistance *, const MissionInput *);
float MotionDistance_GetMm(const MissionDistance *, const MissionInput *);
void MotionEdge_Reset(MissionEdge *);
void MotionEdge_SeedBlack(MissionEdge *, const MissionInput *);
uint8_t MotionEdge_Update(MissionEdge *, const MissionInput *, float distance_mm,
                         float minimum_mm, float maximum_mm, uint8_t target_black);
float Motion_Clamp(float value, float lo, float hi);
float Motion_WrapDegrees(float degrees);
float Motion_Yaw(const MissionSegmentContext *, const MissionInput *);
uint8_t Motion_ValidateTuning(void);
uint8_t Motion_ValidateParams(const MissionSegmentParams *);
MissionSegmentOutput Motion_Output(MissionSegmentContext *, MissionMotionMode, float left, float right);
MissionSegmentOutput Motion_Fail(MissionSegmentContext *, uint32_t fault);
MissionSegmentOutput Motion_Complete(MissionSegmentContext *);
void Motion_Exit(MissionSegmentContext *, const MissionInput *, MissionExitReason);
MissionSegmentOutput MotionStraight_Enter(MissionSegmentContext *, const MissionInput *, const MissionSegmentParams *);
MissionSegmentOutput MotionStraight_Update(MissionSegmentContext *, const MissionInput *, const MissionSegmentParams *);
MissionSegmentOutput MotionArc_Enter(MissionSegmentContext *, const MissionInput *, const MissionSegmentParams *);
MissionSegmentOutput MotionArc_Update(MissionSegmentContext *, const MissionInput *, const MissionSegmentParams *);
#endif
