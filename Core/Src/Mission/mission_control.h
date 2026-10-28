#ifndef MISSION_CONTROL_H
#define MISSION_CONTROL_H
#include "mission_config.h"
#include "mission_segment.h"

typedef enum { MISSION_IDLE, MISSION_RUNNING, MISSION_DONE, MISSION_STOPPED,
               MISSION_REJECTED, MISSION_FAULT } MissionState;
typedef enum { MISSION_OK, MISSION_NOT_IMPLEMENTED, MISSION_USER_STOP,
               MISSION_BAD_CONFIG, MISSION_BAD_TARGET, MISSION_SEGMENT_FAULT,
               MISSION_GYRO_UNAVAILABLE, MISSION_LINE_UNAVAILABLE,
               MISSION_ENCODER_STALE, MISSION_SPEED_FAULT, MISSION_CALIBRATING } MissionReason;
typedef enum { DRIVE_HOLD, DRIVE_BEGIN, DRIVE_TARGET, DRIVE_STOP } MissionDriveAction;
typedef struct {
    MissionDriveAction action;
    float left_cps, right_cps;
    MissionPoint point; /* One arrival per completed segment; none at launch. */
    MissionMotionMode mode;
} MissionOutput;
typedef struct {
    MissionState state;
    MissionReason reason;
    uint32_t mode, started_ms, ended_ms, run_started, segment_fault, arrivals;
    uint32_t lap, laps; /* 0-based completed-lap counter and the configured total. */
    uint32_t laps_override; /* Bench/test total; 0 keeps the mode default. */
    int32_t drift_mdeg; /* Sum of course-heading residuals cancelled at point A. */
    MissionSegmentId segment;
    MissionPoint last_point;
    float course_origin_deg;
    const MissionSegment *segments[SEGMENT_COUNT];
    MissionSegmentContext contexts[SEGMENT_COUNT];
} MissionControl;

/* Pure control: caller owns sampling, actuation and notification. Test doubles
 * may supply descriptors here; firmware always supplies the four real ones. */
void MissionControl_Init(MissionControl *, uint32_t mode, const MissionSegment *const segments[SEGMENT_COUNT]);
MissionOutput MissionControl_Start(MissionControl *, const MissionInput *);
MissionOutput MissionControl_Update(MissionControl *, const MissionInput *);
MissionOutput MissionControl_Stop(MissionControl *, const MissionInput *);
MissionOutput MissionControl_Fail(MissionControl *, const MissionInput *, MissionReason, uint32_t detail);
#endif
