#ifndef MISSION_SEGMENT_H
#define MISSION_SEGMENT_H
#include <stdint.h>

typedef enum { SEGMENT_AC, SEGMENT_CB, SEGMENT_BD, SEGMENT_DA, SEGMENT_COUNT } MissionSegmentId;
typedef enum { POINT_NONE, POINT_A, POINT_B, POINT_C, POINT_D } MissionPoint;
typedef enum { SEG_RUNNING, SEG_COMPLETE, SEG_FAULT, SEG_UNIMPLEMENTED } MissionSegmentStatus;
typedef enum { EXIT_COMPLETE, EXIT_STOP, EXIT_FAULT } MissionExitReason;
enum { SENSOR_GYRO = 1U, SENSOR_LINE = 2U };

/* One immutable foreground snapshot, shared by outgoing/incoming segments.
 * Wheel position is normalized encoder counts; speed is counts/second.
 * Yaw/rate are degrees/degrees per second, never reset at a handoff. */
typedef struct {
    uint32_t now_ms, encoder_sequence, encoder_age_ms;
    int64_t left_counts, right_counts;
    float left_cps, right_cps, yaw_deg, yaw_rate_dps;
    uint32_t gyro_ready, gyro_fault, gyro_age_ms;
    uint32_t line_sequence, line_age_ms, line_calibrated, line_mask;
    uint32_t line_strength[8];
    uint32_t calibrating, speed_fault;
} MissionInput;

typedef struct {
    MissionInput entry;
    uint32_t active, phase, phase_started_ms;
} MissionSegmentContext;

/* Zero values deliberately mean not configured; no guessed track geometry. */
typedef struct { float cruise_cps; uint32_t timeout_ms; } MissionSegmentParams;
typedef struct {
    MissionSegmentStatus status;
    uint32_t fault;
    float left_cps, right_cps;
} MissionSegmentOutput;

typedef struct {
    MissionSegmentId id;
    uint32_t implemented, required_sensors;
    const MissionSegmentParams *params;
    MissionSegmentOutput (*Enter)(MissionSegmentContext *, const MissionInput *, const MissionSegmentParams *);
    MissionSegmentOutput (*Update)(MissionSegmentContext *, const MissionInput *, const MissionSegmentParams *);
    /* Cleanup only: no motor, encoder, IMU or other hardware calls. */
    void (*Exit)(MissionSegmentContext *, const MissionInput *, MissionExitReason);
} MissionSegment;

extern const MissionSegment mission_segment_ac, mission_segment_cb;
extern const MissionSegment mission_segment_bd, mission_segment_da;
#endif
