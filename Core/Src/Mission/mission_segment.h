#ifndef MISSION_SEGMENT_H
#define MISSION_SEGMENT_H
#include <stdint.h>

typedef enum { SEGMENT_AC, SEGMENT_CB, SEGMENT_BD, SEGMENT_DA, SEGMENT_COUNT } MissionSegmentId;
typedef enum { POINT_NONE, POINT_A, POINT_B, POINT_C, POINT_D } MissionPoint;
typedef enum { SEG_RUNNING, SEG_COMPLETE, SEG_FAULT, SEG_UNIMPLEMENTED } MissionSegmentStatus;
typedef enum { EXIT_COMPLETE, EXIT_STOP, EXIT_FAULT } MissionExitReason;
enum { SENSOR_GYRO = 1U, SENSOR_LINE = 2U };
typedef enum { MOTION_FORWARD, MOTION_TURN, MOTION_BRAKE } MissionMotionMode;
typedef enum { PHASE_IDLE, PHASE_STILL, PHASE_TURN, PHASE_SETTLE, PHASE_STRAIGHT,
               PHASE_CAPTURE, PHASE_FOLLOW, PHASE_COMPLETE, PHASE_FAULT } MissionPhase;
enum { MOTION_KIND_TEST, MOTION_KIND_STRAIGHT, MOTION_KIND_ARC };
enum { MOTION_FAULT_NONE, MOTION_FAULT_CONFIG, MOTION_FAULT_DISTANCE,
       MOTION_FAULT_TIMEOUT, MOTION_FAULT_TURN_TIMEOUT, MOTION_FAULT_TURN_DIRECTION,
       MOTION_FAULT_TURN_TRANSLATION, MOTION_FAULT_GYRO_RESPONSE,
       MOTION_FAULT_WHEEL_SYNC, MOTION_FAULT_HEADING, MOTION_FAULT_CAPTURE,
       MOTION_FAULT_NO_LINE, MOTION_FAULT_AMBIGUOUS };

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

typedef struct { int64_t left_origin, right_origin; uint32_t active; } MissionDistance;
typedef struct {
    uint32_t last_sequence, run_frames, candidate_frames;
    uint8_t have_sequence, run_known, run_black, stable_known, stable_black, candidate;
} MissionEdge;
typedef struct {
    MissionDistance distance, turn_distance;
    MissionEdge edge;
    float distance_mm, target_yaw_deg, heading_error_deg;
    float line_position, last_position, derivative;
    float left_target, right_target, recovery_left, recovery_right;
    float turn_origin_yaw, turn_start_error;
    uint32_t turn_started_ms, control_ms, line_sequence, have_line_sequence;
    uint32_t still_since_ms, still_tracking, corrections, uncertain, uncertain_ms;
    uint32_t capture_frames, capture_sequence, have_capture_sequence, have_position;
} MissionMotionState;
typedef struct {
    MissionInput entry;
    uint32_t active, phase, phase_started_ms;
    float course_origin_deg;
    uint32_t from_handoff;
    MissionMotionState motion;
} MissionSegmentContext;

typedef struct {
    float cruise_cps;
    uint32_t timeout_ms;
    uint32_t kind;
    float capture_cps, target_yaw_deg, nominal_mm, point_min_mm, point_max_mm;
} MissionSegmentParams;
typedef struct {
    MissionSegmentStatus status;
    uint32_t fault;
    float left_cps, right_cps;
    MissionMotionMode mode;
} MissionSegmentOutput;

typedef struct {
    MissionSegmentId id;
    uint32_t implemented, required_sensors;
    const MissionSegmentParams *params;
    MissionSegmentOutput (*Enter)(MissionSegmentContext *, const MissionInput *, const MissionSegmentParams *);
    MissionSegmentOutput (*Update)(MissionSegmentContext *, const MissionInput *, const MissionSegmentParams *);
    /* Cleanup only: no motor, encoder, IMU or other hardware calls. */
    void (*Exit)(MissionSegmentContext *, const MissionInput *, MissionExitReason);
    uint8_t (*Validate)(const MissionSegmentParams *);
} MissionSegment;

extern const MissionSegment mission_segment_ac, mission_segment_cb;
extern const MissionSegment mission_segment_bd, mission_segment_da;
#endif
