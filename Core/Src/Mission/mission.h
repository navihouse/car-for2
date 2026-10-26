#ifndef MISSION_H
#define MISSION_H
#include "mission_control.h"

/* Foreground API, after Cube GPIO/TIM/UART initialization. Init leaves drive
 * braked; poll Update continuously. Start returns 0 if the route is not ready. */
void Mission_Init(uint32_t now_ms);
void Mission_Update(uint32_t now_ms);
uint8_t Mission_Start(uint32_t now_ms);
void Mission_Stop(uint32_t now_ms);

/* V2 diagnostic ABI (144 bytes). Old task2/mode2/line-follow layouts are retired.
 * Read while stopped or accept only equal even sequence_begin/sequence_end
 * across a read. Enumerations are defined in mission_control/segment.h. */
typedef struct {
    uint32_t magic, version, sequence_begin, mode;
    uint32_t state, reason, segment, segment_fault;
    uint32_t uptime_ms, elapsed_ms, arrivals, last_point;
    uint32_t last_event_ms, event_count;
    int32_t left_target_cps, right_target_cps;
    uint32_t speed_fault, gyro_ready, gyro_age_ms;
    uint32_t line_calibrated, line_age_ms, line_mask;
    uint32_t calibrating, phase, motion_mode;
    int32_t distance_mm, point_min_mm, point_max_mm, target_yaw_mdeg, heading_error_mdeg;
    int32_t line_position;
    uint32_t point_confirm_frames, turn_corrections;
    int32_t nominal_mm;
    uint32_t capture_frames, sequence_end;
} MissionDiagnostics;
extern volatile MissionDiagnostics mission_diag;
#endif
