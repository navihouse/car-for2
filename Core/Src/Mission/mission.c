#include "mission.h"
#include "mission_port.h"
#include "button.h"
#include <math.h>

volatile MissionDiagnostics mission_diag;
static MissionControl control;
static MissionInput snapshot;
static Button_State button;
static uint32_t press_ms, tracking, consumed, event_count, last_event_ms;
static int32_t left_target, right_target;
static uint32_t last_publish_ms;
static MissionMotionMode commanded_mode;
_Static_assert(sizeof(MissionDiagnostics) == 144U, "Update the mission dump decoder");

static int32_t DiagnosticNumber(float value)
{
    if (!isfinite(value)) return 0;
    if (value>2147483000.0f) return INT32_MAX;
    if (value< -2147483000.0f) return INT32_MIN;
    return (int32_t)value;
}

static void Publish(uint32_t now)
{
    const MissionSegmentContext *ctx=&control.contexts[control.segment];
    const MissionSegmentParams *params=control.segments[control.segment]->params;
    /* Leave a stable window for read-only SWD snapshots. State changes and
     * arrivals are still published in the same tick, independent of cadence. */
    if ((uint32_t)(now - last_publish_ms) < 20U &&
        mission_diag.state == (uint32_t)control.state && mission_diag.reason == (uint32_t)control.reason &&
        mission_diag.segment == (uint32_t)control.segment && mission_diag.event_count == event_count &&
        mission_diag.calibrating == snapshot.calibrating && mission_diag.speed_fault == snapshot.speed_fault &&
        mission_diag.left_target_cps == left_target && mission_diag.right_target_cps == right_target &&
        mission_diag.phase==ctx->phase && mission_diag.motion_mode==(uint32_t)commanded_mode) return;
    uint32_t seq = (mission_diag.sequence_begin + 2U) & ~1U;
    MissionDiagnostics d = {
        .magic = 0x4D534E31U, .version = 2U, .sequence_begin = seq - 1U,
        .mode = control.mode, .state = control.state, .reason = control.reason,
        .segment = control.segment, .segment_fault = control.segment_fault,
        .uptime_ms = now, .arrivals = control.arrivals, .last_point = control.last_point,
        .event_count = event_count, .last_event_ms = last_event_ms,
        .left_target_cps = left_target, .right_target_cps = right_target,
        .speed_fault = snapshot.speed_fault, .gyro_ready = snapshot.gyro_ready, .gyro_age_ms = snapshot.gyro_age_ms,
        .line_calibrated = snapshot.line_calibrated, .line_age_ms = snapshot.line_age_ms,
        .line_mask = snapshot.line_mask, .calibrating = snapshot.calibrating,
        .phase=ctx->phase, .motion_mode=commanded_mode,
        .distance_mm=DiagnosticNumber(ctx->motion.distance_mm),
        .point_min_mm=DiagnosticNumber(params->point_min_mm), .point_max_mm=DiagnosticNumber(params->point_max_mm),
        .target_yaw_mdeg=DiagnosticNumber(ctx->motion.target_yaw_deg*1000.0f),
        .heading_error_mdeg=DiagnosticNumber(ctx->motion.heading_error_deg*1000.0f),
        .line_position=DiagnosticNumber(ctx->motion.line_position),
        .point_confirm_frames=ctx->motion.edge.candidate_frames, .turn_corrections=ctx->motion.corrections,
        .nominal_mm=DiagnosticNumber(params->nominal_mm), .capture_frames=ctx->motion.capture_frames,
        .sequence_end = seq
    };
    if (control.state == MISSION_RUNNING) d.elapsed_ms = now - control.started_ms;
    else if (control.run_started && (control.state == MISSION_DONE || control.state == MISSION_STOPPED || control.state == MISSION_FAULT))
        d.elapsed_ms = control.ended_ms - control.started_ms;
    mission_diag.sequence_begin = seq - 1U;
    mission_diag = d;
    mission_diag.sequence_begin = seq;
    last_publish_ms = now;
}
static void Apply(MissionOutput out, uint32_t now)
{
    if (out.action == DRIVE_BEGIN || out.action == DRIVE_TARGET) {
        MissionPort_Command(out.mode,out.left_cps,out.right_cps,now); commanded_mode=out.mode;
    } else if (out.action == DRIVE_STOP) {
        MissionPort_StopDrive(); commanded_mode=MOTION_BRAKE;
    }

    if (out.action == DRIVE_BEGIN || out.action == DRIVE_TARGET) {
        left_target = (int32_t)out.left_cps;
        right_target = (int32_t)out.right_cps;
    } else if (out.action == DRIVE_STOP) {
        left_target = right_target = 0;
    }
    if (out.point != POINT_NONE) {
        ++event_count; last_event_ms = now;
        MissionPort_NotifyPoint(out.point, now);
    }
}
static uint8_t StartSnapshot(uint32_t now)
{
    if (control.state == MISSION_RUNNING) return 0U;
    Apply(MissionControl_Start(&control, &snapshot), now);
    return control.state == MISSION_RUNNING;
}
void Mission_Init(uint32_t now)
{
    static const MissionSegment *const segments[SEGMENT_COUNT] = {
        &mission_segment_ac, &mission_segment_cb, &mission_segment_bd, &mission_segment_da
    };
    MissionPort_Init(now);
    MissionControl_Init(&control, MISSION_BUILD_MODE, segments);
    mission_diag = (MissionDiagnostics){0};
    tracking = consumed = event_count = last_event_ms = 0U;
    left_target = right_target = 0;
    commanded_mode=MOTION_BRAKE;
    last_publish_ms = now - 20U;
    snapshot = MissionPort_Capture(now);
    Button_Init(&button, MissionPort_ButtonPressed(), now);
    Publish(now);
}
uint8_t Mission_Start(uint32_t now)
{
    snapshot = MissionPort_Capture(now);
    uint8_t ok = StartSnapshot(now);
    /* Public starts use the same fault checks as an Update-driven start. */
    if (ok) {
        uint32_t fault = MissionPort_UpdateDrive(now);
        if (fault) {
            snapshot.speed_fault = fault;
            Apply(MissionControl_Fail(&control, &snapshot, MISSION_SPEED_FAULT, fault), now);
            ok = 0U;
        }
    }
    Publish(now);
    return ok;
}
void Mission_Stop(uint32_t now)
{
    snapshot = MissionPort_Capture(now);
    if (snapshot.calibrating) MissionPort_CalibrationCancel();
    Apply(MissionControl_Stop(&control, &snapshot), now);
    /* Consumes a pending button release so STOP cannot become a restart. */
    consumed = 1U; tracking = 0U;
    Button_Init(&button, MissionPort_ButtonPressed(), now);
    snapshot.calibrating = 0U;
    Publish(now);
}
void Mission_Update(uint32_t now)
{
    MissionPort_Service(now);
    snapshot = MissionPort_Capture(now);
    uint8_t pressed = Button_Update(&button, MissionPort_ButtonPressed(), now);
    if (pressed) {
        tracking = 1U; press_ms = now; consumed = 0U;
        if (control.state == MISSION_RUNNING) {
            Apply(MissionControl_Stop(&control, &snapshot), now); consumed = 1U;
        } else if (snapshot.calibrating) {
            MissionPort_CalibrationCancel(); snapshot.calibrating = 0U; consumed = 1U;
        }
    }
    uint8_t started = 0U;
    if (tracking && !button.stable) {
        tracking = 0U;
        if (!consumed) {
            if ((uint32_t)(now - press_ms) >= MISSION_CAL_HOLD_MS) {
                MissionPort_StopDrive(); MissionPort_CalibrationStart(now); snapshot.calibrating = 1U;
            } else started = StartSnapshot(now);
        }
    }
    if (control.state == MISSION_RUNNING) {
        if (!started) Apply(MissionControl_Update(&control, &snapshot), now);
        if (control.state == MISSION_RUNNING) {
            uint32_t fault = MissionPort_UpdateDrive(now);
            if (fault) {
                snapshot.speed_fault = fault;
                Apply(MissionControl_Fail(&control, &snapshot, MISSION_SPEED_FAULT, fault), now);
            }
        }
    }
    Publish(now);
    MissionDiagnostics display = mission_diag;
    MissionPort_Draw(&display);
}
