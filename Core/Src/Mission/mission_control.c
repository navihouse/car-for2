#include "mission_control.h"
#include <math.h>
#include <stddef.h>

static const MissionPoint endpoints[SEGMENT_COUNT] = { POINT_C, POINT_B, POINT_D, POINT_A };
static MissionOutput Output(MissionDriveAction action)
{
    return (MissionOutput){ .action = action };
}
static uint8_t ValidDescriptor(const MissionSegment *s, uint32_t id)
{
    return s && s->id == id && s->params && s->Enter && s->Update && s->Exit &&
        !(s->required_sensors & ~(SENSOR_GYRO | SENSOR_LINE));
}
static MissionReason CheckInput(const MissionSegment *s, const MissionInput *in)
{
    if (in->calibrating) return MISSION_CALIBRATING;
    if (in->speed_fault) return MISSION_SPEED_FAULT;
    if (in->encoder_age_ms > 100U) return MISSION_ENCODER_STALE;
    if ((s->required_sensors & SENSOR_GYRO) && (!in->gyro_ready || in->gyro_fault ||
        in->gyro_age_ms > MISSION_GYRO_MAX_AGE_MS || !isfinite(in->yaw_deg) || !isfinite(in->yaw_rate_dps)))
        return MISSION_GYRO_UNAVAILABLE;
    if ((s->required_sensors & SENSOR_LINE) && (!in->line_calibrated || !in->line_sequence ||
        in->line_age_ms > MISSION_LINE_MAX_AGE_MS)) return MISSION_LINE_UNAVAILABLE;
    return MISSION_OK;
}
static uint8_t ValidTarget(MissionSegmentOutput out)
{
    /* Positive forward drive only. Final parking/stop is a controller action. */
    return out.left_cps > 0.0f && out.left_cps <= MISSION_MAX_TARGET_CPS &&
           out.right_cps > 0.0f && out.right_cps <= MISSION_MAX_TARGET_CPS;
}
static void Exit(MissionControl *m, const MissionInput *in, MissionExitReason reason)
{
    MissionSegmentContext *ctx = &m->contexts[m->segment];
    if (ctx->active) {
        m->segments[m->segment]->Exit(ctx, in, reason);
        ctx->active = 0U;
    }
}
void MissionControl_Init(MissionControl *m, uint32_t mode, const MissionSegment *const segments[SEGMENT_COUNT])
{
    *m = (MissionControl){ .mode = mode, .segment = SEGMENT_AC };
    for (uint32_t i = 0; i < SEGMENT_COUNT; ++i) m->segments[i] = segments[i];
    if (mode >= MISSION_MODE_AC && mode <= MISSION_MODE_DA) m->segment = (MissionSegmentId)(mode - 1U);
}
MissionOutput MissionControl_Fail(MissionControl *m, const MissionInput *in, MissionReason reason, uint32_t detail)
{
    if (m->state == MISSION_RUNNING) Exit(m, in, EXIT_FAULT);
    m->state = MISSION_FAULT; m->reason = reason; m->segment_fault = detail; m->ended_ms = in->now_ms;
    return Output(DRIVE_STOP);
}
static MissionOutput Accept(MissionControl *m, const MissionInput *in, MissionSegmentOutput result, MissionDriveAction action)
{
    if (result.status == SEG_UNIMPLEMENTED)
        return MissionControl_Fail(m, in, MISSION_NOT_IMPLEMENTED, result.fault);
    if (result.status != SEG_RUNNING)
        return MissionControl_Fail(m, in, MISSION_SEGMENT_FAULT, result.fault);
    if (!ValidTarget(result)) return MissionControl_Fail(m, in, MISSION_BAD_TARGET, 0U);
    MissionOutput out = Output(action);
    out.left_cps = result.left_cps; out.right_cps = result.right_cps;
    return out;
}
static MissionSegmentOutput Enter(MissionControl *m, const MissionInput *in)
{
    const MissionSegment *s = m->segments[m->segment];
    MissionSegmentContext *ctx = &m->contexts[m->segment];
    *ctx = (MissionSegmentContext){ .entry = *in, .active = 1U, .phase_started_ms = in->now_ms };
    return s->Enter(ctx, in, s->params);
}
MissionOutput MissionControl_Start(MissionControl *m, const MissionInput *in)
{
    if (m->state == MISSION_RUNNING) return Output(DRIVE_HOLD);
    m->reason = MISSION_OK; m->segment_fault = m->arrivals = 0U; m->last_point = POINT_NONE;
    m->started_ms = m->ended_ms = m->run_started = 0U;
    uint32_t first = m->mode == MISSION_MODE_FULL ? 0U : m->mode - 1U;
    uint32_t end = m->mode == MISSION_MODE_FULL ? SEGMENT_COUNT : first + 1U;
    if (m->mode > MISSION_MODE_DA) {
        m->state = MISSION_REJECTED; m->reason = MISSION_BAD_CONFIG; return Output(DRIVE_STOP);
    }
    m->segment = (MissionSegmentId)first;
    /* Check the WHOLE selected route before any Enter/drive operation. */
    for (uint32_t i = first; i < end; ++i) {
        if (!ValidDescriptor(m->segments[i], i)) {
            m->segment = (MissionSegmentId)i; m->state = MISSION_REJECTED;
            m->reason = MISSION_BAD_CONFIG; return Output(DRIVE_STOP);
        }
        if (!m->segments[i]->implemented) {
            m->segment = (MissionSegmentId)i; m->state = MISSION_REJECTED;
            m->reason = MISSION_NOT_IMPLEMENTED; return Output(DRIVE_STOP);
        }
    }
    MissionReason reason = CheckInput(m->segments[first], in);
    if (reason != MISSION_OK) {
        m->state = MISSION_REJECTED; m->reason = reason; return Output(DRIVE_STOP);
    }
    m->state = MISSION_RUNNING; m->started_ms = in->now_ms; m->run_started = 1U;
    MissionOutput out = Accept(m, in, Enter(m, in), DRIVE_BEGIN);
    /* An unexpected Enter failure must not energize the motors either. */
    if (m->state == MISSION_FAULT) m->state = MISSION_REJECTED;
    return out;
}
MissionOutput MissionControl_Update(MissionControl *m, const MissionInput *in)
{
    if (m->state != MISSION_RUNNING) return Output(DRIVE_HOLD);
    const MissionSegment *s = m->segments[m->segment];
    MissionReason reason = CheckInput(s, in);
    if (reason != MISSION_OK) return MissionControl_Fail(m, in, reason, in->speed_fault);
    MissionSegmentOutput result = s->Update(&m->contexts[m->segment], in, s->params);
    if (result.status != SEG_COMPLETE) return Accept(m, in, result, DRIVE_TARGET);

    MissionPoint arrived = endpoints[m->segment];
    Exit(m, in, EXIT_COMPLETE);
    m->last_point = arrived; ++m->arrivals;
    MissionOutput out;
    if (m->mode != MISSION_MODE_FULL || m->segment == SEGMENT_DA) {
        m->state = MISSION_DONE; m->reason = MISSION_OK; m->ended_ms = in->now_ms;
        out = Output(DRIVE_STOP);
    } else {
        ++m->segment;
        reason = CheckInput(m->segments[m->segment], in);
        if (reason != MISSION_OK) out = MissionControl_Fail(m, in, reason, in->speed_fault);
        else out = Accept(m, in, Enter(m, in), DRIVE_TARGET);
    }
    /* Incoming Enter consumes this SAME snapshot and supplies this tick's
     * targets. No Stop/Start, zero target, reset or repeated arrival here. */
    out.point = arrived;
    return out;
}
MissionOutput MissionControl_Stop(MissionControl *m, const MissionInput *in)
{
    if (m->state == MISSION_RUNNING) {
        Exit(m, in, EXIT_STOP);
        m->ended_ms = in->now_ms;
    } else if (!m->run_started) m->ended_ms = in->now_ms;
    m->state = MISSION_STOPPED; m->reason = MISSION_USER_STOP;
    return Output(DRIVE_STOP);
}
