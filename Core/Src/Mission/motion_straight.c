#include "motion_control.h"
#include <math.h>

static uint8_t Still(const MissionInput *in)
{
    return fabsf(in->yaw_rate_dps)<=MOTION_STILL_RATE_DPS &&
        fabsf(in->left_cps)<MOTION_STILL_WHEEL_CPS && fabsf(in->right_cps)<MOTION_STILL_WHEEL_CPS;
}
static uint8_t Settled(MissionMotionState *s, const MissionInput *in, uint32_t window_ms)
{
    if (!Still(in)) { s->still_tracking=0U; return 0U; }
    if (!s->still_tracking) { s->still_tracking=1U; s->still_since_ms=in->now_ms; }
    return (uint32_t)(in->now_ms-s->still_since_ms)>=window_ms;
}
/* D has no transverse marker. A right bend on a still-visible narrow line
 * must persist on distinct gray frames inside the distance window. */
static uint8_t BendAtD(MissionMotionState *s, const MissionInput *in,
                       const MissionSegmentParams *p, uint8_t line_visible)
{
    MissionEdge *e=&s->edge;
    uint8_t fresh=in->line_sequence && (!e->have_sequence || in->line_sequence!=e->last_sequence);
    if (fresh) { e->have_sequence=1U; e->last_sequence=in->line_sequence; }
    if (!line_visible || s->distance_mm<p->point_min_mm || s->distance_mm>p->point_max_mm ||
        s->heading_error_deg<MOTION_BD_D_TURN_MIN_DEG ||
        MOTION_YAW_SIGN*in->yaw_rate_dps>-MOTION_BD_D_RATE_MIN_DPS) {
        e->candidate_frames=0U;
        return 0U;
    }
    if (fresh && e->candidate_frames<MOTION_POINT_CONFIRM_FRAMES) ++e->candidate_frames;
    return e->candidate_frames>=MOTION_POINT_CONFIRM_FRAMES;
}
static MissionSegmentOutput FollowBD(MissionSegmentContext *ctx, const MissionInput *in,
                                     const MissionSegmentParams *p)
{
    MissionMotionState *s=&ctx->motion;
    float position=0.0f;
    uint32_t fault=MotionLine_Position(s,in,0U,1U,&position);
    if (BendAtD(s,in,p,fault==MOTION_FAULT_NONE)) return Motion_Complete(ctx);
    /* Before the D window, a large turn suggests the wrong branch. Near
     * D, allow the expected right turn; a large left turn is still a fault. */
    if (s->heading_error_deg < -MOTION_HEADING_FAULT_DEG ||
        (s->distance_mm<p->point_min_mm && s->heading_error_deg>MOTION_HEADING_FAULT_DEG))
        return Motion_Fail(ctx,MOTION_FAULT_HEADING);

    float base=p->cruise_cps, steer;
    float yaw=Motion_Yaw(ctx,in), rate=MOTION_YAW_SIGN*in->yaw_rate_dps;
    if (fault) {
        /* At a crossing or short dropout, hold the last valid heading for
         * a bounded time at reduced speed. No line means no D confirmation. */
        if (!s->uncertain) { s->uncertain=1U; s->uncertain_ms=in->now_ms; }
        if ((uint32_t)(in->now_ms-s->uncertain_ms)>=MOTION_BD_LINE_LOSS_MS)
            return Motion_Fail(ctx,fault);
        base=fminf(base,MOTION_BD_LINE_LOSS_CPS);
        steer=Motion_Clamp(MOTION_HEADING_KP*Motion_WrapDegrees(yaw-s->line_yaw_deg)+
            MOTION_HEADING_KD*rate,-MOTION_HEADING_TRIM_MAX_CPS,MOTION_HEADING_TRIM_MAX_CPS);
    } else {
        if (MotionLine_UpdatePosition(s,in,position,MOTION_BD_LINE_PERIOD_MS,s->uncertain))
            s->line_yaw_deg=yaw;
        s->uncertain=0U;
        steer=MOTION_LINE_KP*s->line_position+MOTION_LINE_KD*s->derivative+MOTION_BD_LINE_YAW_KD*rate;
    }
    float cap=fmaxf(0.0f,fminf(MOTION_LINE_STEERING_MAX_CPS,
        fminf(base-MOTION_LINE_MIN_WHEEL_CPS,(float)MOTION_MAX_CPS-base)));
    steer=Motion_Clamp(steer,-cap,cap);
    return Motion_Output(ctx,MOTION_FORWARD,base+steer,base-steer);
}
MissionSegmentOutput MotionStraight_Enter(MissionSegmentContext *ctx, const MissionInput *in, const MissionSegmentParams *p)
{
    if (!Motion_ValidateParams(p) || p->kind!=MOTION_KIND_STRAIGHT) return Motion_Fail(ctx,MOTION_FAULT_CONFIG);
    ctx->motion=(MissionMotionState){.target_yaw_deg=p->target_yaw_deg,.turn_started_ms=in->now_ms};
    ctx->motion.heading_error_deg=Motion_WrapDegrees(p->target_yaw_deg-Motion_Yaw(ctx,in));
    ctx->phase=PHASE_STILL; ctx->phase_started_ms=in->now_ms;
    return Motion_Output(ctx,MOTION_BRAKE,0.0f,0.0f);
}
MissionSegmentOutput MotionStraight_Update(MissionSegmentContext *ctx, const MissionInput *in, const MissionSegmentParams *p)
{
    MissionMotionState *s=&ctx->motion;
    s->heading_error_deg=Motion_WrapDegrees(s->target_yaw_deg-Motion_Yaw(ctx,in));
    if (ctx->phase!=PHASE_STRAIGHT && (uint32_t)(in->now_ms-s->turn_started_ms)>MOTION_TURN_TIMEOUT_MS)
        return Motion_Fail(ctx,MOTION_FAULT_TURN_TIMEOUT);
    if (ctx->phase==PHASE_STILL) {
        if (!Settled(s,in,MOTION_STILL_MS)) return Motion_Output(ctx,MOTION_BRAKE,0.0f,0.0f);
        if (ctx->return_a && fabsf(s->target_yaw_deg-Motion_Yaw(ctx,in))>MOTION_A_RETURN_MAX_DEG)
            return Motion_Fail(ctx,MOTION_FAULT_A_RETURN_RANGE);
        if (fabsf(s->heading_error_deg)>MOTION_TURN_BRAKE_DEG) {
            MotionTurn_Begin(ctx,in,PHASE_TURN); return MotionTurn_Update(ctx,in);
        }
        ctx->phase=PHASE_SETTLE; s->still_tracking=0U;
        return Motion_Output(ctx,MOTION_BRAKE,0.0f,0.0f);
    }
    /* Keep the unwrapped guard active during correction and settling too,
     * so an unexpected full turn cannot masquerade as a small error. */
    if (ctx->return_a && ctx->phase!=PHASE_STRAIGHT &&
        fabsf(s->target_yaw_deg-Motion_Yaw(ctx,in))>MOTION_A_RETURN_MAX_DEG)
        return Motion_Fail(ctx,MOTION_FAULT_A_RETURN_RANGE);
    if (ctx->phase==PHASE_TURN) return MotionTurn_Update(ctx,in);
    if (ctx->phase==PHASE_SETTLE) {
        if (!Settled(s,in,MOTION_SETTLE_MS)) return Motion_Output(ctx,MOTION_BRAKE,0.0f,0.0f);
        if (fabsf(s->heading_error_deg)>MOTION_TURN_ACCEPT_DEG) {
            if (s->corrections>=MOTION_TURN_MAX_CORRECTIONS) return Motion_Fail(ctx,MOTION_FAULT_TURN_TIMEOUT);
            ++s->corrections; MotionTurn_Begin(ctx,in,PHASE_TURN); return MotionTurn_Update(ctx,in);
        }
        /* The straight trip begins HERE, after all braking/rotation. */
        ctx->phase=PHASE_STRAIGHT; ctx->phase_started_ms=in->now_ms;
        MotionDistance_Begin(&s->distance,in); MotionEdge_Reset(&s->edge);
        s->line_yaw_deg=Motion_Yaw(ctx,in);
        s->control_ms=in->now_ms-MOTION_HEADING_PERIOD_MS;
    }
    if (ctx->phase!=PHASE_STRAIGHT) return Motion_Fail(ctx,MOTION_FAULT_CONFIG);
    s->distance_mm=MotionDistance_GetMm(&s->distance,in);
    if (s->distance_mm>p->point_max_mm || s->distance_mm < -MOTION_REVERSE_DISTANCE_FAULT_MM)
        return Motion_Fail(ctx,MOTION_FAULT_DISTANCE);
    if ((uint32_t)(in->now_ms-ctx->phase_started_ms)>p->timeout_ms) return Motion_Fail(ctx,MOTION_FAULT_TIMEOUT);
    if (p->follow_line) return FollowBD(ctx,in,p);
    if (fabsf(s->heading_error_deg)>MOTION_HEADING_FAULT_DEG) return Motion_Fail(ctx,MOTION_FAULT_HEADING);
    if (MotionEdge_Update(&s->edge,in,s->distance_mm,p->point_min_mm,p->point_max_mm,1U)) return Motion_Complete(ctx);
    if ((uint32_t)(in->now_ms-s->control_ms)>=MOTION_HEADING_PERIOD_MS) {
        /* Hand the next arc its entry speed: MOTION_POINT_APPROACH_MM before the
         * nominal point, ramp down to MOTION_STRAIGHT_HANDOFF_CPS and hold it
         * until the point is confirmed, so the differential that steers the
         * corner is not spent on decelerating from cruise. Anchored on the
         * nominal length, not the window floor: the slowdown must not start
         * early just because the window tolerates a short arrival. */
        float cruise=p->cruise_cps;
        float approach_start=p->nominal_mm-MOTION_POINT_APPROACH_MM;
        if (s->distance_mm>approach_start) {
            float t=Motion_Clamp((s->distance_mm-approach_start)/MOTION_POINT_APPROACH_MM,0.0f,1.0f);
            cruise=cruise+(MOTION_STRAIGHT_HANDOFF_CPS-cruise)*t;
        }
        float trim=Motion_Clamp(MOTION_HEADING_KP*s->heading_error_deg-MOTION_HEADING_KD*MOTION_YAW_SIGN*in->yaw_rate_dps,
            -MOTION_HEADING_TRIM_MAX_CPS,MOTION_HEADING_TRIM_MAX_CPS);
        s->left_target=cruise-trim; s->right_target=cruise+trim;
        /* Leave differential headroom if a user increases cruise speed. */
        float excess=fmaxf(s->left_target,s->right_target)-MOTION_MAX_CPS;
        if (excess>0.0f) { s->left_target-=excess; s->right_target-=excess; }
        s->control_ms=in->now_ms;
    }
    return Motion_Output(ctx,MOTION_FORWARD,s->left_target,s->right_target);
}
