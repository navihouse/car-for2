#include "motion_control.h"
#include <math.h>

/* Extracted from the old line_follow position/PD algorithm. Hardware, lap
 * counting and stop decisions belong to the mission, not this controller. */
static uint32_t Position(MissionMotionState *s, const MissionInput *in, float *position)
{
    static const uint8_t order[8]=MOTION_LINE_ORDER;
    uint32_t groups=0U, previous=0U, total=0U, active=0U;
    uint32_t weights[4]={0U}; int32_t weighted[4]={0};
    for (uint32_t i=0;i<8U;++i) {
        uint32_t raw=order[i], on=(in->line_mask>>raw)&1U;
        if (on) {
            ++active; if (!previous) ++groups;
            uint32_t strength=in->line_strength[raw]>1000U ? 1000U : in->line_strength[raw];
            uint32_t weight=strength>400U ? strength-400U : 1U;
            weights[groups-1U]+=weight;
            weighted[groups-1U]+=((int32_t)i*1000-3500)*(int32_t)weight;
            total+=weight;
        }
        previous=on;
    }
    if (!total) return MOTION_FAULT_NO_LINE;
    if (active>4U) return MOTION_FAULT_AMBIGUOUS;
    if (groups==1U) { *position=(float)weighted[0]/(float)weights[0]; return MOTION_FAULT_NONE; }
    if (!s->have_position || groups!=2U) return MOTION_FAULT_AMBIGUOUS;
    float p0=(float)weighted[0]/(float)weights[0], p1=(float)weighted[1]/(float)weights[1];
    float d0=fabsf(p0-s->last_position), d1=fabsf(p1-s->last_position);
    float nearest=fminf(d0,d1), other=fmaxf(d0,d1);
    if (nearest>MOTION_LINE_HISTORY_MAX || other-nearest<MOTION_LINE_GROUP_MARGIN) return MOTION_FAULT_AMBIGUOUS;
    *position=d0<d1 ? p0 : p1;
    return MOTION_FAULT_NONE;
}
static void Targets(MissionMotionState *s, float cruise)
{
    float floor=cruise<MOTION_LINE_HIGH_SPEED_THRESHOLD_CPS ? MOTION_LINE_CORNER_BASE_CPS : MOTION_LINE_HIGH_BASE_FLOOR_CPS;
    float base=Motion_Clamp(cruise-fabsf(s->line_position)*MOTION_LINE_POSITION_SLOWDOWN,fminf(floor,cruise),cruise);
    float wheel_floor=cruise<MOTION_LINE_HIGH_SPEED_THRESHOLD_CPS ? MOTION_LINE_MIN_WHEEL_CPS : MOTION_LINE_HIGH_WHEEL_FLOOR_CPS;
    float cap=Motion_Clamp(base-wheel_floor,0.0f,MOTION_LINE_STEERING_MAX_CPS);
    float turn=Motion_Clamp(MOTION_LINE_KP*s->line_position+MOTION_LINE_KD*s->derivative,-cap,cap);
    base=fminf(base,(float)MOTION_MAX_CPS-fabsf(turn));
    s->left_target=base+turn; s->right_target=base-turn;
}
MissionSegmentOutput MotionArc_Enter(MissionSegmentContext *ctx, const MissionInput *in, const MissionSegmentParams *p)
{
    if (!Motion_ValidateParams(p) || p->kind!=MOTION_KIND_ARC) return Motion_Fail(ctx,MOTION_FAULT_CONFIG);
    ctx->motion=(MissionMotionState){.target_yaw_deg=p->target_yaw_deg,
        .control_ms=in->now_ms-MOTION_LINE_PERIOD_MS};
    ctx->phase=PHASE_CAPTURE; ctx->phase_started_ms=in->now_ms;
    MotionDistance_Begin(&ctx->motion.distance,in);
    MotionEdge_Reset(&ctx->motion.edge);
    if (ctx->from_handoff) MotionEdge_SeedBlack(&ctx->motion.edge,in);
    /* Give the incoming segment's targets in the SAME handoff snapshot. */
    return MotionArc_Update(ctx,in,p);
}
MissionSegmentOutput MotionArc_Update(MissionSegmentContext *ctx, const MissionInput *in, const MissionSegmentParams *p)
{
    MissionMotionState *s=&ctx->motion;
    s->distance_mm=MotionDistance_GetMm(&s->distance,in);
    /* Arc heading is a diagnostic difference to its exit tangent. Steering
     * remains line-position PD, not a straight heading hold. */
    s->heading_error_deg=Motion_WrapDegrees(s->target_yaw_deg-Motion_Yaw(ctx,in));
    if (s->distance_mm>p->point_max_mm || s->distance_mm < -MOTION_REVERSE_DISTANCE_FAULT_MM)
        return Motion_Fail(ctx,MOTION_FAULT_DISTANCE);
    if ((uint32_t)(in->now_ms-ctx->entry.now_ms)>p->timeout_ms) return Motion_Fail(ctx,MOTION_FAULT_TIMEOUT);
    if (ctx->phase==PHASE_CAPTURE && ((uint32_t)(in->now_ms-ctx->phase_started_ms)>MOTION_CAPTURE_TIMEOUT_MS ||
        s->distance_mm>MOTION_CAPTURE_MAX_MM)) return Motion_Fail(ctx,MOTION_FAULT_CAPTURE);

    /* Exit detection precedes normal no-line handling. Its stable black source
     * is seeded only on a confirmed straight->arc handoff, not a single bit. */
    if (MotionEdge_Update(&s->edge,in,s->distance_mm,p->point_min_mm,p->point_max_mm,0U)) return Motion_Complete(ctx);
    float position=0.0f;
    uint32_t fault=Position(s,in,&position);
    uint8_t new_capture=!s->have_capture_sequence || in->line_sequence!=s->capture_sequence;
    if (new_capture) {
        s->capture_sequence=in->line_sequence; s->have_capture_sequence=1U;
        if (!fault && fabsf(position)<=MOTION_CAPTURE_CENTER_LIMIT) ++s->capture_frames;
        else s->capture_frames=0U;
        if (ctx->phase==PHASE_CAPTURE && s->capture_frames>=MOTION_CAPTURE_CONFIRM_FRAMES) ctx->phase=PHASE_FOLLOW;
    }
    if (fault) {
        if (!s->uncertain) {
            s->uncertain=1U; s->uncertain_ms=in->now_ms;
            /* Scale once per incident. At capture entry with no good position,
             * continue a bounded slow forward search rather than use zeros. */
            float l=s->left_target>0.0f ? s->left_target : p->capture_cps;
            float r=s->right_target>0.0f ? s->right_target : p->capture_cps;
            s->recovery_left=Motion_Clamp(l*MOTION_LINE_RECOVERY_FRACTION,MOTION_LINE_MIN_WHEEL_CPS,l);
            s->recovery_right=Motion_Clamp(r*MOTION_LINE_RECOVERY_FRACTION,MOTION_LINE_MIN_WHEEL_CPS,r);
        }
        if ((uint32_t)(in->now_ms-s->uncertain_ms)>=MOTION_LINE_UNCERTAIN_MS) return Motion_Fail(ctx,fault);
        return Motion_Output(ctx,MOTION_FORWARD,s->recovery_left,s->recovery_right);
    }
    uint8_t recovering=s->uncertain;
    if (recovering) {
        s->uncertain=0U; s->last_position=position; s->derivative=0.0f;
        s->control_ms=in->now_ms-MOTION_LINE_PERIOD_MS;
    }
    if ((!s->have_line_sequence || in->line_sequence!=s->line_sequence) &&
        (uint32_t)(in->now_ms-s->control_ms)>=MOTION_LINE_PERIOD_MS) {
        float dt=(float)(uint32_t)(in->now_ms-s->control_ms)*0.001f;
        if (s->have_position && !recovering) {
            float rate=(position-s->last_position)/dt;
            s->derivative+=dt/(MOTION_LINE_D_FILTER_S+dt)*(rate-s->derivative);
        }
        s->line_position=position; s->last_position=position; s->have_position=1U;
        s->line_sequence=in->line_sequence; s->have_line_sequence=1U; s->control_ms=in->now_ms;
        Targets(s,ctx->phase==PHASE_CAPTURE ? p->capture_cps : p->cruise_cps);
    }
    /* Capture may finish on a fresh frame before the PD period. Update the
     * cruise target immediately without differentiating the same frame twice. */
    if (ctx->phase==PHASE_FOLLOW && s->capture_frames==MOTION_CAPTURE_CONFIRM_FRAMES) Targets(s,p->cruise_cps);
    return Motion_Output(ctx,MOTION_FORWARD,s->left_target,s->right_target);
}
