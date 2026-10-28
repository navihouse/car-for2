#include "motion_control.h"
#include <math.h>

/* Extracted from the old line_follow position/PD algorithm. Hardware, lap
 * counting and stop decisions belong to the mission, not this controller. */
static uint32_t Position(MissionMotionState *s, const MissionInput *in, uint8_t capturing, float *position)
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
    /* A diagonal entry may illuminate most of the bar. Permit only a
     * SINGLE contiguous wide stripe during bounded capture, never at cruise. */
    if (active>4U && (!capturing || groups!=1U)) return MOTION_FAULT_AMBIGUOUS;
    if (groups==1U) { *position=(float)weighted[0]/(float)weights[0]; return MOTION_FAULT_NONE; }
    if (!s->have_position || groups!=2U) return MOTION_FAULT_AMBIGUOUS;
    float p0=(float)weighted[0]/(float)weights[0], p1=(float)weighted[1]/(float)weights[1];
    float d0=fabsf(p0-s->last_position), d1=fabsf(p1-s->last_position);
    float nearest=fminf(d0,d1), other=fmaxf(d0,d1);
    if (nearest>MOTION_LINE_HISTORY_MAX || other-nearest<MOTION_LINE_GROUP_MARGIN) return MOTION_FAULT_AMBIGUOUS;
    *position=d0<d1 ? p0 : p1;
    return MOTION_FAULT_NONE;
}
static float CaptureHeadingError(const MissionSegmentContext *ctx, const MissionInput *in,
                                 const MissionSegmentParams *p)
{
    /* These descriptors are semicircles: entry tangent is 180 deg before exit.
     * CB: 0 deg, DA: 180 deg. Standalone tests retain their existing pose. */
    return Motion_WrapDegrees(p->target_yaw_deg-180.0f-Motion_Yaw(ctx,in));
}
static uint8_t NarrowStripe(uint32_t mask)
{
    uint32_t count=0U;
    for (uint32_t i=0;i<8U;i++) count+=(mask>>i)&1U;
    return count<=4U;
}
static void Targets(MissionMotionState *s, float cruise, float heading_trim, uint8_t following)
{
    float floor=cruise<MOTION_LINE_HIGH_SPEED_THRESHOLD_CPS ? MOTION_LINE_CORNER_BASE_CPS : MOTION_LINE_HIGH_BASE_FLOOR_CPS;
    float base=Motion_Clamp(cruise-fabsf(s->line_position)*MOTION_LINE_POSITION_SLOWDOWN,fminf(floor,cruise),cruise);
    float wheel_floor=cruise<MOTION_LINE_HIGH_SPEED_THRESHOLD_CPS ? MOTION_LINE_MIN_WHEEL_CPS : MOTION_LINE_HIGH_WHEEL_FLOOR_CPS;
    float cap=Motion_Clamp(base-wheel_floor,0.0f,MOTION_LINE_STEERING_MAX_CPS);
    /* Known curvature goes forward, disturbances stay with the PD. The
     * differential is baked in first and only the remainder is left as PD
     * authority, so the inner wheel can never drop below its floor and the
     * outer wheel can never cross MOTION_MAX_CPS. */
    float turn_ff=following ? MOTION_LINE_CURVATURE_FF*cruise : 0.0f;
    float authority=fmaxf(cap-fabsf(turn_ff),0.0f);
    float turn=turn_ff+Motion_Clamp(MOTION_LINE_KP*s->line_position+MOTION_LINE_KD*s->derivative-heading_trim,
        -authority,authority);
    base=fminf(base,(float)MOTION_MAX_CPS-fabsf(turn));
    s->left_target=base+turn; s->right_target=base-turn;
}
/* Capture steering for a rolling corner entry.
 *
 * Holding the entry tangent is not enough: every degree of the 38.66 deg
 * diagonal-to-semicircle error turns into lateral drift while the car crosses
 * the point (measured on the car: 96 mm off the line after 174 mm when the
 * capture only steered to the tangent, i.e. far outside the probe bar). So
 * instead of a frozen heading, track the semicircle the segment already knows:
 * dead-reckon the car in the entry frame and aim at the arc, correcting the
 * lateral offset with a lookahead. While the heading error is still large the
 * gain saturates so the corner is taken at the tightest radius the capture
 * speed allows - the smallest possible bulge - and only then does the path
 * tracking gain take over.
 * Returns the wheel differential in cps: positive steers LEFT (matching the
 * recovery branch's base -/+ trim convention). */
static float CaptureTrim(const MissionMotionState *s, const MissionSegmentParams *p,
                         float yaw_deg, float base)
{
    float radius=p->nominal_mm/3.14159265f;   /* 180 deg semicircle */
    if (!s->have_track || radius<1.0f) return 0.0f;
    float dx=s->track_x, dy=s->track_y-radius;
    float lateral=radius-sqrtf(dx*dx+dy*dy);  /* + = the line lies to the left */
    float progress=atan2f(s->track_x,radius-s->track_y)*57.29578f;
    float steer=atan2f(lateral,MOTION_CAPTURE_LOOKAHEAD_MM)*57.29578f;
    float error=Motion_WrapDegrees(s->track_origin_yaw_deg+progress-steer-yaw_deg);
    float gain=fabsf(error)>MOTION_CAPTURE_ACQUIRE_DEG ? MOTION_CAPTURE_ACQUIRE_KP : MOTION_CAPTURE_HEADING_KP;
    return -MOTION_LINE_CURVATURE_FF*base+gain*error;
}
/* Advance the dead-reckoned pose by the distance covered since the last frame. */
static void TrackAdvance(MissionMotionState *s, float yaw_deg)
{
    if (!s->have_track) return;
    float ds=s->distance_mm-s->track_mm;
    if (ds<=0.0f) return;
    float rel=(yaw_deg-s->track_origin_yaw_deg)*0.01745329f;
    s->track_x+=ds*cosf(rel);
    s->track_y+=ds*sinf(rel);
    s->track_mm=s->distance_mm;
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
    /* A diagonal arrival is NOT tangent to the semicircle: at C the car still
     * holds the AC heading while CB expects its own entry tangent. Stopping to
     * rotate would brake from cruise and start the search from a standstill,
     * exactly where stiction and skidding cost the point. The capture phase
     * therefore owns the 38.66 deg corner and steers it away while rolling. */
    if (ctx->from_handoff) {
        ctx->motion.target_yaw_deg=Motion_WrapDegrees(p->target_yaw_deg-180.0f);
        ctx->motion.heading_error_deg=Motion_WrapDegrees(ctx->motion.target_yaw_deg-Motion_Yaw(ctx,in));
        /* Entry frame for the rolling capture: +x along the semicircle tangent
         * at the point, +y to its left, origin at the car's position there. */
        ctx->motion.track_origin_yaw_deg=Motion_WrapDegrees(p->target_yaw_deg-180.0f);
        ctx->motion.track_x=0.0f; ctx->motion.track_y=0.0f; ctx->motion.track_mm=0.0f;
        ctx->motion.have_track=1U;
    }
    /* Give the incoming segment's targets in the SAME handoff snapshot. */
    return MotionArc_Update(ctx,in,p);
}
MissionSegmentOutput MotionArc_Update(MissionSegmentContext *ctx, const MissionInput *in, const MissionSegmentParams *p)
{
    MissionMotionState *s=&ctx->motion;
    s->distance_mm=MotionDistance_GetMm(&s->distance,in);
    TrackAdvance(s,Motion_Yaw(ctx,in));
    /* Guide C/D diagonal handoffs toward the entry tangent during capture by
     * biasing the keep-rolling targets; normal following retains line PD and
     * exit-heading diagnostics. The bias may use every cps the slow wheel can
     * give up without dropping below its own floor. */
    uint8_t align_capture=ctx->phase==PHASE_CAPTURE && ctx->from_handoff;
    float entry_error=CaptureHeadingError(ctx,in,p);
    float heading_cap=fminf(MOTION_CAPTURE_HEADING_MAX_CPS,
        fmaxf(p->capture_cps-MOTION_LINE_MIN_WHEEL_CPS,0.0f));
    float heading_trim=align_capture ? Motion_Clamp(MOTION_CAPTURE_HEADING_KP*entry_error,
        -heading_cap,heading_cap) : 0.0f;
    s->target_yaw_deg=align_capture ? Motion_WrapDegrees(p->target_yaw_deg-180.0f) : p->target_yaw_deg;
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
    uint32_t fault=Position(s,in,ctx->phase==PHASE_CAPTURE,&position);
    /* Seeing a usable narrow stripe and certifying FOLLOW are different jobs.
     * Steer from the stripe immediately, even while the entry-heading gate is
     * still waiting. Otherwise the tangent bias can fight a stripe moving to
     * the opposite side and sweep it off the bar before three frames arrive.
     * Wide point bars keep the existing entry-heading guidance. */
    uint8_t line_guided=align_capture && !fault && NarrowStripe(in->line_mask);
    uint8_t first_stripe=line_guided && !s->capture_line_tracking;
    s->capture_line_tracking=line_guided;
    if (line_guided) heading_trim=0.0f;
    uint8_t new_capture=!s->have_capture_sequence || in->line_sequence!=s->capture_sequence;
    if (new_capture) {
        s->capture_sequence=in->line_sequence; s->have_capture_sequence=1U;
        uint8_t aligned=!align_capture || fabsf(entry_error)<=MOTION_CAPTURE_HEADING_ACCEPT_DEG;
        if (!fault && NarrowStripe(in->line_mask) && aligned && fabsf(position)<=MOTION_CAPTURE_CENTER_LIMIT)
            ++s->capture_frames;
        else s->capture_frames=0U;
        if (ctx->phase==PHASE_CAPTURE && s->capture_frames>=MOTION_CAPTURE_CONFIRM_FRAMES) {
            ctx->phase=PHASE_FOLLOW;heading_trim=0.0f;
            s->target_yaw_deg=p->target_yaw_deg;
            s->heading_error_deg=Motion_WrapDegrees(s->target_yaw_deg-Motion_Yaw(ctx,in));
        }
    }
    if (fault) {
        if (!s->uncertain) {
            s->uncertain=1U; s->uncertain_ms=in->now_ms;
            if (ctx->phase==PHASE_CAPTURE) {
                /* 0.75 x an already slow capture target lands below the
                 * drivetrain's breakaway duty: the wheels never move, so the
                 * stall guard (or the capture timeout) ends the segment. Crawl
                 * at the capture speed and let the heading trim steer. */
                s->recovery_left=s->recovery_right=p->capture_cps;
            } else {
                /* Scale once per incident. After a lost line, continue a
                 * bounded slow forward search rather than use zeros. */
                float l=s->left_target>0.0f ? s->left_target : p->capture_cps;
                float r=s->right_target>0.0f ? s->right_target : p->capture_cps;
                s->recovery_left=Motion_Clamp(l*MOTION_LINE_RECOVERY_FRACTION,MOTION_LINE_MIN_WHEEL_CPS,l);
                s->recovery_right=Motion_Clamp(r*MOTION_LINE_RECOVERY_FRACTION,MOTION_LINE_MIN_WHEEL_CPS,r);
            }
        }
        uint32_t grace=ctx->phase==PHASE_CAPTURE ? MOTION_CAPTURE_UNCERTAIN_MS : MOTION_LINE_UNCERTAIN_MS;
        float base=0.5f*(s->recovery_left+s->recovery_right);
        float cap=fmaxf(0.0f,fminf(base-MOTION_LINE_MIN_WHEEL_CPS,(float)MOTION_MAX_CPS-base));
        /* A rolling corner entry has just crossed the point bar, so the only
         * black under the sensor is behind the car and the arc line is still off
         * to the side: losing it here is expected, not a fault. Keep steering
         * the known semicircle and let the capture time/distance budget bound
         * the search - that budget already reports MOTION_FAULT_CAPTURE. */
        if (align_capture) {
            s->uncertain_ms=in->now_ms;
            float steer=Motion_Clamp(CaptureTrim(s,p,Motion_Yaw(ctx,in),base),-cap,cap);
            return Motion_Output(ctx,MOTION_FORWARD,base-steer,base+steer);
        }
        if ((uint32_t)(in->now_ms-s->uncertain_ms)>=grace) return Motion_Fail(ctx,fault);
        /* Preserve positive forward motion while seeking the entry tangent.
         * The overall capture time/distance bounds remain in force above. */
        return Motion_Output(ctx,MOTION_FORWARD,s->recovery_left,s->recovery_right);
    }
    uint32_t line_period=line_guided ? MOTION_CAPTURE_LINE_PERIOD_MS : MOTION_LINE_PERIOD_MS;
    uint8_t recovering=s->uncertain || first_stripe;
    if (recovering) {
        s->uncertain=0U; s->last_position=position; s->derivative=0.0f;
        s->control_ms=in->now_ms-line_period;
    }
    if ((!s->have_line_sequence || in->line_sequence!=s->line_sequence) &&
        (uint32_t)(in->now_ms-s->control_ms)>=line_period) {
        float dt=(float)(uint32_t)(in->now_ms-s->control_ms)*0.001f;
        if (s->have_position && !recovering) {
            float rate=(position-s->last_position)/dt;
            s->derivative+=dt/(MOTION_LINE_D_FILTER_S+dt)*(rate-s->derivative);
        }
        s->line_position=position; s->last_position=position; s->have_position=1U;
        s->line_sequence=in->line_sequence; s->have_line_sequence=1U; s->control_ms=in->now_ms;
        Targets(s,ctx->phase==PHASE_CAPTURE ? p->capture_cps : p->cruise_cps,heading_trim,
            ctx->phase==PHASE_FOLLOW || line_guided);
    }
    /* Capture may finish on a fresh frame before the PD period. Update the
     * cruise target immediately without differentiating the same frame twice. */
    if (ctx->phase==PHASE_FOLLOW && s->capture_frames==MOTION_CAPTURE_CONFIRM_FRAMES)
        Targets(s,p->cruise_cps,0.0f,1U);
    return Motion_Output(ctx,MOTION_FORWARD,s->left_target,s->right_target);
}
