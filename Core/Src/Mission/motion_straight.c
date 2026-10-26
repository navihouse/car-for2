#include "motion_control.h"
#include <math.h>

static uint8_t Still(const MissionInput *in)
{
    return fabsf(in->yaw_rate_dps)<=MOTION_STILL_RATE_DPS &&
        fabsf(in->left_cps)<MOTION_STILL_WHEEL_CPS && fabsf(in->right_cps)<MOTION_STILL_WHEEL_CPS;
}
static uint8_t Settled(MissionMotionState *s, const MissionInput *in)
{
    if (!Still(in)) { s->still_tracking=0U; return 0U; }
    if (!s->still_tracking) { s->still_tracking=1U; s->still_since_ms=in->now_ms; }
    return (uint32_t)(in->now_ms-s->still_since_ms)>=MOTION_SETTLE_MS;
}
static void BeginTurn(MissionSegmentContext *ctx, const MissionInput *in)
{
    ctx->phase=PHASE_TURN; ctx->phase_started_ms=in->now_ms;
    ctx->motion.turn_origin_yaw=Motion_Yaw(ctx,in);
    ctx->motion.turn_start_error=ctx->motion.heading_error_deg;
    ctx->motion.control_ms=in->now_ms-MOTION_HEADING_PERIOD_MS;
    MotionDistance_Begin(&ctx->motion.turn_distance,in);
}
static MissionSegmentOutput Turn(MissionSegmentContext *ctx, const MissionInput *in)
{
    MissionMotionState *s=&ctx->motion;
    float left=(float)(in->left_counts-s->turn_distance.left_origin)/MOTION_LEFT_COUNTS_PER_MM;
    float right=(float)(in->right_counts-s->turn_distance.right_origin)/MOTION_RIGHT_COUNTS_PER_MM;
    uint32_t elapsed=in->now_ms-ctx->phase_started_ms;
    if (fabsf((left+right)*0.5f)>MOTION_TURN_TRANSLATION_MAX_MM)
        return Motion_Fail(ctx,MOTION_FAULT_TURN_TRANSLATION);
    if (elapsed>MOTION_TURN_RESPONSE_GRACE_MS) {
        float leading=fmaxf(fabsf(left),fabsf(right)), following=fminf(fabsf(left),fabsf(right));
        if (leading>MOTION_TURN_WHEEL_MISMATCH_MM && following<leading*MOTION_TURN_FOLLOWING_MIN_FRACTION)
            return Motion_Fail(ctx,MOTION_FAULT_WHEEL_SYNC);
        if (fabsf(left)+fabsf(right)>MOTION_TURN_GYRO_RESPONSE_MM &&
            fabsf(Motion_Yaw(ctx,in)-s->turn_origin_yaw)<MOTION_TURN_MIN_RESPONSE_DEG)
            return Motion_Fail(ctx,MOTION_FAULT_GYRO_RESPONSE);
    }
    if (elapsed>MOTION_TURN_DIRECTION_GRACE_MS && s->turn_start_error*(Motion_Yaw(ctx,in)-s->turn_origin_yaw)<
        -fabsf(s->turn_start_error)*MOTION_TURN_WRONG_DIRECTION_DEG)
        return Motion_Fail(ctx,MOTION_FAULT_TURN_DIRECTION);
    if (fabsf(s->heading_error_deg)<=MOTION_TURN_BRAKE_DEG || s->turn_start_error*s->heading_error_deg<=0.0f) {
        ctx->phase=PHASE_SETTLE; ctx->phase_started_ms=in->now_ms; s->still_tracking=0U;
        return Motion_Output(ctx,MOTION_BRAKE,0.0f,0.0f);
    }
    if ((uint32_t)(in->now_ms-s->control_ms)>=MOTION_HEADING_PERIOD_MS) {
        float desired=Motion_Clamp(MOTION_TURN_ANGLE_KP*s->heading_error_deg,-MOTION_TURN_RATE_MAX_DPS,MOTION_TURN_RATE_MAX_DPS);
        float command=MOTION_TURN_RATE_FF*desired+MOTION_TURN_RATE_KP*(desired-MOTION_YAW_SIGN*in->yaw_rate_dps);
        command=s->heading_error_deg>0.0f ? Motion_Clamp(command,MOTION_TURN_MIN_CPS,MOTION_TURN_MAX_CPS) :
            Motion_Clamp(command,-(float)MOTION_TURN_MAX_CPS,-MOTION_TURN_MIN_CPS);
        s->left_target=-command; s->right_target=command; s->control_ms=in->now_ms;
    }
    return Motion_Output(ctx,MOTION_TURN,s->left_target,s->right_target);
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
        if (!Settled(s,in)) return Motion_Output(ctx,MOTION_BRAKE,0.0f,0.0f);
        if (fabsf(s->heading_error_deg)>MOTION_TURN_BRAKE_DEG) {
            BeginTurn(ctx,in); return Turn(ctx,in);
        }
        ctx->phase=PHASE_SETTLE; s->still_tracking=0U;
        return Motion_Output(ctx,MOTION_BRAKE,0.0f,0.0f);
    }
    if (ctx->phase==PHASE_TURN) return Turn(ctx,in);
    if (ctx->phase==PHASE_SETTLE) {
        if (!Settled(s,in)) return Motion_Output(ctx,MOTION_BRAKE,0.0f,0.0f);
        if (fabsf(s->heading_error_deg)>MOTION_TURN_ACCEPT_DEG) {
            if (s->corrections>=MOTION_TURN_MAX_CORRECTIONS) return Motion_Fail(ctx,MOTION_FAULT_TURN_TIMEOUT);
            ++s->corrections; BeginTurn(ctx,in); return Turn(ctx,in);
        }
        /* The straight trip begins HERE, after all braking/rotation. */
        ctx->phase=PHASE_STRAIGHT; ctx->phase_started_ms=in->now_ms;
        MotionDistance_Begin(&s->distance,in); MotionEdge_Reset(&s->edge);
        s->control_ms=in->now_ms-MOTION_HEADING_PERIOD_MS;
    }
    if (ctx->phase!=PHASE_STRAIGHT) return Motion_Fail(ctx,MOTION_FAULT_CONFIG);
    s->distance_mm=MotionDistance_GetMm(&s->distance,in);
    if (s->distance_mm>p->point_max_mm || s->distance_mm < -MOTION_REVERSE_DISTANCE_FAULT_MM)
        return Motion_Fail(ctx,MOTION_FAULT_DISTANCE);
    if ((uint32_t)(in->now_ms-ctx->phase_started_ms)>p->timeout_ms) return Motion_Fail(ctx,MOTION_FAULT_TIMEOUT);
    if (fabsf(s->heading_error_deg)>MOTION_HEADING_FAULT_DEG) return Motion_Fail(ctx,MOTION_FAULT_HEADING);
    if (MotionEdge_Update(&s->edge,in,s->distance_mm,p->point_min_mm,p->point_max_mm,1U)) return Motion_Complete(ctx);
    if ((uint32_t)(in->now_ms-s->control_ms)>=MOTION_HEADING_PERIOD_MS) {
        float trim=Motion_Clamp(MOTION_HEADING_KP*s->heading_error_deg-MOTION_HEADING_KD*MOTION_YAW_SIGN*in->yaw_rate_dps,
            -MOTION_HEADING_TRIM_MAX_CPS,MOTION_HEADING_TRIM_MAX_CPS);
        s->left_target=p->cruise_cps-trim; s->right_target=p->cruise_cps+trim;
        /* Leave differential headroom if a user increases cruise speed. */
        float excess=fmaxf(s->left_target,s->right_target)-MOTION_MAX_CPS;
        if (excess>0.0f) { s->left_target-=excess; s->right_target-=excess; }
        s->control_ms=in->now_ms;
    }
    return Motion_Output(ctx,MOTION_FORWARD,s->left_target,s->right_target);
}
