#include "motion_control.h"
#include <math.h>

/* Shared in-place rotation. The straight segments use it before cruise; the arc
 * segments use it to align with the entry tangent after a diagonal handoff.
 * The caller passes the phase name, so each segment keeps its own state machine
 * and diagnostics, while the angle->rate->wheel law stays in ONE place. */
void MotionTurn_Begin(MissionSegmentContext *ctx, const MissionInput *in, uint32_t phase)
{
    ctx->phase=phase; ctx->phase_started_ms=in->now_ms;
    ctx->motion.turn_origin_yaw=Motion_Yaw(ctx,in);
    ctx->motion.turn_start_error=ctx->motion.heading_error_deg;
    ctx->motion.control_ms=in->now_ms-MOTION_HEADING_PERIOD_MS;
    MotionDistance_Begin(&ctx->motion.turn_distance,in);
}

MissionSegmentOutput MotionTurn_Update(MissionSegmentContext *ctx, const MissionInput *in)
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
    /* Anticipate the short braking delay only while rotating TOWARDS the
     * target. Cap the lead inside the accepted angle, avoiding an early stop
     * far outside tolerance and an unnecessary extra correction cycle. */
    float toward_rate=s->heading_error_deg>0.0f ? MOTION_YAW_SIGN*in->yaw_rate_dps :
        -MOTION_YAW_SIGN*in->yaw_rate_dps;
    float brake_deg=Motion_Clamp(MOTION_TURN_BRAKE_DEG+
        fmaxf(toward_rate,0.0f)*MOTION_TURN_BRAKE_LEAD_S,
        MOTION_TURN_BRAKE_DEG,MOTION_TURN_ACCEPT_DEG);
    if (fabsf(s->heading_error_deg)<=brake_deg || s->turn_start_error*s->heading_error_deg<=0.0f) {
        ctx->phase=PHASE_SETTLE; ctx->phase_started_ms=in->now_ms; s->still_tracking=0U;
        return Motion_Output(ctx,MOTION_BRAKE,0.0f,0.0f);
    }
    if ((uint32_t)(in->now_ms-s->control_ms)>=MOTION_HEADING_PERIOD_MS) {
        float desired=Motion_Clamp(MOTION_TURN_ANGLE_KP*s->heading_error_deg,-MOTION_TURN_RATE_MAX_DPS,MOTION_TURN_RATE_MAX_DPS);
        float command=MOTION_TURN_RATE_FF*desired+MOTION_TURN_RATE_KP*(desired-MOTION_YAW_SIGN*in->yaw_rate_dps);
        /* Fade the coarse floor near the target. The old fixed 350 cps
         * floor overrode deceleration even when only 2-3 degrees remained. */
        float minimum=Motion_Clamp(MOTION_TURN_MIN_CPS*fabsf(s->heading_error_deg)/MOTION_TURN_APPROACH_DEG,
            MOTION_TURN_APPROACH_MIN_CPS,MOTION_TURN_MIN_CPS);
        command=s->heading_error_deg>0.0f ? Motion_Clamp(command,minimum,MOTION_TURN_MAX_CPS) :
            Motion_Clamp(command,-(float)MOTION_TURN_MAX_CPS,-minimum);
        s->left_target=-command; s->right_target=command; s->control_ms=in->now_ms;
    }
    return Motion_Output(ctx,MOTION_TURN,s->left_target,s->right_target);
}
