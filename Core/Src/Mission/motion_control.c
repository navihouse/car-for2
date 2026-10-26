#include "motion_control.h"
#include <math.h>

float Motion_Clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
float Motion_WrapDegrees(float degrees)
{
    float result = fmodf(degrees, 360.0f);
    if (result > 180.0f) result -= 360.0f;
    if (result < -180.0f) result += 360.0f;
    return result;
}
float Motion_Yaw(const MissionSegmentContext *ctx, const MissionInput *in)
{
    return MOTION_YAW_SIGN * (in->yaw_deg - ctx->course_origin_deg);
}
uint8_t Motion_ValidateTuning(void)
{
    const float positive[] = {MOTION_LEFT_COUNTS_PER_MM,MOTION_RIGHT_COUNTS_PER_MM,
        MOTION_MAX_CPS,MOTION_TURN_MIN_CPS,MOTION_TURN_MAX_CPS,MOTION_TURN_RATE_MAX_DPS,
        MOTION_SETTLE_MS,MOTION_STILL_WHEEL_CPS,MOTION_STILL_RATE_DPS,MOTION_LINE_D_FILTER_S,
        MOTION_LINE_HISTORY_MAX,MOTION_LINE_GROUP_MARGIN,MOTION_LINE_MIN_WHEEL_CPS,
        MOTION_LINE_CORNER_BASE_CPS,MOTION_LINE_STEERING_MAX_CPS,MOTION_CAPTURE_CENTER_LIMIT,
        MOTION_CAPTURE_MAX_MM,MOTION_CAPTURE_TIMEOUT_MS,MOTION_LINE_UNCERTAIN_MS,
        MOTION_HEADING_PERIOD_MS,MOTION_LINE_PERIOD_MS,MOTION_TURN_TIMEOUT_MS,
        MOTION_HEADING_TRIM_MAX_CPS,MOTION_HEADING_FAULT_DEG,MOTION_TURN_BRAKE_DEG,
        MOTION_TURN_ACCEPT_DEG,MOTION_TURN_TRANSLATION_MAX_MM,MOTION_TURN_WHEEL_MISMATCH_MM,
        MOTION_TURN_GYRO_RESPONSE_MM,MOTION_REVERSE_DISTANCE_FAULT_MM,MOTION_TARGET_RAMP_CPS_S,
        MOTION_TURN_MIN_RESPONSE_DEG,MOTION_TURN_WRONG_DIRECTION_DEG,MOTION_LINE_HIGH_SPEED_THRESHOLD_CPS,
        MOTION_LINE_HIGH_BASE_FLOOR_CPS,MOTION_LINE_HIGH_WHEEL_FLOOR_CPS};
    const float gains[] = {MOTION_SPEED_KP,MOTION_SPEED_KI,MOTION_HEADING_KP,MOTION_HEADING_KD,
        MOTION_TURN_ANGLE_KP,MOTION_TURN_RATE_FF,MOTION_TURN_RATE_KP,MOTION_LINE_KP,MOTION_LINE_KD,
        MOTION_LINE_POSITION_SLOWDOWN};
    for (uint32_t i=0;i<sizeof(positive)/sizeof(positive[0]);++i)
        if (!isfinite(positive[i]) || positive[i]<=0.0f) return 0U;
    for (uint32_t i=0;i<sizeof(gains)/sizeof(gains[0]);++i)
        if (!isfinite(gains[i]) || gains[i]<0.0f) return 0U;
    if ((MOTION_YAW_SIGN!=1.0f && MOTION_YAW_SIGN!=-1.0f) || MOTION_POINT_CONFIRM_FRAMES<1U ||
        MOTION_CAPTURE_CONFIRM_FRAMES<1U || MOTION_TURN_MIN_CPS>MOTION_TURN_MAX_CPS ||
        MOTION_HEADING_TRIM_MAX_CPS>=(float)MOTION_MAX_CPS*0.5f ||
        MOTION_TURN_ACCEPT_DEG<MOTION_TURN_BRAKE_DEG ||
        !(MOTION_PWM_LIMIT>0.0f && MOTION_PWM_LIMIT<=100.0f) ||
        !(MOTION_SPEED_INTEGRAL_LIMIT>0.0f && isfinite(MOTION_SPEED_INTEGRAL_LIMIT)) ||
        !(MOTION_LINE_RECOVERY_FRACTION>0.0f && MOTION_LINE_RECOVERY_FRACTION<=1.0f) ||
        !(MOTION_TURN_FOLLOWING_MIN_FRACTION>0.0f && MOTION_TURN_FOLLOWING_MIN_FRACTION<=1.0f)) return 0U;
    static const uint8_t order[8]=MOTION_LINE_ORDER;
    uint32_t seen=0U;
    for (uint32_t i=0;i<8U;++i) {
        if (order[i]>=8U || (seen & (1U<<order[i]))) return 0U;
        seen |= 1U<<order[i];
    }
    return seen==255U;
}
uint8_t Motion_ValidateParams(const MissionSegmentParams *p)
{
    if (!p || !Motion_ValidateTuning() || !p->timeout_ms ||
        !(p->cruise_cps>0.0f && p->cruise_cps<=MOTION_MAX_CPS) ||
        !isfinite(p->point_min_mm) || !isfinite(p->point_max_mm) ||
        !(p->point_min_mm>=0.0f && p->point_max_mm>p->point_min_mm) ||
        !(p->nominal_mm>0.0f && isfinite(p->nominal_mm)) || !isfinite(p->target_yaw_deg)) return 0U;
    if (p->kind==MOTION_KIND_STRAIGHT)
        return p->cruise_cps>MOTION_HEADING_TRIM_MAX_CPS;
    if (p->kind==MOTION_KIND_ARC)
        return p->capture_cps>=MOTION_LINE_MIN_WHEEL_CPS && p->capture_cps<=p->cruise_cps &&
            p->cruise_cps>=MOTION_LINE_MIN_WHEEL_CPS;
    return 0U;
}
MissionSegmentOutput Motion_Output(MissionSegmentContext *ctx, MissionMotionMode mode, float left, float right)
{
    ctx->motion.left_target=left; ctx->motion.right_target=right;
    return (MissionSegmentOutput){.status=SEG_RUNNING,.left_cps=left,.right_cps=right,.mode=mode};
}
MissionSegmentOutput Motion_Fail(MissionSegmentContext *ctx, uint32_t fault)
{
    ctx->phase=PHASE_FAULT;
    ctx->motion.left_target=ctx->motion.right_target=0.0f;
    return (MissionSegmentOutput){.status=SEG_FAULT,.fault=fault,.mode=MOTION_BRAKE};
}
MissionSegmentOutput Motion_Complete(MissionSegmentContext *ctx)
{
    ctx->phase=PHASE_COMPLETE;
    return (MissionSegmentOutput){.status=SEG_COMPLETE,.mode=MOTION_BRAKE};
}
void Motion_Exit(MissionSegmentContext *ctx, const MissionInput *in, MissionExitReason reason)
{
    (void)in; (void)reason; ctx->active=0U;
}
