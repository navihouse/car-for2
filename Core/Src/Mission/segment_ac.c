#include "motion_control.h"

/* AC: independently tunable; shared controller owns no hardware. */
static const MissionSegmentParams params = {
    .kind=MOTION_KIND_STRAIGHT, .cruise_cps=MOTION_AC_CRUISE_CPS,
    .timeout_ms=MOTION_DRIVE_TIMEOUT_MS,
    .capture_cps=0.0f,
    .target_yaw_deg=MOTION_AC_TARGET_DEG,
    .nominal_mm=MOTION_AC_NOMINAL_MM,
    .point_min_mm=MOTION_AC_POINT_MIN_MM, .point_max_mm=MOTION_AC_POINT_MAX_MM
};
static uint8_t Validate(const MissionSegmentParams *p)
{
    if (!Motion_ValidateParams(p)) return 0U;
    MissionSegmentParams returning=*p;
    returning.target_yaw_deg=MOTION_AC_RETURN_TARGET_DEG;
    return Motion_ValidateParams(&returning);
}
static MissionSegmentOutput Enter(MissionSegmentContext *ctx, const MissionInput *in,
                                 const MissionSegmentParams *p)
{
    /* AC only receives a handoff when DA closes a non-final lap. Do not
     * re-zero the course at the probe crossing or at the end of braking. */
    if (!Validate(p)) return Motion_Fail(ctx,MOTION_FAULT_CONFIG);
    MissionSegmentParams selected=*p;
    if (ctx->from_handoff) selected.target_yaw_deg=MOTION_AC_RETURN_TARGET_DEG;
    return MotionStraight_Enter(ctx,in,&selected);
}
const MissionSegment mission_segment_ac = {
    .id=SEGMENT_AC, .implemented=1U, .required_sensors=SENSOR_GYRO|SENSOR_LINE,
    .params=&params, .Enter=Enter, .Update=MotionStraight_Update,
    .Exit=Motion_Exit, .Validate=Validate
};
