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
const MissionSegment mission_segment_ac = {
    .id=SEGMENT_AC, .implemented=1U, .required_sensors=SENSOR_GYRO|SENSOR_LINE,
    .params=&params, .Enter=MotionStraight_Enter, .Update=MotionStraight_Update,
    .Exit=Motion_Exit, .Validate=Motion_ValidateParams
};
