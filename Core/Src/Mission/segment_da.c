#include "motion_control.h"

/* DA: independently tunable; shared controller owns no hardware. */
static const MissionSegmentParams params = {
    .kind=MOTION_KIND_ARC, .cruise_cps=MOTION_DA_CRUISE_CPS,
    .timeout_ms=MOTION_ARC_TIMEOUT_MS,
    .capture_cps=MOTION_DA_CAPTURE_CPS,
    .target_yaw_deg=0.0f,
    .nominal_mm=MOTION_DA_NOMINAL_MM,
    .point_min_mm=MOTION_DA_POINT_MIN_MM, .point_max_mm=MOTION_DA_POINT_MAX_MM
};
const MissionSegment mission_segment_da = {
    .id=SEGMENT_DA, .implemented=1U, .required_sensors=SENSOR_GYRO|SENSOR_LINE,
    .params=&params, .Enter=MotionArc_Enter, .Update=MotionArc_Update,
    .Exit=Motion_Exit, .Validate=Motion_ValidateParams
};
