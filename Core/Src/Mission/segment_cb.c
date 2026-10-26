#include "motion_control.h"

/* CB: independently tunable; shared controller owns no hardware. */
static const MissionSegmentParams params = {
    .kind=MOTION_KIND_ARC, .cruise_cps=MOTION_CB_CRUISE_CPS,
    .timeout_ms=MOTION_ARC_TIMEOUT_MS,
    .capture_cps=MOTION_CB_CAPTURE_CPS,
    .target_yaw_deg=180.0f,
    .nominal_mm=MOTION_CB_NOMINAL_MM,
    .point_min_mm=MOTION_CB_POINT_MIN_MM, .point_max_mm=MOTION_CB_POINT_MAX_MM
};
const MissionSegment mission_segment_cb = {
    .id=SEGMENT_CB, .implemented=1U, .required_sensors=SENSOR_GYRO|SENSOR_LINE,
    .params=&params, .Enter=MotionArc_Enter, .Update=MotionArc_Update,
    .Exit=Motion_Exit, .Validate=Motion_ValidateParams
};
