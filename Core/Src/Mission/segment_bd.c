#include "motion_control.h"

/* BD: independently tunable; shared controller owns no hardware. */
static const MissionSegmentParams params = {
    .kind=MOTION_KIND_STRAIGHT, .cruise_cps=MOTION_BD_CRUISE_CPS,
    .timeout_ms=MOTION_DRIVE_TIMEOUT_MS,
    .capture_cps=0.0f,
    .target_yaw_deg=MOTION_BD_TARGET_DEG,
    .nominal_mm=MOTION_BD_NOMINAL_MM,
    .point_min_mm=MOTION_BD_POINT_MIN_MM, .point_max_mm=MOTION_BD_POINT_MAX_MM
};
const MissionSegment mission_segment_bd = {
    .id=SEGMENT_BD, .implemented=1U, .required_sensors=SENSOR_GYRO|SENSOR_LINE,
    .params=&params, .Enter=MotionStraight_Enter, .Update=MotionStraight_Update,
    .Exit=Motion_Exit, .Validate=Motion_ValidateParams
};
