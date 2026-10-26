#ifndef MOTION_TUNING_H
#define MOTION_TUNING_H

/* All physical defaults below are FIRST-TRIAL values, not new car calibration.
 * cps = normalized encoder counts/second; mm = ground distance; deg = degrees.
 * Higher gains/speeds can cause oscillation or missed lines: change one group
 * at a time and use the distance/heading/phase diagnostics to check the result. */

/* 1. Wheel calibration: increase counts/mm to reduce displayed travel. */
#define MOTION_LEFT_COUNTS_PER_MM  6.730141612f
#define MOTION_RIGHT_COUNTS_PER_MM 6.730141612f
#define MOTION_ENCODER_PERIOD_MS 10U

/* 2. Actuator limits and speed PI. Feed-forward remains the measured old fit. */
#define MOTION_MAX_CPS 3200U
#define MOTION_SPEED_KP 0.010f
#define MOTION_SPEED_KI 0.030f
#define MOTION_PWM_LIMIT 65.0f
#define MOTION_SPEED_INTEGRAL_LIMIT 20.0f
#define MOTION_TARGET_RAMP_CPS_S 5000.0f

/* 3. Gyro convention: +1 means raw positive yaw is a physical left turn.
 * The course reference is east/right = 0, north/up = +90, west/left = 180.
 * Probe order lists raw indices from VEHICLE LEFT to VEHICLE RIGHT. */
#define MOTION_YAW_SIGN 1.0f
#define MOTION_LINE_ORDER {0U,1U,2U,3U,4U,5U,6U,7U}

/* 4. Per-segment controls. AB=1000, AD=BC=800, radius=400 mm.
 * Angles are course headings, not an increment from a possibly wrong arrival.
 * Identification windows are inclusive. Nominal lengths do NOT end a segment. */
#define MOTION_AC_TARGET_DEG (-38.66f)
#define MOTION_BD_TARGET_DEG (180.0f + 38.66f)
#define MOTION_AC_CRUISE_CPS 900.0f
#define MOTION_BD_CRUISE_CPS 900.0f
#define MOTION_CB_CRUISE_CPS 900.0f
#define MOTION_DA_CRUISE_CPS 900.0f
#define MOTION_CB_CAPTURE_CPS 600.0f
#define MOTION_DA_CAPTURE_CPS 600.0f
#define MOTION_AC_NOMINAL_MM 1281.0f
#define MOTION_BD_NOMINAL_MM 1281.0f
#define MOTION_CB_NOMINAL_MM 1257.0f
#define MOTION_DA_NOMINAL_MM 1257.0f
#define MOTION_AC_POINT_MIN_MM 1000.0f
#define MOTION_AC_POINT_MAX_MM 1550.0f
#define MOTION_BD_POINT_MIN_MM 1000.0f
#define MOTION_BD_POINT_MAX_MM 1550.0f
#define MOTION_CB_POINT_MIN_MM 1000.0f
#define MOTION_CB_POINT_MAX_MM 1500.0f
#define MOTION_DA_POINT_MIN_MM 1000.0f
#define MOTION_DA_POINT_MAX_MM 1500.0f
#define MOTION_DRIVE_TIMEOUT_MS 30000U
#define MOTION_ARC_TIMEOUT_MS 30000U

/* 5. Heading PD: stronger Kp corrects angle sooner; Kd damps yaw rate. */
#define MOTION_HEADING_KP 10.0f
#define MOTION_HEADING_KD 3.0f
#define MOTION_HEADING_TRIM_MAX_CPS 150.0f
#define MOTION_HEADING_PERIOD_MS 20U
#define MOTION_HEADING_FAULT_DEG 15.0f

/* 6. In-place turn: angle -> desired angular rate -> opposite wheel targets. */
#define MOTION_TURN_ANGLE_KP 2.0f
#define MOTION_TURN_RATE_FF 16.0f
#define MOTION_TURN_RATE_KP 6.0f
#define MOTION_TURN_RATE_MAX_DPS 50.0f
#define MOTION_TURN_MIN_CPS 350.0f
#define MOTION_TURN_MAX_CPS 1200U
#define MOTION_TURN_BRAKE_DEG 2.0f
#define MOTION_TURN_ACCEPT_DEG 3.0f
#define MOTION_STILL_RATE_DPS 3.0f
#define MOTION_STILL_WHEEL_CPS 60.0f
#define MOTION_SETTLE_MS 500U
#define MOTION_TURN_MAX_CORRECTIONS 3U
#define MOTION_TURN_TIMEOUT_MS 25000U
#define MOTION_TURN_TRANSLATION_MAX_MM 80.0f
#define MOTION_TURN_WHEEL_MISMATCH_MM 20.0f
#define MOTION_TURN_GYRO_RESPONSE_MM 40.0f
#define MOTION_TURN_RESPONSE_GRACE_MS 500U
#define MOTION_TURN_DIRECTION_GRACE_MS 300U
#define MOTION_TURN_MIN_RESPONSE_DEG 1.0f
#define MOTION_TURN_WRONG_DIRECTION_DEG 5.0f
#define MOTION_TURN_FOLLOWING_MIN_FRACTION 0.15f

/* 7. Old line-position PD, in probe-position units (-3500 left, +3500 right).
 * Preserve the old speed shaping and clear-group selection. */
#define MOTION_LINE_KP 0.18f
#define MOTION_LINE_KD 0.006f
#define MOTION_LINE_D_FILTER_S 0.060f
#define MOTION_LINE_PERIOD_MS 50U
#define MOTION_LINE_STEERING_MAX_CPS 650.0f
#define MOTION_LINE_CORNER_BASE_CPS 300.0f
#define MOTION_LINE_MIN_WHEEL_CPS 150.0f
#define MOTION_LINE_HISTORY_MAX 1000.0f
#define MOTION_LINE_GROUP_MARGIN 1500.0f
#define MOTION_LINE_POSITION_SLOWDOWN 0.10f
#define MOTION_LINE_RECOVERY_FRACTION 0.75f
#define MOTION_LINE_HIGH_SPEED_THRESHOLD_CPS 900.0f
#define MOTION_LINE_HIGH_BASE_FLOOR_CPS 900.0f
#define MOTION_LINE_HIGH_WHEEL_FLOOR_CPS 450.0f

/* 8. Capture and point confirmation: count NEW frames, not main-loop polls. */
#define MOTION_POINT_CONFIRM_FRAMES 3U
#define MOTION_CAPTURE_CONFIRM_FRAMES 3U
#define MOTION_CAPTURE_CENTER_LIMIT 1000.0f
#define MOTION_CAPTURE_TIMEOUT_MS 1500U
#define MOTION_CAPTURE_MAX_MM 200.0f
#define MOTION_LINE_UNCERTAIN_MS 100U
#define MOTION_REVERSE_DISTANCE_FAULT_MM 30.0f
#endif
