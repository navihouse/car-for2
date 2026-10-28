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
/* A duty below this cannot overcome drivetrain stiction, so encoder silence is
 * expected rather than proof of a stall. The no-feedback guard only counts time
 * while the output pushes at least this hard. 25% is above the ~13% faded kick
 * and below the ~34% that actually moves the wheels on the bench. */
#define MOTION_STALL_MIN_PWM 25.0f
/* Static-friction assist for the rolling modes. The speed integral needs over a
 * second to lift the duty from feed-forward (~9% at 600 cps) to the ~34% that
 * actually breaks the wheels loose, and the no-feedback guard fires long before
 * that - which is how a straight that slowed to 600 cps stopped dead and then
 * reported SPEED_FAULT. When a wheel is asked for real speed but reads nothing,
 * add this fixed duty; it vanishes the moment the wheel turns. */
#define MOTION_BREAKAWAY_MIN_CPS 200.0f
#define MOTION_BREAKAWAY_STALL_CPS 60.0f
#define MOTION_BREAKAWAY_PWM 15.0f

/* 3. Gyro convention: +1 means raw positive yaw is a physical left turn.
 * The course reference is east/right = 0, north/up = +90, west/left = 180.
 * Probe order lists raw indices from VEHICLE LEFT to VEHICLE RIGHT. */
#define MOTION_YAW_SIGN 1.0f
#define MOTION_LINE_ORDER {0U,1U,2U,3U,4U,5U,6U,7U}

/* 4. Per-segment controls. AB=1000, AD=BC=800, radius=400 mm.
 * Angles are course headings, not an increment from a possibly wrong arrival.
 * Identification windows are inclusive. Nominal lengths do NOT end a segment. */
#define MOTION_AC_TARGET_DEG (-38.66f)
/* B点实测修正：CB 出弯后车在 B 停稳、原地左转到 BD 航向时，车头比黑线方向多偏了
 * 约 5°（目测，2026-10-28 第六次上板）。因此把 BD 的目标航向比标称 180+38.66 收回
 * 5°，即该处左转角从 38.66° 变为 33.66°。只影响 B 点这一次转向：
 * AC 的 -38.66°、CB/DA 的出弯切线、A 点转向都不动。若换赛道或换了转向参数，
 * 先用 mission_diag 的 Heading_error_deg 复核再调整这个偏移量。 */
#define MOTION_BD_CORNER_OFFSET_DEG (-5.0f)
#define MOTION_BD_TARGET_DEG (180.0f + 38.66f + MOTION_BD_CORNER_OFFSET_DEG)
#define MOTION_AC_CRUISE_CPS 2000.0f
#define MOTION_BD_CRUISE_CPS 2000.0f
#define MOTION_CB_CRUISE_CPS 1400.0f
#define MOTION_DA_CRUISE_CPS 1400.0f
#define MOTION_CB_CAPTURE_CPS 1000.0f
#define MOTION_DA_CAPTURE_CPS 1000.0f
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

/* 6. In-place turn: angle -> desired angular rate -> opposite wheel targets.
 * The turn time is dominated by the exponential tail of desired = KP*angle, not
 * by the rate ceiling: at KP 2.0 the 25 deg -> 3 deg tail alone takes 1.06 s.
 * KP 3.2 shortens it to 0.66 s and the 65 dps ceiling shortens the saturated
 * lead-in, which brings a 38.66 deg corner from ~1.35 s to ~0.9 s. Overshoot is
 * absorbed by MOTION_TURN_ACCEPT_DEG as long as the 0.04 s brake lead holds. */
#define MOTION_TURN_ANGLE_KP 3.2f
#define MOTION_TURN_RATE_FF 16.0f
#define MOTION_TURN_RATE_KP 6.0f
#define MOTION_TURN_RATE_MAX_DPS 65.0f
#define MOTION_TURN_MIN_CPS 350.0f
#define MOTION_TURN_MAX_CPS 1400U
/* Fine approach is separate from the coarse stiction/minimum-speed floor.
 * Existing cruise/rate/gain values above are left for the operator to tune. */
#define MOTION_TURN_APPROACH_DEG 12.0f
#define MOTION_TURN_APPROACH_MIN_CPS 120.0f
#define MOTION_TURN_BRAKE_LEAD_S 0.04f
#define MOTION_TURN_BRAKE_DEG 2.0f
#define MOTION_TURN_ACCEPT_DEG 3.0f
/* Breakaway escalation for the faded fine-approach friction kick. While a wheel
 * stays stalled the full 30% kick is restored; once the wheel turns it decays
 * again, so the low free-running effort (and low overshoot) is kept. */
#define MOTION_TURN_BREAKAWAY_MS 150U
#define MOTION_TURN_BOOST_RELEASE_MS 300U
/* A faded fine approach needs longer than a coarse turn to prove a wheel is
 * really dead, because the kick has to ramp in first. */
#define MOTION_TURN_NO_FEEDBACK_MS 700U
#define MOTION_STILL_RATE_DPS 3.0f
#define MOTION_STILL_WHEEL_CPS 60.0f
/* Both windows are pure waiting: the real proof of standstill is the rate and
 * wheel thresholds, sampled every 10 ms by the encoder. 200 ms is already 20
 * consecutive samples; the old 500 ms cost 0.6 s per corner (two windows) with
 * no extra safety. Raise these two if the car ever starts a turn still rolling. */
#define MOTION_STILL_MS 200U
#define MOTION_SETTLE_MS 200U
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
/* Curvature feed-forward for the semicircles. A pure PD has no integral term,
 * so the only way it can hold the differential a 400 mm arc needs is to keep a
 * steady line offset: turn = KP*pos with turn = 0.1625*cruise leaves
 * pos = 0.9*cruise probe units (~900 of 3500 at cruise 1000), which both costs
 * speed (base drops by 0.1*pos) and eats the steering authority. Feeding the
 * known curvature forward removes that offset and leaves the PD for
 * disturbances only.
 * Derivation: diff_cps = v/R * track * counts_per_mm = cruise * 130 / 400, and
 * turn = diff_cps/2, i.e. 0.1625 per unit cruise.
 * Sign: both arcs are left hand (CB sweeps 0 -> 180, DA 180 -> 360, yaw
 * increases), and a left turn is the branch where left = base + turn must be
 * the slower wheel, so the fraction is negative. Flip it if the arena is ever
 * driven clockwise. */
#define MOTION_LINE_CURVATURE_FF (-0.1625f)
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
/* During a rolling entry the stripe can sweep across the bar before the normal
 * 50 ms following period expires. React to fresh narrow-line frames at this
 * cadence; confirmation still requires three distinct frames and heading. */
#define MOTION_CAPTURE_LINE_PERIOD_MS 10U
/* Three captured frames prove visibility, not that the rolling corner has
 * settled. Keep capture speed and fast control until these conditions hold
 * continuously on fresh narrow-line frames, then permit normal arc cruise. */
#define MOTION_ENTRY_SETTLE_MS 200U
#define MOTION_ENTRY_CENTER_LIMIT 1000.0f
#define MOTION_ENTRY_RATE_MARGIN_DPS 10.0f
/* Wheel cps per deg/s of yaw above the nominal arc rate. Positive excess left
 * yaw adds rightward steering. Applies while acquiring/settling a rolling entry. */
#define MOTION_ENTRY_YAW_KD 3.0f
/* Accept the line anywhere inside the probe bar during capture. The old +-1000
 * window (one probe pitch either side of centre) rejected exactly the case the
 * rolling entry produces: the line enters at the edge of the bar, so the first
 * usable frames were thrown away and capture_frames stayed 0 while the car
 * drove past. Three fresh frames plus NarrowStripe are what make it safe. */
#define MOTION_CAPTURE_CENTER_LIMIT 3000.0f
/* The budget covers a corner taken at the handoff speed, not at cruise: the
 * bar behind the point is the only thing under the bar when the entry starts,
 * and the arc line only comes back into view after the car has travelled the
 * first ~80 mm of the corner. 2000 ms / 320 mm leaves that dropout plus the
 * 38.66 deg rotation inside the window. */
#define MOTION_CAPTURE_TIMEOUT_MS 2000U
#define MOTION_CAPTURE_MAX_MM 320.0f
/* Diagonal -> semicircle: roll through the 38.66 deg corner instead of stopping.
 * The capture keeps capture_cps on both wheels and steers with a differential
 * bias, so the wheels are already turning when the turn starts and stiction is
 * not in play. The effective bias is capped at capture_cps - MIN_WHEEL_CPS
 * (850 cps at 1000), which is ~111 dps and turns the corner in ~50 mm.
 * Acquisition gain: while the heading error is still wide the bias saturates,
 * because every degree left over becomes lateral drift away from the line (the
 * frozen-tangent version ended 96 mm off the line after 174 mm). Once inside
 * this band the path-tracking gain takes over and pulls the lateral offset in
 * through the MOTION_CAPTURE_LOOKAHEAD_MM lookahead. */
#define MOTION_CAPTURE_HEADING_KP 10.0f
#define MOTION_CAPTURE_ACQUIRE_DEG 8.0f
#define MOTION_CAPTURE_ACQUIRE_KP 40.0f
#define MOTION_CAPTURE_LOOKAHEAD_MM 150.0f
#define MOTION_CAPTURE_HEADING_MAX_CPS 650.0f
#define MOTION_CAPTURE_HEADING_ACCEPT_DEG 20.0f
#define MOTION_CAPTURE_UNCERTAIN_MS 350U
#define MOTION_LINE_UNCERTAIN_MS 100U
/* Straight segments hand the arc its own entry speed. A rolling 38.66 deg
 * corner is steered by wheel differential, and decelerating from cruise eats
 * exactly that differential (the PI saturates, both wheels coast) while the
 * corner is still owed - which is how a 2000 cps handoff ran 100 mm dead
 * straight and lost the line. Slow the last MOTION_POINT_APPROACH_MM before
 * the nominal point down to the handoff speed, and stay there until the point
 * is actually confirmed.
 * Kept at 1200 cps (178 mm/s) on purpose: 600 cps is below the duty the
 * drivetrain needs to keep rolling, so slowing that far stopped the wheels and
 * the speed guard then reported a stall on a perfectly healthy chassis. */
#define MOTION_STRAIGHT_HANDOFF_CPS 1200.0f
#define MOTION_POINT_APPROACH_MM 140.0f
#define MOTION_REVERSE_DISTANCE_FAULT_MM 30.0f
#endif
