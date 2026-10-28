#ifndef MISSION_CONFIG_H
#define MISSION_CONFIG_H
#include "motion_tuning.h"

/* CMake -DMISSION_MODE=FULL|AC|CB|BD|DA selects one firmware. */
#define MISSION_MODE_FULL 0
#define MISSION_MODE_AC   1
#define MISSION_MODE_CB   2
#define MISSION_MODE_BD   3
#define MISSION_MODE_DA   4
#ifndef MISSION_BUILD_MODE
#define MISSION_BUILD_MODE MISSION_MODE_FULL
#endif
#if MISSION_BUILD_MODE < MISSION_MODE_FULL || MISSION_BUILD_MODE > MISSION_MODE_DA
#error Invalid MISSION_BUILD_MODE
#endif

/* FULL route is one circuit A->C->B->D->A; the competition covers four of them.
 * Only FULL repeats: a single-segment build drives exactly one segment. */
#define MISSION_FULL_LAPS 4U
/* Keep the course frame anchored at the first A for the whole run. Re-zeroing
 * the origin at every A crossing re-anchored the frame to whatever pose the car
 * happened to hold when the finish stripe was confirmed - in the field that was
 * 21 deg off, so the next AC leg ran along a rotated diagonal and drove into
 * the CB arc. Accumulated gyro drift over four laps is only a few degrees
 * (0.04 dps), i.e. the smaller of the two errors. The per-lap residual is still
 * accumulated into mission_diag.drift_mdeg so the drift stays observable. */
#define MISSION_LAP_ORIGIN_RESET 0
#define MISSION_MAX_TARGET_CPS ((float)MOTION_MAX_CPS)
#define MISSION_GYRO_MAX_AGE_MS 30U
#define MISSION_LINE_MAX_AGE_MS 150U
#define MISSION_CAL_HOLD_MS 6000U
/* C10B/car-02-: PA15 active-high buzzer, PC2 active-low LED. */
#define MISSION_POINT_SIGNAL_MS 300U
#define MISSION_BUZZER_ACTIVE_HIGH 1
#endif
