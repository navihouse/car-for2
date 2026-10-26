#include "mission_segment.h"

/* CB: tune this segment here when its control algorithm is added.
 * Entry/Update must only compute targets; Exit must never stop hardware. */
static const MissionSegmentParams params = { .cruise_cps = 0.0f, .timeout_ms = 0U };
static MissionSegmentOutput Enter(MissionSegmentContext *ctx, const MissionInput *in,
                                  const MissionSegmentParams *config)
{
    (void)ctx; (void)in; (void)config;
    return (MissionSegmentOutput){ .status = SEG_UNIMPLEMENTED };
}
static MissionSegmentOutput Update(MissionSegmentContext *ctx, const MissionInput *in,
                                   const MissionSegmentParams *config)
{
    (void)ctx; (void)in; (void)config;
    return (MissionSegmentOutput){ .status = SEG_UNIMPLEMENTED };
}
static void Exit(MissionSegmentContext *ctx, const MissionInput *in, MissionExitReason reason)
{
    (void)in; (void)reason;
    ctx->active = 0U;
}
const MissionSegment mission_segment_cb = {
    .id = SEGMENT_CB, .implemented = 0U, .required_sensors = SENSOR_GYRO | SENSOR_LINE,
    .params = &params, .Enter = Enter, .Update = Update, .Exit = Exit
};
