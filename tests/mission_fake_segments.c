#include <assert.h>
#include <string.h>
#include "mission_fake_segments.h"

uint32_t fake_enter_count[4], fake_exit_count[4], fake_update_count[4];
MissionSegmentStatus fake_segment_status[4];
MissionInput fake_entries[4];
void FakeSegments_Reset(void)
{
    memset(fake_enter_count,0,sizeof(fake_enter_count));
    memset(fake_exit_count,0,sizeof(fake_exit_count));
    memset(fake_update_count,0,sizeof(fake_update_count));
    for(uint32_t i=0;i<4U;++i) fake_segment_status[i]=SEG_RUNNING;
}
static MissionSegmentOutput Enter(MissionSegmentContext *ctx, const MissionInput *in, const MissionSegmentParams *p)
{
    uint32_t id=p->timeout_ms;
    assert(id<4U && ctx->active);
    ++fake_enter_count[id]; fake_entries[id]=*in; ctx->phase=id;
    return (MissionSegmentOutput){SEG_RUNNING,0U,p->cruise_cps,p->cruise_cps+20.0f,MOTION_FORWARD};
}
static MissionSegmentOutput Update(MissionSegmentContext *ctx, const MissionInput *in, const MissionSegmentParams *p)
{
    (void)ctx; (void)in;
    uint32_t id=p->timeout_ms; ++fake_update_count[id];
    return (MissionSegmentOutput){fake_segment_status[id],42U,p->cruise_cps,p->cruise_cps+20.0f,MOTION_FORWARD};
}
static void Exit(MissionSegmentContext *ctx, const MissionInput *in, MissionExitReason reason)
{
    (void)in; (void)reason;
    assert(ctx->phase<4U); ++fake_exit_count[ctx->phase];
}
static const MissionSegmentParams params[4]={
    {.cruise_cps=500.0f,.timeout_ms=0U},{.cruise_cps=700.0f,.timeout_ms=1U},
    {.cruise_cps=900.0f,.timeout_ms=2U},{.cruise_cps=600.0f,.timeout_ms=3U}};
const MissionSegment mission_segment_ac={SEGMENT_AC,1U,SENSOR_GYRO,&params[0],Enter,Update,Exit,NULL};
const MissionSegment mission_segment_cb={SEGMENT_CB,1U,SENSOR_GYRO|SENSOR_LINE,&params[1],Enter,Update,Exit,NULL};
const MissionSegment mission_segment_bd={SEGMENT_BD,1U,SENSOR_GYRO,&params[2],Enter,Update,Exit,NULL};
const MissionSegment mission_segment_da={SEGMENT_DA,1U,SENSOR_GYRO|SENSOR_LINE,&params[3],Enter,Update,Exit,NULL};
