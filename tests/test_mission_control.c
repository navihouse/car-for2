#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "mission_control.h"

static uint32_t enters, exits, updates;
static MissionSegmentStatus enter_status, update_status;
static float target;
static float right_delta;
static MissionMotionMode fake_mode;
static MissionExitReason exit_reason;
static MissionInput seen_entry;
static MissionSegmentOutput FakeEnter(MissionSegmentContext *ctx, const MissionInput *in, const MissionSegmentParams *p)
{
    (void)p;
    assert(ctx->active && ctx->entry.left_counts == in->left_counts);
    ++enters; seen_entry = *in;
    return (MissionSegmentOutput){enter_status, 17U, target, target + right_delta, fake_mode};
}
static MissionSegmentOutput FakeUpdate(MissionSegmentContext *ctx, const MissionInput *in, const MissionSegmentParams *p)
{
    (void)ctx; (void)in; (void)p; ++updates;
    return (MissionSegmentOutput){update_status, 17U, target, target + right_delta, fake_mode};
}
static void FakeExit(MissionSegmentContext *ctx, const MissionInput *in, MissionExitReason reason)
{
    (void)ctx; (void)in; ++exits; exit_reason = reason;
}
static MissionInput Fresh(uint32_t now)
{
    return (MissionInput){.now_ms=now, .encoder_sequence=1U, .left_counts=123456,
        .right_counts=654321, .yaw_deg=37.0f, .gyro_ready=1U, .line_sequence=1U, .line_calibrated=1U};
}
static void Init(MissionControl *m, MissionSegment segments[4], const MissionSegment **table, uint32_t mode)
{
    static const MissionSegmentParams params = {.cruise_cps=500.0f, .timeout_ms=10000U};
    enters = exits = updates = 0U; target=500.0f;
    right_delta=20.0f; fake_mode=MOTION_FORWARD;
    enter_status=update_status=SEG_RUNNING;
    for (uint32_t i=0; i<4U; ++i) {
        segments[i]=(MissionSegment){(MissionSegmentId)i,1U,SENSOR_GYRO|SENSOR_LINE,
            &params,FakeEnter,FakeUpdate,FakeExit,NULL}; table[i]=&segments[i];
    }
    MissionControl_Init(m,mode,table);
    /* The fake route is single-lap; the FULL lap roll-over has its own case. */
    m->laps_override=1U;
}
int main(void)
{
    MissionControl m; MissionSegment s[4]; const MissionSegment *table[4];
    MissionInput in=Fresh(UINT32_MAX-10U);
    Init(&m,s,table,MISSION_MODE_FULL);
    MissionOutput out=MissionControl_Start(&m,&in);
    assert(out.action==DRIVE_BEGIN && out.point==POINT_NONE && enters==1U);
    assert(MissionControl_Start(&m,&in).action==DRIVE_HOLD && enters==1U);
    update_status=SEG_COMPLETE;
    const MissionPoint expected[]={POINT_C,POINT_B,POINT_D,POINT_A};
    for (uint32_t i=0;i<4U;++i) {
        in.now_ms+=20U; in.left_counts+=1000; in.right_counts+=900; in.yaw_deg+=15.0f;
        MissionInput before=in;
        out=MissionControl_Update(&m,&in);
        assert(!memcmp(&before,&in,sizeof(in)));
        assert(out.point==expected[i] && m.arrivals==i+1U && exits==i+1U);
        if(i<3U) {
            assert(out.action==DRIVE_TARGET && out.left_cps==500.0f);
            assert(m.segment==i+1U && enters==i+2U && m.state==MISSION_RUNNING);
            assert(seen_entry.now_ms==in.now_ms && seen_entry.left_counts==in.left_counts);
            assert(seen_entry.yaw_deg==in.yaw_deg && exit_reason==EXIT_COMPLETE);
        } else assert(out.action==DRIVE_STOP && m.state==MISSION_DONE);
    }
    assert(MissionControl_Update(&m,&in).point==POINT_NONE && exits==4U);
    assert((uint32_t)(m.ended_ms-m.started_ms)==80U); /* millis rollover */

    /* FULL repeats the four segments until the configured lap total. */
    Init(&m,s,table,MISSION_MODE_FULL); m.laps_override=2U;
    in=Fresh(0U); out=MissionControl_Start(&m,&in);
    assert(out.action==DRIVE_BEGIN && m.laps==2U && !m.lap && !m.arrivals);
    update_status=SEG_COMPLETE;
    for (uint32_t i=0U;i<3U;++i) { in.now_ms+=20U; (void)MissionControl_Update(&m,&in); }
    /* The course frame stays anchored at the first A: the roll-over only banks
     * the residual as drift evidence, so the next lap's AC turn self-corrects. */
    in.now_ms+=20U; in.yaw_deg+=5.0f; (void)MissionControl_Update(&m,&in);
    assert(m.state==MISSION_RUNNING && m.lap==1U && m.segment==SEGMENT_AC);
    assert(m.arrivals==4U && enters==5U);
    assert(m.course_origin_deg==Fresh(0U).yaw_deg);
    assert(m.drift_mdeg==(int32_t)(MOTION_YAW_SIGN*5.0f*1000.0f));
    for (uint32_t i=0U;i<3U;++i) { in.now_ms+=20U; (void)MissionControl_Update(&m,&in); }
    in.now_ms+=20U; out=MissionControl_Update(&m,&in);
    assert(out.action==DRIVE_STOP && m.state==MISSION_DONE && m.arrivals==8U && m.lap==1U);
    assert(MissionControl_Update(&m,&in).action==DRIVE_HOLD && exits==8U);

    for(uint32_t mode=MISSION_MODE_AC;mode<=MISSION_MODE_DA;++mode) {
        Init(&m,s,table,mode); out=MissionControl_Start(&m,&in);
        assert(out.action==DRIVE_BEGIN && m.segment==mode-1U);
        update_status=SEG_COMPLETE; out=MissionControl_Update(&m,&in);
        assert(out.action==DRIVE_STOP && out.point==expected[mode-1U] && enters==1U);
    }
    /* Any unimplemented leg blocks the entire route BEFORE Enter. */
    for(uint32_t i=0;i<4U;++i) {
        Init(&m,s,table,MISSION_MODE_FULL); s[i].implemented=0U;
        out=MissionControl_Start(&m,&in);
        assert(out.action==DRIVE_STOP && m.state==MISSION_REJECTED);
        assert(m.reason==MISSION_NOT_IMPLEMENTED && m.segment==i && !enters && !m.arrivals);
    }
    Init(&m,s,table,99U);
    assert(MissionControl_Start(&m,&in).action==DRIVE_STOP && m.reason==MISSION_BAD_CONFIG);
    Init(&m,s,table,0U); s[2].id=SEGMENT_AC;
    assert(MissionControl_Start(&m,&in).action==DRIVE_STOP && m.reason==MISSION_BAD_CONFIG);
    Init(&m,s,table,0U); s[0].Enter=NULL;
    assert(MissionControl_Start(&m,&in).action==DRIVE_STOP && !enters);

    /* Enter cannot claim an arrival or issue invalid wheel commands. */
    const float bad[]={0.0f,-1.0f,3201.0f,NAN,INFINITY};
    for(uint32_t i=0;i<sizeof(bad)/sizeof(bad[0]);++i) {
        Init(&m,s,table,0U); target=bad[i]; out=MissionControl_Start(&m,&in);
        assert(out.action==DRIVE_STOP && m.reason==MISSION_BAD_TARGET && exits==1U);
    }
    for(uint32_t status=SEG_COMPLETE;status<=SEG_UNIMPLEMENTED;++status) {
        Init(&m,s,table,0U); enter_status=(MissionSegmentStatus)status;
        out=MissionControl_Start(&m,&in);
        assert(out.action==DRIVE_STOP && m.state==MISSION_REJECTED && !m.arrivals);
    }
    Init(&m,s,table,0U); (void)MissionControl_Start(&m,&in);
    out=MissionControl_Stop(&m,&in);
    assert(out.action==DRIVE_STOP && exit_reason==EXIT_STOP && exits==1U);
    uint32_t stopped_at=m.ended_ms; in.now_ms+=50U;
    (void)MissionControl_Stop(&m,&in); (void)MissionControl_Update(&m,&in);
    assert(exits==1U && !m.arrivals && m.ended_ms==stopped_at);

    for(uint32_t fault=0;fault<5U;++fault) {
        Init(&m,s,table,0U); in=Fresh(100U); (void)MissionControl_Start(&m,&in);
        if(fault==0U) in.gyro_age_ms=31U;
        if(fault==1U) in.line_age_ms=151U;
        if(fault==2U) in.encoder_age_ms=101U;
        if(fault==3U) in.speed_fault=3U;
        if(fault==4U) in.calibrating=1U;
        out=MissionControl_Update(&m,&in);
        assert(out.action==DRIVE_STOP && m.state==MISSION_FAULT && exits==1U);
        assert(exit_reason==EXIT_FAULT && !m.arrivals);
        assert(MissionControl_Update(&m,&in).action==DRIVE_HOLD && exits==1U);
    }
    in=Fresh(200U); Init(&m,s,table,0U); (void)MissionControl_Start(&m,&in);
    update_status=SEG_FAULT; out=MissionControl_Update(&m,&in);
    assert(out.action==DRIVE_STOP && m.reason==MISSION_SEGMENT_FAULT && m.segment_fault==17U);
    Init(&m,s,table,0U); (void)MissionControl_Start(&m,&in);
    update_status=SEG_UNIMPLEMENTED;
    assert(MissionControl_Update(&m,&in).action==DRIVE_STOP && m.reason==MISSION_NOT_IMPLEMENTED);
    /* A single straight leg must not require a black line in a blank area. */
    Init(&m,s,table,MISSION_MODE_AC); s[0].required_sensors=SENSOR_GYRO;
    in.line_calibrated=0U; in.line_sequence=0U;
    assert(MissionControl_Start(&m,&in).action==DRIVE_BEGIN);
    in=Fresh(300U); Init(&m,s,table,0U); fake_mode=MOTION_TURN;
    target=-500.0f; right_delta=1000.0f; out=MissionControl_Start(&m,&in);
    assert(out.action==DRIVE_BEGIN && out.mode==MOTION_TURN);
    Init(&m,s,table,0U); fake_mode=MOTION_TURN;
    out=MissionControl_Start(&m,&in); assert(out.action==DRIVE_STOP && m.reason==MISSION_BAD_TARGET);
    Init(&m,s,table,0U); fake_mode=MOTION_BRAKE; target=right_delta=0.0f;
    out=MissionControl_Start(&m,&in); assert(out.action==DRIVE_BEGIN && out.mode==MOTION_BRAKE);
    Init(&m,s,table,0U); fake_mode=MOTION_BRAKE;
    out=MissionControl_Start(&m,&in); assert(out.action==DRIVE_STOP && m.reason==MISSION_BAD_TARGET);
    Init(&m,s,table,0U); fake_mode=(MissionMotionMode)99;
    assert(MissionControl_Start(&m,&in).action==DRIVE_STOP);
    puts("mission control: route, drive profiles, continuous handoff, rejection, faults and rollover passed");
    return 0;
}
