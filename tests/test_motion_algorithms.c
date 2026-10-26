#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "motion_control.h"
#include "mission_control.h"

static MissionInput Fresh(void)
{
    return (MissionInput){.gyro_ready=1U,.line_calibrated=1U,.line_sequence=1U};
}
static void Frame(MissionInput *in,uint32_t mask)
{
    in->now_ms+=40U; ++in->line_sequence; in->line_mask=mask;
    for(uint32_t i=0;i<8U;++i) in->line_strength[i]=(mask & (1U<<i)) ? 1000U : 0U;
}
static MissionSegmentContext Context(const MissionInput *in)
{
    return (MissionSegmentContext){.entry=*in,.active=1U};
}
static void EdgeTests(void)
{
    MissionEdge e; MissionInput in=Fresh(); MotionEdge_Reset(&e);
    for(uint32_t i=0;i<3U;++i) { Frame(&in,0U); assert(!MotionEdge_Update(&e,&in,0,100,200,1)); }
    Frame(&in,0x18U); assert(!MotionEdge_Update(&e,&in,100,100,200,1));
    for(uint32_t i=0;i<100U;++i) assert(!MotionEdge_Update(&e,&in,100,100,200,1));
    assert(e.candidate_frames==1U); /* repeated foreground polls do not count */
    Frame(&in,0x18U); assert(!MotionEdge_Update(&e,&in,150,100,200,1));
    Frame(&in,0x18U); assert(MotionEdge_Update(&e,&in,200,100,200,1));
    assert(!MotionEdge_Update(&e,&in,200,100,200,1));
    MotionEdge_Reset(&e);
    for(uint32_t i=0;i<3U;++i) { Frame(&in,0U); (void)MotionEdge_Update(&e,&in,0,100,200,1); }
    Frame(&in,0x18U); assert(!MotionEdge_Update(&e,&in,99,100,200,1));
    Frame(&in,0x18U); assert(!MotionEdge_Update(&e,&in,100,100,200,1));
    Frame(&in,0x18U); assert(!MotionEdge_Update(&e,&in,101,100,200,1)); /* old edge stays ineligible */
    for(uint32_t i=0;i<3U;++i) { Frame(&in,0U); (void)MotionEdge_Update(&e,&in,120,100,200,1); }
    Frame(&in,0x18U); assert(!MotionEdge_Update(&e,&in,199,100,200,1));
    Frame(&in,0x18U); assert(!MotionEdge_Update(&e,&in,200,100,200,1));
    Frame(&in,0x18U); assert(!MotionEdge_Update(&e,&in,201,100,200,1)); /* confirmation crossed upper bound */
    MotionEdge_SeedBlack(&e,&in); in.line_age_ms=151U;
    for(uint32_t i=0;i<4U;++i) { Frame(&in,0U); assert(!MotionEdge_Update(&e,&in,150,100,200,0)); }
    in.line_age_ms=0U; MotionEdge_SeedBlack(&e,&in);
    for(uint32_t i=0;i<3U;++i) {
        Frame(&in,0U); assert(MotionEdge_Update(&e,&in,150,100,200,0)==(i==2U));
    }
}
static void DistanceTests(void)
{
    MissionInput in=Fresh(); MissionDistance a,b;
    in.left_counts=INT64_C(1000000000000); in.right_counts=INT64_C(2000000000000);
    MotionDistance_Begin(&a,&in); in.left_counts+=6730; in.right_counts+=6730;
    assert(fabsf(MotionDistance_GetMm(&a,&in)-1000.0f)<0.1f);
    MotionDistance_Begin(&b,&in); in.left_counts-=100; in.right_counts+=100;
    assert(fabsf(MotionDistance_GetMm(&b,&in))<0.001f);
    assert(fabsf(MotionDistance_GetMm(&a,&in)-1000.0f)<0.1f);
}
static void StraightTests(void)
{
    MissionInput in=Fresh(); MissionSegmentContext ctx=Context(&in);
    const MissionSegmentParams *p=mission_segment_ac.params;
    assert(Motion_ValidateParams(p));
    MissionSegmentOutput out=MotionStraight_Enter(&ctx,&in,p);
    assert(out.mode==MOTION_BRAKE && ctx.phase==PHASE_STILL);
    (void)MotionStraight_Update(&ctx,&in,p);
    in.now_ms=500U; out=MotionStraight_Update(&ctx,&in,p);
    assert(out.mode==MOTION_TURN && out.left_cps>0 && out.right_cps<0); /* AC right */
    assert(!ctx.motion.distance.active);
    in.left_counts=400; in.right_counts=-400; in.yaw_deg=p->target_yaw_deg;
    in.now_ms=520U; out=MotionStraight_Update(&ctx,&in,p);
    assert(out.mode==MOTION_BRAKE && ctx.phase==PHASE_SETTLE);
    in.now_ms=540U; (void)MotionStraight_Update(&ctx,&in,p);
    in.now_ms=1040U; out=MotionStraight_Update(&ctx,&in,p);
    assert(ctx.phase==PHASE_STRAIGHT && out.mode==MOTION_FORWARD);
    assert(ctx.motion.distance.left_origin==400 && ctx.motion.distance.right_origin==-400);
    assert(ctx.motion.distance_mm==0.0f); /* excludes rotation entirely */
    in.yaw_deg=p->target_yaw_deg-2.0f; in.now_ms+=20U;
    out=MotionStraight_Update(&ctx,&in,p); assert(out.left_cps<out.right_cps); /* steer left */
    in.yaw_deg=p->target_yaw_deg+2.0f; in.now_ms+=20U;
    out=MotionStraight_Update(&ctx,&in,p); assert(out.left_cps>out.right_cps); /* steer right */
    in.left_counts+=11000; in.right_counts+=11000;
    out=MotionStraight_Update(&ctx,&in,p); assert(out.status==SEG_FAULT && out.fault==MOTION_FAULT_DISTANCE);

    in=Fresh(); in.yaw_deg=180.0f; ctx=Context(&in); p=mission_segment_bd.params;
    (void)MotionStraight_Enter(&ctx,&in,p); (void)MotionStraight_Update(&ctx,&in,p);
    in.now_ms=500U; out=MotionStraight_Update(&ctx,&in,p);
    assert(out.mode==MOTION_TURN && out.left_cps<0 && out.right_cps>0); /* BD left */
    in.now_ms=26000U; out=MotionStraight_Update(&ctx,&in,p);
    assert(out.status==SEG_FAULT && out.fault==MOTION_FAULT_TURN_TIMEOUT);
    /* Fixed route heading does not absorb an erroneous B arrival. */
    in=Fresh(); in.yaw_deg=170.0f; ctx=Context(&in);
    (void)MotionStraight_Enter(&ctx,&in,p);
    assert(fabsf(ctx.motion.heading_error_deg-48.66f)<0.01f);
}
static void ArcTests(void)
{
    const MissionSegmentParams *p=mission_segment_cb.params;
    MissionInput in=Fresh(); in.line_mask=0x18U; in.line_strength[3]=in.line_strength[4]=1000U;
    MissionSegmentContext ctx=Context(&in);
    MissionSegmentOutput out=MotionArc_Enter(&ctx,&in,p);
    assert(ctx.phase==PHASE_CAPTURE && out.left_cps==600.0f && out.right_cps==600.0f);
    for(uint32_t i=0;i<100U;++i) (void)MotionArc_Update(&ctx,&in,p);
    assert(ctx.motion.capture_frames==1U);
    Frame(&in,0x18U); (void)MotionArc_Update(&ctx,&in,p);
    Frame(&in,0x18U); out=MotionArc_Update(&ctx,&in,p);
    assert(ctx.phase==PHASE_FOLLOW && out.left_cps==900.0f);
    Frame(&in,0x20U); in.now_ms+=20U; out=MotionArc_Update(&ctx,&in,p);
    assert(out.left_cps>out.right_cps); /* black right -> turn right */
    Frame(&in,0x04U); in.now_ms+=20U; out=MotionArc_Update(&ctx,&in,p);
    assert(out.left_cps<out.right_cps);
    /* Ambiguous disconnected groups without a continuity winner. */
    Frame(&in,0x81U); out=MotionArc_Update(&ctx,&in,p); assert(out.status==SEG_RUNNING);
    in.now_ms+=100U; out=MotionArc_Update(&ctx,&in,p);
    assert(out.status==SEG_FAULT && out.fault==MOTION_FAULT_AMBIGUOUS);
    /* A short all-white excursion is recovered, not an end before the window. */
    in=Fresh(); Frame(&in,0x18U); ctx=Context(&in); (void)MotionArc_Enter(&ctx,&in,p);
    Frame(&in,0U); out=MotionArc_Update(&ctx,&in,p); assert(out.status==SEG_RUNNING);
    Frame(&in,0x18U); out=MotionArc_Update(&ctx,&in,p); assert(out.status==SEG_RUNNING && !ctx.motion.uncertain);
    in.now_ms=ctx.entry.now_ms+1501U; out=MotionArc_Update(&ctx,&in,p);
    assert(out.status==SEG_FAULT && out.fault==MOTION_FAULT_CAPTURE);
    /* Distance capture cap is independent of its time cap. */
    in=Fresh(); Frame(&in,0x01U); ctx=Context(&in); (void)MotionArc_Enter(&ctx,&in,p);
    in.left_counts=in.right_counts=1400; out=MotionArc_Update(&ctx,&in,p);
    assert(out.status==SEG_FAULT && out.fault==MOTION_FAULT_CAPTURE);
    /* Expected arc exit wins over the normal 100ms no-line protection. */
    in=Fresh(); Frame(&in,0x18U); ctx=Context(&in); ctx.from_handoff=1U;
    (void)MotionArc_Enter(&ctx,&in,p);
    for(uint32_t i=0;i<2U;++i) { Frame(&in,0x18U); (void)MotionArc_Update(&ctx,&in,p); }
    in.left_counts=in.right_counts=8000; /* ~1189mm, inside arc window */
    for(uint32_t i=0;i<3U;++i) {
        Frame(&in,0U); out=MotionArc_Update(&ctx,&in,p);
        assert(out.status==(i==2U ? SEG_COMPLETE : SEG_RUNNING));
    }
}
static void TurnFailureTests(void)
{
    const MissionSegmentParams *p=mission_segment_ac.params;
    for(uint32_t fault=0U;fault<4U;++fault) {
        MissionInput in=Fresh(); MissionSegmentContext ctx=Context(&in);
        (void)MotionStraight_Enter(&ctx,&in,p); (void)MotionStraight_Update(&ctx,&in,p);
        in.now_ms=500U; (void)MotionStraight_Update(&ctx,&in,p);
        in.now_ms=1100U;
        if(fault==0U) in.yaw_deg=6.0f;
        if(fault==1U) in.left_counts=in.right_counts=1200;
        if(fault==2U) { in.left_counts=300; in.right_counts=-300; }
        if(fault==3U) in.left_counts=200;
        const uint32_t expected[]={MOTION_FAULT_TURN_DIRECTION,MOTION_FAULT_TURN_TRANSLATION,
            MOTION_FAULT_GYRO_RESPONSE,MOTION_FAULT_WHEEL_SYNC};
        MissionSegmentOutput out=MotionStraight_Update(&ctx,&in,p);
        assert(out.status==SEG_FAULT && out.fault==expected[fault]);
    }
    MissionInput in=Fresh(); MissionSegmentContext ctx=Context(&in);
    (void)MotionStraight_Enter(&ctx,&in,p);
    ctx.phase=PHASE_SETTLE; ctx.motion.corrections=3U;
    in.yaw_deg=p->target_yaw_deg-4.0f;
    (void)MotionStraight_Update(&ctx,&in,p);
    in.now_ms=500U; MissionSegmentOutput out=MotionStraight_Update(&ctx,&in,p);
    assert(out.status==SEG_FAULT && out.fault==MOTION_FAULT_TURN_TIMEOUT);
    ctx.phase=PHASE_SETTLE; ctx.motion.corrections=2U; ctx.motion.still_tracking=0U;
    (void)MotionStraight_Update(&ctx,&in,p); in.now_ms+=500U;
    out=MotionStraight_Update(&ctx,&in,p);
    assert(out.mode==MOTION_TURN && ctx.motion.corrections==3U);
}
static void ConfigTests(void)
{
    MissionSegmentParams p=*mission_segment_ac.params;
    p.point_max_mm=p.point_min_mm; assert(!Motion_ValidateParams(&p));
    p=*mission_segment_ac.params; p.cruise_cps=NAN; assert(!Motion_ValidateParams(&p));
    p=*mission_segment_cb.params; p.capture_cps=INFINITY; assert(!Motion_ValidateParams(&p));
    const MissionSegment *table[]={&mission_segment_ac,&mission_segment_cb,&mission_segment_bd,&mission_segment_da};
    MissionSegment invalid=mission_segment_da; p=*invalid.params; p.point_max_mm=0;
    invalid.params=&p; table[3]=&invalid;
    MissionControl c; MissionInput in=Fresh(); MissionControl_Init(&c,0U,table);
    MissionOutput out=MissionControl_Start(&c,&in);
    assert(out.action==DRIVE_STOP && c.reason==MISSION_BAD_CONFIG && c.segment==SEGMENT_DA);
}
int main(void)
{
    assert(Motion_ValidateTuning()); EdgeTests(); DistanceTests(); StraightTests(); ArcTests(); TurnFailureTests(); ConfigTests();
    assert(Motion_WrapDegrees(218.66f)<0.0f);
    puts("actual motion: edge windows, distinct frames, per-leg distance, turn/heading/line PD and faults passed");
    return 0;
}
