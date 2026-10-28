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
    /* Last stretch before the point hands the arc its entry speed. */
    const int32_t approach=(int32_t)((p->nominal_mm-20.0f)*MOTION_LEFT_COUNTS_PER_MM);
    in.left_counts=400+approach; in.right_counts=-400+approach;
    in.yaw_deg=p->target_yaw_deg; in.now_ms+=20U;
    out=MotionStraight_Update(&ctx,&in,p);
    assert(out.left_cps<p->cruise_cps && out.left_cps>MOTION_STRAIGHT_HANDOFF_CPS);
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
    assert(fabsf(ctx.motion.heading_error_deg-Motion_WrapDegrees(p->target_yaw_deg-170.0f))<0.01f);
}
static void ArcTests(void)
{
    const MissionSegmentParams *p=mission_segment_cb.params;
    MissionInput in=Fresh(); in.line_mask=0x18U; in.line_strength[3]=in.line_strength[4]=1000U;
    MissionSegmentContext ctx=Context(&in);
    MissionSegmentOutput out=MotionArc_Enter(&ctx,&in,p);
    assert(ctx.phase==PHASE_CAPTURE && out.left_cps==p->capture_cps && out.right_cps==p->capture_cps);
    for(uint32_t i=0;i<100U;++i) (void)MotionArc_Update(&ctx,&in,p);
    assert(ctx.motion.capture_frames==1U);
    Frame(&in,0x18U); (void)MotionArc_Update(&ctx,&in,p);
    Frame(&in,0x18U); out=MotionArc_Update(&ctx,&in,p);
    /* Centred line in FOLLOW: the arc differential is already made by the
     * curvature feed-forward, so the wheels are biased against each other
     * instead of running equal, and the PD only spends what is left. Right
     * wheel leads: this course runs its semicircles counter-clockwise. */
    assert(ctx.phase==PHASE_FOLLOW);
    float differential=out.right_cps-out.left_cps;
    float arc_differential=-2.0f*MOTION_LINE_CURVATURE_FF*p->cruise_cps;
    assert(fabsf(differential-arc_differential)<5.0f);
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
    in.now_ms=ctx.entry.now_ms+MOTION_CAPTURE_TIMEOUT_MS+1U; out=MotionArc_Update(&ctx,&in,p);
    assert(out.status==SEG_FAULT && out.fault==MOTION_FAULT_CAPTURE);
    /* Distance capture cap is independent of its time cap. */
    in=Fresh(); Frame(&in,0x01U); ctx=Context(&in); (void)MotionArc_Enter(&ctx,&in,p);
    const int32_t cap_counts=(int32_t)((MOTION_CAPTURE_MAX_MM+40.0f)*MOTION_LEFT_COUNTS_PER_MM);
    in.left_counts=in.right_counts=cap_counts; out=MotionArc_Update(&ctx,&in,p);
    assert(out.status==SEG_FAULT && out.fault==MOTION_FAULT_CAPTURE);
    /* Real-track dropout, exactly what the car did at C: only the bar it just
     * crossed is black, the arc line sits off to the side, and 38.66 deg of
     * entry tangent is still owed. The car must keep steering on the gyro -
     * never run straight - and must not fault while the tangent is owed. */
    in=Fresh(); in.yaw_deg=-38.66f; ctx=Context(&in); ctx.from_handoff=1U;
    out=MotionArc_Enter(&ctx,&in,p);
    for(uint32_t i=0;i<25U;++i) {
        Frame(&in,0U);
        out=MotionArc_Update(&ctx,&in,p);
        assert(out.status==SEG_RUNNING && out.mode==MOTION_FORWARD);
        assert(fabsf(out.right_cps-out.left_cps)>=MOTION_LINE_MIN_WHEEL_CPS*2.0f);
    }
    /* Reaching the tangent does not re-arm NO_LINE: a rolling entry keeps
     * steering the known semicircle until the capture budget is spent, and the
     * budget - not a 350 ms dropout - is what reports the failure. */
    in.yaw_deg=0.0f;
    for(uint32_t i=0;i<12U;++i) {
        Frame(&in,0U); out=MotionArc_Update(&ctx,&in,p);
        assert(out.status==SEG_RUNNING || out.fault==MOTION_FAULT_CAPTURE);
    }
    in.now_ms=ctx.entry.now_ms+MOTION_CAPTURE_TIMEOUT_MS+1U;
    out=MotionArc_Update(&ctx,&in,p);
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
static void TurnApproachTests(void)
{
    const MissionSegmentParams *p=mission_segment_ac.params;
    for (int sign=-1;sign<=1;sign+=2) {
        MissionInput in=Fresh();in.yaw_deg=p->target_yaw_deg-(float)sign*5.0f;
        MissionSegmentContext ctx=Context(&in);
        (void)MotionStraight_Enter(&ctx,&in,p);(void)MotionStraight_Update(&ctx,&in,p);
        in.now_ms=500U;MissionSegmentOutput out=MotionStraight_Update(&ctx,&in,p);
        assert(out.mode==MOTION_TURN);
        /* 5 degrees out: the rate feed-forward alone is 16 cps per deg/s times
         * KP*5, and the approach floor keeps the low end. Well inside the
         * approach regime, nowhere near the full speed kick. */
        assert(fabsf(out.left_cps)>MOTION_TURN_APPROACH_MIN_CPS);
        assert(fabsf(out.left_cps)<0.5f*MOTION_TURN_MAX_CPS && fabsf(out.right_cps)<0.5f*MOTION_TURN_MAX_CPS);
        assert(out.left_cps*(float)sign<0.0f && out.right_cps*(float)sign>0.0f);
        /* Moving away must not receive predictive early braking. */
        in.yaw_deg=p->target_yaw_deg-(float)sign*2.8f;in.yaw_rate_dps=-(float)sign*40.0f;
        in.now_ms+=20U;out=MotionStraight_Update(&ctx,&in,p);assert(out.mode==MOTION_TURN);
        /* Moving towards the target brakes inside tolerance before crossing. */
        in.yaw_rate_dps=(float)sign*40.0f;in.now_ms+=20U;
        out=MotionStraight_Update(&ctx,&in,p);
        assert(out.mode==MOTION_BRAKE && ctx.phase==PHASE_SETTLE);
        in.yaw_rate_dps=0.0f;in.now_ms+=20U;(void)MotionStraight_Update(&ctx,&in,p);
        in.now_ms+=MOTION_SETTLE_MS;out=MotionStraight_Update(&ctx,&in,p);
        assert(out.mode==MOTION_FORWARD && ctx.phase==PHASE_STRAIGHT && !ctx.motion.corrections);
    }
}
static void CaptureHandoffTests(void)
{
    const MissionSegmentParams *p=mission_segment_cb.params;
    MissionInput in=Fresh();Frame(&in,0xFFU);in.yaw_deg=MOTION_AC_TARGET_DEG;
    MissionSegmentContext ctx=Context(&in);ctx.from_handoff=1U;
    MissionSegmentOutput out=MotionArc_Enter(&ctx,&in,p);
    /* The diagonal leaves C on the AC heading; CB's entry tangent is 0 deg, so
       the capture steers the corner away while both wheels keep rolling. */
    assert(out.status==SEG_RUNNING && ctx.phase==PHASE_CAPTURE && out.mode==MOTION_FORWARD);
    assert(out.left_cps>0.0f && out.right_cps>out.left_cps);
    const float entry_cap=fminf(MOTION_CAPTURE_HEADING_MAX_CPS,
        fmaxf(p->capture_cps-MOTION_LINE_MIN_WHEEL_CPS,0.0f));
    const float entry_trim=fminf(MOTION_CAPTURE_HEADING_KP*fabsf(MOTION_AC_TARGET_DEG),entry_cap);
    assert(fabsf((out.right_cps-out.left_cps)-2.0f*entry_trim)<2.0f);
    in.yaw_deg=0.0f;in.yaw_rate_dps=0.0f;
    out=MotionArc_Update(&ctx,&in,p);
    assert(out.status==SEG_RUNNING && ctx.phase==PHASE_CAPTURE);
    assert(ctx.motion.capture_frames==0U); /* Wide stripe cannot certify capture. */
    for (uint32_t i=0;i<MOTION_CAPTURE_CONFIRM_FRAMES;i++) {
        Frame(&in,0x18U);out=MotionArc_Update(&ctx,&in,p);
    }
    assert(ctx.phase==PHASE_FOLLOW && out.status==SEG_RUNNING);
    /* Wide masks still fault during normal following. */
    Frame(&in,0xFFU);out=MotionArc_Update(&ctx,&in,p);assert(out.status==SEG_RUNNING);
    in.now_ms+=MOTION_LINE_UNCERTAIN_MS;out=MotionArc_Update(&ctx,&in,p);
    assert(out.status==SEG_FAULT && out.fault==MOTION_FAULT_AMBIGUOUS);

    p=mission_segment_da.params;in=Fresh();Frame(&in,0xFFU);in.yaw_deg=MOTION_BD_TARGET_DEG;
    ctx=Context(&in);ctx.from_handoff=1U;out=MotionArc_Enter(&ctx,&in,p);
    /* DA is the mirror image: tangent 180 deg, the bias steers the other way. */
    assert(out.status==SEG_RUNNING && ctx.phase==PHASE_CAPTURE && out.mode==MOTION_FORWARD);
    assert(out.left_cps>0.0f && out.right_cps>0.0f && out.left_cps>out.right_cps);
    in.yaw_deg=180.0f;in.yaw_rate_dps=0.0f;
    out=MotionArc_Update(&ctx,&in,p);
    assert(out.status==SEG_RUNNING && ctx.phase==PHASE_CAPTURE);
    Frame(&in,0U);out=MotionArc_Update(&ctx,&in,p);
    assert(out.status==SEG_RUNNING);
    in.now_ms+=MOTION_LINE_UNCERTAIN_MS;out=MotionArc_Update(&ctx,&in,p);
    assert(out.status==SEG_RUNNING); /* Capture has its own bounded search grace. */
    /* Rolling capture keeps searching past the no-line grace - it steers the
     * known semicircle instead - so only the capture budget ends the search. */
    in.now_ms=ctx.phase_started_ms+MOTION_CAPTURE_TIMEOUT_MS+1U;
    out=MotionArc_Update(&ctx,&in,p);assert(out.status==SEG_FAULT && out.fault==MOTION_FAULT_CAPTURE);

    /* A wide stripe can never bypass the absolute capture timeout. */
    in=Fresh();Frame(&in,0xFFU);in.yaw_deg=p->target_yaw_deg-180.0f; /* Already on the tangent. */
    ctx=Context(&in);ctx.from_handoff=1U;
    (void)MotionArc_Enter(&ctx,&in,p);
    assert(ctx.phase==PHASE_CAPTURE);
    in.now_ms=ctx.phase_started_ms+MOTION_CAPTURE_TIMEOUT_MS+1U;
    out=MotionArc_Update(&ctx,&in,p);assert(out.status==SEG_FAULT && out.fault==MOTION_FAULT_CAPTURE);
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
    assert(Motion_ValidateTuning()); EdgeTests(); DistanceTests(); StraightTests(); ArcTests(); TurnFailureTests(); TurnApproachTests(); CaptureHandoffTests(); ConfigTests();
    assert(Motion_WrapDegrees(218.66f)<0.0f);
    puts("actual motion: edge windows, distinct frames, per-leg distance, turn/heading/line PD and faults passed");
    return 0;
}
