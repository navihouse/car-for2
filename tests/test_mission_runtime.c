#include <assert.h>
#include <stdio.h>
#include "mission_fake_port.h"
#include "mission_fake_segments.h"

static void Init(void)
{
    fake_pressed=0U; FakeSegments_Reset(); Mission_Init(0U); Mission_SetLaps(1U);
}
static void TickRange(uint32_t first,uint32_t last)
{
    for(uint32_t now=first;now<=last;now+=10U) Fake_Tick(now);
}
int main(void)
{
    Init(); assert(Mission_Start(0U));
    assert(fake_starts==1U && !fake_resets);
    TickRange(10U,100U);
    assert(speed_control.left_reference_cps>0.0f && fake_motor_writes>0U);
    uint32_t stops=fake_stops;
    const MissionPoint expected[]={POINT_C,POINT_B,POINT_D,POINT_A};
    for(uint32_t i=0;i<4U;++i) {
        float previous_ref=speed_control.left_reference_cps;
        int64_t previous_total=encoder_data.left_total;
        uint32_t services=fake_services, captures=fake_captures;
        fake_segment_status[i]=SEG_COMPLETE; Fake_Tick(110U+i*10U);
        assert(fake_notifications==i+1U && fake_points[i]==expected[i]);
        assert(fake_services==services+1U && fake_captures==captures+1U);
        assert(fake_exit_count[i]==1U && fake_starts==1U && !fake_resets);
        assert(fake_imu_initializations==1U && encoder_data.left_total==previous_total+5);
        if(i<3U) {
            assert(fake_stops==stops && mission_diag.state==MISSION_RUNNING);
            assert(fake_enter_count[i+1U]==1U);
            assert(fake_entries[i+1U].now_ms==110U+i*10U);
            assert(fake_entries[i+1U].left_counts==encoder_data.left_total);
            assert(fake_entries[i+1U].yaw_deg==33.0f);
            assert(speed_control.running && speed_control.left_reference_cps>=previous_ref);
            assert(speed_control.left_reference_cps-previous_ref<=50.01f); /* real speed ramp */
            const float next[]={700.0f,900.0f,600.0f};
            assert(speed_control.requested_left_cps==next[i]);
        } else assert(fake_stops==stops+1U && mission_diag.state==MISSION_DONE && !speed_control.running);
    }
    TickRange(150U,250U);
    assert(fake_notifications==4U && mission_diag.arrivals==4U && mission_diag.event_count==4U);
    assert(mission_diag.elapsed_ms==140U && mission_diag.last_point==POINT_A);
    assert(!mission_diag.left_target_cps && !mission_diag.right_target_cps);

    /* Fresh run is explicit; contexts and arrivals restart, total events do not. */
    FakeSegments_Reset(); assert(Mission_Start(260U));
    assert(mission_diag.arrivals==0U && mission_diag.event_count==4U && fake_starts==2U);
    Mission_Stop(260U); assert(mission_diag.state==MISSION_STOPPED);
    assert(fake_exit_count[0]==1U && !speed_control.running);
    Mission_Stop(260U); assert(fake_exit_count[0]==1U);

    Init(); assert(Mission_Start(0U)); fake_input.gyro_age_ms=31U; Fake_Tick(10U);
    assert(mission_diag.state==MISSION_FAULT && mission_diag.reason==MISSION_GYRO_UNAVAILABLE);
    assert(!speed_control.running && fake_exit_count[0]==1U && !fake_notifications);
    Init(); assert(Mission_Start(0U)); fake_segment_status[0]=SEG_FAULT; Fake_Tick(10U);
    assert(mission_diag.reason==MISSION_SEGMENT_FAULT && mission_diag.segment_fault==42U && !fake_notifications);
    /* Straight is allowed without line data; incoming arc is guarded at handoff. */
    Init(); fake_input.line_calibrated=0U; assert(Mission_Start(0U));
    fake_segment_status[0]=SEG_COMPLETE; Fake_Tick(10U);
    assert(mission_diag.reason==MISSION_LINE_UNAVAILABLE && !speed_control.running);
    assert(fake_notifications==1U && fake_points[0]==POINT_C && !fake_enter_count[1]);
    Init(); assert(Mission_Start(0U)); speed_control.fault=SPEED_FAULT_LEFT_NO_FEEDBACK; Fake_Tick(10U);
    assert(mission_diag.reason==MISSION_SPEED_FAULT && !speed_control.running);

    /* Idle click starts on release; a running press stops immediately and its
     * release cannot restart. Button debounce is the real shared module. */
    Init(); TickRange(10U,50U); fake_pressed=1U; TickRange(60U,110U);
    assert(!fake_starts); fake_pressed=0U; TickRange(120U,170U);
    assert(fake_starts==1U && mission_diag.state==MISSION_RUNNING);
    fake_pressed=1U; TickRange(180U,230U);
    assert(mission_diag.state==MISSION_STOPPED);
    fake_pressed=0U; TickRange(240U,290U); assert(fake_starts==1U);
    /* Long hold runs calibration only and never starts a motor. */
    Init(); TickRange(10U,50U); fake_pressed=1U; TickRange(60U,6100U);
    fake_pressed=0U; TickRange(6110U,6160U);
    assert(fake_cal_starts==1U && !fake_starts && mission_diag.calibrating);
    fake_pressed=1U; TickRange(6170U,6230U);
    assert(fake_cal_cancels==1U && !fake_input.calibrating && !fake_starts);
    fake_pressed=0U; TickRange(6240U,6300U); assert(!fake_starts);
    /* External STOP during an un-debounced press must consume that key. */
    Init(); TickRange(10U,50U); fake_pressed=1U; Mission_Stop(51U);
    TickRange(60U,110U); fake_pressed=0U; TickRange(120U,170U);
    assert(!fake_starts && !speed_control.running);
    puts("mission runtime: real scheduler/button/speed ramp, continuous handoff and parked safety passed");
    return 0;
}
