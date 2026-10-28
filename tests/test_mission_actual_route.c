#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "mission_fake_port.h"
#include "motion_drive.h"

/* Synthetic feedback, not a tire/track simulation: actual controllers, drive
 * adapter and speed PI execute, while input trace supplies ideal wheel motion
 * and line readings. This certifies integration, not physical car performance. */
int main(int argc, char **argv)
{
    fake_pressed=0U; Mission_Init(0U); fake_auto_feedback=0U;
    assert(Mission_Start(0U) && !speed_control.running);
    const MissionPoint expected[]={POINT_C,POINT_B,POINT_D,POINT_A};
    uint32_t now=0U, arrivals=0U, turns_seen=0U;
    /* Per-phase time of the first lap: the only way to see how the budget is
     * split between capture, following and the two stop-and-turn corners. */
    uint32_t phase_ms[10]={0U,0U,0U,0U,0U,0U,0U,0U,0U,0U};
    float left_fraction=0.0f,right_fraction=0.0f;
    while(mission_diag.state==MISSION_RUNNING && now<400000U) {
        now+=10U;
        if(mission_diag.lap==0U && mission_diag.phase<10U) phase_ms[mission_diag.phase]+=10U;
        float left=speed_control.running ? speed_control.left_reference_cps : 0.0f;
        float right=speed_control.running ? speed_control.right_reference_cps : 0.0f;
        left_fraction+=left*0.01f; right_fraction+=right*0.01f;
        int32_t ld=(int32_t)roundf(left_fraction), rd=(int32_t)roundf(right_fraction);
        left_fraction-=ld; right_fraction-=rd;
        encoder_data.left_delta=ld; encoder_data.right_delta=rd;
        encoder_data.left_total+=ld; encoder_data.right_total+=rd;
        encoder_data.left_cps=(float)ld*100.0f; encoder_data.right_cps=(float)rd*100.0f;
        encoder_data.sample_ms=10U; ++encoder_data.sample_count;
        float yaw_rate=((right/MOTION_RIGHT_COUNTS_PER_MM-left/MOTION_LEFT_COUNTS_PER_MM)/130.0f)*57.2957795f;
        fake_input.yaw_rate_dps=yaw_rate/MOTION_YAW_SIGN;
        fake_input.yaw_deg+=fake_input.yaw_rate_dps*0.01f;
        if(MotionDrive_Mode()==MOTION_TURN) {
            if(mission_diag.segment==SEGMENT_AC) { assert(left>=0.0f && right<=0.0f); turns_seen|=1U; }
            if(mission_diag.segment==SEGMENT_BD) { assert(left<=0.0f && right>=0.0f); turns_seen|=2U; }
        }
        if(now%40U==0U) {
            uint32_t mask=0U, strength[8]={0U,0U,0U,0U,0U,0U,0U,0U};
            if(mission_diag.segment==SEGMENT_AC || mission_diag.segment==SEGMENT_BD) {
                if(mission_diag.phase==PHASE_STRAIGHT && mission_diag.distance_mm>=1281) mask=0x18U;
            } else if(mission_diag.distance_mm<1257) {
                {
                    /* Arc feedback, not a fixed offset: the line of a semicircle
                     * sits where the tangent is, so what the bar sees is the
                     * heading lag against the tangent of the arc. A car that has
                     * turned less than the tangent finds the line on its left,
                     * which maps to the negative probes and steers it back.
                     * Use the same observation during CAPTURE: an unconditional
                     * centred stripe there falsely says the line is underneath
                     * the bar even while the car faces the wrong way. Outside
                     * the probe field of view the observation must be white,
                     * rather than an indefinitely clamped edge stripe. This is
                     * still a heading-only model, not a lateral tire model. */
                    float entry=mission_diag.segment==SEGMENT_CB ? 0.0f : 180.0f;
                    float tangent=entry+180.0f*((float)mission_diag.distance_mm/1257.0f);
                    float lag=fake_input.yaw_deg-tangent;
                    while(lag>180.0f) lag-=360.0f;
                    while(lag<-180.0f) lag+=360.0f;
                    float pos=lag*100.0f;
                    if(fabsf(pos)<=3500.0f) {
                        int32_t idx=(int32_t)lroundf((pos+3500.0f)/1000.0f);
                        idx=idx<0 ? 0 : (idx>7 ? 7 : idx);
                        mask=1U<<(uint32_t)idx;
                        strength[idx]=1000U;
                    }
                }
            }
            fake_input.line_mask=mask; ++fake_input.line_sequence;
            for(uint32_t i=0;i<8U;++i) fake_input.line_strength[i]=strength[i];
        }
        uint32_t motor_stops=fake_motor_stops, brakes=fake_motor_brakes, starts=fake_starts;
        Fake_Tick(now);
        if(mission_diag.arrivals>arrivals) {
            assert(mission_diag.arrivals==arrivals+1U && fake_points[arrivals]==expected[arrivals%4U]);
            if(arrivals%4U==0U || arrivals%4U==2U) {
                /* C and D hand their arc an entry-tangent corner that the capture
                 * steers away while rolling: the drive stays in FORWARD, so the
                 * speed PI is never torn down and the car does not stop. */
                assert(MotionDrive_Mode()==MOTION_FORWARD && speed_control.running);
                assert(fake_starts==starts && fake_motor_stops==motor_stops);
                assert(fake_motor_brakes==brakes);
                assert(mission_diag.phase==PHASE_CAPTURE && mission_diag.distance_mm==0);
            }
            if(arrivals%4U==1U) assert(MotionDrive_Mode()==MOTION_BRAKE && mission_diag.phase==PHASE_STILL);
            ++arrivals;
        }
    }
    if(mission_diag.state!=MISSION_DONE) {
        fprintf(stderr,"route failed at %u ms segment=%u phase=%u reason=%u detail=%u dist=%d yaw=%.2f\n",
            now,mission_diag.segment,mission_diag.phase,mission_diag.reason,mission_diag.segment_fault,
            mission_diag.distance_mm,fake_input.yaw_deg);
    }
    assert(mission_diag.state==MISSION_DONE && arrivals==16U && mission_diag.lap==3U && turns_seen==3U);
    assert(!speed_control.running && MotionDrive_Mode()==MOTION_BRAKE);
    assert(!fake_resets && fake_imu_initializations==1U && fake_notifications==16U);
    for(uint32_t i=0;i<100U;++i) { now+=10U; Fake_Tick(now); }
    assert(fake_notifications==16U && !speed_control.running && mission_diag.state==MISSION_DONE);
    if(argc==2) {
        FILE *dump=fopen(argv[1],"w"); assert(dump);
        MissionDiagnostics record=mission_diag;
        uint32_t words[sizeof(record)/sizeof(uint32_t)]; memcpy(words,&record,sizeof(record));
        const uint32_t count=sizeof(words)/sizeof(words[0]);
        for(uint32_t i=0;i<count;i+=4U) {
            fprintf(dump,"0x%08X :",0x20000000U+i*4U);
            for(uint32_t j=0U;j<4U&&i+j<count;++j) fprintf(dump," %08X",words[i+j]);
            fputc('\n',dump);
        }
        assert(!fclose(dump));
    }
    printf("actual FULL route trace completed in %u ms over %u laps; C/D continuous, B turn, A brake, %u events\n",
        mission_diag.elapsed_ms, mission_diag.lap+1U, mission_diag.event_count);
    printf("lap 1 phase ms: STILL=%u TURN=%u SETTLE=%u STRAIGHT=%u CAPTURE=%u FOLLOW=%u\n",
        phase_ms[PHASE_STILL],phase_ms[PHASE_TURN],phase_ms[PHASE_SETTLE],phase_ms[PHASE_STRAIGHT],
        phase_ms[PHASE_CAPTURE],phase_ms[PHASE_FOLLOW]);
    return 0;
}
