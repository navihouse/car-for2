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
    float left_fraction=0.0f,right_fraction=0.0f;
    while(mission_diag.state==MISSION_RUNNING && now<120000U) {
        now+=10U;
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
            uint32_t mask=0U;
            if(mission_diag.segment==SEGMENT_AC || mission_diag.segment==SEGMENT_BD) {
                if(mission_diag.phase==PHASE_STRAIGHT && mission_diag.distance_mm>=1281) mask=0x18U;
            } else if(mission_diag.distance_mm<1257) {
                mask=mission_diag.phase==PHASE_CAPTURE ? 0x18U : (mission_diag.segment==SEGMENT_CB ? 0x0CU : 0x30U);
            }
            fake_input.line_mask=mask; ++fake_input.line_sequence;
            for(uint32_t i=0;i<8U;++i) fake_input.line_strength[i]=(mask&(1U<<i)) ? 1000U : 0U;
            if(mask==0x0CU) fake_input.line_strength[2]=600U;
            if(mask==0x30U) fake_input.line_strength[5]=600U;
        }
        uint32_t motor_stops=fake_motor_stops, brakes=fake_motor_brakes, starts=fake_starts;
        float previous_ref=speed_control.left_reference_cps;
        Fake_Tick(now);
        if(mission_diag.arrivals>arrivals) {
            assert(mission_diag.arrivals==arrivals+1U && fake_points[arrivals]==expected[arrivals]);
            if(arrivals==0U || arrivals==2U) {
                assert(MotionDrive_Mode()==MOTION_FORWARD && speed_control.running);
                assert(fake_motor_stops==motor_stops && fake_motor_brakes==brakes && fake_starts==starts);
                assert(fabsf(speed_control.left_reference_cps-previous_ref)<=50.01f);
                assert(mission_diag.phase==PHASE_CAPTURE && mission_diag.distance_mm==0);
            }
            if(arrivals==1U) assert(MotionDrive_Mode()==MOTION_BRAKE && mission_diag.phase==PHASE_STILL);
            ++arrivals;
        }
    }
    if(mission_diag.state!=MISSION_DONE) {
        fprintf(stderr,"route failed at %u ms segment=%u phase=%u reason=%u detail=%u dist=%d yaw=%.2f\n",
            now,mission_diag.segment,mission_diag.phase,mission_diag.reason,mission_diag.segment_fault,
            mission_diag.distance_mm,fake_input.yaw_deg);
    }
    assert(mission_diag.state==MISSION_DONE && arrivals==4U && turns_seen==3U);
    assert(!speed_control.running && MotionDrive_Mode()==MOTION_BRAKE);
    assert(!fake_resets && fake_imu_initializations==1U && fake_notifications==4U);
    for(uint32_t i=0;i<100U;++i) { now+=10U; Fake_Tick(now); }
    assert(fake_notifications==4U && !speed_control.running && mission_diag.state==MISSION_DONE);
    if(argc==2) {
        FILE *dump=fopen(argv[1],"w"); assert(dump);
        MissionDiagnostics record=mission_diag;
        uint32_t words[sizeof(record)/sizeof(uint32_t)]; memcpy(words,&record,sizeof(record));
        for(uint32_t i=0;i<sizeof(words)/sizeof(words[0]);i+=4U) {
            fprintf(dump,"0x%08X : %08X %08X %08X %08X\n",0x20000000U+i*4U,
                words[i],words[i+1U],words[i+2U],words[i+3U]);
        }
        assert(!fclose(dump));
    }
    printf("actual FULL route trace completed in %u ms; C/D continuous, B turn, A brake, 4 unique events\n",mission_diag.elapsed_ms);
    return 0;
}
