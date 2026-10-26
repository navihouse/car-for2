#include <assert.h>
#include "mission_fake_port.h"
#include "motor.h"
#include "motion_drive.h"

volatile Encoder_Data encoder_data;
MissionInput fake_input;
uint32_t fake_pressed, fake_starts, fake_stops, fake_targets, fake_resets;
uint32_t fake_services, fake_captures, fake_notifications, fake_motor_writes;
uint32_t fake_cal_starts, fake_cal_cancels, fake_imu_initializations;
uint32_t fake_motor_brakes, fake_motor_stops, fake_auto_feedback;
MissionPoint fake_points[16];
void Motor_Stop(void) { ++fake_motor_stops; }
void Motor_Brake(void) { ++fake_motor_brakes; }
void Motor_SetOutput(int16_t left, int16_t right)
{
    assert(left >= -100 && left <= 100 && right >= -100 && right <= 100);
    ++fake_motor_writes;
}
void Encoder_Reset(uint32_t now)
{
    (void)now; ++fake_resets;
    encoder_data=(Encoder_Data){0};
}
void MissionPort_Init(uint32_t now)
{
    (void)now;
    fake_starts=fake_stops=fake_targets=fake_resets=fake_services=fake_captures=0U;
    fake_notifications=fake_motor_writes=fake_cal_starts=fake_cal_cancels=0U;
    fake_imu_initializations=1U;
    fake_motor_brakes=fake_motor_stops=0U; fake_auto_feedback=1U;
    fake_input=(MissionInput){.gyro_ready=1U,.line_sequence=1U,.line_calibrated=1U,.yaw_deg=33.0f};
    encoder_data=(Encoder_Data){.left_total=1000,.right_total=2000};
    speed_control=(Speed_ControlData){0};
    MotionDrive_Init(); fake_stops=1U;
}
void MissionPort_Service(uint32_t now)
{
    (void)now; ++fake_services;
    if (!fake_auto_feedback) return;
    encoder_data.sample_count++; encoder_data.sample_ms=10U;
    encoder_data.left_delta=5; encoder_data.right_delta=5;
    encoder_data.left_total+=5; encoder_data.right_total+=5;
    encoder_data.left_cps=500.0f; encoder_data.right_cps=500.0f;
}
MissionInput MissionPort_Capture(uint32_t now)
{
    ++fake_captures;
    MissionInput in=fake_input;
    in.now_ms=now; in.left_counts=encoder_data.left_total; in.right_counts=encoder_data.right_total;
    in.encoder_sequence=encoder_data.sample_count;
    in.left_cps=encoder_data.left_cps; in.right_cps=encoder_data.right_cps;
    in.speed_fault=speed_control.fault;
    return in;
}
uint8_t MissionPort_ButtonPressed(void) { return fake_pressed != 0U; }
void MissionPort_Command(MissionMotionMode mode, float left, float right, uint32_t now)
{
    if (mode!=MOTION_BRAKE && mode!=MotionDrive_Mode()) ++fake_starts;
    if (mode==MOTION_BRAKE && MotionDrive_Mode()!=MOTION_BRAKE) ++fake_stops;
    ++fake_targets; MotionDrive_Command(mode,left,right,now);
}
uint32_t MissionPort_UpdateDrive(uint32_t now)
{
    SpeedControl_Update(now); return speed_control.fault;
}
void MissionPort_StopDrive(void) { ++fake_stops; MotionDrive_Stop(); }
void MissionPort_CalibrationStart(uint32_t now) { (void)now; ++fake_cal_starts; fake_input.calibrating=1U; }
void MissionPort_CalibrationCancel(void) { ++fake_cal_cancels; fake_input.calibrating=0U; }
void MissionPort_NotifyPoint(MissionPoint point, uint32_t now)
{
    (void)now; assert(fake_notifications < 16U);
    fake_points[fake_notifications++]=point;
}
void MissionPort_Draw(const MissionDiagnostics *d)
{
    assert(d->sequence_begin==d->sequence_end && !(d->sequence_begin & 1U));
}
void Fake_Tick(uint32_t now) { Mission_Update(now); }
