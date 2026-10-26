#include "mission_port.h"
#include "main.h"
#include "motor.h"
#include "encoder.h"
#include "speed_control.h"
#include "line_sensor.h"
#include "line_calibration.h"
#include "imu_mpu6050.h"
#include "oled.h"
#include "motion_drive.h"

static uint32_t last_draw, shown_state, shown_reason, shown_segment, shown_cal;
static uint32_t shown_phase;
static uint32_t point_lit, point_ms;
_Static_assert(SPEED_CONTROL_MAX_CPS == MOTION_MAX_CPS, "Review mission target limit");

void MissionPort_Init(uint32_t now)
{
    Motor_Init(); Encoder_Init(); MotionDrive_Init();
    LineSensor_Init(now); Imu_Init(now); OLED_Init(now);
    last_draw = now - 200U; shown_state = shown_reason = shown_segment = shown_cal = UINT32_MAX;
    shown_phase=UINT32_MAX;
    point_lit = 0U;
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
}
void MissionPort_Service(uint32_t now)
{
    /* Each producer is polled once per loop, including while parked. */
    Encoder_Update(now); LineSensor_Update(now); Imu_Update(now);
    if (point_lit && (uint32_t)(now - point_ms) >= 150U) {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        point_lit = 0U;
    }
    OLED_Update(now);
}
MissionInput MissionPort_Capture(uint32_t now)
{
    ImuHeadingSample heading;
    Imu_GetHeadingSample(now, &heading);
    MissionInput in = {
        .now_ms = now, .encoder_sequence = encoder_data.sample_count,
        .encoder_age_ms = now - Encoder_LastSampleTime(),
        .left_counts = encoder_data.left_total, .right_counts = encoder_data.right_total,
        .left_cps = encoder_data.left_cps, .right_cps = encoder_data.right_cps,
        .yaw_deg = heading.yaw_deg, .yaw_rate_dps = heading.rate_dps,
        .gyro_ready = heading.ready, .gyro_fault = heading.fault, .gyro_age_ms = heading.age_ms,
        .line_sequence = line_sensor.sample_sequence,
        .line_age_ms = line_sensor.sample_sequence ? now - line_sensor.last_frame_ms : UINT32_MAX,
        .line_calibrated = line_sensor.cal_valid, .line_mask = line_sensor.mask,
        .calibrating = LineCalibration_IsActive(), .speed_fault = speed_control.fault
    };
    for (uint32_t i = 0; i < 8U; ++i) in.line_strength[i] = line_sensor.strength[i];
    return in;
}
uint8_t MissionPort_ButtonPressed(void)
{
    return HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == GPIO_PIN_SET;
}
void MissionPort_Command(MissionMotionMode mode, float left, float right, uint32_t now)
{
    MotionDrive_Command(mode,left,right,now);
}
uint32_t MissionPort_UpdateDrive(uint32_t now) { SpeedControl_Update(now); return speed_control.fault; }
void MissionPort_StopDrive(void) { MotionDrive_Stop(); }
void MissionPort_CalibrationStart(uint32_t now) { LineCalibration_Start(now); }
void MissionPort_CalibrationCancel(void) { LineCalibration_Cancel(); }
void MissionPort_NotifyPoint(MissionPoint point, uint32_t now)
{
    (void)point;
    /* LED implementation only. Add a nonblocking buzzer here once its actual
     * board pin/driver is confirmed; never delay the motion loop for a beep. */
    point_ms = now; point_lit = 1U;
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
}
void MissionPort_Draw(const MissionDiagnostics *d)
{
    static const char *const modes[] = {"FULL", "AC", "CB", "BD", "DA"};
    static const char *const states[] = {"IDLE", "RUNNING", "DONE", "STOPPED", "REJECTED", "FAULT"};
    static const char *const segments[] = {"AC", "CB", "BD", "DA"};
    static const char *const phases[] = {"IDLE","STILL","TURN","SETTLE","STRAIGHT","CAPTURE","FOLLOW","COMPLETE","FAULT"};
    static const char *const reasons[] = {
        "READY", "NOT IMPLEMENTED", "USER STOP", "BAD CONFIG", "BAD TARGET", "SEGMENT FAULT",
        "GYRO NOT READY", "LINE NOT READY", "ENCODER STALE", "SPEED FAULT", "CALIBRATING"
    };
    uint8_t changed = shown_state != d->state || shown_reason != d->reason ||
        shown_segment != d->segment || shown_cal != d->calibrating || shown_phase!=d->phase;
    if ((!changed && (uint32_t)(d->uptime_ms - last_draw) < 200U) || !OLED_CanDraw()) return;
    OLED_Clear();
    OLED_Text(0, 0, "MISSION"); OLED_Text(0, 9, d->mode <= MISSION_MODE_DA ? modes[d->mode] : "INVALID");
    OLED_Text(1, 0, d->state <= MISSION_FAULT ? states[d->state] : "INVALID");
    OLED_Text(1, 13, d->segment < SEGMENT_COUNT ? segments[d->segment] : "??");
    OLED_Text(2, 0, d->reason <= MISSION_CALIBRATING ? reasons[d->reason] : "UNKNOWN");
    OLED_Text(3, 0, d->phase<=PHASE_FAULT ? phases[d->phase] : "INVALID");
    OLED_Text(3, 11, "MM:"); OLED_Number(3,14,d->distance_mm);
    OLED_Text(4, 0, "WIN:"); OLED_Number(4,4,d->point_min_mm); OLED_Text(4,9,"/"); OLED_Number(4,10,d->point_max_mm);
    OLED_Text(5, 0, "DEG:"); OLED_Number(5,4,d->target_yaw_mdeg/1000); OLED_Text(5,10,"ERR:"); OLED_Number(5,14,d->heading_error_mdeg/1000);
    OLED_Text(6, 0, "CONF:"); OLED_Number(6,5,(int32_t)d->point_confirm_frames); OLED_Text(6,9,"PTS:"); OLED_Number(6,13,(int32_t)d->arrivals);
    OLED_Text(7, 0, d->calibrating ? "CALIBRATING / CANCEL" : (d->state==MISSION_RUNNING ? "PRESS: STOP" : "CLICK START HOLD6 CAL"));
    OLED_Present();
    last_draw = d->uptime_ms; shown_state = d->state; shown_reason = d->reason;
    shown_segment = d->segment; shown_cal = d->calibrating;
    shown_phase=d->phase;
}
