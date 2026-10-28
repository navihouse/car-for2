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

static void PointSignal(uint8_t on)
{
    HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,on ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,
        (on == (MISSION_BUZZER_ACTIVE_HIGH != 0)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
void MissionPort_Init(uint32_t now)
{
    Motor_Init(); Encoder_Init(); MotionDrive_Init();
    /* Same PA15 buzzer mapping as car-02-. HAL MSP has disabled JTAG while
     * retaining PA13/PA14 SWD. Set the idle level before enabling output. */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    PointSignal(0U);
    GPIO_InitTypeDef buzzer={0};buzzer.Pin=GPIO_PIN_15;
    buzzer.Mode=GPIO_MODE_OUTPUT_PP;buzzer.Pull=GPIO_NOPULL;buzzer.Speed=GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA,&buzzer);
    LineSensor_Init(now); Imu_Init(now); OLED_Init(now);
    last_draw = now - 200U; shown_state = shown_reason = shown_segment = shown_cal = UINT32_MAX;
    shown_phase=UINT32_MAX;
    point_lit = 0U;
    PointSignal(0U);
}
void MissionPort_Service(uint32_t now)
{
    /* Each producer is polled once per loop, including while parked. */
    Encoder_Update(now); LineSensor_Update(now);
    uint8_t wheels_still = encoder_data.sample_count != 0U &&
        (uint32_t)(now - Encoder_LastSampleTime()) <= 30U &&
        encoder_data.left_delta == 0 && encoder_data.right_delta == 0 &&
        !speed_control.running;
    Imu_Update(now, wheels_still);
    if (point_lit && (uint32_t)(now - point_ms) >= MISSION_POINT_SIGNAL_MS) {
        PointSignal(0U);
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
    /* Nonblocking C/B/D/A sound+light pulse, including arrivals whose
     * incoming segment fails. Completion events are owned by the scheduler. */
    point_ms = now; point_lit = 1U;
    PointSignal(1U);
}
void MissionPort_Draw(const MissionDiagnostics *d)
{
    static const char *const modes[] = {"FULL", "AC", "CB", "BD", "DA"};
    static const char *const states[] = {"IDLE", "RUNNING", "DONE", "STOPPED", "REJECTED", "FAULT"};
    static const char *const segments[] = {"AC", "CB", "BD", "DA"};
    static const char *const phases[] = {"IDLE","STILL","TURN","SETTLE","STRAIGHT","CAPTURE","FOLLOW","COMPLETE","FAULT"};
    static const char *const points[] = {"-","A","B","C","D"};
    static const char *const faults[] = {"NONE","CONFIG","DISTANCE","TIMEOUT","TURN TIMEOUT",
        "TURN DIRECTION","TURN TRANSLATION","GYRO RESPONSE","WHEEL SYNC","HEADING",
        "CAPTURE TIME/DIST","NO LINE","LINE AMBIGUOUS"};
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
    const char *reason=d->reason <= MISSION_CALIBRATING ? reasons[d->reason] : "UNKNOWN";
    if (d->reason==MISSION_SEGMENT_FAULT && d->segment_fault<sizeof(faults)/sizeof(faults[0]))
        reason=faults[d->segment_fault];
    OLED_Text(2, 0, reason);
    OLED_Text(3, 0, d->phase<=PHASE_FAULT ? phases[d->phase] : "INVALID");
    OLED_Text(3, 11, "MM:"); OLED_Number(3,14,d->distance_mm);
    OLED_Text(4, 0, "WIN:"); OLED_Number(4,4,d->point_min_mm); OLED_Text(4,9,"/"); OLED_Number(4,10,d->point_max_mm);
    OLED_Text(5, 0, "DEG:"); OLED_Number(5,4,d->target_yaw_mdeg/1000); OLED_Text(5,10,"ERR:"); OLED_Number(5,14,d->heading_error_mdeg/1000);
    OLED_Text(6,0,"P:"); OLED_Text(6,2,d->last_point<=POINT_D ? points[d->last_point] : "?");
    OLED_Text(6,4,"N:"); OLED_Number(6,6,(int32_t)d->arrivals);
    OLED_Text(6,8,"M:"); OLED_Number(6,10,(int32_t)d->line_mask);
    OLED_Text(6,15,"C:"); OLED_Number(6,17,(int32_t)d->point_confirm_frames);
    OLED_Text(7, 0, d->calibrating ? "CALIBRATING / CANCEL" : (d->state==MISSION_RUNNING ? "PRESS: STOP" : "CLICK START HOLD6 CAL"));
    OLED_Present();
    last_draw = d->uptime_ms; shown_state = d->state; shown_reason = d->reason;
    shown_segment = d->segment; shown_cal = d->calibrating;
    shown_phase=d->phase;
}
