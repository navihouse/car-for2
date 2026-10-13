#include "car_control.h"
#include "car_config.h"
#include "buttons.h"

static int32_t Clamp(int32_t value, int32_t low, int32_t high)
{
    return (value < low) ? low : ((value > high) ? high : value);
}

void CarControl_Stop(CarControl *car, CarState state)
{
    car->state = state;
    car->left_pwm = car->right_pwm = 0U;
    car->left_target_mm_s = car->right_target_mm_s = 0;
    car->left_integral_milli = car->right_integral_milli = 0;
    car->previous_error_valid = false;
    car->line_missing = false;
}

void CarControl_Init(CarControl *car, uint32_t now, bool marker_present)
{
    *car = (CarControl){0};
    car->state = CAR_STOPPED;
    car->set_speed_mm_s = CAR_SPEED_DEFAULT_MM_S;
    car->target_laps = CAR_LAPS_DEFAULT;
    car->marker_raw = car->marker_stable = marker_present;
    car->marker_changed_at = now;
    car->last_control_at = car->last_speed_at = now;
}

static void UpdateMarker(CarControl *car, uint32_t now, bool present)
{
    if (present != car->marker_raw)
    {
        car->marker_raw = present;
        car->marker_changed_at = now;
    }
    if ((uint32_t)(now - car->marker_changed_at) >= CAR_MARKER_DEBOUNCE_MS)
    {
        car->marker_stable = car->marker_raw;
    }
}

static uint16_t SpeedPi(int32_t target, int32_t measured, uint32_t dt, int32_t *integral)
{
    int32_t error;
    int32_t proposed;
    int32_t duty;
    if (target <= 0)
    {
        *integral = 0;
        return 0U;
    }
    error = Clamp(target - measured, -2000, 2000);
    proposed = Clamp(*integral + error * CAR_SPEED_KI * (int32_t)dt,
                     -CAR_SPEED_INTEGRAL_LIMIT * 1000, CAR_SPEED_INTEGRAL_LIMIT * 1000);
    duty = target * CAR_SPEED_FF_NUM / CAR_SPEED_FF_DEN + CAR_SPEED_KP * error + proposed / 1000;
    /* 输出饱和且误差会继续推向饱和时，不积累积分。 */
    if (!((duty > CAR_PWM_LIMIT && error > 0) || (duty < 0 && error < 0)))
    {
        *integral = proposed;
    }
    duty = target * CAR_SPEED_FF_NUM / CAR_SPEED_FF_DEN + CAR_SPEED_KP * error + *integral / 1000;
    return (uint16_t)Clamp(duty, 0, CAR_PWM_LIMIT);
}

static void SetTargets(CarControl *car, uint32_t now, int32_t left, int32_t right)
{
    if (car->left_target_mm_s < 50 || left < 50)
    {
        car->left_command_since = now;
    }
    if (car->right_target_mm_s < 50 || right < 50)
    {
        car->right_command_since = now;
    }
    car->left_target_mm_s = left;
    car->right_target_mm_s = right;
    if (left == 0)
    {
        car->left_pwm = 0U;
        car->left_integral_milli = 0;
    }
    if (right == 0)
    {
        car->right_pwm = 0U;
        car->right_integral_milli = 0;
    }
}

static void TrackLine(CarControl *car, uint32_t now, uint8_t mask)
{
    static const int16_t weights[5] = {-200, -100, 0, 100, 200};
    int32_t sum = 0, count = 0, error, derivative, correction;
    int32_t base = car->set_speed_mm_s;
    uint32_t lap_distance = car->trip_distance_mm - car->lap_distance_at;
    if (car->completed_laps + 1U == car->target_laps &&
        car->last_lap_length_mm > CAR_APPROACH_DISTANCE_MM &&
        lap_distance >= car->last_lap_length_mm - CAR_APPROACH_DISTANCE_MM &&
        base > CAR_MARKER_SPEED_MM_S)
    {
        base = CAR_MARKER_SPEED_MM_S;
    }
    if (mask == 0U)
    {
        if (!car->line_missing)
        {
            car->line_missing = true;
            car->line_missing_at = now;
        }
        if ((uint32_t)(now - car->line_missing_at) >= CAR_LINE_LOST_TIMEOUT_MS)
        {
            CarControl_Stop(car, CAR_FAULT_LINE_LOST);
            return;
        }
        SetTargets(car, now, car->last_direction < 0 ? 0 : CAR_SEARCH_SPEED_MM_S,
                   car->last_direction > 0 ? 0 : CAR_SEARCH_SPEED_MM_S);
        car->previous_error_valid = false;
        return;
    }
    car->line_missing = false;
    if (mask == 0x1FU)
    {
        SetTargets(car, now, CAR_MARKER_SPEED_MM_S, CAR_MARKER_SPEED_MM_S);
        car->previous_error_valid = false;
        return;
    }
    for (uint8_t i = 0U; i < 5U; ++i)
    {
        if ((mask & (1U << i)) != 0U)
        {
            sum += weights[i];
            ++count;
        }
    }
    error = sum / count;
    derivative = car->previous_error_valid ? error - car->previous_error : 0;
    correction = CAR_STEER_KP * error + CAR_STEER_KD * derivative;
    base -= ((error < 0) ? -error : error) / CAR_STEER_SLOWDOWN_DIVISOR;
    SetTargets(car, now, Clamp(base + correction, 0, CAR_WHEEL_TARGET_MAX_MM_S),
               Clamp(base - correction, 0, CAR_WHEEL_TARGET_MAX_MM_S));
    car->previous_error = (int16_t)error;
    car->previous_error_valid = true;
    if (error != 0)
    {
        car->last_direction = error < 0 ? -1 : 1;
    }
}

void CarControl_Update(CarControl *car, uint32_t now, const CarInput *input)
{
    uint32_t dt;
    car->reset_trip = false;
    car->trip_distance_mm = input->distance_mm;
    UpdateMarker(car, now, input->marker_present);

    /* 先处理启动/停止；同一时刻按多个键时启停优先。 */
    if ((input->key_events & CAR_KEY_START) != 0U)
    {
        if (car->state == CAR_RUNNING)
        {
            CarControl_Stop(car, CAR_STOPPED);
            return;
        }
        if (!car->marker_stable || !input->marker_present)
        {
            CarControl_Stop(car, CAR_START_REQUIRED);
            return;
        }
        CarControl_Stop(car, CAR_RUNNING);
        car->completed_laps = 0U;
        car->last_direction = 0;
        car->previous_error = 0;
        car->lap_armed = false;
        car->trip_distance_mm = car->lap_distance_at = 0U;
        car->last_lap_length_mm = 0U;
        car->lap_started_at = car->last_speed_at = now;
        car->left_command_since = car->right_command_since = now;
        car->last_control_at = now - CAR_CONTROL_PERIOD_MS;
        car->reset_trip = true;
        return;
    }
    if ((input->key_events & CAR_KEY_SPEED) != 0U)
    {
        car->set_speed_mm_s = (car->set_speed_mm_s + CAR_SPEED_STEP_MM_S > CAR_SPEED_MAX_MM_S)
                             ? CAR_SPEED_MIN_MM_S : car->set_speed_mm_s + CAR_SPEED_STEP_MM_S;
    }
    if ((input->key_events & CAR_KEY_LAPS) != 0U && car->state != CAR_RUNNING)
    {
        car->target_laps = car->target_laps >= CAR_LAPS_MAX ? CAR_LAPS_MIN : car->target_laps + 1U;
    }
    if (car->state != CAR_RUNNING)
    {
        return;
    }

    /* 起始位置不计圈；离开标记并走够最小距离/时间后才允许再次计圈。 */
    if (!car->marker_stable &&
        (uint32_t)(now - car->lap_started_at) >= CAR_MIN_LAP_TIME_MS &&
        car->trip_distance_mm - car->lap_distance_at >= CAR_MIN_LAP_DISTANCE_MM)
    {
        car->lap_armed = true;
    }
    if (car->marker_stable && input->marker_present && car->lap_armed)
    {
        ++car->completed_laps;
        car->last_lap_length_mm = car->trip_distance_mm - car->lap_distance_at;
        car->lap_distance_at = car->trip_distance_mm;
        car->lap_started_at = now;
        car->lap_armed = false;
        if (car->completed_laps >= car->target_laps)
        {
            CarControl_Stop(car, CAR_DONE);
            return;
        }
    }
    if ((uint32_t)(now - car->lap_started_at) >= CAR_MAX_LAP_TIME_MS)
    {
        CarControl_Stop(car, CAR_FAULT_MARKER);
        return;
    }
    if ((uint32_t)(now - car->last_control_at) >= CAR_CONTROL_PERIOD_MS)
    {
        car->last_control_at = now;
        TrackLine(car, now, input->black_mask & 0x1FU);
    }
    if (car->state != CAR_RUNNING)
    {
        return;
    }
    if ((car->left_target_mm_s >= 50 &&
         (uint32_t)(now - car->left_command_since) >= CAR_ENCODER_STALL_MS &&
         (uint32_t)(now - input->left_motion_at) >= CAR_ENCODER_STALL_MS) ||
        (car->right_target_mm_s >= 50 &&
         (uint32_t)(now - car->right_command_since) >= CAR_ENCODER_STALL_MS &&
         (uint32_t)(now - input->right_motion_at) >= CAR_ENCODER_STALL_MS))
    {
        CarControl_Stop(car, CAR_FAULT_ENCODER);
        return;
    }
    if (input->speed_sample)
    {
        if ((car->left_target_mm_s > 0 && input->left_speed_mm_s < -20) ||
            (car->right_target_mm_s > 0 && input->right_speed_mm_s < -20))
        {
            CarControl_Stop(car, CAR_FAULT_ENCODER);
            return;
        }
        dt = (uint32_t)(now - car->last_speed_at);
        car->last_speed_at = now;
        if (dt > 4U * CAR_SPEED_PERIOD_MS)
        {
            dt = 4U * CAR_SPEED_PERIOD_MS;
        }
        car->left_pwm = SpeedPi(car->left_target_mm_s, input->left_speed_mm_s, dt, &car->left_integral_milli);
        car->right_pwm = SpeedPi(car->right_target_mm_s, input->right_speed_mm_s, dt, &car->right_integral_milli);
    }
}

const char *CarControl_StateName(CarState state)
{
    switch (state)
    {
        case CAR_STOPPED: return "READY";
        case CAR_RUNNING: return "RUN";
        case CAR_DONE: return "DONE";
        case CAR_START_REQUIRED: return "MOVE TO START";
        case CAR_FAULT_LINE_LOST: return "LINE LOST";
        case CAR_FAULT_ENCODER: return "ENCODER FAULT";
        case CAR_FAULT_MARKER: return "MARKER LOST";
        case CAR_FAULT_OLED: return "OLED FAULT";
        default: return "FAULT";
    }
}
