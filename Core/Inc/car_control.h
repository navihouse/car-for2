#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    CAR_STOPPED = 0, CAR_RUNNING, CAR_DONE, CAR_START_REQUIRED,
    CAR_FAULT_LINE_LOST, CAR_FAULT_ENCODER, CAR_FAULT_MARKER, CAR_FAULT_OLED
} CarState;

typedef struct {
    uint8_t black_mask;
    uint8_t key_events;
    bool marker_present;
    bool speed_sample;
    uint32_t distance_mm;
    int32_t left_speed_mm_s;
    int32_t right_speed_mm_s;
    uint32_t left_motion_at;
    uint32_t right_motion_at;
} CarInput;

typedef struct {
    CarState state;
    uint16_t set_speed_mm_s;
    uint8_t target_laps;
    uint8_t completed_laps;
    uint16_t left_pwm;
    uint16_t right_pwm;
    int32_t left_target_mm_s;
    int32_t right_target_mm_s;
    int32_t left_integral_milli;
    int32_t right_integral_milli;
    int16_t previous_error;
    int8_t last_direction;
    bool previous_error_valid;
    bool line_missing;
    bool marker_raw;
    bool marker_stable;
    bool lap_armed;
    bool reset_trip; /* 单次请求，板级主循环收到后重置编码器里程。 */
    uint32_t marker_changed_at;
    uint32_t line_missing_at;
    uint32_t last_control_at;
    uint32_t last_speed_at;
    uint32_t lap_started_at;
    uint32_t lap_distance_at;
    uint32_t last_lap_length_mm;
    uint32_t trip_distance_mm;
    uint32_t left_command_since;
    uint32_t right_command_since;
} CarControl;

void CarControl_Init(CarControl *car, uint32_t now, bool marker_present);
void CarControl_Update(CarControl *car, uint32_t now, const CarInput *input);
void CarControl_Stop(CarControl *car, CarState state);
const char *CarControl_StateName(CarState state);
#endif
