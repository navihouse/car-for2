#include "odometry.h"
#include "car_config.h"
#include <limits.h>

static int32_t CounterDelta(uint16_t current, uint16_t previous)
{
    uint32_t value = (uint16_t)(current - previous);
    return (value < 32768U) ? (int32_t)value : (int32_t)value - 65536;
}

void Odometry_Init(Odometry *odo, uint32_t now, uint16_t left, uint16_t right)
{
    *odo = (Odometry){0};
    odo->previous_left = left;
    odo->previous_right = right;
    odo->previous_tick = now;
    odo->left_motion_at = now;
    odo->right_motion_at = now;
}

void Odometry_ResetTrip(Odometry *odo, uint32_t now, uint16_t left, uint16_t right)
{
    int32_t left_speed = odo->left_speed_mm_s;
    int32_t right_speed = odo->right_speed_mm_s;
    Odometry_Init(odo, now, left, right);
    /* 重置本次里程不伪造新的速度测量，继续保留最后一次采样值。 */
    odo->left_speed_mm_s = left_speed;
    odo->right_speed_mm_s = right_speed;
    odo->speed_mm_s = (int32_t)(((int64_t)left_speed + right_speed) / 2);
}

bool Odometry_Update(Odometry *odo, uint32_t now, uint16_t left, uint16_t right)
{
    uint32_t dt = (uint32_t)(now - odo->previous_tick);
    int32_t dl;
    int32_t dr;
    uint64_t left_um;
    uint64_t right_um;
    uint64_t distance;
    if (dt < CAR_SPEED_PERIOD_MS)
    {
        return false;
    }

    dl = CounterDelta(left, odo->previous_left) * CAR_LEFT_ENCODER_SIGN;
    dr = CounterDelta(right, odo->previous_right) * CAR_RIGHT_ENCODER_SIGN;
    odo->previous_left = left;
    odo->previous_right = right;
    odo->previous_tick = now;
    if (dl != 0)
    {
        odo->left_motion_at = now;
    }
    if (dr != 0)
    {
        odo->right_motion_at = now;
    }

    /* um/ms 数值上等于 mm/s。保留方向，便于发现编码器极性接反。 */
    odo->left_speed_mm_s = (int32_t)((int64_t)dl * CAR_LEFT_WHEEL_CIRCUMFERENCE_UM /
                                    ((int64_t)CAR_LEFT_ENCODER_CPR * dt));
    odo->right_speed_mm_s = (int32_t)((int64_t)dr * CAR_RIGHT_WHEEL_CIRCUMFERENCE_UM /
                                     ((int64_t)CAR_RIGHT_ENCODER_CPR * dt));
    odo->speed_mm_s = (int32_t)(((int64_t)odo->left_speed_mm_s + odo->right_speed_mm_s) / 2);
    odo->left_travel_counts += (uint32_t)((dl < 0) ? -dl : dl);
    odo->right_travel_counts += (uint32_t)((dr < 0) ? -dr : dr);
    left_um = odo->left_travel_counts * CAR_LEFT_WHEEL_CIRCUMFERENCE_UM / CAR_LEFT_ENCODER_CPR;
    right_um = odo->right_travel_counts * CAR_RIGHT_WHEEL_CIRCUMFERENCE_UM / CAR_RIGHT_ENCODER_CPR;
    distance = (left_um + right_um) / 2000U;
    odo->distance_mm = (distance > UINT32_MAX) ? UINT32_MAX : (uint32_t)distance;
    return true;
}
