#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t previous_left;
    uint16_t previous_right;
    uint32_t previous_tick;
    uint32_t left_motion_at;
    uint32_t right_motion_at;
    uint64_t left_travel_counts;
    uint64_t right_travel_counts;
    uint32_t distance_mm;
    int32_t left_speed_mm_s;
    int32_t right_speed_mm_s;
    int32_t speed_mm_s;
} Odometry;

void Odometry_Init(Odometry *odo, uint32_t now, uint16_t left, uint16_t right);
/* 使用定时器原始计数，每次调用间隔应使单轮变化小于 32768 个计数。 */
bool Odometry_Update(Odometry *odo, uint32_t now, uint16_t left, uint16_t right);
/* 新任务开始时重置里程及采样基准，不重置硬件定时器。 */
void Odometry_ResetTrip(Odometry *odo, uint32_t now, uint16_t left, uint16_t right);

#endif
