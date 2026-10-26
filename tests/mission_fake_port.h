#ifndef MISSION_FAKE_PORT_H
#define MISSION_FAKE_PORT_H
#include "mission_port.h"
#include "encoder.h"
#include "speed_control.h"
extern MissionInput fake_input;
extern uint32_t fake_pressed, fake_starts, fake_stops, fake_targets, fake_resets;
extern uint32_t fake_services, fake_captures, fake_notifications, fake_motor_writes;
extern uint32_t fake_cal_starts, fake_cal_cancels, fake_imu_initializations;
extern MissionPoint fake_points[16];
void Fake_Tick(uint32_t now_ms);
#endif
