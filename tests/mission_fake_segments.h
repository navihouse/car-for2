#ifndef MISSION_FAKE_SEGMENTS_H
#define MISSION_FAKE_SEGMENTS_H
#include "mission_segment.h"
extern uint32_t fake_enter_count[4], fake_exit_count[4], fake_update_count[4];
extern MissionSegmentStatus fake_segment_status[4];
extern MissionInput fake_entries[4];
void FakeSegments_Reset(void);
#endif
