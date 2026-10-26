#include <assert.h>
#include <stdio.h>
#include "mission_fake_port.h"

int main(void)
{
    fake_pressed=0U; Mission_Init(0U);
    assert(mission_diag.state==MISSION_IDLE && !speed_control.running);
    assert(!fake_starts && !fake_motor_writes && !fake_notifications);
    assert(mission_diag.mode==MISSION_BUILD_MODE && mission_diag.version==2U);
    /* Every actual leg needs calibrated, fresh gray data, even on white. */
    fake_input.line_calibrated=0U; assert(!Mission_Start(1U));
    assert(mission_diag.reason==MISSION_LINE_UNAVAILABLE && !fake_starts);
    fake_input.line_calibrated=1U; fake_input.gyro_ready=0U;
    assert(!Mission_Start(2U) && mission_diag.reason==MISSION_GYRO_UNAVAILABLE);
    fake_input.gyro_ready=1U;
    uint8_t arc=MISSION_BUILD_MODE==MISSION_MODE_CB || MISSION_BUILD_MODE==MISSION_MODE_DA;
    if (arc) {
        assert(!Mission_Start(3U) && mission_diag.reason==MISSION_LINE_UNAVAILABLE);
        fake_input.line_mask=0x18U; fake_input.line_strength[3]=fake_input.line_strength[4]=1000U;
    }
    assert(Mission_Start(4U));
    assert(mission_diag.state==MISSION_RUNNING && mission_diag.reason==MISSION_OK);
    if (arc) assert(mission_diag.phase==PHASE_CAPTURE && fake_starts==1U);
    else assert(mission_diag.phase==PHASE_STILL && !fake_starts && !fake_motor_writes);
    Mission_Stop(4U); assert(!speed_control.running && !fake_resets && !fake_notifications);
    /* A key held at boot must first be released; no automatic launch. */
    fake_pressed=1U; Mission_Init(1000U);
    for(uint32_t now=1010U;now<=1100U;now+=10U) Fake_Tick(now);
    fake_pressed=0U;
    for(uint32_t now=1110U;now<=1200U;now+=10U) Fake_Tick(now);
    assert(mission_diag.state==MISSION_IDLE && !fake_starts && !speed_control.running);
    printf("mode %u: real algorithms, parked boot, sensor preflight and explicit start passed\n",MISSION_BUILD_MODE);
    return 0;
}
