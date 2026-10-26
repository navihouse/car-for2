#include <assert.h>
#include <stdio.h>
#include "mission_fake_port.h"

int main(void)
{
    fake_pressed=0U; Mission_Init(0U);
    assert(mission_diag.state==MISSION_IDLE && !speed_control.running);
    assert(fake_starts==0U && fake_motor_writes==0U && fake_notifications==0U);
    assert(mission_diag.mode==MISSION_BUILD_MODE);
    assert(!Mission_Start(1U));
    assert(mission_diag.state==MISSION_REJECTED && mission_diag.reason==MISSION_NOT_IMPLEMENTED);
    uint32_t expected=MISSION_BUILD_MODE==0U ? 0U : MISSION_BUILD_MODE-1U;
    assert(mission_diag.segment==expected);
    for(uint32_t now=10U;now<=100U;now+=10U) Fake_Tick(now);
    fake_pressed=1U;
    for(uint32_t now=110U;now<=200U;now+=10U) Fake_Tick(now);
    fake_pressed=0U;
    for(uint32_t now=210U;now<=300U;now+=10U) Fake_Tick(now);
    assert(mission_diag.state==MISSION_REJECTED && mission_diag.reason==MISSION_NOT_IMPLEMENTED);
    assert(!fake_starts && !fake_motor_writes && !fake_resets && !fake_notifications);
    assert(!mission_diag.arrivals && !mission_diag.left_target_cps && !mission_diag.right_target_cps);
    assert(!speed_control.running && fake_imu_initializations==1U);
    /* A key held at boot must first be released, with no automatic launch. */
    fake_pressed=1U; Mission_Init(1000U);
    for(uint32_t now=1010U;now<=1100U;now+=10U) Fake_Tick(now);
    fake_pressed=0U;
    for(uint32_t now=1110U;now<=1200U;now+=10U) Fake_Tick(now);
    assert(mission_diag.state==MISSION_IDLE && !fake_starts);
    printf("placeholder mode %u: boot/API/button launch remain parked and reject unimplemented route\n", MISSION_BUILD_MODE);
    return 0;
}
