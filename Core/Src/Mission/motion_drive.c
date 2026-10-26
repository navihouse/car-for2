#include "motion_drive.h"
#include "motion_tuning.h"
#include "speed_control.h"
#include "motor.h"

static MissionMotionMode actuator_mode=MOTION_BRAKE;
MissionMotionMode MotionDrive_Mode(void) { return actuator_mode; }
void MotionDrive_Init(void)
{
    SpeedControl_Stop(); Motor_Brake(); actuator_mode=MOTION_BRAKE;
}
void MotionDrive_Stop(void)
{
    if (actuator_mode!=MOTION_BRAKE || speed_control.running) {
        SpeedControl_Stop(); Motor_Brake();
    }
    actuator_mode=MOTION_BRAKE;
}
void MotionDrive_Command(MissionMotionMode mode, float left, float right, uint32_t now)
{
    if (mode==MOTION_BRAKE) { MotionDrive_Stop(); return; }
    if (mode!=MOTION_FORWARD && mode!=MOTION_TURN) { MotionDrive_Stop(); return; }
    if (mode!=actuator_mode) {
        /* Only stopped A/B phases change profile. C/D stay FORWARD, retaining
         * the PI, encoder/filter history and speed ramp. */
        if (mode==MOTION_FORWARD) SpeedControl_StartDifferentialPreserveCounts(left,right,now);
        else {
            SpeedControl_StartDifferentialPreserveCounts(MOTION_TURN_MIN_CPS,MOTION_TURN_MIN_CPS,now);
            if (speed_control.running) SpeedControl_EnterTurn();
        }
        actuator_mode=mode;
    }
    if (mode==MOTION_TURN) SpeedControl_SetTurnTargets(left,right);
    else SpeedControl_SetDifferentialTargets(left,right);
}
