#include "motion_control.h"

void MotionDistance_Begin(MissionDistance *d, const MissionInput *in)
{
    *d = (MissionDistance){in->left_counts, in->right_counts, 1U};
}
float MotionDistance_GetMm(const MissionDistance *d, const MissionInput *in)
{
    if (!d->active) return 0.0f;
    /* Subtract int64 counts BEFORE converting to float, preserving small trips
     * even after a long uptime. Signed wheel means cancel in-place rotation. */
    return 0.5f * ((float)(in->left_counts - d->left_origin) / MOTION_LEFT_COUNTS_PER_MM +
                   (float)(in->right_counts - d->right_origin) / MOTION_RIGHT_COUNTS_PER_MM);
}
