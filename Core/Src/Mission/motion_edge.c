#include "motion_control.h"
#include "mission_config.h"

void MotionEdge_Reset(MissionEdge *e) { *e = (MissionEdge){0}; }
void MotionEdge_SeedBlack(MissionEdge *e, const MissionInput *in)
{
    *e = (MissionEdge){.last_sequence=in->line_sequence, .have_sequence=1U,
        .run_known=1U, .run_black=1U, .run_frames=MOTION_POINT_CONFIRM_FRAMES,
        .stable_known=1U, .stable_black=1U};
}
uint8_t MotionEdge_Update(MissionEdge *e, const MissionInput *in, float mm,
                         float minimum, float maximum, uint8_t target_black)
{
    uint8_t window = mm >= minimum && mm <= maximum;
    if (!window) { e->candidate=0U; e->candidate_frames=0U; }
    /* The caller also rejects stale sensors; this gate keeps the standalone
     * detector from treating a driver's stale mask=0 as white. */
    if (!in->line_calibrated || !in->line_sequence || in->calibrating ||
        in->line_age_ms > MISSION_LINE_MAX_AGE_MS) {
        e->candidate=0U; e->candidate_frames=0U; return 0U;
    }
    if (e->have_sequence && e->last_sequence == in->line_sequence) return 0U;
    e->have_sequence=1U; e->last_sequence=in->line_sequence;
    uint8_t black = in->line_mask != 0U;
    if (!e->run_known || e->run_black != black) {
        e->run_known=1U; e->run_black=black; e->run_frames=1U;
        /* Eligibility is decided on the FIRST raw frame of the transition.
         * Opening the window halfway through an old transition cannot arm it. */
        e->candidate=window && e->stable_known && e->stable_black != target_black && black==target_black;
    } else if (e->run_frames < MOTION_POINT_CONFIRM_FRAMES) ++e->run_frames;
    e->candidate_frames=e->candidate ? e->run_frames : 0U;
    if (e->run_frames < MOTION_POINT_CONFIRM_FRAMES) return 0U;
    uint8_t event = e->candidate && window && black==target_black;
    e->stable_known=1U; e->stable_black=black;
    e->candidate=0U;
    return event;
}
