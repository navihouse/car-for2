#include "motion_control.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* The diagonals leave C/D on the previous segment's heading, which is NOT the
 * semicircle entry tangent (38.66 deg apart). These tests pin the arc segments
 * to steering that corner away WHILE ROLLING: the capture must never stop the
 * car, because a standstill is where stiction and skidding cost the point. The
 * plant is a first-order kinematic stand-in driven by the emitted wheel cps. */

#define TRACK_MM 130.0f
#define RAD_TO_DEG (180.0f/3.14159265f)

static int failures;
#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); ++failures; } } while (0)

static const MissionSegmentParams CB = {
    .cruise_cps = MOTION_CB_CRUISE_CPS, .timeout_ms = MOTION_ARC_TIMEOUT_MS, .kind = MOTION_KIND_ARC,
    .capture_cps = MOTION_CB_CAPTURE_CPS, .target_yaw_deg = 180.0f,
    .nominal_mm = MOTION_CB_NOMINAL_MM,
    .point_min_mm = MOTION_CB_POINT_MIN_MM, .point_max_mm = MOTION_CB_POINT_MAX_MM};
static const MissionSegmentParams DA = {
    .cruise_cps = MOTION_DA_CRUISE_CPS, .timeout_ms = MOTION_ARC_TIMEOUT_MS, .kind = MOTION_KIND_ARC,
    .capture_cps = MOTION_DA_CAPTURE_CPS, .target_yaw_deg = 0.0f,
    .nominal_mm = MOTION_DA_NOMINAL_MM,
    .point_min_mm = MOTION_DA_POINT_MIN_MM, .point_max_mm = MOTION_DA_POINT_MAX_MM};

typedef struct {
    MissionSegmentContext ctx;
    MissionInput in;
    uint32_t now_ms;
} Rig;

static void Plant(Rig *rig, MissionSegmentOutput out, uint32_t dt_ms)
{
    if (out.mode == MOTION_BRAKE) { rig->in.yaw_rate_dps = 0.0f; return; }
    float dt = (float)dt_ms * 0.001f;
    rig->in.left_counts += (int64_t)(out.left_cps * dt);
    rig->in.right_counts += (int64_t)(out.right_cps * dt);
    rig->in.left_cps = out.left_cps;
    rig->in.right_cps = out.right_cps;
    rig->in.yaw_rate_dps = MOTION_YAW_SIGN *
        (out.right_cps - out.left_cps) / MOTION_RIGHT_COUNTS_PER_MM / TRACK_MM * RAD_TO_DEG;
    rig->in.yaw_deg += rig->in.yaw_rate_dps * dt;
}

static void RigInit(Rig *rig, float yaw_deg)
{
    memset(rig, 0, sizeof *rig);
    rig->ctx.active = 1U;
    rig->ctx.from_handoff = 1U;
    rig->ctx.course_origin_deg = 0.0f;
    rig->in.yaw_deg = yaw_deg;
    rig->in.gyro_ready = 1U;
    rig->in.line_calibrated = 1U;
}

static MissionSegmentOutput RigStep(Rig *rig, const MissionSegmentParams *p)
{
    MissionSegmentOutput out = MotionArc_Update(&rig->ctx, &rig->in, p);
    Plant(rig, out, MOTION_ENCODER_PERIOD_MS);
    rig->now_ms += MOTION_ENCODER_PERIOD_MS;
    rig->in.now_ms = rig->now_ms;
    return out;
}

/* A wide single stripe is accepted during bounded capture: it is what a car
 * sitting on the C stripe sees while it is still turning. */
static void ShowWideStripe(Rig *rig)
{
    rig->in.line_mask = 0xFFU;
    for (uint32_t i = 0; i < 8U; ++i) rig->in.line_strength[i] = 800U;
    rig->in.line_sequence += 1U;
}

/* Narrow centred stripe: bits 3 and 4, equal strength -> position 0. */
static void ShowNarrowStripe(Rig *rig)
{
    rig->in.line_mask = 0x18U;
    for (uint32_t i = 0; i < 8U; ++i) rig->in.line_strength[i] = 800U;
    rig->in.line_sequence += 1U;
}

/* Entry tangent error for CB: 0 - (-38.66) = +38.66 deg, beyond the accept
 * band. The very first output must already be a forward push, never a brake or
 * an opposite-wheel pivot. */
static void TestHandoffKeepsRolling(void)
{
    Rig rig;
    RigInit(&rig, -38.66f);
    MissionSegmentOutput out = MotionArc_Enter(&rig.ctx, &rig.in, &CB);
    CHECK(rig.ctx.phase == PHASE_CAPTURE);
    CHECK(out.mode == MOTION_FORWARD);
    CHECK(out.left_cps > 0.0f && out.right_cps > 0.0f);
    /* Left is the inside wheel: it slows, the right one speeds up. */
    CHECK(out.right_cps - out.left_cps > 0.0f);
    CHECK(out.left_cps >= MOTION_LINE_MIN_WHEEL_CPS);
    CHECK(failures == 0);
}

/* The bias must use the whole differential headroom the capture speed allows,
 * otherwise the corner is not finished inside the 320 mm budget. Outside the
 * acquisition band the steering saturates at that headroom. */
static void TestBiasUsesCaptureHeadroom(void)
{
    Rig rig;
    RigInit(&rig, -38.66f);
    MissionSegmentOutput out = MotionArc_Enter(&rig.ctx, &rig.in, &CB);
    float cap = MOTION_CB_CAPTURE_CPS - MOTION_LINE_MIN_WHEEL_CPS;
    float base = 0.5f * (out.left_cps + out.right_cps);
    CHECK(fabsf(base - MOTION_CB_CAPTURE_CPS) < 1.0f);
    CHECK(fabsf(0.5f * (out.right_cps - out.left_cps) - cap) < 1.0f);
}

/* Rolling through the corner: the yaw error closes to the accept band without
 * ever braking, and the arc budget is respected. */
static void TestCornerClosesWhileMoving(void)
{
    Rig rig;
    RigInit(&rig, -38.66f);
    (void)MotionArc_Enter(&rig.ctx, &rig.in, &CB);
    MissionSegmentOutput out = {0};
    uint32_t elapsed = 0U;
    while (elapsed < 1400U) {
        ShowWideStripe(&rig);
        out = RigStep(&rig, &CB);
        elapsed += MOTION_ENCODER_PERIOD_MS;
        CHECK(out.status == SEG_RUNNING);
        if (out.status != SEG_RUNNING) return;
        CHECK(out.mode == MOTION_FORWARD && out.left_cps > 0.0f && out.right_cps > 0.0f);
        if (out.status != SEG_RUNNING) return;
        if (fabsf(rig.ctx.motion.heading_error_deg) <= MOTION_CAPTURE_HEADING_ACCEPT_DEG) break;
    }
    CHECK(fabsf(rig.ctx.motion.heading_error_deg) <= MOTION_CAPTURE_HEADING_ACCEPT_DEG);
    CHECK(rig.ctx.phase == PHASE_CAPTURE);
    CHECK(rig.ctx.motion.distance_mm <= MOTION_CAPTURE_MAX_MM);
    CHECK(elapsed <= MOTION_CAPTURE_TIMEOUT_MS);
}

/* Once the corner is closed and a narrow stripe is centred, the capture hands
 * over to line following on the very same forward push. */
static void TestNarrowStripeConfirmsFollow(void)
{
    Rig rig;
    RigInit(&rig, -38.66f);
    (void)MotionArc_Enter(&rig.ctx, &rig.in, &CB);
    for (uint32_t i = 0; i < 140U && fabsf(rig.ctx.motion.heading_error_deg) >
         MOTION_CAPTURE_HEADING_ACCEPT_DEG; ++i) {
        ShowWideStripe(&rig);
        if (RigStep(&rig, &CB).status != SEG_RUNNING) return;
    }
    MissionSegmentOutput out = {0};
    for (uint32_t i = 0; i < MOTION_CAPTURE_CONFIRM_FRAMES + 2U; ++i) {
        ShowNarrowStripe(&rig);
        out = RigStep(&rig, &CB);
    }
    CHECK(rig.ctx.phase == PHASE_FOLLOW);
    CHECK(out.status == SEG_RUNNING && out.mode == MOTION_FORWARD);
    CHECK(out.left_cps > 0.0f && out.right_cps > 0.0f);
}

/* An aligned arrival no longer runs straight: the capture rolls along the arc,
 * so the curvature feed-forward is already in the differential even when the
 * car is exactly on the tangent. */
static void TestAlignedEntryKeepsCurvatureBias(void)
{
    Rig rig;
    RigInit(&rig, 0.0f);
    MissionSegmentOutput out = MotionArc_Enter(&rig.ctx, &rig.in, &CB);
    float expect = -2.0f * MOTION_LINE_CURVATURE_FF * MOTION_CB_CAPTURE_CPS;
    CHECK(rig.ctx.phase == PHASE_CAPTURE);
    CHECK(out.mode == MOTION_FORWARD);
    CHECK(fabsf((out.right_cps - out.left_cps) - expect) < 1.0f);
    CHECK(out.left_cps > 0.0f && out.right_cps > 0.0f);
}

/* DA is the mirror case: tangent 180 deg, arriving on the BD exit heading
 * (MOTION_BD_TARGET_DEG), so the left wheel speeds up and the right one slows. */
static void TestDaRollsTheOtherWay(void)
{
    Rig rig;
    RigInit(&rig, MOTION_BD_TARGET_DEG);
    MissionSegmentOutput out = MotionArc_Enter(&rig.ctx, &rig.in, &DA);
    CHECK(rig.ctx.phase == PHASE_CAPTURE);
    CHECK(out.mode == MOTION_FORWARD);
    CHECK(out.left_cps > 0.0f && out.right_cps > 0.0f);
    CHECK(out.left_cps - out.right_cps > 0.0f);
}

/* Single stripe under probe `index`: position is index*1000-3500, so index 6 is
 * +2500 - an edge stripe, which the capture must now accept. */
static void ShowStripeAt(Rig *rig, uint32_t index)
{
    rig->in.line_mask = 1U << index;
    for (uint32_t i = 0; i < 8U; ++i) rig->in.line_strength[i] = (i == index) ? 1000U : 0U;
    rig->in.line_sequence += 1U;
}

/* The real C handoff: after the point bar the sensor sees white for a while and
 * the arc line first shows up at the edge of the bar. Neither may end the
 * segment - the capture has to steer the known semicircle until the line is
 * confirmed, and only the capture budget may report a failure. */
static void TestBlindZoneThenEdgeStripeConfirms(void)
{
    Rig rig;
    RigInit(&rig, -38.66f);
    MissionSegmentOutput out = MotionArc_Enter(&rig.ctx, &rig.in, &CB);
    /* 600 ms of all-white travel: twice the old 350 ms no-line grace. */
    for (uint32_t i = 0; i < 60U; ++i) {
        rig.in.line_mask = 0U;
        for (uint32_t k = 0; k < 8U; ++k) rig.in.line_strength[k] = 0U;
        rig.in.line_sequence += 1U;
        out = RigStep(&rig, &CB);
        if (out.status != SEG_RUNNING) { CHECK(out.fault == MOTION_FAULT_CAPTURE); return; }
        CHECK(out.mode == MOTION_FORWARD && out.left_cps > 0.0f && out.right_cps > 0.0f);
    }
    CHECK(rig.ctx.phase == PHASE_CAPTURE);
    CHECK(out.status == SEG_RUNNING);
    for (uint32_t i = 0; i < MOTION_CAPTURE_CONFIRM_FRAMES + 1U; ++i) {
        ShowStripeAt(&rig, 6U);
        out = RigStep(&rig, &CB);
    }
    CHECK(rig.ctx.phase == PHASE_FOLLOW);
    CHECK(out.status == SEG_RUNNING);
}

/* The arc stripe can appear before the entry-heading gate accepts FOLLOW.
 * Its observed side must win over an opposing tangent bias immediately. */
static void TestStripeWinsBeforeHeadingConfirmation(void)
{
    const MissionSegmentParams *segments[2]={&CB,&DA};
    const float yaw[2]={-30.0f,210.0f};
    const uint32_t probe[2]={5U,2U};
    for (uint32_t i=0U;i<2U;++i) {
        Rig rig;
        RigInit(&rig,yaw[i]);
        ShowWideStripe(&rig);
        (void)MotionArc_Enter(&rig.ctx,&rig.in,segments[i]);
        rig.in.now_ms=MOTION_CAPTURE_LINE_PERIOD_MS;
        ShowStripeAt(&rig,probe[i]);
        MissionSegmentOutput out=MotionArc_Update(&rig.ctx,&rig.in,segments[i]);
        CHECK(out.status==SEG_RUNNING && out.mode==MOTION_FORWARD);
        CHECK(rig.ctx.phase==PHASE_CAPTURE && rig.ctx.motion.capture_frames==0U);
        CHECK(out.left_cps>0.0f && out.right_cps>0.0f);
        if (i==0U) CHECK(out.left_cps>out.right_cps); /* stripe right -> right */
        else CHECK(out.right_cps>out.left_cps);       /* stripe left -> left */
    }
}

/* A left-to-right sweep over two fresh frames fits entirely inside the normal
 * 50 ms PD period. Both sides must change the output before FOLLOW, while
 * repeated foreground polls must neither count nor differentiate a frame twice. */
static void TestFreshCaptureFramesReactBeforeFollow(void)
{
    Rig rig;
    RigInit(&rig,0.0f);
    ShowWideStripe(&rig);
    (void)MotionArc_Enter(&rig.ctx,&rig.in,&CB);
    rig.in.now_ms=MOTION_CAPTURE_LINE_PERIOD_MS;
    ShowStripeAt(&rig,2U);
    MissionSegmentOutput out=MotionArc_Update(&rig.ctx,&rig.in,&CB);
    CHECK(rig.ctx.phase==PHASE_CAPTURE && rig.ctx.motion.capture_frames==1U);
    CHECK(out.right_cps>out.left_cps);
    CHECK(rig.ctx.motion.derivative==0.0f); /* no point-bar derivative kick */
    rig.in.now_ms+=MOTION_CAPTURE_LINE_PERIOD_MS;
    ShowStripeAt(&rig,5U);
    out=MotionArc_Update(&rig.ctx,&rig.in,&CB);
    CHECK(rig.ctx.phase==PHASE_CAPTURE && rig.ctx.motion.capture_frames==2U);
    CHECK(out.left_cps>out.right_cps);
    CHECK(rig.ctx.motion.line_position==1500.0f);
    float derivative=rig.ctx.motion.derivative;
    float left=out.left_cps,right=out.right_cps;
    for (uint32_t i=0U;i<10U;++i) {
        out=MotionArc_Update(&rig.ctx,&rig.in,&CB);
        CHECK(rig.ctx.motion.capture_frames==2U && rig.ctx.phase==PHASE_CAPTURE);
        CHECK(rig.ctx.motion.derivative==derivative);
        CHECK(out.left_cps==left && out.right_cps==right);
    }
    rig.in.now_ms+=MOTION_CAPTURE_LINE_PERIOD_MS;
    ShowNarrowStripe(&rig);
    out=MotionArc_Update(&rig.ctx,&rig.in,&CB);
    CHECK(rig.ctx.phase==PHASE_FOLLOW && out.status==SEG_RUNNING);
}

int main(void)
{
    TestHandoffKeepsRolling();
    TestBiasUsesCaptureHeadroom();
    TestCornerClosesWhileMoving();
    TestNarrowStripeConfirmsFollow();
    TestBlindZoneThenEdgeStripeConfirms();
    TestAlignedEntryKeepsCurvatureBias();
    TestDaRollsTheOtherWay();
    TestStripeWinsBeforeHeadingConfirmation();
    TestFreshCaptureFramesReactBeforeFollow();
    if (failures) { fprintf(stderr, "%d check(s) failed\n", failures); return 1; }
    printf("arc rolling entry: 9 cases passed\n");
    return 0;
}
