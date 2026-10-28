/* Host test for the turn breakaway escalation in SpeedControl.
 *
 * The fine approach fades the fixed 30% friction kick so a small yaw-rate
 * command is not overwhelmed. That faded value can sit below static friction,
 * so the kick has to come back while the wheel is really stalled, and has to
 * fade again once the wheel turns. The 700 ms stall guard must still fire. */
#include <stdio.h>
#include <stdint.h>
#include "speed_control.h"
#include "encoder.h"

volatile Encoder_Data encoder_data;

static int32_t out_left, out_right;

void Motor_Stop(void) { out_left = out_right = 0; }
void Motor_SetOutput(int16_t left, int16_t right) { out_left = left; out_right = right; }
void Encoder_Reset(uint32_t now)
{
    (void)now;
    encoder_data = (Encoder_Data){0};
}

static int failures;
static uint32_t now_ms;

#define CHECK(condition)                                                      \
    do {                                                                      \
        if (!(condition)) {                                                   \
            printf("FAIL %s:%u %s\n", __func__, (unsigned)__LINE__, #condition); \
            ++failures;                                                       \
        }                                                                     \
    } while (0)

static int32_t Abs32(int32_t value) { return value < 0 ? -value : value; }

/* One 10 ms encoder sample. Delta is counts in this period, so cps = 100*delta. */
static void Tick(int32_t left_delta, int32_t right_delta)
{
    now_ms += 10U;
    encoder_data.sample_count++;
    encoder_data.sample_ms = 10U;
    encoder_data.left_delta = left_delta;
    encoder_data.right_delta = right_delta;
    encoder_data.left_cps = (float)left_delta * 100.0f;
    encoder_data.right_cps = (float)right_delta * 100.0f;
    encoder_data.left_total += left_delta;
    encoder_data.right_total += right_delta;
    SpeedControl_Update(now_ms);
}

static void BeginTurn(void)
{
    now_ms = 1000U;
    encoder_data = (Encoder_Data){0};
    SpeedControl_StartDifferentialPreserveCounts(300.0f, 300.0f, now_ms);
    SpeedControl_EnterTurn();
    SpeedControl_SetTurnTargets(-MOTION_TURN_APPROACH_MIN_CPS, MOTION_TURN_APPROACH_MIN_CPS);
}

static void TestStalledWheelRestoresKick(void)
{
    int32_t faded, boosted;
    BeginTurn();
    for (uint32_t i = 0U; i < 10U; ++i) Tick(0, 0);
    faded = Abs32(out_left);
    /* 100 ms in: still inside the breakaway window, so the faded kick applies. */
    CHECK(faded >= 8);
    CHECK(faded <= 22);
    for (uint32_t i = 0U; i < 30U; ++i) Tick(0, 0);
    boosted = Abs32(out_left);
    printf("stalled turn: faded duty %d%%, boosted duty %d%%\n", (int)faded, (int)boosted);
    /* 400 ms of stall: the full kick must be back, well above the faded value. */
    CHECK(boosted >= 28);
    CHECK(boosted > faded + 8);
    CHECK(speed_control.fault == SPEED_FAULT_NONE);
}

static void TestTurningWheelKeepsFadedKick(void)
{
    BeginTurn();
    for (uint32_t i = 0U; i < 40U; ++i) Tick(-1, 1);
    /* The wheel turns at 100 cps against a 120 cps command: no escalation. */
    CHECK(Abs32(out_left) >= 5);
    CHECK(Abs32(out_left) <= 20);
    CHECK(speed_control.fault == SPEED_FAULT_NONE);
}

static void TestBoostDecaysAfterBreakaway(void)
{
    int32_t boosted, released;
    BeginTurn();
    for (uint32_t i = 0U; i < 30U; ++i) Tick(0, 0);
    boosted = Abs32(out_left);
    CHECK(boosted >= 28);
    for (uint32_t i = 0U; i < 40U; ++i) Tick(-1, 1);
    released = Abs32(out_left);
    /* Escalation must not latch on: it decays once the wheel moves again. */
    CHECK(released <= 20);
    CHECK(speed_control.fault == SPEED_FAULT_NONE);
}

static void TestStallGuardStillFaults(void)
{
    BeginTurn();
    for (uint32_t i = 0U; i < 60U; ++i) Tick(0, 0);
    /* 600 ms of stall is inside the widened 700 ms budget. */
    CHECK(speed_control.fault == SPEED_FAULT_NONE);
    for (uint32_t i = 0U; i < 15U; ++i) Tick(0, 0);
    CHECK(speed_control.fault == SPEED_FAULT_LEFT_NO_FEEDBACK);
}

int main(void)
{
    TestStalledWheelRestoresKick();
    TestTurningWheelKeepsFadedKick();
    TestBoostDecaysAfterBreakaway();
    TestStallGuardStillFaults();
    if (failures) {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    printf("speed control turn breakaway: all checks passed\n");
    return 0;
}
