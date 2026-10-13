#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "buttons.h"
#include "car_control.h"
#include "car_config.h"
#include "odometry.h"

typedef struct {
    CarControl car;
    CarButtons buttons;
    uint32_t now, sample_at, distance;
    uint32_t left_motion_at, right_motion_at;
    int32_t left_speed, right_speed;
    bool motion;
} Simulation;

static void Init(Simulation *sim, uint32_t now, bool marker, uint8_t held)
{
    *sim = (Simulation){0};
    sim->now = sim->sample_at = sim->left_motion_at = sim->right_motion_at = now;
    sim->left_speed = sim->right_speed = CAR_SPEED_DEFAULT_MM_S;
    sim->motion = true;
    CarControl_Init(&sim->car, now, marker);
    Buttons_Init(&sim->buttons, now, held);
}

static void Advance(Simulation *sim, uint32_t duration, uint8_t mask, bool marker, uint8_t held)
{
    for (uint32_t i = 0; i < duration; ++i)
    {
        bool sample;
        ++sim->now;
        if (sim->motion) { sim->left_motion_at = sim->right_motion_at = sim->now; }
        sample = (uint32_t)(sim->now - sim->sample_at) >= CAR_SPEED_PERIOD_MS;
        if (sample) { sim->sample_at = sim->now; }
        CarInput input = {
            .black_mask = mask, .key_events = Buttons_Update(&sim->buttons, sim->now, held),
            .marker_present = marker, .speed_sample = sample, .distance_mm = sim->distance,
            .left_speed_mm_s = sim->left_speed, .right_speed_mm_s = sim->right_speed,
            .left_motion_at = sim->left_motion_at, .right_motion_at = sim->right_motion_at
        };
        CarControl_Update(&sim->car, sim->now, &input);
        if (sim->car.reset_trip) { sim->distance = 0U; }
        assert(sim->car.left_pwm <= CAR_PWM_LIMIT && sim->car.right_pwm <= CAR_PWM_LIMIT);
        if (sim->car.state != CAR_RUNNING)
        {
            assert(sim->car.left_pwm == 0U && sim->car.right_pwm == 0U);
        }
    }
}

static void Click(Simulation *sim, uint8_t key, uint8_t mask, bool marker)
{
    Advance(sim, CAR_BUTTON_DEBOUNCE_MS + 2U, mask, marker, key);
    Advance(sim, CAR_BUTTON_DEBOUNCE_MS + 2U, mask, marker, 0U);
}

static void Start(Simulation *sim, uint8_t laps)
{
    Init(sim, 0U, true, 0U);
    for (uint8_t i = 1U; i < laps; ++i) { Click(sim, CAR_KEY_LAPS, 0x04U, true); }
    Click(sim, CAR_KEY_START, 0x04U, true);
    assert(sim->car.state == CAR_RUNNING && sim->car.completed_laps == 0U);
}

static void TestKeysAndSettings(void)
{
    Simulation sim;
    Init(&sim, 0U, true, CAR_KEY_START | CAR_KEY_SPEED | CAR_KEY_LAPS);
    Advance(&sim, 100U, 0x04U, true, CAR_KEY_START | CAR_KEY_SPEED | CAR_KEY_LAPS);
    assert(sim.car.state == CAR_STOPPED && sim.car.set_speed_mm_s == CAR_SPEED_DEFAULT_MM_S);
    Advance(&sim, 50U, 0x04U, true, 0U);
    for (unsigned i = 0U; i < 8U; ++i)
    {
        Advance(&sim, 3U, 0x04U, true, CAR_KEY_START);
        Advance(&sim, 3U, 0x04U, true, 0U);
    }
    assert(sim.car.state == CAR_STOPPED);
    for (unsigned i = 0U; i < 10U; ++i) { Click(&sim, CAR_KEY_SPEED, 0x04U, true); }
    assert(sim.car.set_speed_mm_s == CAR_SPEED_DEFAULT_MM_S);
    for (unsigned i = 0U; i < CAR_LAPS_MAX; ++i) { Click(&sim, CAR_KEY_LAPS, 0x04U, true); }
    assert(sim.car.target_laps == CAR_LAPS_DEFAULT);
    Advance(&sim, 500U, 0x04U, true, CAR_KEY_START);
    assert(sim.car.state == CAR_RUNNING); /* 长按只启动一次。 */
    Advance(&sim, 50U, 0x04U, true, 0U);
    Click(&sim, CAR_KEY_SPEED, 0x04U, true);
    assert(sim.car.set_speed_mm_s == CAR_SPEED_DEFAULT_MM_S + CAR_SPEED_STEP_MM_S);
    Click(&sim, CAR_KEY_LAPS, 0x04U, true);
    assert(sim.car.target_laps == CAR_LAPS_DEFAULT); /* 行驶期间圈数锁定。 */
    Click(&sim, CAR_KEY_START, 0x04U, true);
    assert(sim.car.state == CAR_STOPPED);
    Click(&sim, CAR_KEY_LAPS, 0x04U, true);
    assert(sim.car.target_laps == 2U);
}

static void TestStartPositionAndRestart(void)
{
    Simulation sim;
    Init(&sim, 0U, false, 0U);
    Click(&sim, CAR_KEY_START, 0x04U, false);
    assert(sim.car.state == CAR_START_REQUIRED);
    Advance(&sim, 30U, 0x04U, true, 0U);
    assert(sim.car.state == CAR_START_REQUIRED);
    Click(&sim, CAR_KEY_START, 0x04U, true);
    assert(sim.car.state == CAR_RUNNING);
    sim.distance = 1000U;
    Click(&sim, CAR_KEY_START, 0x04U, true);
    assert(sim.car.state == CAR_STOPPED && sim.car.trip_distance_mm == 1000U);
    Click(&sim, CAR_KEY_START, 0x04U, true);
    assert(sim.distance == 0U && sim.car.completed_laps == 0U);
}

static void CompleteLap(Simulation *sim)
{
    sim->distance += 500U;
    Advance(sim, CAR_MIN_LAP_TIME_MS + 50U, 0x04U, false, 0U);
    assert(sim->car.lap_armed);
    sim->distance += 1500U;
    Advance(sim, CAR_MARKER_DEBOUNCE_MS + 5U, 0x04U, true, 0U);
}

static void TestLapCountingAndFinish(void)
{
    Simulation sim;
    Start(&sim, 3U);
    Advance(&sim, 1200U, 0x04U, true, 0U);
    assert(sim.car.completed_laps == 0U); /* 起点停留不算一圈。 */
    sim.distance = 10U;
    Advance(&sim, 30U, 0x04U, false, 0U);
    Advance(&sim, 30U, 0x04U, true, 0U);
    assert(sim.car.completed_laps == 0U); /* 起点附近短距离晃动不算圈。 */
    CompleteLap(&sim);
    assert(sim.car.completed_laps == 1U && sim.car.state == CAR_RUNNING);
    Advance(&sim, 1300U, 0x04U, true, 0U);
    assert(sim.car.completed_laps == 1U); /* 同一次经过不能重复计数。 */
    CompleteLap(&sim);
    assert(sim.car.completed_laps == 2U && sim.car.state == CAR_RUNNING);
    CompleteLap(&sim);
    assert(sim.car.completed_laps == 3U && sim.car.state == CAR_DONE);
    Advance(&sim, 500U, 0x04U, true, 0U);
    assert(sim.car.state == CAR_DONE && sim.car.completed_laps == 3U);
    Click(&sim, CAR_KEY_START, 0x04U, true);
    assert(sim.car.state == CAR_RUNNING && sim.car.completed_laps == 0U && sim.distance == 0U);
    Start(&sim, 1U);
    CompleteLap(&sim);
    assert(sim.car.state == CAR_DONE && sim.car.completed_laps == 1U);
}

static void TestSpeedFeedbackAndSteering(void)
{
    Simulation sim;
    uint16_t low_feedback_pwm;
    Start(&sim, 2U);
    sim.left_speed = sim.right_speed = 100;
    Advance(&sim, 100U, 0x04U, false, 0U);
    low_feedback_pwm = sim.car.left_pwm;
    sim.left_speed = sim.right_speed = 400;
    Advance(&sim, 100U, 0x04U, false, 0U);
    assert(sim.car.left_pwm < low_feedback_pwm); /* 同样目标，实测过快就降低 PWM。 */
    sim.left_speed = sim.right_speed = 100;
    Advance(&sim, 50U, 0x02U, false, 0U);
    assert(sim.car.left_target_mm_s < sim.car.right_target_mm_s);
    Advance(&sim, 50U, 0x08U, false, 0U);
    assert(sim.car.left_target_mm_s > sim.car.right_target_mm_s);
    Advance(&sim, 50U, 0x04U, false, 0U);
    assert(sim.car.left_target_mm_s == sim.car.right_target_mm_s);
    Advance(&sim, 50U, 0x1FU, false, 0U);
    assert(sim.car.state == CAR_RUNNING && sim.car.completed_laps == 0U);
    /* 证明计圈使用独立起点输入，不是遇到全黑就计圈。 */
    for (uint8_t mask = 1U; mask < 32U; ++mask)
    {
        Advance(&sim, 10U, mask, false, 0U);
        assert(sim.car.state == CAR_RUNNING);
    }
}

static void TestMarkerDebounceAndApproach(void)
{
    Simulation sim;
    Start(&sim, 2U);
    sim.distance = 500U;
    Advance(&sim, CAR_MIN_LAP_TIME_MS + 50U, 0x04U, false, 0U);
    assert(sim.car.lap_armed);
    sim.distance = 2000U;
    for (unsigned i = 0U; i < 10U; ++i)
    {
        Advance(&sim, 2U, 0x04U, true, 0U);
        Advance(&sim, 2U, 0x04U, false, 0U);
    }
    assert(sim.car.completed_laps == 0U);
    Advance(&sim, CAR_MARKER_DEBOUNCE_MS + 2U, 0x04U, true, 0U);
    assert(sim.car.completed_laps == 1U);
    sim.distance = 2500U;
    Advance(&sim, CAR_MIN_LAP_TIME_MS + 50U, 0x04U, false, 0U);
    sim.distance = 3700U;
    Advance(&sim, 50U, 0x04U, false, 0U);
    assert(sim.car.left_target_mm_s == CAR_MARKER_SPEED_MM_S);
    assert(sim.car.right_target_mm_s == CAR_MARKER_SPEED_MM_S);
    sim.distance = 5000U;
    Advance(&sim, 50U, 0x04U, false, 0U);
    assert(sim.car.state == CAR_RUNNING && sim.car.completed_laps == 1U);
    /* 超过上圈测得长度不擅自停车，仍等实际起点信号。 */
    Advance(&sim, CAR_MARKER_DEBOUNCE_MS + 2U, 0x04U, true, 0U);
    assert(sim.car.state == CAR_DONE);
}

static void PlantRun(Simulation *sim, unsigned samples)
{
    for (unsigned i = 0U; i < samples; ++i)
    {
        Advance(sim, CAR_SPEED_PERIOD_MS, 0x04U, true, 0U);
        sim->left_speed += ((int32_t)sim->car.left_pwm * 550 / 800 - sim->left_speed) / 3;
        sim->right_speed += ((int32_t)sim->car.right_pwm * 550 / 800 - sim->right_speed) / 3;
    }
}

static void TestClosedLoopPlant(void)
{
    Simulation sim;
    Start(&sim, 1U);
    sim.left_speed = sim.right_speed = 0;
    PlantRun(&sim, 150U);
    assert(sim.left_speed >= 235 && sim.left_speed <= 265);
    Click(&sim, CAR_KEY_SPEED, 0x04U, true);
    PlantRun(&sim, 150U);
    assert(sim.left_speed >= 285 && sim.left_speed <= 315);
    assert(sim.car.state == CAR_RUNNING);
}

static void TestFaultsAndRecovery(void)
{
    Simulation sim;
    Start(&sim, 2U);
    Advance(&sim, 50U, 0x02U, false, 0U);
    Advance(&sim, 100U, 0U, false, 0U);
    assert(sim.car.state == CAR_RUNNING && sim.car.left_target_mm_s < sim.car.right_target_mm_s);
    Advance(&sim, 50U, 0x04U, false, 0U);
    assert(!sim.car.line_missing && sim.car.state == CAR_RUNNING);
    Advance(&sim, CAR_LINE_LOST_TIMEOUT_MS + 20U, 0U, false, 0U);
    assert(sim.car.state == CAR_FAULT_LINE_LOST);
    Advance(&sim, 100U, 0x04U, true, 0U);
    assert(sim.car.state == CAR_FAULT_LINE_LOST);
    Click(&sim, CAR_KEY_START, 0x04U, true);
    assert(sim.car.state == CAR_RUNNING);
    sim.motion = false;
    sim.left_speed = sim.right_speed = 0;
    Advance(&sim, CAR_ENCODER_STALL_MS + 100U, 0x04U, false, 0U);
    assert(sim.car.state == CAR_FAULT_ENCODER);
    Start(&sim, 1U);
    sim.left_speed = -100;
    Advance(&sim, 100U, 0x04U, false, 0U);
    assert(sim.car.state == CAR_FAULT_ENCODER);
    Start(&sim, 1U);
    Advance(&sim, CAR_MAX_LAP_TIME_MS + 1U, 0x04U, false, 0U);
    assert(sim.car.state == CAR_FAULT_MARKER);
    CarControl_Stop(&sim.car, CAR_FAULT_OLED);
    assert(sim.car.state == CAR_FAULT_OLED && sim.car.left_pwm == 0U && sim.car.right_pwm == 0U);
}

static void TestOdometry(void)
{
    Odometry odo;
    Odometry_Init(&odo, 0U, 0U, 0U);
    assert(!Odometry_Update(&odo, 20U, 1U, 1U));
    assert(Odometry_Update(&odo, 1000U, 1560U, 1560U));
    assert(odo.distance_mm == 210U && odo.speed_mm_s == 210);
    assert(Odometry_Update(&odo, 2000U, 3120U, 1560U));
    assert(odo.distance_mm == 315U && odo.left_speed_mm_s == 210 && odo.right_speed_mm_s == 0);
    Odometry_ResetTrip(&odo, 2000U, 3120U, 1560U);
    assert(odo.distance_mm == 0U && odo.speed_mm_s == 105);
    assert(Odometry_Update(&odo, 2050U, 3120U, 1560U));
    assert(odo.speed_mm_s == 0); /* 停车速度来自编码器零增量。 */
    Odometry_Init(&odo, 0U, 65530U, 65530U);
    assert(Odometry_Update(&odo, 50U, 20U, 20U));
    assert(odo.left_speed_mm_s > 0 && odo.distance_mm == 3U);
    assert(Odometry_Update(&odo, 100U, 65530U, 65530U));
    assert(odo.speed_mm_s < 0 && odo.distance_mm == 7U);
    Odometry_Init(&odo, 0U, 0U, 0U);
    for (uint16_t i = 1U; i <= 100U; ++i) { assert(Odometry_Update(&odo, 50U * i, i, i)); }
    assert(odo.distance_mm == 13U); /* 小于 1 mm 的每次增量不会丢失。 */
    Odometry_Init(&odo, UINT32_MAX - 10U, 0U, 0U);
    assert(Odometry_Update(&odo, 50U, 10U, 10U));
    assert(odo.speed_mm_s > 0);
}

static void TestTickWrap(void)
{
    Simulation sim;
    Init(&sim, UINT32_MAX - 10U, true, 0U);
    Click(&sim, CAR_KEY_START, 0x04U, true);
    assert(sim.now < 100U && sim.car.state == CAR_RUNNING);
    CompleteLap(&sim);
    assert(sim.car.state == CAR_DONE);
}

int main(void)
{
    TestKeysAndSettings();
    TestStartPositionAndRestart();
    TestLapCountingAndFinish();
    TestSpeedFeedbackAndSteering();
    TestMarkerDebounceAndApproach();
    TestClosedLoopPlant();
    TestFaultsAndRecovery();
    TestOdometry();
    TestTickWrap();
    puts("PASS: buttons/settings, start validation, lap debounce/finish, speed feedback, steering, faults, measured odometry, counter/tick wrap");
    return 0;
}
