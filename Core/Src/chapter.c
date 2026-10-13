#include "main.h"
#include "chapter.h"
#include "car_config.h"
#include "car_control.h"
#include "buttons.h"
#include "odometry.h"
#include "oled.h"
#include <stdio.h>

/* 可在调试器中查看 state、left_pwm、right_pwm。 */
static CarControl car;
static CarButtons buttons;
static Odometry odometry;

static uint8_t ReadKeys(void)
{
    uint32_t levels = CAR_BUTTON_PORT->IDR;
    uint8_t mask = 0U;
    if ((levels & CAR_BUTTON_PIN) == 0U) { mask |= CAR_KEY_START; }
    if ((levels & CAR_SPEED_BUTTON_PIN) == 0U) { mask |= CAR_KEY_SPEED; }
    if ((levels & CAR_LAPS_BUTTON_PIN) == 0U) { mask |= CAR_KEY_LAPS; }
    return mask;
}

static bool ReadMarker(uint8_t black_mask)
{
#if CAR_START_MARKER_MODE == CAR_START_MARKER_BLACK_BAR
    return (black_mask & 0x1FU) == 0x1FU;
#else
    (void)black_mask;
    bool high = HAL_GPIO_ReadPin(CAR_MARKER_PORT, CAR_MARKER_PIN) == GPIO_PIN_SET;
    return CAR_MARKER_ACTIVE_LOW ? !high : high;
#endif
}

static uint8_t ReadBlackMask(void)
{
    static const uint16_t sensor_pins[5] = {
        CAR_SENSOR_LEFT_OUTER_PIN, CAR_SENSOR_LEFT_PIN, CAR_SENSOR_CENTER_PIN,
        CAR_SENSOR_RIGHT_PIN, CAR_SENSOR_RIGHT_OUTER_PIN
    };
    /* 一次读取端口，五路传感器使用同一时刻的快照。 */
    uint32_t levels = CAR_SENSOR_PORT->IDR;
    uint8_t mask = 0U;

    for (uint8_t i = 0U; i < 5U; ++i)
    {
        bool high = (levels & sensor_pins[i]) != 0U;
        bool black = CAR_SENSOR_BLACK_IS_LOW ? !high : high;
        if (black)
        {
            mask |= (uint8_t)(1U << i);
        }
    }
    return mask;
}

static uint32_t ScaledDuty(uint16_t duty, uint32_t scale_percent)
{
    uint32_t scaled = (uint32_t)duty * scale_percent / 100U;
    if (scaled > CAR_PWM_LIMIT)
    {
        scaled = CAR_PWM_LIMIT;
    }
    return (TIM3->ARR + 1U) * scaled / CAR_PWM_MAX;
}

static void MotorApply(uint16_t left, uint16_t right)
{
    /* TB6612FNG：IN1=IN2=1 是短路制动；停车时持续保持制动。 */
    HAL_GPIO_WritePin(CAR_MOTOR_PORT, CAR_LEFT_IN1_PIN,
                     (left == 0U || CAR_LEFT_FORWARD_IN1_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CAR_MOTOR_PORT, CAR_LEFT_IN2_PIN,
                     (left == 0U || !CAR_LEFT_FORWARD_IN1_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CAR_MOTOR_PORT, CAR_RIGHT_IN1_PIN,
                     (right == 0U || CAR_RIGHT_FORWARD_IN1_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CAR_MOTOR_PORT, CAR_RIGHT_IN2_PIN,
                     (right == 0U || !CAR_RIGHT_FORWARD_IN1_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    TIM3->CCR1 = ScaledDuty(left, CAR_LEFT_SCALE_PERCENT);
    TIM3->CCR2 = ScaledDuty(right, CAR_RIGHT_SCALE_PERCENT);
    HAL_GPIO_WritePin(CAR_MOTOR_PORT, CAR_STANDBY_PIN, GPIO_PIN_SET);
}

static void EncoderInit(TIM_TypeDef *timer)
{
    timer->CR1 = 0U;
    timer->PSC = 0U;
    timer->ARR = 0xFFFFU;
    timer->CNT = 0U;
    timer->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 |
                   (8U << TIM_CCMR1_IC1F_Pos) | (8U << TIM_CCMR1_IC2F_Pos);
    timer->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E; /* 启用两个编码器输入通道。 */
    timer->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1; /* TI12，AB 相四倍频。 */
    timer->EGR = TIM_EGR_UG;
    timer->SR = 0U;
    timer->CR1 = TIM_CR1_CEN;
}

static void HardwareInit(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t timer_clock;
    uint32_t period;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    /* TIM3 使用默认 PA6/PA7；外设重映射后最后明确保留 SWD。 */
    __HAL_AFIO_REMAP_TIM2_PARTIAL_1(); /* 左轮 CH1=PA15、CH2=PB3。 */
    __HAL_AFIO_REMAP_TIM3_DISABLE();
    __HAL_AFIO_REMAP_TIM4_DISABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    HAL_GPIO_WritePin(CAR_MOTOR_PORT,
                     CAR_LEFT_IN1_PIN | CAR_LEFT_IN2_PIN | CAR_RIGHT_IN1_PIN |
                     CAR_RIGHT_IN2_PIN | CAR_STANDBY_PIN, GPIO_PIN_RESET);
    gpio.Pin = CAR_LEFT_IN1_PIN | CAR_LEFT_IN2_PIN | CAR_RIGHT_IN1_PIN |
               CAR_RIGHT_IN2_PIN | CAR_STANDBY_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CAR_MOTOR_PORT, &gpio);

    gpio.Pin = CAR_SENSOR_PINS;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(CAR_SENSOR_PORT, &gpio);
    gpio.Pin = CAR_BUTTON_PIN | CAR_SPEED_BUTTON_PIN | CAR_LAPS_BUTTON_PIN;
    HAL_GPIO_Init(CAR_BUTTON_PORT, &gpio);

    gpio.Pin = CAR_LEFT_ENCODER_A_PIN;
    HAL_GPIO_Init(CAR_LEFT_ENCODER_A_PORT, &gpio);
    gpio.Pin = CAR_LEFT_ENCODER_B_PIN;
    HAL_GPIO_Init(CAR_LEFT_ENCODER_B_PORT, &gpio);
    gpio.Pin = CAR_RIGHT_ENCODER_A_PIN | CAR_RIGHT_ENCODER_B_PIN;
    HAL_GPIO_Init(CAR_RIGHT_ENCODER_PORT, &gpio);
#if CAR_START_MARKER_MODE == CAR_START_MARKER_HALL
    gpio.Pin = CAR_MARKER_PIN;
    HAL_GPIO_Init(CAR_MARKER_PORT, &gpio);
#endif
    __HAL_RCC_TIM2_FORCE_RESET();
    __HAL_RCC_TIM2_RELEASE_RESET();
    __HAL_RCC_TIM4_FORCE_RESET();
    __HAL_RCC_TIM4_RELEASE_RESET();
    EncoderInit(TIM2);
    EncoderInit(TIM4);


    HAL_GPIO_WritePin(CAR_LED_PORT, CAR_LED_PIN, GPIO_PIN_SET);
    gpio.Pin = CAR_LED_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CAR_LED_PORT, &gpio);

    /* 不依赖项目中尚未引入的 HAL_TIM 驱动，使用现有 CMSIS 寄存器定义。 */
    timer_clock = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
    {
        timer_clock *= 2U;
    }
    period = timer_clock / CAR_PWM_FREQUENCY_HZ;
    if (period < 2U || period > 65536U)
    {
        /* 此时 STBY 仍为低，电机尚未使能。 */
        Error_Handler();
    }
    __HAL_RCC_TIM3_FORCE_RESET();
    __HAL_RCC_TIM3_RELEASE_RESET();
    TIM3->PSC = 0U;
    TIM3->ARR = period - 1U;   /* 72 MHz / 3600 = 20 kHz。 */
    TIM3->CCR1 = 0U;
    TIM3->CCR2 = 0U;
    TIM3->CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 |
                  TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM3->CR1 = TIM_CR1_ARPE;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0U;

    gpio.Pin = CAR_PWM_LEFT_PIN | CAR_PWM_RIGHT_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CAR_PWM_PORT, &gpio);
    TIM3->CR1 |= TIM_CR1_CEN;
    MotorApply(0U, 0U);
}

static void RenderDisplay(bool marker_present)
{
    char line[24];
    int32_t actual = odometry.speed_mm_s;
    uint32_t magnitude = (uint32_t)((actual < 0) ? -(int64_t)actual : actual);
    uint32_t distance = odometry.distance_mm;
    OLED_Clear();
    snprintf(line, sizeof(line), "STATE:%s", CarControl_StateName(car.state));
    OLED_Text(0U, line);
    snprintf(line, sizeof(line), "SET:%lu.%03lu M/S", (unsigned long)(car.set_speed_mm_s / 1000U),
             (unsigned long)(car.set_speed_mm_s % 1000U));
    OLED_Text(1U, line);
    snprintf(line, sizeof(line), "ACT:%c%lu.%03lu M/S", actual < 0 ? '-' : '+',
             (unsigned long)(magnitude / 1000U), (unsigned long)(magnitude % 1000U));
    OLED_Text(2U, line);
    snprintf(line, sizeof(line), "DIST:%lu.%03lu M", (unsigned long)(distance / 1000U),
             (unsigned long)(distance % 1000U));
    OLED_Text(3U, line);
    snprintf(line, sizeof(line), "LAP:%u TARGET:%u", (unsigned)car.completed_laps, (unsigned)car.target_laps);
    OLED_Text(4U, line);
    OLED_Text(5U, marker_present ? "START:DETECTED" : "START:AWAY");
    OLED_Text(6U, car.state == CAR_RUNNING ? "SPEED:SET LAP:LOCK" : "SPEED/LAP:SET");
    OLED_Text(7U, car.state == CAR_RUNNING ? "KEY START:STOP" : "KEY START:RUN");
    OLED_StartFrame();
}

void chapter_main(void)
{
    uint32_t display_at;
    uint32_t oled_retry_at;
    uint32_t now;
    HardwareInit();
    (void)OLED_Init();
    now = HAL_GetTick();
    CarControl_Init(&car, now, ReadMarker(ReadBlackMask()));
    Buttons_Init(&buttons, now, ReadKeys());
    Odometry_Init(&odometry, now, (uint16_t)TIM2->CNT, (uint16_t)TIM4->CNT);
    display_at = now - CAR_OLED_REFRESH_MS;
    oled_retry_at = now;

    while (1)
    {
        uint8_t black_mask;
        uint8_t events;
        bool marker_present;
        bool speed_sample;
        CarInput input;
        now = HAL_GetTick();
        black_mask = ReadBlackMask();
        marker_present = ReadMarker(black_mask);
        events = Buttons_Update(&buttons, now, ReadKeys());
        speed_sample = Odometry_Update(&odometry, now, (uint16_t)TIM2->CNT, (uint16_t)TIM4->CNT);
        if (!OLED_IsReady())
        {
            CarControl_Stop(&car, CAR_FAULT_OLED);
            events &= (uint8_t)~CAR_KEY_START; /* 显示器故障时不允许启动。 */
        }
        input = (CarInput){
            .black_mask = black_mask, .key_events = events, .marker_present = marker_present,
            .speed_sample = speed_sample, .distance_mm = odometry.distance_mm,
            .left_speed_mm_s = odometry.left_speed_mm_s, .right_speed_mm_s = odometry.right_speed_mm_s,
            .left_motion_at = odometry.left_motion_at, .right_motion_at = odometry.right_motion_at
        };
        CarControl_Update(&car, now, &input);
        if (car.reset_trip)
        {
            Odometry_ResetTrip(&odometry, now, (uint16_t)TIM2->CNT, (uint16_t)TIM4->CNT);
        }
        MotorApply(car.left_pwm, car.right_pwm);

        if (!OLED_IsReady() && car.state != CAR_RUNNING &&
            (uint32_t)(now - oled_retry_at) >= 2000U)
        {
            oled_retry_at = now;
            (void)OLED_Init(); /* 电机已制动，仅停车时尝试重新初始化。 */
        }
        if (OLED_CanRender() && (uint32_t)(now - display_at) >= CAR_OLED_REFRESH_MS)
        {
            display_at = now;
            RenderDisplay(marker_present);
        }
        OLED_Task();
        if (!OLED_IsReady())
        {
            CarControl_Stop(&car, CAR_FAULT_OLED);
            MotorApply(0U, 0U);
        }
        bool fault = car.state != CAR_RUNNING && car.state != CAR_STOPPED && car.state != CAR_DONE;
        bool led_on = car.state == CAR_RUNNING || (fault && (now / 100U) % 2U == 0U);
        HAL_GPIO_WritePin(CAR_LED_PORT, CAR_LED_PIN, led_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
        HAL_Delay(1U);
    }
}
