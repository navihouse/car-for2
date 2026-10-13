#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/* car-for2：速度用 mm/s，PWM 仍使用千分比。编码器参数必须实车标定。 */
#define CAR_PWM_MAX                 1000U
#define CAR_PWM_FREQUENCY_HZ        20000U
#define CAR_PWM_LIMIT               800
#define CAR_SPEED_MIN_MM_S          100U
#define CAR_SPEED_MAX_MM_S          550U
#define CAR_SPEED_STEP_MM_S         50U
#define CAR_SPEED_DEFAULT_MM_S      250U
#define CAR_WHEEL_TARGET_MAX_MM_S   700
#define CAR_SEARCH_SPEED_MM_S       100
#define CAR_MARKER_SPEED_MM_S       120
#define CAR_STEER_KP                1
#define CAR_STEER_KD                1
#define CAR_STEER_SLOWDOWN_DIVISOR  4
#define CAR_LAPS_MIN                1U
#define CAR_LAPS_MAX                9U
#define CAR_LAPS_DEFAULT            1U

/* 仅为可编译的参考值：轮周长 210 mm，每车轮一圈四倍频计数 1560。 */
#define CAR_LEFT_WHEEL_CIRCUMFERENCE_UM   210000U
#define CAR_RIGHT_WHEEL_CIRCUMFERENCE_UM  210000U
#define CAR_LEFT_ENCODER_CPR              1560U
#define CAR_RIGHT_ENCODER_CPR             1560U
#define CAR_LEFT_ENCODER_SIGN             1
#define CAR_RIGHT_ENCODER_SIGN            1

/* 双轮独立速度 PI：前馈比例 1.2，积分单位为 PWM 千分比的千分之一。 */
#define CAR_SPEED_FF_NUM            12
#define CAR_SPEED_FF_DEN            10
#define CAR_SPEED_KP                1
#define CAR_SPEED_KI                3
#define CAR_SPEED_INTEGRAL_LIMIT    300

#define CAR_CONTROL_PERIOD_MS       5U
#define CAR_SPEED_PERIOD_MS         50U
#define CAR_BUTTON_DEBOUNCE_MS      25U
#define CAR_LINE_LOST_TIMEOUT_MS    600U
#define CAR_ENCODER_STALL_MS        1000U
#define CAR_MARKER_DEBOUNCE_MS      5U
#define CAR_START_MARKER_HALL        1
#define CAR_START_MARKER_BLACK_BAR   2
/* 默认使用起点磁铁 + 非锁存霍尔开关，不依赖全黑区域计圈。 */
#define CAR_START_MARKER_MODE        CAR_START_MARKER_HALL
#define CAR_MARKER_PORT              GPIOA
#define CAR_MARKER_PIN               GPIO_PIN_5
#define CAR_MARKER_ACTIVE_LOW        1
#define CAR_MIN_LAP_DISTANCE_MM     300U
#define CAR_MIN_LAP_TIME_MS         1000U
#define CAR_MAX_LAP_TIME_MS         180000U
#define CAR_APPROACH_DISTANCE_MM    300U
#define CAR_OLED_REFRESH_MS         200U
#define CAR_OLED_ADDRESS            0x3CU

/* 模块在黑线上输出低电平时为 1，高电平时改为 0。 */
#define CAR_SENSOR_BLACK_IS_LOW     1
/* 某个车轮方向反了，就把对应的 1 改为 0。 */
#define CAR_LEFT_FORWARD_IN1_HIGH   1
#define CAR_RIGHT_FORWARD_IN1_HIGH  1
/* 用于补偿左右电机差异，100 表示保持原占空比。 */
#define CAR_LEFT_SCALE_PERCENT      100U
#define CAR_RIGHT_SCALE_PERCENT     100U

/* 从车头朝前看，传感器按最左到最右排列；安装于后轮前方。 */
#define CAR_SENSOR_PORT             GPIOA
#define CAR_SENSOR_LEFT_OUTER_PIN   GPIO_PIN_0
#define CAR_SENSOR_LEFT_PIN         GPIO_PIN_1
#define CAR_SENSOR_CENTER_PIN       GPIO_PIN_2
#define CAR_SENSOR_RIGHT_PIN        GPIO_PIN_3
#define CAR_SENSOR_RIGHT_OUTER_PIN  GPIO_PIN_4
#define CAR_SENSOR_PINS             (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | \
                                     GPIO_PIN_3 | GPIO_PIN_4)

/* PA6/PA7 固定对应未重映射的 TIM3_CH1/CH2，修改 PWM 接线须同步改定时器。 */
#define CAR_PWM_PORT                GPIOA
#define CAR_PWM_LEFT_PIN            GPIO_PIN_6
#define CAR_PWM_RIGHT_PIN           GPIO_PIN_7
#define CAR_MOTOR_PORT              GPIOB
#define CAR_LEFT_IN1_PIN            GPIO_PIN_0
#define CAR_LEFT_IN2_PIN            GPIO_PIN_1
#define CAR_RIGHT_IN1_PIN           GPIO_PIN_10
#define CAR_RIGHT_IN2_PIN           GPIO_PIN_11
#define CAR_STANDBY_PIN             GPIO_PIN_12
#define CAR_BUTTON_PORT             GPIOB
#define CAR_BUTTON_PIN              GPIO_PIN_5
#define CAR_SPEED_BUTTON_PIN        GPIO_PIN_13
#define CAR_LAPS_BUTTON_PIN         GPIO_PIN_14
#define CAR_LEFT_ENCODER_A_PORT     GPIOA
#define CAR_LEFT_ENCODER_A_PIN      GPIO_PIN_15
#define CAR_LEFT_ENCODER_B_PORT     GPIOB
#define CAR_LEFT_ENCODER_B_PIN      GPIO_PIN_3
#define CAR_RIGHT_ENCODER_PORT      GPIOB
#define CAR_RIGHT_ENCODER_A_PIN     GPIO_PIN_6
#define CAR_RIGHT_ENCODER_B_PIN     GPIO_PIN_7
#define CAR_OLED_PORT               GPIOB
#define CAR_OLED_SCL_PIN            GPIO_PIN_8
#define CAR_OLED_SDA_PIN            GPIO_PIN_9
#define CAR_LED_PORT                GPIOC
#define CAR_LED_PIN                 GPIO_PIN_13

#if (CAR_START_MARKER_MODE != CAR_START_MARKER_HALL) && \
    (CAR_START_MARKER_MODE != CAR_START_MARKER_BLACK_BAR)
#error "Select Hall marker or black-bar marker"
#endif
#if (CAR_PWM_FREQUENCY_HZ == 0U)
#error "PWM frequency must be positive"
#endif
#if (CAR_SPEED_MIN_MM_S == 0U) || (CAR_SPEED_STEP_MM_S == 0U) || \
    (CAR_SPEED_MIN_MM_S > CAR_SPEED_DEFAULT_MM_S) || \
    (CAR_SPEED_DEFAULT_MM_S > CAR_SPEED_MAX_MM_S) || \
    (CAR_SPEED_MAX_MM_S > CAR_WHEEL_TARGET_MAX_MM_S) || \
    (CAR_PWM_LIMIT > CAR_PWM_MAX) || (CAR_PWM_LIMIT <= 0) || \
    (CAR_STEER_SLOWDOWN_DIVISOR <= 0) || (CAR_SPEED_FF_DEN <= 0) || \
    (CAR_LAPS_DEFAULT < CAR_LAPS_MIN) || (CAR_LAPS_DEFAULT > CAR_LAPS_MAX)
#error "Invalid speed, PWM or lap configuration"
#endif
#if (CAR_CONTROL_PERIOD_MS == 0U) || (CAR_BUTTON_DEBOUNCE_MS == 0U) || \
    (CAR_SPEED_PERIOD_MS == 0U) || (CAR_LINE_LOST_TIMEOUT_MS <= CAR_CONTROL_PERIOD_MS)
#error "Invalid car timing configuration"
#endif
#if (CAR_LEFT_ENCODER_CPR == 0U) || (CAR_RIGHT_ENCODER_CPR == 0U) || \
    (CAR_LEFT_WHEEL_CIRCUMFERENCE_UM == 0U) || (CAR_RIGHT_WHEEL_CIRCUMFERENCE_UM == 0U) || \
    ((CAR_LEFT_ENCODER_SIGN != 1) && (CAR_LEFT_ENCODER_SIGN != -1)) || \
    ((CAR_RIGHT_ENCODER_SIGN != 1) && (CAR_RIGHT_ENCODER_SIGN != -1))
#error "Invalid encoder calibration"
#endif
#if (CAR_LEFT_SCALE_PERCENT == 0U) || (CAR_LEFT_SCALE_PERCENT > 200U) || \
    (CAR_RIGHT_SCALE_PERCENT == 0U) || (CAR_RIGHT_SCALE_PERCENT > 200U)
#error "Motor scaling must be between 1 and 200 percent"
#endif

#endif /* CAR_CONFIG_H */
