#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

/* MPU6050 settings ported from car-02-/Core/Src/mpu6050.c. */
#define IMU_GYRO_CONFIG 0x08U /* FS_SEL=1, +/-500 degrees/second */
#define IMU_GYRO_LSB_PER_DPS 65.5f
#define IMU_GYRO_MAX_ABS_DPS 490.0f
#define IMU_SAMPLE_DIVIDER 0U /* DLPF=3: 1 kHz sensor, 5 ms host polling */
#define IMU_DLPF_CONFIG 3U
#define IMU_POLL_MS 5U
#define IMU_MAX_SAMPLE_GAP_MS 30U

/* Installed sensor's measured stationary reference, from car-02-.
 * This ONLY centers the calibration gate. Each boot measures its own bias.
 * Re-measure this reference after replacing the sensor. Raw sensor sign is
 * retained here; MOTION_YAW_SIGN is applied once by the mission motion controller. */
#define IMU_Z_BIAS_PRIOR_DPS (-23.0f)
#define IMU_CAL_WINDOW_DPS 10.0f
#define IMU_CAL_SAMPLES 400U
#define IMU_CAL_MIN_MS ((IMU_CAL_SAMPLES - 1U) * IMU_POLL_MS)
#define IMU_CAL_STD_DPS 0.8f

#endif
