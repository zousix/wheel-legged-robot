#pragma once
#include "imu963ra.h"

typedef enum {
    IMU_NOT_STARTED, IMU_STARTING, IMU_SETTLING, IMU_READY, IMU_FAILED
} imu_phase_t;

typedef struct {
    imu_phase_t phase;
    esp_err_t error;
    uint8_t device_id;
    bool magnetometer_available;
    unsigned int samples;
    int64_t sample_time_us, magnetic_time_us;
    imu963ra_sample_t sensor;
    float gyro_bias_dps[3], corrected_gyro_dps[3];
    float roll_deg, pitch_deg, yaw_deg;
    float body_roll_deg, body_pitch_deg;
    float roll_zero_deg, pitch_zero_deg;
} imu_state_t;

esp_err_t start_imu(void);
void imu_get_state(imu_state_t *state);
void print_imu(void);
bool imu_request_zero(void);
