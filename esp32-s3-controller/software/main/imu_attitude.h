#pragma once

#include <stdbool.h>
#include "fusion/FusionAhrs.h"
#include "fusion/FusionOffset.h"

#define IMU_SAMPLE_RATE_HZ 208U

// Sensor axes, g and degrees/s. Body-axis mapping must be measured before LQR use.
typedef struct {
    FusionAhrs ahrs;
    FusionOffset offset;
    float bias_dps[3];
    bool ready;
    float roll_deg, pitch_deg, yaw_deg;
    float corrected_gyro_dps[3];
} imu_attitude_t;

void imu_attitude_init(imu_attitude_t *state);
float imu_wrap_degrees(float degrees);
void imu_body_angles(float roll, float pitch, float roll_zero, float pitch_zero, float body[2]);
bool imu_attitude_update(imu_attitude_t *state, const float acceleration_g[3],
                         const float gyro_dps[3], float dt_s);
