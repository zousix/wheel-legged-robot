#include <math.h>
#include <string.h>
#include "imu_attitude.h"

float imu_wrap_degrees(float degrees)
{
    return remainderf(degrees, 360.0f);
}

void imu_body_angles(float roll, float pitch, float roll_zero, float pitch_zero, float body[2])
{
    // Local upright convention established by manual tilts: left/forward positive.
    // This Euler offset is for upright commissioning, not an arbitrary mounting transform.
    body[0] = imu_wrap_degrees(roll - roll_zero);
    body[1] = -(pitch - pitch_zero);
}

void imu_attitude_init(imu_attitude_t *state)
{
    memset(state, 0, sizeof(*state));
    FusionAhrsInitialise(&state->ahrs);
    FusionOffsetInitialise(&state->offset, IMU_SAMPLE_RATE_HZ);
}

bool imu_attitude_update(imu_attitude_t *state, const float acceleration_g[3],
                         const float gyro_dps[3], float dt_s)
{
    if (!isfinite(dt_s) || dt_s <= 0 || dt_s > 0.05f) return false;
    float norm_squared = 0;
    for (unsigned int axis = 0; axis < 3; ++axis) {
        if (!isfinite(acceleration_g[axis]) || !isfinite(gyro_dps[axis])) return false;
        norm_squared += acceleration_g[axis] * acceleration_g[axis];
    }
    if (norm_squared < 0.0001f) return false;
    FusionVector acceleration, gyroscope;
    for (unsigned int axis = 0; axis < 3; ++axis) {
        acceleration.array[axis] = acceleration_g[axis];
        gyroscope.array[axis] = gyro_dps[axis];
    }
    gyroscope = FusionOffsetUpdate(&state->offset, gyroscope);
    memcpy(state->bias_dps, state->offset.gyroscopeOffset.array, sizeof(state->bias_dps));
    memcpy(state->corrected_gyro_dps, gyroscope.array, sizeof(state->corrected_gyro_dps));
    // Preserve the supplied gpio archive's six-axis algorithm. Magnetometer is
    // exposed separately until hard/soft-iron calibration and axis mapping exist.
    FusionAhrsUpdateNoMagnetometer(&state->ahrs, gyroscope, acceleration, dt_s);
    if (!state->ready && !FusionAhrsGetFlags(&state->ahrs).initialising) {
        FusionAhrsSetHeading(&state->ahrs, 0);
        state->ready = true;
    }
    FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&state->ahrs));
    state->roll_deg = euler.angle.roll;
    state->pitch_deg = euler.angle.pitch;
    state->yaw_deg = euler.angle.yaw;
    return true;
}
