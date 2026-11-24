#include <math.h>
#include <stdio.h>
#include "esp_timer.h"
#include "robot_state.h"
#include "leg_control_internal.h"
#include "leg_spd.h"

static bool fail(robot_state_t *state, const char *reason)
{
    state->valid = false;
    state->reason = reason;
    for (unsigned int i = 0; i < 6; ++i) state->s[i] = NAN;
    return false;
}

static bool fresh(int64_t now, int64_t timestamp, int64_t timeout)
{
    return timestamp > 0 && timestamp <= now && now - timestamp < timeout;
}

bool robot_state_calculate(const robot_state_input_t *input, robot_state_t *state)
{
    *state = (robot_state_t){0};
    if (input->imu.phase != IMU_READY) return fail(state, "imu-not-ready");
    if (!fresh(input->now_us, input->imu.sample_time_us, 30000)) return fail(state, "imu-feedback-stale");
    if (!input->wheels.valid) return fail(state, "wheel-reference-unavailable");
    const int64_t now_ms = input->now_us / 1000;
    for (unsigned int i = 0; i < 2; ++i)
        if (!fresh(now_ms, input->wheels.last_feedback_ms[i], 30)) return fail(state, "wheel-feedback-stale");
    if (!input->calibrated[0] || !input->calibrated[1]) return fail(state, "legs-uncalibrated");
    for (unsigned int i = 0; i < 4; ++i)
        if (!fresh(now_ms, input->joints[i].last_feedback_ms, 30)) return fail(state, "leg-feedback-stale");
    return robot_state_calculate_axes(input, state);
}

bool robot_state_calculate_axes(const robot_state_input_t *input, robot_state_t *state)
{
    *state = (robot_state_t){0};
    if (input->imu.phase != IMU_READY) return fail(state, "imu-not-ready");
    if (!input->calibrated[0] || !input->calibrated[1]) return fail(state, "legs-uncalibrated");
    const float rad = 3.14159265358979323846f / 180.0f;
    const float half_pi = 1.57079632679489661923f;
    const float direction[2] = {-1, 1};
    for (unsigned int pair = 0; pair < 2; ++pair) {
        const unsigned int r = pair * 2, l = r + 1;
        for (unsigned int i = r; i <= l; ++i) {
            if (!isfinite(input->joints[i].angle) || !isfinite(input->joints[i].speed_rpm) ||
                !isfinite(input->reference[i])) return fail(state, "leg-invalid-data");
        }
        float pos[2], speed[2];
        if (!solve_leg_directed(direction[pair], input->joints[r].angle, input->joints[l].angle,
                                input->reference[r], input->reference[l], pos))
            return fail(state, "invalid-leg-geometry");
        const float phi1 = half_pi + direction[pair] * (input->joints[l].angle - input->reference[l]);
        const float phi4 = half_pi + direction[pair] * (input->joints[r].angle - input->reference[r]);
        leg_spd(direction[pair] * input->joints[l].speed_rpm * rad * 6,
                direction[pair] * input->joints[r].speed_rpm * rad * 6, phi1, phi4, speed);
        if (!isfinite(speed[0]) || !isfinite(speed[1])) return fail(state, "invalid-leg-rate");
        state->leg_length_m[pair] = pos[0];
        state->leg_angle_rad[pair] = pos[1];
        state->leg_rate_rad_s[pair] = speed[1];
        state->leg_length_rate_mps[pair] = speed[0];
    }
    const float sensor_roll = input->imu.roll_deg * rad;
    const float q = input->imu.corrected_gyro_dps[1], r = input->imu.corrected_gyro_dps[2];
    if (!isfinite(sensor_roll) || !isfinite(q) || !isfinite(r)) return fail(state, "invalid-imu-rate");
    const float phi = input->imu.body_pitch_deg * rad;
    // Standard RPY: sensor pitch_dot = q*cos(roll)-r*sin(roll). Body forward
    // pitch is its negative. This handles the measured sensor roll near +/-pi.
    const float dphi = -(q * cosf(sensor_roll) - r * sinf(sensor_roll)) * rad;
    // 2026-10-19 physical forward swings: ID2/3 (left) angle increases;
    // ID5/4 (right) angle decreases.
    // sys_sim uses +X forward, +Y up: theta is hip-ahead-of-wheel positive,
    // phi is counterclockwise/backward positive. IMU body pitch is forward positive.
    state->s[0] = (state->leg_angle_rad[1] - state->leg_angle_rad[0]) * 0.5f + phi;
    state->s[1] = (state->leg_rate_rad_s[1] - state->leg_rate_rad_s[0]) * 0.5f + dphi;
    state->s[2] = input->wheels.position_m_avg;
    state->s[3] = input->wheels.speed_mps_avg;
    state->s[4] = -phi;
    state->s[5] = -dphi;
    for (unsigned int i = 0; i < 6; ++i)
        if (!isfinite(state->s[i])) return fail(state, "non-finite-state");
    state->valid = true;
    state->reason = "model-axes-no-control";
    return true;
}

void robot_state_snapshot(robot_state_t *state)
{
    robot_state_input_t input = {0};
    imu_get_state(&input.imu);
    wheel_get_state(&input.wheels);
    portENTER_CRITICAL(&motor_mux);
    input.joints[0] = motors[1];
    input.joints[1] = motors[2];
    input.joints[2] = motors[LEG_RIGHT_INDEX];
    input.joints[3] = motors[LEG_LEFT_INDEX];
    secondary_reference_locked(input.reference);
    input.reference[2] = leg_zero_right;
    input.reference[3] = leg_zero_left;
    input.calibrated[0] = secondary_calibrated;
    input.calibrated[1] = leg_calibrated;
    portEXIT_CRITICAL(&motor_mux);
    input.now_us = esp_timer_get_time();
    robot_state_calculate(&input, state);
}

void print_robot_state(void)
{
    robot_state_t state;
    robot_state_snapshot(&state);
    if (!state.valid) {
        printf("STATE unavailable: %s; no control output\r\n", state.reason);
        return;
    }
    printf("STATE model: theta=%.5f rad dtheta=%.5f rad/s x=%.5f m dx=%.5f m/s phi=%.5f rad dphi=%.5f rad/s\r\n",
           state.s[0], state.s[1], state.s[2], state.s[3], state.s[4], state.s[5]);
    printf("STATE legs: ID2/3 L=%.2f mm angle=%.3f deg rate=%.5f rad/s; ID5/4 L=%.2f mm angle=%.3f deg rate=%.5f rad/s; no control output\r\n",
           state.leg_length_m[0] * 1000, state.leg_angle_rad[0] * 180 / 3.14159265358979323846f, state.leg_rate_rad_s[0],
           state.leg_length_m[1] * 1000, state.leg_angle_rad[1] * 180 / 3.14159265358979323846f, state.leg_rate_rad_s[1]);
}
