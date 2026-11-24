#include <math.h>
#include <stdio.h>
#include "esp_timer.h"
#include "wheel_odometry.h"

static wheel_odometry_t odometry;
static portMUX_TYPE wheel_mux = portMUX_INITIALIZER_UNLOCKED;

void wheel_odometry_update(wheel_odometry_t *state, const motor_state_t feedback[2], int64_t now_ms, bool zero)
{
    bool fresh = true, gap = false;
    for (unsigned int i = 0; i < 2; ++i) {
        state->age_ms[i] = feedback[i].last_feedback_ms ? now_ms - feedback[i].last_feedback_ms : -1;
        if (state->age_ms[i] < 0 || state->age_ms[i] >= WHEEL_FEEDBACK_TIMEOUT_MS ||
            !isfinite(feedback[i].angle) || !isfinite(feedback[i].speed_rpm)) fresh = false;
        if (state->initialized && state->last_feedback_ms[i] &&
            feedback[i].last_feedback_ms - state->last_feedback_ms[i] >= WHEEL_FEEDBACK_TIMEOUT_MS) gap = true;
        state->last_feedback_ms[i] = feedback[i].last_feedback_ms;
    }
    state->valid = false;
    state->position_m_avg = state->speed_mps_avg = NAN;
    for (unsigned int i = 0; i < 2; ++i) state->position_m[i] = state->speed_mps[i] = NAN;
    if (!fresh || gap) state->reference_valid = false;
    if (!fresh) return;
    if (!state->initialized || zero) {
        for (unsigned int i = 0; i < 2; ++i) state->zero_angle[i] = feedback[i].angle;
        state->initialized = state->reference_valid = true;
    }
    // After a feedback gap the motor's cumulative angle may have reset. Require
    // an explicit zero instead of silently joining two incompatible references.
    if (!state->reference_valid) return;
    const float direction[] = {WHEEL_LEFT_FORWARD_SIGN, WHEEL_RIGHT_FORWARD_SIGN};
    const float rpm_to_mps = (2.0f * 3.14159265358979323846f / 60.0f) * WHEEL_RADIUS_M;
    for (unsigned int i = 0; i < 2; ++i) {
        // Feedback is cumulative radians, so do not wrap at +/-pi or 2*pi.
        state->position_m[i] = direction[i] * (feedback[i].angle - state->zero_angle[i]) * WHEEL_RADIUS_M;
        state->speed_mps[i] = direction[i] * feedback[i].speed_rpm * rpm_to_mps;
    }
    state->position_m_avg = (state->position_m[0] + state->position_m[1]) * 0.5f;
    state->speed_mps_avg = (state->speed_mps[0] + state->speed_mps[1]) * 0.5f;
    state->valid = true;
}

static void snapshot_feedback(motor_state_t feedback[2])
{
    portENTER_CRITICAL(&motor_mux);
    feedback[0] = motors[WHEEL_LEFT_ID - 1];
    feedback[1] = motors[WHEEL_RIGHT_ID - 1];
    portEXIT_CRITICAL(&motor_mux);
}

void wheel_process(void)
{
    motor_state_t feedback[2];
    snapshot_feedback(feedback);
    const int64_t now_ms = esp_timer_get_time() / 1000;
    portENTER_CRITICAL(&wheel_mux);
    wheel_odometry_update(&odometry, feedback, now_ms, false);
    portEXIT_CRITICAL(&wheel_mux);
}

bool wheel_zero(void)
{
    motor_state_t feedback[2];
    snapshot_feedback(feedback);
    // Zero only with fresh, near-stationary wheel feedback. It never drives a motor.
    for (unsigned int i = 0; i < 2; ++i)
        if (!isfinite(feedback[i].speed_rpm) || fabsf(feedback[i].speed_rpm) > 2.0f) return false;
    portENTER_CRITICAL(&wheel_mux);
    wheel_odometry_update(&odometry, feedback, esp_timer_get_time() / 1000, true);
    const bool valid = odometry.valid;
    portEXIT_CRITICAL(&wheel_mux);
    return valid;
}

void wheel_get_state(wheel_odometry_t *state)
{
    portENTER_CRITICAL(&wheel_mux);
    *state = odometry;
    portEXIT_CRITICAL(&wheel_mux);
}

void print_wheels(void)
{
    wheel_process();
    wheel_odometry_t state;
    wheel_get_state(&state);
    printf("WHEELS %s left=ID%u/%+.0f right=ID%u/%+.0f diameter=65 mm age=%lld,%lld ms\r\n",
           state.valid ? "valid" : "unavailable", WHEEL_LEFT_ID, WHEEL_LEFT_FORWARD_SIGN,
           WHEEL_RIGHT_ID, WHEEL_RIGHT_FORWARD_SIGN, (long long)state.age_ms[0], (long long)state.age_ms[1]);
    if (!state.valid) {
        printf("WHEELS need fresh feedback; after an interruption, stop wheels then send wheel zero\r\n");
        return;
    }
    printf("WHEELS left_x=%.5f m right_x=%.5f m x=%.5f m left_v=%.5f m/s right_v=%.5f m/s v=%.5f m/s\r\n",
           state.position_m[0], state.position_m[1], state.position_m_avg,
           state.speed_mps[0], state.speed_mps[1], state.speed_mps_avg);
}
