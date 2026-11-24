#pragma once
#include "controller_state.h"

// 2026-10-19: physical sides and forward signs verified by separate hand turns.
#define WHEEL_LEFT_ID 1U
#define WHEEL_RIGHT_ID 6U
#define WHEEL_LEFT_FORWARD_SIGN 1.0f
#define WHEEL_RIGHT_FORWARD_SIGN (-1.0f)
#define WHEEL_RADIUS_M 0.0325f // User's 2026-10-18 tyre outer diameter: 65 mm.
#define WHEEL_FEEDBACK_TIMEOUT_MS 100

typedef struct {
    bool initialized, reference_valid, valid;
    float zero_angle[2], position_m[2], speed_mps[2]; // left, right; forward positive
    float position_m_avg, speed_mps_avg;
    int64_t age_ms[2], last_feedback_ms[2];
} wheel_odometry_t;

void wheel_odometry_update(wheel_odometry_t *state, const motor_state_t feedback[2], int64_t now_ms, bool zero);
void wheel_process(void);
bool wheel_zero(void);
void wheel_get_state(wheel_odometry_t *state);
void print_wheels(void);
