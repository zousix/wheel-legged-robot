#pragma once
#include "robot_state.h"

typedef struct {
    bool valid, wheel_limited;
    const char *reason;
    float length_m, position_error_m, speed_error_mps;
    float model_torque_nm, model_hip_torque_nm;
    float wheel_forward_raw_nm, wheel_forward_limited_nm;
} lqr_preview_t;

// Read-only diagnostic. Tp is before VMC/joint limits, not a motor command.
bool lqr_preview_calculate(const robot_state_t *state, float position_reference_m,
                           float speed_reference_mps, lqr_preview_t *preview);
void print_lqr_preview(void);
