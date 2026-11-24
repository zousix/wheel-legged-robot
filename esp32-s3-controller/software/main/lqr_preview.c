#include <math.h>
#include <stdio.h>
#include "lqr_preview.h"
#include "lqr_k.h"

static bool fail(lqr_preview_t *preview, const char *reason)
{
    *preview = (lqr_preview_t){.reason = reason};
    return false;
}

bool lqr_preview_calculate(const robot_state_t *state, float position_reference_m,
                           float speed_reference_mps, lqr_preview_t *preview)
{
    *preview = (lqr_preview_t){0};
    if (!state->valid) return fail(preview, state->reason ? state->reason : "state-invalid");
    if (!isfinite(position_reference_m) || !isfinite(speed_reference_mps))
        return fail(preview, "reference-invalid");
    for (unsigned int i = 0; i < 6; ++i)
        if (!isfinite(state->s[i])) return fail(preview, "state-non-finite");
    const float half_pi = 1.57079632679489661923f;
    const float angle_limit = half_pi / 2; // Original controller's 45-degree pose boundary.
    if (fabsf(state->s[4]) > angle_limit) return fail(preview, "body-pitch-limit");
    for (unsigned int side = 0; side < 2; ++side) {
        const float length = state->leg_length_m[side], angle = state->leg_angle_rad[side];
        // Intersection of current leg pose limits and K's fitted length range.
        if (!isfinite(length) || length < 0.085f || length > 0.14f)
            return fail(preview, "leg-length-outside-85..140mm");
        const float theta = (side == 0 ? half_pi - angle : angle - half_pi) - state->s[4];
        if (!isfinite(theta) || fabsf(theta) > angle_limit) return fail(preview, "leg-angle-limit");
    }
    preview->length_m = (state->leg_length_m[0] + state->leg_length_m[1]) * 0.5f;
    float k[12], error[6];
    lqr_k(preview->length_m, k);
    for (unsigned int i = 0; i < 6; ++i) error[i] = state->s[i];
    error[2] -= position_reference_m;
    error[3] -= speed_reference_mps;
    preview->position_error_m = error[2];
    preview->speed_error_mps = error[3];
    // sys_sim wires +K*state to its actuator coordinates. Its positive wheel
    // torque rolls toward -X; original Ctrl_Task also applies -k[0]*error.
    for (unsigned int i = 0; i < 6; ++i) {
        preview->model_torque_nm += k[i * 2] * error[i];
        preview->model_hip_torque_nm += k[i * 2 + 1] * error[i];
    }
    if (!isfinite(preview->model_torque_nm) || !isfinite(preview->model_hip_torque_nm))
        return fail(preview, "non-finite-torque");
    preview->wheel_forward_raw_nm = -preview->model_torque_nm;
    // Model wheel-torque limit. This does not replace the final 4 V wheel limit.
    preview->wheel_forward_limited_nm = fmaxf(-0.03f, fminf(0.03f, preview->wheel_forward_raw_nm));
    preview->wheel_limited = fabsf(preview->wheel_forward_raw_nm) > 0.03f;
    preview->valid = true;
    preview->reason = "preview-no-actuation";
    return true;
}

void print_lqr_preview(void)
{
    robot_state_t state;
    lqr_preview_t preview;
    robot_state_snapshot(&state);
    // Diagnostic reference: current wheel-zero origin and zero speed.
    if (!lqr_preview_calculate(&state, 0, 0, &preview)) {
        printf("LQR unavailable: %s; no motor output\r\n", preview.reason);
        return;
    }
    printf("LQR preview: L=%.2f mm ex=%.5f m ev=%.5f m/s T_model=%.6f Nm Tp_model=%.6f Nm\r\n",
           preview.length_m * 1000, preview.position_error_m, preview.speed_error_mps,
           preview.model_torque_nm, preview.model_hip_torque_nm);
    printf("LQR wheel_forward: raw=%.6f Nm limited=%.6f Nm saturated=%u; refs=wheel-zero/0mps; Tp before VMC; no motor output\r\n",
           preview.wheel_forward_raw_nm, preview.wheel_forward_limited_nm, (unsigned int)preview.wheel_limited);
}
