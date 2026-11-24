#include <math.h>
#include <stdio.h>
#include "esp_timer.h"
#include "leg_control_internal.h"

#include "leg_pos.h"

void load_leg_references(void)
{
    // Called once at boot, before command/control tasks can enable any output.
    portENTER_CRITICAL(&motor_mux);
    leg_zero_right = LEG_RIGHT_ZERO;
    leg_zero_left = LEG_LEFT_ZERO;
    leg_calibrated = true;
    secondary_load_reference_locked(LEG23_RIGHT_ZERO, LEG23_LEFT_ZERO);
    portEXIT_CRITICAL(&motor_mux);
    printf("LEG references loaded from firmware Flash: ID2=%.3f ID3=%.3f ID5=%.3f ID4=%.3f rad; outputs remain off\r\n",
           LEG23_RIGHT_ZERO, LEG23_LEFT_ZERO, LEG_RIGHT_ZERO, LEG_LEFT_ZERO);
}

// Photo coordinates: x right, y down; left ID4=phi1, right ID5=phi4.
bool solve_leg_directed(float direction, float raw_right, float raw_left, float zero_right, float zero_left, float pos[2])
{
    const float phi1 = (float)M_PI_2 + direction * (raw_left - zero_left);
    const float phi4 = (float)M_PI_2 + direction * (raw_right - zero_right);
    const float dx = 0.06f + 0.05f * (cosf(phi4) - cosf(phi1));
    const float dy = 0.05f * (sinf(phi4) - sinf(phi1));
    const float separation = hypotf(dx, dy);
    if (!isfinite(separation) || separation < 0.001f || separation > 0.209f) return false;
    leg_pos(phi1, phi4, pos);
    return isfinite(pos[0]) && isfinite(pos[1]) && pos[0] > 0.01f && pos[0] < 0.16f;
}

bool solve_leg(float raw_right, float raw_left, float zero_right, float zero_left, float pos[2])
{
    return solve_leg_directed(LEG_JOINT_SIGN, raw_right, raw_left, zero_right, zero_left, pos);
}

void calibrate_leg(bool restore, float reference_right, float reference_left)
{
    const int64_t now = esp_timer_get_time() / 1000;
    bool ready = !restore || (isfinite(reference_right) && isfinite(reference_left));
    portENTER_CRITICAL(&motor_mux);
    ready &= !spring_active && !pulse_active && !secondary_active && !balance_active;
    for (size_t i = 0; i < 8; ++i) ready &= motors[i].target_voltage == 0.0f;
    const size_t leg_indices[] = {LEG_RIGHT_INDEX, LEG_LEFT_INDEX};
    for (size_t side = 0; side < 2; ++side) {
        const size_t i = leg_indices[side];
        ready &= motors[i].last_feedback_ms != 0 && now - motors[i].last_feedback_ms < 100 &&
                 fabsf(motors[i].speed_rpm) < 3.0f;
    }
    if (ready) {
        ready &= !restore || (fabsf(motors[LEG_RIGHT_INDEX].angle - reference_right) < (float)M_PI &&
                              fabsf(motors[LEG_LEFT_INDEX].angle - reference_left) < (float)M_PI);
    }
    if (ready) {
        leg_zero_right = restore ? reference_right : motors[LEG_RIGHT_INDEX].angle;
        leg_zero_left = restore ? reference_left : motors[LEG_LEFT_INDEX].angle;
        leg_calibrated = true;
    }
    const float zero_right = leg_zero_right, zero_left = leg_zero_left;
    portEXIT_CRITICAL(&motor_mux);
    if (ready) printf("LEG calibrated: vertical-down ID5=%.3f ID4=%.3f rad; RAM only\r\n", zero_right, zero_left);
    else printf("LEG calibration rejected: stop all motors; ID5/ID4 must be online and stationary\r\n");
}
