#include <math.h>
#include <stdio.h>
#include "esp_timer.h"
#include "leg_control_internal.h"

bool pulse_self_test(void)
{
    return !pulse_should_stop(100, 200, 99, 99, 0, 0) &&
           pulse_should_stop(200, 200, 199, 199, 0, 0) &&
           pulse_should_stop(100, 200, 70, 99, 0, 0) &&
           pulse_should_stop(100, 200, 99, 70, 0, 0) &&
           !pulse_should_stop(100, 200, 99, 99, 30.0f * (float)M_PI / 180.0f, 0) &&
           pulse_should_stop(100, 200, 99, 99, -PULSE_TRAVEL_LIMIT_RAD, 0) &&
           pulse_should_stop(100, 200, 99, 99, 0, PULSE_TRAVEL_LIMIT_RAD);
}

bool spring_self_test(void)
{
    // Self-tests use their own PID state; legtest must not disturb active control.
    leg_pid_t pid;
    init_leg_pid(&pid);
    for (unsigned int i = 0; i < 1000; ++i) {
        PID_CascadeCalc(&pid.length, 0.15f, 0.10f, 0);
        if (!isfinite(pid.length.output) || !isfinite(pid.length.outer.output) ||
            fabsf(pid.length.outer.output) > pid.length.outer.maxOutput + 0.0001f ||
            fabsf(pid.length.output) > pid.length.inner.maxOutput + 0.0001f ||
            !isfinite(pid.length.outer.integral) || !isfinite(pid.length.inner.integral) ||
            fabsf(pid.length.outer.integral) > pid.length.outer.maxIntegral + 0.0001f ||
            fabsf(pid.length.inner.integral) > pid.length.inner.maxIntegral + 0.0001f) return false;
    }
    // Test negative-error direction independently of the preceding integral buildup.
    PID_Clear(&pid.length.outer);
    PID_Clear(&pid.length.inner);
    PID_CascadeCalc(&pid.length, 0.10f, 0.15f, 0);
    if (pid.length.output >= 0) return false;
    init_leg_pid(&pid);
    if (pid.length.inner.error != 0 || pid.length.inner.output != 0) return false;
    const float rpm_one_radian = 30.0f / (float)M_PI;
    const float emf = motor_rev_voltage_4010(rpm_one_radian);
    if (fabsf(emf - 0.22878f) > 0.00001f ||
        fabsf(emf + motor_rev_voltage_4010(-rpm_one_radian)) > 0.00001f ||
        motor_rev_voltage_4010(0) != 0) return false;
    float output[2];
    // Regression: this centered pose previously produced NaN in leg_spd/leg_conv.
    for (int moving = 0; moving <= 1; ++moving) {
        if (spring_voltage(2.365f, 0.248f, 2.466f, 0.147f, -5.0f * moving, 5.0f * moving, SPRING_TARGET_LENGTH, SPRING_TARGET_ANGLE, SPRING_LOAD_FEEDFORWARD, output, NULL) != NULL || !isfinite(output[0]) || !isfinite(output[1])) return false;
    }
    const float p = (float)M_PI / 6;
    const float nominal_length = 0.05f * cosf(p) + sqrtf(0.105f * 0.105f - 0.055f * 0.055f);
    if (spring_voltage(-p, p, 0, 0, 0, 0, nominal_length, (float)M_PI_2, 0, output, NULL) != NULL ||
        fabsf(output[0]) > 0.001f || fabsf(output[1]) > 0.001f) return false;
    if (spring_voltage(-(float)M_PI / 4, (float)M_PI / 4, 0, 0, 0, 0, nominal_length, (float)M_PI_2, 0, output, NULL) != NULL ||
        output[0] <= 0 || output[1] >= 0) return false;
    if (spring_voltage(-p, p, 0, 0, -5, 5, nominal_length, (float)M_PI_2, 0, output, NULL) != NULL ||
        output[0] <= 0 || output[1] >= 0) return false;
    // Check the restoring component separately from the support feedforward.
    if (spring_voltage(1.899f, 6.933f, 2.458f, 6.421f, 35.3f, -30.9f, SPRING_TARGET_LENGTH, SPRING_TARGET_ANGLE, 0, output, NULL) != NULL ||
        output[0] >= 0 || output[1] <= 0) return false;
    if (spring_voltage(0, 0, 0, 0, 0, 0, SPRING_TARGET_LENGTH, SPRING_TARGET_ANGLE, 0, output, NULL) != NULL ||
        output[0] >= 0 || output[1] <= 0) return false;
    if (spring_voltage(NAN, p, 0, 0, 0, 0, nominal_length, (float)M_PI_2, 0, output, NULL) == NULL) return false;
    if (spring_voltage(-p, p, 0, 0, NAN, 0, nominal_length, (float)M_PI_2, 0, output, NULL) == NULL) return false;
    return true;
}

bool leg_geometry_self_test(void)
{
    // Previously measured vertical, compressed and left-shifted poses.
    const float cases[][4] = {
        {-2.469f, -6.428f, 0.150623f, (float)M_PI_2},
        {-1.713f, -7.086f, 0.12241f, 88.12f * (float)M_PI / 180.0f},
        {-2.015f, -7.105f, 0.13054f, 94.10f * (float)M_PI / 180.0f},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        float pos[2];
        if (!solve_leg(-cases[i][0], -cases[i][1], 2.469f, 6.428f, pos) ||
            fabsf(pos[0] - cases[i][2]) > 0.0001f || fabsf(pos[1] - cases[i][3]) > 0.001f) return false;
    }
    float pos[2];
    return !solve_leg(NAN, 0, 0, 0, pos);
}
