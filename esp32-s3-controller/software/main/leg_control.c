#include <math.h>
#include <stdio.h>
#include <string.h>
#include "esp_timer.h"
#include "leg_control_internal.h"
#include "imu_service.h"

bool leg_calibrated;
float leg_zero_right, leg_zero_left;
bool pulse_active;
bool spring_active;
bool balance_active;
bool spring_autostart_pending;
const char *spring_result = "off";
const char *pulse_result = "idle";

bool project_pose_control_allowed(int64_t now_ms)
{
    (void)now_ms;
    portENTER_CRITICAL(&motor_mux);
    const bool active = spring_active || secondary_active;
    portEXIT_CRITICAL(&motor_mux);
    if (!active) return true;
    imu_state_t imu;
    imu_get_state(&imu);
    // User removed the 45-degree pause and 4-second recovery; reject invalid data only.
    if (isfinite(imu.body_pitch_deg)) return true;
    portENTER_CRITICAL(&motor_mux);
    for (size_t i = 0; i < 8; ++i) motors[i].target_voltage = 0;
    portEXIT_CRITICAL(&motor_mux);
    return false;
}

void print_leg(void)
{
    const int64_t now = esp_timer_get_time() / 1000;
    portENTER_CRITICAL(&motor_mux);
    const motor_state_t right_motor = motors[LEG_RIGHT_INDEX], left_motor = motors[LEG_LEFT_INDEX];
    const bool calibrated = leg_calibrated;
    const float zero_right = leg_zero_right, zero_left = leg_zero_left;
    const char *mode = spring_result;
    const bool active = spring_active, pulsing = pulse_active;
    portEXIT_CRITICAL(&motor_mux);
    secondary_print();
    if (!calibrated) {
        portENTER_CRITICAL(&motor_mux);
        const bool pending = spring_autostart_pending;
        portEXIT_CRITICAL(&motor_mux);
        printf("LEG uncalibrated: autostart=%s spring=%s; place both active links vertically down, then send cal\r\n",
               pending ? "waiting" : "cancelled", mode);
        return;
    }
    if (now - right_motor.last_feedback_ms >= 100 || now - left_motor.last_feedback_ms >= 100) {
        printf("LEG feedback stale; references retained\r\n");
        return;
    }
    float pos[2];
    if (!solve_leg(right_motor.angle, left_motor.angle, zero_right, zero_left, pos)) {
        printf("LEG invalid geometry; check assembly and calibration\r\n");
        return;
    }
    printf("LEG length=%.1f mm angle=%.1f deg x=%.1f mm y=%.1f mm output=%s spring=%s feedforward=%.2f N controller=project-cascade+emf\r\n",
           pos[0] * 1000, pos[1] * 180 / (float)M_PI,
           pos[0] * cosf(pos[1]) * 1000, pos[0] * sinf(pos[1]) * 1000,
           active ? "spring" : pulsing ? "pulse" : "manual", mode,
           active ? SPRING_LOAD_FEEDFORWARD : 0.0f);
    print_spring_control();
}

bool set_motor_voltage(unsigned int id, float voltage)
{
    if (id < 1 || id > 8 || !isfinite(voltage)) {
        return false;
    }

    voltage = limit_motor_voltage(id, voltage);
    portENTER_CRITICAL(&motor_mux);
    if (pulse_active || spring_active || secondary_active || balance_active) {
        portEXIT_CRITICAL(&motor_mux);
        return false;
    }
    secondary_stop_locked("off");
    motors[id - 1].target_voltage = voltage;
    spring_autostart_pending = false;
    portEXIT_CRITICAL(&motor_mux);
    return true;
}

void stop_all_motors(void)
{
    portENTER_CRITICAL(&motor_mux);
    spring_autostart_pending = false;
    secondary_stop_locked("off");
    if (pulse_active) pulse_result = "cancelled";
    spring_active = false;
    balance_active = false;
    spring_result = "off";
    pulse_active = false;
    for (size_t i = 0; i < 8; ++i) motors[i].target_voltage = 0.0f;
    portEXIT_CRITICAL(&motor_mux);
}
