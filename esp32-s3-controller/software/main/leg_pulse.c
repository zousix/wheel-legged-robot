#include <math.h>
#include <stdio.h>
#include "esp_timer.h"
#include "leg_control_internal.h"
#include "wheel_odometry.h"

static unsigned int pulse_id;
static int64_t pulse_deadline_ms;
static float pulse_start[2], pulse_delta[2];
static unsigned int monitor_ids[2] = {LEG_RIGHT_ID, LEG_LEFT_ID};
static float peak_rpm[2];

bool pulse_should_stop(int64_t now, int64_t deadline, int64_t feedback_right, int64_t feedback_left,
                              float delta_right, float delta_left)
{
    return !isfinite(delta_right) || !isfinite(delta_left) ||
           now >= deadline || now < feedback_right || now < feedback_left ||
           now - feedback_right >= 30 || now - feedback_left >= 30 ||
           fabsf(delta_right) >= PULSE_TRAVEL_LIMIT_RAD || fabsf(delta_left) >= PULSE_TRAVEL_LIMIT_RAD;
}

static bool start_pulse_internal(unsigned int id, float voltage, float other_voltage, unsigned int duration_ms)
{
    const bool wheel = id == WHEEL_LEFT_ID || id == WHEEL_RIGHT_ID;
    const bool leg = id >= 2 && id <= 5;
    if ((!wheel && !leg) || !isfinite(voltage) ||
        fabsf(voltage) > 1.0f || !isfinite(other_voltage) || fabsf(other_voltage)>1.0f ||
        (voltage == 0 && other_voltage == 0) ||
        duration_ms < 20 || duration_ms > (wheel ? 100U : 150U)) return false;
    const int64_t now = esp_timer_get_time() / 1000;
    portENTER_CRITICAL(&motor_mux);
    bool ready = !pulse_active && !spring_active && !secondary_active && !balance_active;
    for (size_t i = 0; i < 8; ++i) ready &= motors[i].target_voltage == 0;
    const unsigned int selected_ids[] = {
        wheel ? WHEEL_RIGHT_ID : (id <= 3 ? 2U : LEG_RIGHT_ID),
        wheel ? WHEEL_LEFT_ID : (id <= 3 ? 3U : LEG_LEFT_ID),
    };
    for (size_t side = 0; side < 2; ++side) {
        const size_t i = selected_ids[side] - 1;
        ready &= motors[i].last_feedback_ms != 0 && now >= motors[i].last_feedback_ms &&
                 now - motors[i].last_feedback_ms < 30 && isfinite(motors[i].angle) &&
                 isfinite(motors[i].speed_rpm) && fabsf(motors[i].speed_rpm) < 3;
    }
    if (ready) {
        pulse_id = id;
        for (size_t side = 0; side < 2; ++side) {
            monitor_ids[side] = selected_ids[side];
            pulse_start[side] = motors[monitor_ids[side] - 1].angle;
            peak_rpm[side] = 0;
        }
        pulse_delta[0] = pulse_delta[1] = 0;
        pulse_deadline_ms = now + duration_ms;
        pulse_result = "active";
        pulse_active = true;
        motors[id - 1].target_voltage = voltage;
        if (other_voltage != 0) motors[selected_ids[1] - 1].target_voltage = other_voltage;
    }
    portEXIT_CRITICAL(&motor_mux);
    return ready;
}

bool start_pulse(unsigned int id, float voltage, unsigned int duration_ms)
{
    return start_pulse_internal(id,voltage,0,duration_ms);
}

bool start_pair_pulse(unsigned int id, float voltage, float other_voltage, unsigned int duration_ms)
{
    if ((id!=2 && id!=5) || duration_ms>80) return false;
    return start_pulse_internal(id,voltage,other_voltage,duration_ms);
}

void pulse_process(int64_t now)
{
    portENTER_CRITICAL(&motor_mux);
    if (pulse_active) {
        bool invalid_speed = false;
        const bool wheel = pulse_id == WHEEL_LEFT_ID || pulse_id == WHEEL_RIGHT_ID;
        for (size_t side = 0; side < 2; ++side) {
            const motor_state_t *motor = &motors[monitor_ids[side] - 1];
            pulse_delta[side] = motor->angle - pulse_start[side];
            if (fabsf(motor->speed_rpm) > fabsf(peak_rpm[side])) peak_rpm[side] = motor->speed_rpm;
            invalid_speed |= !isfinite(motor->speed_rpm) || (wheel && fabsf(motor->speed_rpm) > 80);
        }
        if (invalid_speed || pulse_should_stop(now, pulse_deadline_ms, motors[monitor_ids[0] - 1].last_feedback_ms,
                              motors[monitor_ids[1] - 1].last_feedback_ms, pulse_delta[0], pulse_delta[1])) {
            for (size_t i = 0; i < 8; ++i) motors[i].target_voltage = 0;
            pulse_active = false;
            pulse_result = now >= pulse_deadline_ms ? "timeout" : "feedback-speed-or-travel-limit";
        }
    }
    portEXIT_CRITICAL(&motor_mux);
}

void print_pulse(void)
{
    portENTER_CRITICAL(&motor_mux);
    const unsigned int id = pulse_id;
    const float delta_right = pulse_delta[0], delta_left = pulse_delta[1];
    const char *result = pulse_result;
    const unsigned int right_id = monitor_ids[0], left_id = monitor_ids[1];
    const float right_rpm = peak_rpm[0], left_rpm = peak_rpm[1];
    portEXIT_CRITICAL(&motor_mux);
    printf("PULSE ID=%u result=%s delta_ID%u=%.4f rad delta_ID%u=%.4f rad peak_ID%u=%.1f rpm peak_ID%u=%.1f rpm\r\n",
           id, result, right_id, delta_right, left_id, delta_left, right_id, right_rpm, left_id, left_rpm);
}
