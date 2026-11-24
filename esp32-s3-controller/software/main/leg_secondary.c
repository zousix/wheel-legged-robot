#include <math.h>
#include <stdio.h>
#include "esp_timer.h"
#include "leg_control_internal.h"

// ID2 (right) / ID3 (left) uses the original raw-to-joint direction.
bool secondary_active, secondary_calibrated;
static bool pending;
static float zero_right, zero_left;
static int64_t stationary_ms = -1, retry_ms, start_ms, last_ms;
static leg_pid_t pid_state;
static unsigned int pid_revision;
static const char *result = "off";

bool secondary_length_state_locked(CascadePID *length)
{
    if (!secondary_active || !leg_at_start_pose(&pid_state) ||
        esp_timer_get_time()/1000-last_ms>=30) return false;
    *length=pid_state.length;
    return true;
}

bool secondary_spring_snapshot_locked(leg_pid_t *pid, float reference[2], int64_t *session)
{
    if (!secondary_active) return false;
    *pid=pid_state;
    if (pid_revision!=tuning_revision) copy_tuning_locked(pid);
    reference[0]=zero_right;reference[1]=zero_left;*session=start_ms;
    return true;
}

bool secondary_spring_matches_locked(int64_t session)
{
    return secondary_active && start_ms==session;
}

void secondary_spring_commit_locked(int64_t now, const leg_pid_t *pid, const float output[2])
{
    pid_state=*pid;pid_revision=tuning_revision;last_ms=now;
    result=fabsf(SPRING_TARGET_LENGTH-pid->pos[0])<.003f &&
        fabsf(SPRING_TARGET_ANGLE-pid->pos[1])<3*(float)M_PI/180 ? "holding" : "settling";
    motors[1].target_voltage=output[0];motors[2].target_voltage=output[1];
}

void secondary_reference_locked(float reference[2])
{
    reference[0] = zero_right;
    reference[1] = zero_left;
}

void secondary_load_reference_locked(float reference_right, float reference_left)
{
    zero_right = reference_right;
    zero_left = reference_left;
    secondary_calibrated = true;
}

static bool secondary_set_reference(bool restore, float reference_right, float reference_left)
{
    const int64_t now = esp_timer_get_time() / 1000;
    portENTER_CRITICAL(&motor_mux);
    const motor_state_t right = motors[1], left = motors[2];
    bool ready = !spring_active && !secondary_active && !pulse_active && !balance_active &&
        right.last_feedback_ms != 0 && left.last_feedback_ms != 0 &&
        now - right.last_feedback_ms < 100 && now - left.last_feedback_ms < 100 &&
        isfinite(right.angle) && isfinite(left.angle) &&
        fabsf(right.speed_rpm) < 3 && fabsf(left.speed_rpm) < 3;
    for (size_t i = 0; i < 8; ++i) ready &= motors[i].target_voltage == 0;
    if (restore) ready &= isfinite(reference_right) && isfinite(reference_left) &&
        fabsf(right.angle - reference_right) < (float)M_PI && fabsf(left.angle - reference_left) < (float)M_PI;
    if (ready) {
        zero_right = restore ? reference_right : right.angle;
        zero_left = restore ? reference_left : left.angle;
        secondary_calibrated = true;
        pending = false;
    }
    const float zr = zero_right, zl = zero_left;
    portEXIT_CRITICAL(&motor_mux);
    if (ready) printf("LEG23 %s: ID2=%.3f ID3=%.3f rad; RAM only\r\n", restore ? "reference restored" : "calibrated", zr, zl);
    else printf("LEG23 calibration rejected: stop both legs; ID2/ID3 must be online and stationary\r\n");
    return ready;
}

bool secondary_calibrate(void)
{
    return secondary_set_reference(false, 0, 0);
}

bool secondary_restore_reference(float reference_right, float reference_left)
{
    return secondary_set_reference(true, reference_right, reference_left);
}

void secondary_stop_locked(const char *reason)
{
    secondary_active = pending = false;
    result = reason;
    motors[1].target_voltage = motors[2].target_voltage = 0;
}

void secondary_arm_locked(void)
{
    pending = true;
    stationary_ms = -1;
    retry_ms = 0;
}

static void start_secondary(bool automatic)
{
    const int64_t now = esp_timer_get_time() / 1000;
    portENTER_CRITICAL(&motor_mux);
    bool ready = secondary_calibrated && !secondary_active && !pulse_active && !balance_active &&
        (!automatic || pending);
    for (size_t i = 0; i < 8; ++i)
        if (i != LEG_RIGHT_INDEX && i != LEG_LEFT_INDEX) ready &= motors[i].target_voltage == 0;
    portEXIT_CRITICAL(&motor_mux);
    portENTER_CRITICAL(&motor_mux);
    if (ready && secondary_calibrated && !secondary_active && !pulse_active &&
        (!automatic || pending)) {
        copy_tuning_locked(&pid_state);
        pid_revision = tuning_revision;
        start_ms = last_ms = now;
        secondary_active = true;
        pending = false;
        result = "settling";
    } else ready = false;
    portEXIT_CRITICAL(&motor_mux);
    printf(ready ? "LEG23 active: independent leg-length PID, target=128.2 mm / 90 deg\r\n" :
                   "LEG23 rejected: check feedback, references and pose limits\r\n");
}

void secondary_autostart_process(int64_t now)
{
    portENTER_CRITICAL(&motor_mux);
    if (!pending) {
        portEXIT_CRITICAL(&motor_mux);
        return;
    }
    const motor_state_t right = motors[1], left = motors[2];
    const bool ready = !secondary_active && !pulse_active && !balance_active &&
        right.target_voltage == 0 && left.target_voltage == 0 &&
        right.last_feedback_ms != 0 && left.last_feedback_ms != 0 &&
        now - right.last_feedback_ms < 30 && now - left.last_feedback_ms < 30 &&
        isfinite(right.angle) && isfinite(left.angle) &&
        fabsf(right.speed_rpm) < 3 && fabsf(left.speed_rpm) < 3;
    if (!ready) stationary_ms = -1;
    else if (stationary_ms < 0) stationary_ms = now;
    const bool start = ready && now >= retry_ms && now - stationary_ms >= 300;
    if (start) {
        retry_ms = now + 1000;
        if (!secondary_calibrated) {
            const float turn = 2 * (float)M_PI;
            zero_right = LEG23_RIGHT_ZERO + roundf((right.angle - LEG23_RIGHT_ZERO) / turn) * turn;
            zero_left = LEG23_LEFT_ZERO + roundf((left.angle - LEG23_LEFT_ZERO) / turn) * turn;
            secondary_calibrated = true;
        }
    }
    portEXIT_CRITICAL(&motor_mux);
    if (start) start_secondary(true);
}

void secondary_process(int64_t now)
{
    portENTER_CRITICAL(&motor_mux);
    const bool active = secondary_active && !spring_active, calibrated = secondary_calibrated;
    const motor_state_t right = motors[1], left = motors[2];
    const float zr = zero_right, zl = zero_left;
    const int64_t session = start_ms;
    const unsigned int revision = tuning_revision;
    leg_pid_t pid = pid_state;
    if (active && revision != pid_revision) copy_tuning_locked(&pid);
    portEXIT_CRITICAL(&motor_mux);
    if (!active) return;
    const char *fault = NULL;
    float output[2];
    if (!calibrated) fault = "ID2/ID3-calibration-lost";
    else fault = spring_voltage_directed(-1, right.angle, left.angle, zr, zl,
        right.speed_rpm, left.speed_rpm, SPRING_TARGET_LENGTH, SPRING_TARGET_ANGLE,
        SPRING_LOAD_FEEDFORWARD, output, &pid);
    portENTER_CRITICAL(&motor_mux);
    if (secondary_active && start_ms == session) {
        if (fault || !secondary_calibrated) {
            fault = fault ? fault : "ID2/ID3-calibration-lost";
            secondary_stop_locked(fault);
            spring_active = spring_autostart_pending = false;
            spring_result = fault;
            for (size_t i = 0; i < 8; ++i) motors[i].target_voltage = 0;
        } else if (revision == tuning_revision) {
            pid_state = pid;
            pid_revision = revision;
            last_ms = now;
            result = fabsf(SPRING_TARGET_LENGTH - pid.pos[0]) < 0.003f &&
                fabsf(SPRING_TARGET_ANGLE - pid.pos[1]) < 3 * (float)M_PI / 180 ? "holding" : "settling";
            motors[1].target_voltage = fmaxf(-SPRING_VOLTAGE_LIMIT, fminf(SPRING_VOLTAGE_LIMIT, output[0]));
            motors[2].target_voltage = fmaxf(-SPRING_VOLTAGE_LIMIT, fminf(SPRING_VOLTAGE_LIMIT, output[1]));
        }
    }
    portEXIT_CRITICAL(&motor_mux);
}

void secondary_print(void)
{
    portENTER_CRITICAL(&motor_mux);
    const motor_state_t right = motors[1], left = motors[2];
    const bool calibrated = secondary_calibrated, active = secondary_active, waiting = pending;
    const float zr = zero_right, zl = zero_left;
    const char *mode = result;
    const leg_pid_t pid=pid_state;
    const int64_t sample_ms=last_ms;
    portEXIT_CRITICAL(&motor_mux);
    float pos[2];
    const int64_t now = esp_timer_get_time() / 1000;
    if (calibrated && now - right.last_feedback_ms < 100 && now - left.last_feedback_ms < 100 &&
        solve_leg_directed(-1, right.angle, left.angle, zr, zl, pos)) {
        printf("LEG23 length=%.1f mm angle=%.1f deg output=%s spring=%s autostart=%s\r\n",
            pos[0] * 1000, pos[1] * 180 / (float)M_PI, active ? "spring" : "manual",
            mode, waiting ? "waiting" : "cancelled");
        if (active && pid.feedback_valid) printf("LEG_PID side=23 t_ms=%lld age_ms=%lld L_mm=%.3f dl_mps=%.5f length_N=%.4f integral_N=%.4f ff_N=%.3f force_N=%.4f hip_Nm=%.5f V23=%.3f,%.3f outer_integral_mps=%.5f speed_ref_mps=%.5f\r\n",
            (long long)sample_ms,(long long)(now-sample_ms),pid.pos[0]*1000,pid.spd[0],
            pid.length.output,pid.length.inner.integral,SPRING_LOAD_FEEDFORWARD,
            pid.length.output+SPRING_LOAD_FEEDFORWARD,pid.angle.output,right.target_voltage,left.target_voltage,pid.length.outer.integral,pid.length.outer.output);
    } else printf("LEG23 calibration/feedback unavailable; spring=%s autostart=%s\r\n",
        mode, waiting ? "waiting" : "cancelled");
}

void secondary_start(void)
{
    start_secondary(false);
}

