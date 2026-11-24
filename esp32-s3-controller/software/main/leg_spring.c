#include <math.h>
#include <stdio.h>
#include <string.h>
#include "esp_timer.h"
#include "leg_control_internal.h"

static void start_primary_spring(bool automatic);
static int64_t spring_start_ms, spring_last_ms;
static leg_pid_t spring_pid;
static float spring_waveform[12];
static int64_t autostart_stationary_ms = -1;
static int64_t autostart_retry_ms;
static leg_pid_t tuned_pid;
static bool tuning_initialized;
unsigned int tuning_revision;

void print_spring_control(void)
{
    portENTER_CRITICAL(&motor_mux);
    const bool valid=spring_active && spring_pid.feedback_valid;
    const leg_pid_t pid=spring_pid;
    const float voltage[2]={motors[LEG_RIGHT_INDEX].target_voltage,motors[LEG_LEFT_INDEX].target_voltage};
    const int64_t sample_ms=spring_last_ms;
    portEXIT_CRITICAL(&motor_mux);
    if (valid) printf("LEG_PID side=54 t_ms=%lld age_ms=%lld L_mm=%.3f dl_mps=%.5f length_N=%.4f integral_N=%.4f ff_N=%.3f force_N=%.4f hip_Nm=%.5f V54=%.3f,%.3f outer_integral_mps=%.5f speed_ref_mps=%.5f\r\n",
        (long long)sample_ms,(long long)(esp_timer_get_time()/1000-sample_ms),pid.pos[0]*1000,pid.spd[0],
        pid.length.output,pid.length.inner.integral,SPRING_LOAD_FEEDFORWARD,
        pid.length.output+SPRING_LOAD_FEEDFORWARD,pid.angle.output,voltage[0],voltage[1],pid.length.outer.integral,pid.length.outer.output);
}

bool spring_balance_handoff_locked(CascadePID length[2])
{
    CascadePID left;
    if (!spring_active || !leg_at_start_pose(&spring_pid) ||
        esp_timer_get_time()/1000-spring_last_ms>=30 ||
        !secondary_length_state_locked(&left)) return false;
    // Keep each leg's complete controller state and physical-side mapping.
    length[0]=left;
    length[1]=spring_pid.length;
    return true;
}

// Caller holds motor_mux; this template contains parameters with zero PID history.
static void init_tuning(void)
{
    if (!tuning_initialized) {
        init_leg_pid(&tuned_pid);
        tuning_initialized = true;
    }
}

void copy_tuning_locked(leg_pid_t *pid)
{
    init_tuning();
    *pid = tuned_pid;
}

static void print_pid_parameters(void)
{
    portENTER_CRITICAL(&motor_mux);
    init_tuning();
    const leg_pid_t parameters = tuned_pid;
    portEXIT_CRITICAL(&motor_mux);
    const PID *loops[] = {&parameters.length.outer, &parameters.length.inner,
                          &parameters.angle.outer, &parameters.angle.inner};
    const char *names[] = {"lo", "li", "ao", "ai"};
    for (size_t i = 0; i < 4; ++i) {
        const PID *p = loops[i];
        printf("PID %s kp=%.6g ki=%.6g kd=%.6g imax=%.6g max=%.6g lpf=%.6g\r\n",
               names[i], p->kp, p->ki, p->kd, p->maxIntegral, p->maxOutput, p->errLpfRatio);
    }
}

bool spring_pid_command(const char *line)
{
    if (strcmp(line, "pid") == 0) {
        print_pid_parameters();
        return true;
    }
    if (strncmp(line, "pid ", 4) != 0) return false;
    char loop[8], field[8], extra;
    float value;
    if (sscanf(line, "pid %7s %7s %f %c", loop, field, &value, &extra) != 3 ||
        !isfinite(value) || value < 0) {
        printf("PID rejected: pid <lo|li|ao|ai> <kp|ki|kd|imax|max|lpf> <finite nonnegative value>\r\n");
        return true;
    }
    portENTER_CRITICAL(&motor_mux);
    if (balance_active) {
        portEXIT_CRITICAL(&motor_mux);
        printf("PID rejected: stop balance mode before changing gains\r\n");
        return true;
    }
    init_tuning();
    PID *p = strcmp(loop, "lo") == 0 ? &tuned_pid.length.outer :
             strcmp(loop, "li") == 0 ? &tuned_pid.length.inner :
             strcmp(loop, "ao") == 0 ? &tuned_pid.angle.outer :
             strcmp(loop, "ai") == 0 ? &tuned_pid.angle.inner : NULL;
    float *parameter = NULL;
    if (p != NULL) {
        if (strcmp(field, "kp") == 0) parameter = &p->kp;
        else if (strcmp(field, "ki") == 0) parameter = &p->ki;
        else if (strcmp(field, "kd") == 0) parameter = &p->kd;
        else if (strcmp(field, "imax") == 0) parameter = &p->maxIntegral;
        else if (strcmp(field, "max") == 0 && value > 0) parameter = &p->maxOutput;
        else if (strcmp(field, "lpf") == 0 && value <= 1) parameter = &p->errLpfRatio;
    }
    const bool accepted = parameter != NULL;
    if (accepted) {
        *parameter = value;
        spring_pid = tuned_pid;
        ++tuning_revision;
    }
    portEXIT_CRITICAL(&motor_mux);
    if (accepted) {
        printf("PID updated %s %s=%.6g; history cleared; RAM only\r\n", loop, field, value);
        print_pid_parameters();
    } else printf("PID rejected: unknown loop/field, max must be positive, lpf must be 0..1\r\n");
    return true;
}

void arm_spring_autostart(void)
{
    portENTER_CRITICAL(&motor_mux);
    autostart_stationary_ms = -1;
    autostart_retry_ms = 0;
    spring_autostart_pending = true;
    secondary_arm_locked();
    portEXIT_CRITICAL(&motor_mux);
    printf("SPRING support armed: leg-length/angle PID + VMC, wheels zero; stop cancels\r\n");
}

void spring_autostart_process(int64_t now)
{
    portENTER_CRITICAL(&motor_mux);
    if (!spring_autostart_pending) {
        portEXIT_CRITICAL(&motor_mux);
        return;
    }
    bool idle = !spring_active && !pulse_active && !balance_active;
    for (size_t i = 0; i < 8; ++i)
        if (i != 1 && i != 2) idle &= motors[i].target_voltage == 0;
    if (!idle) spring_autostart_pending = false;
    const motor_state_t right_motor = motors[LEG_RIGHT_INDEX], left_motor = motors[LEG_LEFT_INDEX];
    const bool calibrated = leg_calibrated;
    const bool ready = idle && right_motor.last_feedback_ms != 0 && left_motor.last_feedback_ms != 0 &&
        now - right_motor.last_feedback_ms < 30 && now - left_motor.last_feedback_ms < 30 &&
        isfinite(right_motor.angle) && isfinite(left_motor.angle) &&
        fabsf(right_motor.speed_rpm) < 3 && fabsf(left_motor.speed_rpm) < 3;
    if (!ready) autostart_stationary_ms = -1;
    else if (autostart_stationary_ms < 0) autostart_stationary_ms = now;
    const bool start = ready && now >= autostart_retry_ms && now - autostart_stationary_ms >= 300;
    if (start) autostart_retry_ms = now + 1000;
    portEXIT_CRITICAL(&motor_mux);
    if (!start) return;
    if (!calibrated) {
        // Restore this leg's measured reference, matching only equivalent full turns.
        const float turn = 2 * (float)M_PI;
        const float zero_right = LEG_RIGHT_ZERO + roundf((right_motor.angle - LEG_RIGHT_ZERO) / turn) * turn;
        const float zero_left = LEG_LEFT_ZERO + roundf((left_motor.angle - LEG_LEFT_ZERO) / turn) * turn;
        calibrate_leg(true, zero_right, zero_left);
    }
    portENTER_CRITICAL(&motor_mux);
    const bool pending = spring_autostart_pending;
    portEXIT_CRITICAL(&motor_mux);
    if (!pending) return;
    start_primary_spring(true);
}

bool print_spring_waveform(void)
{
    const int64_t now = esp_timer_get_time() / 1000;
    float data[12];
    portENTER_CRITICAL(&motor_mux);
    const bool active = spring_active;
    const bool valid = active && spring_pid.feedback_valid && now - spring_last_ms < 30;
    if (valid) {
        for (size_t i = 0; i < 12; ++i) data[i] = spring_waveform[i];
    }
    portEXIT_CRITICAL(&motor_mux);
    if (valid) {
        // FireWater: length ref/fdb, speed ref/fdb, angle ref/fdb,
        // angular speed ref/fdb, ID5 rpm/voltage, ID4 rpm/voltage.
        printf("leg:%.3f,%.3f,%.5f,%.5f,%.3f,%.3f,%.5f,%.5f,%.2f,%.3f,%.2f,%.3f\r\n",
               data[0], data[1], data[2], data[3], data[4], data[5],
               data[6], data[7], data[8], data[9], data[10], data[11]);
    }
    return active;
}

void spring_process(int64_t now)
{
    portENTER_CRITICAL(&motor_mux);
    const bool active = spring_active, calibrated = leg_calibrated;
    const float zero_right = leg_zero_right, zero_left = leg_zero_left;
    const motor_state_t right_motor = motors[LEG_RIGHT_INDEX], left_motor = motors[LEG_LEFT_INDEX];
    const int64_t start_ms = spring_start_ms;
    leg_pid_t pair[2]={0};pair[0]=spring_pid;
    float reference[2][2]={{zero_right,zero_left},{0}};
    int64_t secondary_session=0;
    const bool paired=secondary_spring_snapshot_locked(&pair[1],reference[1],&secondary_session);
    const motor_state_t motor[2][2]={{right_motor,left_motor},{motors[1],motors[2]}};
    const bool pair_calibrated=secondary_calibrated;
    leg_pid_t pid = spring_pid;
    const unsigned int revision = tuning_revision;
    portEXIT_CRITICAL(&motor_mux);
    if (!active) return;
    float output[2], pair_output[2][2];
    const float target_length = SPRING_TARGET_LENGTH;
    const float target_angle = SPRING_TARGET_ANGLE;
    const char *fault = NULL;
    bool at_target = false;
    if (!calibrated) fault = "calibration-lost";
    else if (paired) {
        fault=pair_calibrated ? spring_voltage_pair(motor,reference,pair,pair_output) : "ID2/ID3-calibration-lost";
        pid=pair[0];
        if (!fault) {
            output[0]=pair_output[0][0];output[1]=pair_output[0][1];
            at_target=fabsf(target_length-pid.pos[0])<.003f && fabsf(target_angle-pid.pos[1])<3*(float)M_PI/180;
        }
    } else {
        float pos[2];
        if (solve_leg(right_motor.angle, left_motor.angle, zero_right, zero_left, pos)) {
            at_target = fabsf(target_length - pos[0]) < 0.003f && fabsf(target_angle - pos[1]) < 3 * (float)M_PI / 180;
        }
        fault = spring_voltage(right_motor.angle, left_motor.angle, zero_right, zero_left, right_motor.speed_rpm, left_motor.speed_rpm,
                               target_length, target_angle, SPRING_LOAD_FEEDFORWARD, output, &pid);
    }
    portENTER_CRITICAL(&motor_mux);
    if (spring_active && spring_start_ms == start_ms &&
        (!paired || secondary_spring_matches_locked(secondary_session))) {
        if (fault || !leg_calibrated) {
            spring_active = false;
            spring_result = fault ? fault : "calibration-lost";
            spring_autostart_pending = false;
            secondary_stop_locked(spring_result);
            for (size_t i = 0; i < 8; ++i) motors[i].target_voltage = 0;
        } else if (tuning_revision == revision) {
            if (paired) secondary_spring_commit_locked(now,&pair[1],pair_output[1]);
            spring_pid = pid;
            spring_last_ms = now;
            spring_result = at_target ? "holding" : "settling";
            for (size_t i = 0; i < 2; ++i) {
                const float limited = fmaxf(-SPRING_VOLTAGE_LIMIT, fminf(SPRING_VOLTAGE_LIMIT, output[i]));
                motors[i == 0 ? LEG_RIGHT_INDEX : LEG_LEFT_INDEX].target_voltage = limited;
            }
            spring_waveform[0] = target_length * 1000;
            spring_waveform[1] = pid.pos[0] * 1000;
            spring_waveform[2] = pid.length.outer.output;
            spring_waveform[3] = pid.spd[0];
            spring_waveform[4] = target_angle * 180 / (float)M_PI;
            spring_waveform[5] = pid.pos[1] * 180 / (float)M_PI;
            spring_waveform[6] = pid.angle.outer.output;
            spring_waveform[7] = pid.spd[1];
            spring_waveform[8] = right_motor.speed_rpm;
            spring_waveform[9] = motors[LEG_RIGHT_INDEX].target_voltage;
            spring_waveform[10] = left_motor.speed_rpm;
            spring_waveform[11] = motors[LEG_LEFT_INDEX].target_voltage;
        }
    }
    portEXIT_CRITICAL(&motor_mux);
}

static void start_primary_spring(bool automatic)
{
    const int64_t now = esp_timer_get_time() / 1000;
    portENTER_CRITICAL(&motor_mux);
    bool ready = leg_calibrated && !pulse_active && !spring_active && !balance_active &&
        (!automatic || spring_autostart_pending);
    for (size_t i = 0; i < 8; ++i)
        if (i != 1 && i != 2) ready &= motors[i].target_voltage == 0;
    portEXIT_CRITICAL(&motor_mux);
    portENTER_CRITICAL(&motor_mux);
    if (ready && leg_calibrated && !pulse_active && !spring_active && !balance_active &&
        (!automatic || spring_autostart_pending)) {
        spring_active = true;
        spring_autostart_pending = false; // Once motion starts, faults must never trigger a restart.
        init_tuning();
        spring_pid = tuned_pid;
        spring_start_ms = spring_last_ms = now;
        spring_result = "settling";
    } else ready = false;
    portEXIT_CRITICAL(&motor_mux);
    printf(ready ? "SPRING active: independent leg-length and angle PID + 4010 back-EMF; target=128.2 mm / 90.0 deg, 2 s angle ramp, max=6.9 V; stop to disable\r\n" :
                   "SPRING rejected: calibrate and stop other output modes\r\n");
}

void start_spring(void)
{
    start_primary_spring(false);
    secondary_start();
}

