#pragma once

#include "controller_state.h"
#include "leg_control.h"
#include "leg_config.h"
#include "PID.h"
#include "motor_output.h"

// Shared mode/calibration state: access only while holding motor_mux.
extern float leg_zero_right, leg_zero_left;
extern bool pulse_active, spring_active;
extern bool balance_active;
extern bool spring_autostart_pending;
extern const char *pulse_result, *spring_result;

typedef struct {
    CascadePID length, angle;
    float pos[2], spd[2];
    float angle_ramp_start;
    unsigned int angle_ramp_cycle;
    bool feedback_valid;
} leg_pid_t;

// Startup/handoff only; this is not a running balance stop condition.
static inline bool leg_at_start_pose(const leg_pid_t *pid)
{
    // Explicit balance-on accepts the measured pose under load.
    return pid->feedback_valid &&
        fabsf(pid->pos[0]-SPRING_TARGET_LENGTH)<=BALANCE_START_LENGTH_TOLERANCE_M &&
        fabsf(pid->pos[1]-SPRING_TARGET_ANGLE)<=3*(float)M_PI/180 &&
        fabsf(pid->spd[0])<0.02f && fabsf(pid->spd[1])<0.2f;
}
// Caller holds motor_mux. Snapshot only; no mode or output mutation.
bool spring_balance_handoff_locked(CascadePID length[2]); // Physical left, right.
bool secondary_length_state_locked(CascadePID *length);

bool solve_leg(float raw_right, float raw_left, float zero_right, float zero_left, float pos[2]);
bool project_pose_control_allowed(int64_t now_ms);
void init_leg_pid(leg_pid_t *pid);
const char *spring_voltage(float raw_right, float raw_left, float zero_right, float zero_left,
                           float rpm_right, float rpm_left, float target_length, float target_angle,
                           float holding_force, float output[2], leg_pid_t *pid);
bool pulse_should_stop(int64_t now, int64_t deadline, int64_t feedback_right, int64_t feedback_left,
                       float delta_right, float delta_left);

// ID2/ID3 keeps its own angle state; paired length state is shared. motor_mux protects access.
extern bool secondary_active, secondary_calibrated;
extern unsigned int tuning_revision;
void copy_tuning_locked(leg_pid_t *pid);
void print_spring_control(void);
void secondary_stop_locked(const char *reason);
void secondary_arm_locked(void);
void secondary_process(int64_t now);
void secondary_autostart_process(int64_t now);
void secondary_start(void);
void secondary_print(void);
bool secondary_calibrate(void);
bool secondary_restore_reference(float reference_right, float reference_left);
// Caller holds motor_mux; right=ID2, left=ID3.
void secondary_reference_locked(float reference[2]);
void secondary_load_reference_locked(float reference_right, float reference_left);
bool solve_leg_directed(float direction, float raw_right, float raw_left,
                        float zero_right, float zero_left, float pos[2]);
const char *spring_voltage_directed(float direction, float raw_right, float raw_left,
    float zero_right, float zero_left, float rpm_right, float rpm_left,
    float target_length, float target_angle, float holding_force, float output[2], leg_pid_t *pid);

// Caller holds motor_mux; pair processing publishes both legs in one transaction.
bool secondary_spring_snapshot_locked(leg_pid_t *pid, float reference[2], int64_t *session);
bool secondary_spring_matches_locked(int64_t session);
void secondary_spring_commit_locked(int64_t now, const leg_pid_t *pid, const float output[2]);
const char *spring_voltage_pair(const motor_state_t motor[2][2], const float zero[2][2],
    leg_pid_t pid[2], float output[2][2]);
