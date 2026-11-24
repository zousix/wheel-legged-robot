#pragma once
#include "robot_state.h"
#include "PID.h"

typedef struct {
    CascadePID length, roll, yaw, leg_difference;
    bool initialized;
    float position_reference, yaw_reference, last_yaw, yaw_unwrapped;
    float wheel_body_rate_ratio, wheel_speed_ratio;
    int64_t support_handoff_start_us; // Zero when not handing over from leg PID.
    float last_dl[2], dd_length[2];
    bool support_test;
    CascadePID support_angle[2], support_length[2];
    float support_feedforward;
    int64_t support_deadline_ms;
} balance_core_t;

typedef struct {
    robot_state_t state;
    bool grounded; // Estimated contact diagnostic, not a mode switch.
    const char *reason;
    float voltage[6], torque[6]; // ID1..ID6, raw motor coordinates
    float force[2], hip[2], support[2]; // ID2/3 (physical left), ID5/4 (right); hip is body model torque
    float model_torque, model_hip_torque;
    float lqr_error[6]; // Exact six-state error used for this output, including position reference.
    float lqr_output_ratio, wheel_body_rate_ratio, wheel_speed_ratio;
    float length_force, length_integral, feedforward; // Normal balance diagnostics from this output sample.
    float support_blend, roll_force; // Applied support transition and roll correction.
    int64_t control_time_us, control_gap_us, imu_age_us; // Sampled by the control task, not the printer.
} balance_output_t;

bool balance_core_init(balance_core_t *core, const robot_state_input_t *input, const CascadePID *length);
bool balance_core_step(balance_core_t *core, const robot_state_input_t *input, balance_output_t *output);
bool support_test_step(balance_core_t *core, const robot_state_input_t *input, balance_output_t *output);
void start_balance(void);
void start_support_test(float feedforward);
void balance_process(int64_t now_ms);
void print_balance(void);
void print_balance_sample(void); // One synchronized line for low-overhead live capture.
void print_balance_trace(void); // Last 1024 control samples; stopped outputs only.
bool balance_rate_command(const char *line); // RAM-only wheel body-rate/speed tuning; never starts control.
bool print_balance_waveform(void);
