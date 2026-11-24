#pragma once
#include "imu_service.h"
#include "wheel_odometry.h"

// Diagnostic only, no actuation. Model +X is forward; theta is hip-ahead-of-wheel
// positive, phi is backward positive (opposite IMU body_pitch_deg).
// Local leg arrays are left ID2/3, right ID5/4; their forward-swing signs differ.
// Order follows sys_calc.m: theta,dtheta,x,dx,phi,dphi; rad,rad/s,m,m/s,rad,rad/s.
typedef struct {
    bool valid;
    const char *reason;
    float s[6];
    float leg_length_m[2], leg_angle_rad[2], leg_rate_rad_s[2];
    float leg_length_rate_mps[2];
} robot_state_t;

typedef struct {
    imu_state_t imu;
    wheel_odometry_t wheels;
    motor_state_t joints[4]; // ID2,ID3,ID5,ID4: right,left for each leg
    float reference[4];
    bool calibrated[2];
    int64_t now_us;
} robot_state_input_t;

bool robot_state_calculate(const robot_state_input_t *input, robot_state_t *state);
// Coordinate conversion for the original controller's latest cached feedback;
// no diagnostic age thresholds are applied here.
bool robot_state_calculate_axes(const robot_state_input_t *input, robot_state_t *state);
void robot_state_snapshot(robot_state_t *state);
void print_robot_state(void);
