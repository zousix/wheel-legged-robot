#pragma once
#include <math.h>
#include "wheel_odometry.h"

// Current tuning leg: photo coordinates, left=phi1, right=phi4.
// Measured: outward motion increases left raw angle and decreases right raw angle.
#define LEG_JOINT_SIGN 1.0f
#define LEG_LEFT_ID 4U
#define LEG_RIGHT_ID 5U
#define LEG_LEFT_INDEX (LEG_LEFT_ID - 1U)
#define LEG_RIGHT_INDEX (LEG_RIGHT_ID - 1U)
// Confirmed mechanical references stored in firmware Flash and loaded at boot.
// Rechecked against the vertical-down pose; do not replace with a relaxed pose.
#define LEG_RIGHT_ZERO 5.671f
#define LEG_LEFT_ZERO 1.259f

// ID2/ID3 retains its original measured references and opposite joint direction.
#define LEG23_RIGHT_ZERO (-2.428f)
#define LEG23_LEFT_ZERO (-1.742f)
#define LEG_AUTOSTART_ENABLED 0 // Start preparation and balance only by explicit commands.

// 目标姿态与电压限幅
#define SPRING_TARGET_LENGTH 0.1282f       // m
#define BALANCE_START_LENGTH_TOLERANCE_M 0.010f // Startup acceptance; leg-length target remains 128.2 mm.
#define BALANCE_SUPPORT_HANDOFF_US 500000LL // Blend new support corrections after PID handoff.
#define SPRING_TARGET_ANGLE ((float)M_PI_2) // rad
#define SPRING_ANGLE_RAMP_CYCLES 500U       // 2 s at the 4 ms preparation loop
#define SPRING_VOLTAGE_LIMIT MAX_MOTOR_VOLTAGE // Original joint limit, capped by STM32 6.9 V
// 984 g assembled total; wheel+tyre 66 g/side; modeled rods 24.8 g/side.
// Unitemized fasteners are provisionally lumped into the body.
#define ROBOT_TOTAL_MASS_KG 0.984f
#define WHEEL_ASSEMBLY_MASS_KG 0.066f
#define LEG_RODS_MASS_KG 0.0248f
#define BODY_MASS_PER_SIDE_KG (ROBOT_TOTAL_MASS_KG / 2 - WHEEL_ASSEMBLY_MASS_KG - LEG_RODS_MASS_KG)
// User-selected body-weight support, shared with the balance handoff.
#define SPRING_LOAD_FEEDFORWARD 4.0f
// Diagnostic LQR output ratio; preserves leg PID, support, yaw and roll tuning.
// Candidate from the linear-model delay check; hardware stability is not yet verified.
#define BALANCE_LQR_OUTPUT_RATIO 0.3f
// B comparison: scale only wheel-torque position feedback; baseline is 1.0.
#define BALANCE_WHEEL_POSITION_RATIO 0.8f
// B baseline: full combined body-rate feedback; RAM tuning preserves joint-rate feedback.
#define BALANCE_WHEEL_BODY_RATE_RATIO 0.8f
// Known moving-link estimate only; bearing/fastener mass is not yet itemized.
// Do not substitute the reference robot's unexplained 0.05 kg as our measurement.
#define SUPPORT_ESTIMATOR_LEG_MASS_KG LEG_RODS_MASS_KG
#define MAX_MOTOR_VOLTAGE 6.9f             // 手动电压模式，V
#define MAX_WHEEL_VOLTAGE 4.0f             // 原项目 2804 车轮限幅，V

// Final voltage, after torque conversion and back-EMF compensation.
static inline float limit_motor_voltage(unsigned int id, float voltage)
{
    if (id < 1 || id > 8 || !isfinite(voltage)) return 0.0f;
    const float limit = (id == WHEEL_LEFT_ID || id == WHEEL_RIGHT_ID) ?
                        MAX_WHEEL_VOLTAGE : MAX_MOTOR_VOLTAGE;
    return fmaxf(-limit, fminf(limit, voltage));
}
#define PULSE_TRAVEL_LIMIT_RAD (40.0f * (float)M_PI / 180.0f)

// PID 参数直接写在 leg_algorithm.c 的 init_leg_pid() 中。
