#include <math.h>
#include <stdio.h>
#include "esp_timer.h"
#include "leg_control_internal.h"
#include "leg_spd.h"
#include "leg_conv.h"

void init_leg_pid(leg_pid_t *pid)
{
    *pid = (leg_pid_t){0};
    // 参数顺序：P、I、D、积分限幅、输出限幅。控制周期 4 ms。
    // 积分按控制周期累加，手压时需注意积分饱和。
    // Restore the user's KP=15 leg-length tuning table, including limits.
    PID_Init(&pid->length.outer, 15.0f, 0.045f, 0, 30.0f, 1.0f); // 腿长外环，输出目标速度 m/s
    PID_Init(&pid->length.inner, 20.0f, 0.1f, 30.0f, 30.0f, 10.0f); // 腿长内环，输出轴向力 N
    PID_SetErrLpfRatio(&pid->length.inner, 0.5f);            // 内环误差滤波比例
    // 单腿回正；原项目 legAnglePID 控制的是左右两条腿的角度差。
    PID_Init(&pid->angle.outer, 6.5f, 0, 0, 0, 3.0f);   // 腿角外环，输出目标角速度 rad/s
    PID_Init(&pid->angle.inner, 0.25f, 0, 0, 0, 1.0f);   // 腿角内环，输出力矩 Nm
}

static const char *spring_voltage_step(const CascadePID *shared_length, bool feedback_only, float direction, float raw_right, float raw_left, float zero_right, float zero_left,
                           float rpm_right, float rpm_left, float target_length, float target_angle,
                           float holding_force, float output[2], leg_pid_t *pid)
{
    leg_pid_t fresh_pid;
    if (direction != 1.0f && direction != -1.0f) return "invalid-leg-direction";
    if (!isfinite(rpm_right) || !isfinite(rpm_left) || !isfinite(target_length) ||
        !isfinite(target_angle) || !isfinite(holding_force)) return "non-finite-input";
    if (pid == NULL) {
        init_leg_pid(&fresh_pid);
        pid = &fresh_pid;
    }
    const bool start_angle_ramp = !pid->feedback_valid;
    pid->feedback_valid = false;
    float pos[2];
    if (!solve_leg_directed(direction, raw_right, raw_left, zero_right, zero_left, pos)) return "invalid-geometry";
    const float phi1 = (float)M_PI_2 + direction * (raw_left - zero_left);
    const float phi4 = (float)M_PI_2 + direction * (raw_right - zero_right);
    const float dphi1 = direction * rpm_left * (float)M_PI / 30;
    const float dphi4 = direction * rpm_right * (float)M_PI / 30;
    float spd[2];
    leg_spd(dphi1, dphi4, phi1, phi4, spd);
    if (!isfinite(spd[0]) || !isfinite(spd[1])) return "non-finite-speed";
    for (size_t i = 0; i < 2; ++i) {
        pid->pos[i] = pos[i];
        pid->spd[i] = spd[i];
    }
    if (start_angle_ramp) {
        pid->angle_ramp_start = pos[1];
        pid->angle_ramp_cycle = 0;
    }
    pid->feedback_valid = true;
    if (feedback_only) return NULL;
    const float dl = spd[0], dt = spd[1];
    if (shared_length) pid->length=*shared_length;
    else PID_CascadeCalc(&pid->length, target_length, pos[0], dl);
    const float force = pid->length.output + holding_force;
    const float angle_blend = fminf(1.0f,
        (float)pid->angle_ramp_cycle / (float)SPRING_ANGLE_RAMP_CYCLES);
    const float ramped_angle = pid->angle_ramp_start +
        (target_angle - pid->angle_ramp_start) * angle_blend;
    PID_CascadeCalc(&pid->angle, ramped_angle, pos[1], dt);
    if (pid->angle_ramp_cycle < SPRING_ANGLE_RAMP_CYCLES) ++pid->angle_ramp_cycle;
    const float torque = pid->angle.output;
    float motor_torque[2];
    leg_conv(force, torque, phi1, phi4, motor_torque);
    // Convert VMC torque to raw motor coordinates once, then compensate raw rpm.
    // Positive-direction pair: physical right ID5/4; negative: physical left ID2/3.
    output[0] = motor_torque_voltage(direction > 0 ? LEG_RIGHT_ID : 2U,
                                   direction * motor_torque[1], rpm_right);
    output[1] = motor_torque_voltage(direction > 0 ? LEG_LEFT_ID : 3U,
                                   direction * motor_torque[0], rpm_left);
    return isfinite(output[0]) && isfinite(output[1]) ? NULL : "non-finite-output";
}

const char *spring_voltage_directed(float direction, float raw_right, float raw_left, float zero_right, float zero_left,
    float rpm_right, float rpm_left, float target_length, float target_angle,
    float holding_force, float output[2], leg_pid_t *pid)
{
    return spring_voltage_step(NULL,false,direction,raw_right,raw_left,zero_right,zero_left,
        rpm_right,rpm_left,target_length,target_angle,holding_force,output,pid);
}

// Pair order: physical right ID5/4, physical left ID2/3. Each leg owns its PID history.
const char *spring_voltage_pair(const motor_state_t motor[2][2], const float zero[2][2],
    leg_pid_t pid[2], float output[2][2])
{
    for (unsigned int side=0;side<2;++side) {
        const char *fault=spring_voltage_step(NULL,false,side==0 ? 1 : -1,
            motor[side][0].angle,motor[side][1].angle,zero[side][0],zero[side][1],
            motor[side][0].speed_rpm,motor[side][1].speed_rpm,SPRING_TARGET_LENGTH,
            SPRING_TARGET_ANGLE,SPRING_LOAD_FEEDFORWARD,output[side],&pid[side]);
        if (fault) return fault;
    }
    return NULL;
}

const char *spring_voltage(float raw_right, float raw_left, float zero_right, float zero_left,
                           float rpm_right, float rpm_left, float target_length, float target_angle,
                           float holding_force, float output[2], leg_pid_t *pid)
{
    return spring_voltage_directed(LEG_JOINT_SIGN, raw_right, raw_left, zero_right, zero_left,
        rpm_right, rpm_left, target_length, target_angle, holding_force, output, pid);
}
