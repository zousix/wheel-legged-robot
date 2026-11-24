#include <math.h>
#include <stdio.h>
#include <string.h>
#include "esp_timer.h"
#include "balance_control.h"
#include "leg_control_internal.h"
#include "leg_conv.h"
#include "lqr_k.h"

static balance_core_t controller;
static balance_output_t last_output;
static unsigned int session;
static const char *result = "off";
static float tuned_body_rate = BALANCE_WHEEL_BODY_RATE_RATIO;
static float tuned_speed = 1.0f;

#define BALANCE_TRACE_CAPACITY 1024U
// Control-task snapshots only; printing is allowed only after all output stops.
typedef struct {
    int64_t time_us;
    int32_t gap_us, imu_age_us, wheel_age_us[2];
    float error[6], torque, hip_torque, mean_length, length_force, roll_force, hip[2], voltage[6];
} balance_trace_sample_t;
static balance_trace_sample_t trace_samples[BALANCE_TRACE_CAPACITY];
static unsigned int trace_head, trace_count;

bool balance_rate_command(const char *line)
{
    const bool speed=strncmp(line,"balance speed",13)==0;
    if (!speed && strncmp(line,"balance rate",12)!=0) return false;
    float value;char extra;
    const bool query=strcmp(line,speed ? "balance speed" : "balance rate")==0;
    if (!query && (sscanf(line,speed ? "balance speed %f %c" : "balance rate %f %c",&value,&extra)!=1 ||
                   !isfinite(value) || (speed ? value<=0 : (value<.5f || value>1.0f)))) {
        printf("BALANCE %s rejected: %s; RAM only\r\n",speed ? "speed" : "rate",
            speed ? "use balance speed <positive finite gain>" : "use balance rate <0.5..1.0>");
        return true;
    }
    portENTER_CRITICAL(&motor_mux);
    float *tuned=speed ? &tuned_speed : &tuned_body_rate;
    if (!query && value!=*tuned) {
        *tuned=value;
        if (speed) controller.wheel_speed_ratio=value;
        else controller.wheel_body_rate_ratio=value;
        // Discard an in-flight old-gain calculation; retain PID history and output.
        ++session;trace_head=trace_count=0;
    }
    const float current=*tuned;
    portEXIT_CRITICAL(&motor_mux);
    printf("BALANCE %s=%.3f; RAM only; changes clear trace, preserve PID and output\r\n",speed ? "speed" : "rate",current);
    return true;
}

static bool trace_stopped_locked(void)
{
    if (balance_active || spring_active || secondary_active || pulse_active || spring_autostart_pending) return false;
    for (unsigned int i=0;i<8;++i) if (motors[i].target_voltage!=0) return false;
    return true;
}

void print_balance_trace(void)
{
    portENTER_CRITICAL(&motor_mux);
    const bool stopped=trace_stopped_locked();
    const unsigned int count=trace_count, generation=session;
    const float body_rate=controller.wheel_body_rate_ratio;
    const float speed_ratio=controller.wheel_speed_ratio;
    const unsigned int first=(trace_head+BALANCE_TRACE_CAPACITY-count)%BALANCE_TRACE_CAPACITY;
    portEXIT_CRITICAL(&motor_mux);
    if (!stopped) { printf("BALANCE_TRACE rejected: stop all outputs first\r\n");return; }
    printf("BALANCE_TRACE_BEGIN count=%u capacity=%u gain_ratio=%.3f position_ratio=%.3f body_rate_ratio=%.3f speed_ratio=%.3f\r\n",
        count,BALANCE_TRACE_CAPACITY,BALANCE_LQR_OUTPUT_RATIO,BALANCE_WHEEL_POSITION_RATIO,body_rate,speed_ratio);
    for (unsigned int i=0;i<count;++i) {
        portENTER_CRITICAL(&motor_mux);
        const bool unchanged=trace_stopped_locked() && generation==session;
        const balance_trace_sample_t sample=trace_samples[(first+i)%BALANCE_TRACE_CAPACITY];
        portEXIT_CRITICAL(&motor_mux);
        if (!unchanged) { printf("BALANCE_TRACE_ABORT outputs restarted\r\n");return; }
        printf("BT,%lld,%ld,%ld,%ld,%ld,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
            (long long)sample.time_us,(long)sample.gap_us,(long)sample.imu_age_us,
            (long)sample.wheel_age_us[0],(long)sample.wheel_age_us[1],
            sample.error[0],sample.error[1],sample.error[2],sample.error[3],sample.error[4],sample.error[5],
            sample.torque,sample.hip_torque,sample.mean_length,sample.length_force,sample.roll_force,sample.hip[0],sample.hip[1],
            sample.voltage[0],sample.voltage[1],sample.voltage[2],sample.voltage[3],sample.voltage[4],sample.voltage[5]);
    }
    printf("BALANCE_TRACE_END count=%u\r\n",count);
}


static bool reject(balance_output_t *out, const char *reason)
{
    *out = (balance_output_t){.reason = reason};
    return false;
}

bool balance_core_init(balance_core_t *core, const robot_state_input_t *input, const CascadePID *length)
{
    robot_state_t state;
    if (!robot_state_calculate_axes(input, &state) || !isfinite(input->imu.yaw_deg) ||
        input->wheels.last_feedback_ms[0] == 0 || input->wheels.last_feedback_ms[1] == 0) return false;
    for (unsigned int i = 0; i < 4; ++i)
        if (input->joints[i].last_feedback_ms == 0) return false;
    *core = (balance_core_t){0};
    core->wheel_body_rate_ratio=BALANCE_WHEEL_BODY_RATE_RATIO;
    core->wheel_speed_ratio=1.0f;
    core->length = *length;
    PID_Clear(&core->length.outer);
    PID_Clear(&core->length.inner);
    for (unsigned int side=0;side<2;++side) {
        core->support_length[side]=core->length;
    }
    // Original Ctrl_Init gains; length uses the same restored tuning template.
    PID_Init(&core->yaw.inner, .01f, 0, 0, 0, .1f);
    PID_Init(&core->yaw.outer, 10, 0, 0, 0, 2);
    PID_Init(&core->roll.inner, 1, 0, 5, 0, 5);
    PID_Init(&core->roll.outer, 20, 0, 0, 0, 3);
    PID_SetErrLpfRatio(&core->roll.inner, .1f);
    PID_Init(&core->leg_difference.inner, .04f, 0, 0, 0, 1);
    PID_Init(&core->leg_difference.outer, 12, 0, 0, 0, 20);
    PID_SetErrLpfRatio(&core->leg_difference.outer, .5f);
    core->position_reference = state.s[2];
    core->last_yaw = core->yaw_reference = core->yaw_unwrapped = input->imu.yaw_deg * (float)M_PI / 180;
    for (unsigned int i = 0; i < 2; ++i) core->last_dl[i] = state.leg_length_rate_mps[i];
    core->initialized = true;
    return true;
}

bool balance_core_step(balance_core_t *core, const robot_state_input_t *input, balance_output_t *out)
{
    *out = (balance_output_t){0};
    if (!core->initialized) return reject(out, "not-initialized");
    if (!robot_state_calculate_axes(input, &out->state)) return reject(out, out->state.reason);
    const robot_state_t *state = &out->state;
    const float rad = (float)M_PI / 180;
    const float roll = input->imu.roll_deg * rad, pitch = input->imu.pitch_deg * rad;
    const float body_roll = input->imu.body_roll_deg * rad, yaw = input->imu.yaw_deg * rad;
    const float p = input->imu.corrected_gyro_dps[0] * rad;
    const float q = input->imu.corrected_gyro_dps[1] * rad, r = input->imu.corrected_gyro_dps[2] * rad;
    if (!isfinite(roll) || !isfinite(pitch) || !isfinite(body_roll) || !isfinite(yaw) || !isfinite(p))
        return reject(out, "invalid-attitude");
    core->yaw_unwrapped += remainderf(yaw - core->last_yaw, 2 * (float)M_PI);
    core->last_yaw = yaw;
    // Raw sensor acceleration includes gravity. Project into world vertical and
    // remove 1 g, replacing original MPU dmpGetLinearAccel().z for this mounting.
    const float gravity[3] = {-sinf(pitch), sinf(roll)*cosf(pitch), cosf(roll)*cosf(pitch)};
    float vertical_g = 0;
    for (unsigned int i = 0; i < 3; ++i) {
        if (!isfinite(input->imu.sensor.acceleration_g[i])) return reject(out, "invalid-acceleration");
        vertical_g += input->imu.sensor.acceleration_g[i] * gravity[i];
    }
    const float vertical_accel = (vertical_g - 1) * 9.8f;
    const float roll_rate = p + (q*sinf(roll) + r*cosf(roll))*tanf(pitch);
    const float yaw_rate = (q*sinf(roll) + r*cosf(roll))/cosf(pitch);
    if (!isfinite(roll_rate) || !isfinite(yaw_rate)) return reject(out, "invalid-euler-rate");
    const float length = (state->leg_length_m[0] + state->leg_length_m[1]) * .5f;
    float k[12] = {0}, error[6];
    memcpy(error, state->s, sizeof(error));
    error[2] -= core->position_reference; // Initial standing request: speed=0, fixed position.
    memcpy(out->lqr_error, error, sizeof(error));
    // Original Ctrl_Task evaluates K at the current mean leg length.
    lqr_k(length, k);
    // Standing balance: commanded-force estimates must not select airborne gains.
    out->lqr_output_ratio=BALANCE_LQR_OUTPUT_RATIO;
    out->wheel_body_rate_ratio=core->wheel_body_rate_ratio;
    out->wheel_speed_ratio=core->wheel_speed_ratio;
    for (unsigned int i = 0; i < 6; ++i) {
        out->model_torque += out->lqr_output_ratio * k[i*2] * error[i] *
            (i==2 ? BALANCE_WHEEL_POSITION_RATIO : (i==3 ? out->wheel_speed_ratio : 1.0f));
        out->model_hip_torque += out->lqr_output_ratio * k[i*2+1] * error[i];
    }
    // s1 contains -s5: body-rate contribution is (K10-K2)*s5, not K10*s5 alone.
    out->model_torque += out->lqr_output_ratio * (out->wheel_body_rate_ratio-1.0f) * (k[10]-k[2]) * error[5];
    PID_CascadeCalc(&core->yaw, core->yaw_reference, core->yaw_unwrapped, yaw_rate);
    // Same standing wheel command as Ctrl_Task, then measured raw motor signs.
    const float left = -out->model_torque - core->yaw.output;
    const float right = -out->model_torque + core->yaw.output;
    out->torque[WHEEL_LEFT_ID-1] = WHEEL_LEFT_FORWARD_SIGN*left;
    out->torque[WHEEL_RIGHT_ID-1] = WHEEL_RIGHT_FORWARD_SIGN*right;
    PID_CascadeCalc(&core->roll, 0, body_roll, roll_rate);
    out->feedforward=SPRING_LOAD_FEEDFORWARD;
    out->support_blend=1;
    // Leg PID already supplies length force and gravity; introduce only the new corrections.
    out->roll_force=core->roll.output*out->support_blend;
    // Physical left is ID2/3, right is ID5/4. Left-positive roll needs more left support.
    for (unsigned int side = 0; side < 2; ++side) {
        PID_CascadeCalc(&core->support_length[side],SPRING_TARGET_LENGTH,
            state->leg_length_m[side],state->leg_length_rate_mps[side]);
        out->force[side] = core->support_length[side].output + SPRING_LOAD_FEEDFORWARD +
            (side == 0 ? -out->roll_force : out->roll_force);
        if (state->leg_length_m[side] > .12f) out->force[side] -= out->support_blend * (state->leg_length_m[side] - .12f) * 100;
        core->dd_length[side] = .5f * (state->leg_length_rate_mps[side] - core->last_dl[side]) / .004f + .5f * core->dd_length[side];
        core->last_dl[side] = state->leg_length_rate_mps[side];
        out->support[side] = out->force[side] + SUPPORT_ESTIMATOR_LEG_MASS_KG * 9.8f -
                            SUPPORT_ESTIMATOR_LEG_MASS_KG * (core->dd_length[side] - vertical_accel);
    }
    // Legacy scalar trace fields summarize the two independent controllers only.
    out->length_force=(core->support_length[0].output+core->support_length[1].output)*.5f;
    out->length_integral=(core->support_length[0].inner.integral+core->support_length[1].inner.integral)*.5f;
    // Diagnostic only: no contact sensor measures the force used by this estimate.
    out->grounded = out->support[0] > 3 && out->support[1] > 3;
    // Local mirror signs are separate from physical left/right support assignment.
    const float difference = (float)M_PI - state->leg_angle_rad[1] - state->leg_angle_rad[0];
    const float difference_rate = -state->leg_rate_rad_s[1] - state->leg_rate_rad_s[0];
    PID_CascadeCalc(&core->leg_difference, 0, difference, difference_rate);
    out->hip[1] = out->model_hip_torque + core->leg_difference.output * (state->leg_length_m[1]/.07f);
    out->hip[0] = out->model_hip_torque - core->leg_difference.output * (state->leg_length_m[0]/.07f);
    const unsigned int ids[2][2] = {{2,3},{5,4}}; // local phi4, phi1
    for (unsigned int side = 0; side < 2; ++side) {
        const float dir = side == 0 ? -1 : 1;
        const float phi1 = (float)M_PI_2 + dir*(input->joints[side*2+1].angle-input->reference[side*2+1]);
        const float phi4 = (float)M_PI_2 + dir*(input->joints[side*2].angle-input->reference[side*2]);
        float torque[2];
        // Tp is body torque; the local angle convention determines its mirror sign.
        leg_conv(out->force[side], (side == 0 ? 1 : -1)*out->hip[side], phi1, phi4, torque);
        for (unsigned int joint = 0; joint < 2; ++joint) {
            const unsigned int id = ids[side][joint];
            out->torque[id-1] = dir*torque[joint == 0 ? 1 : 0];
            out->voltage[id-1] = motor_torque_voltage(id,out->torque[id-1],input->joints[side*2+joint].speed_rpm);
        }
    }
    const float rpm_left = WHEEL_LEFT_FORWARD_SIGN*input->wheels.speed_mps[0]/WHEEL_RADIUS_M * 30/(float)M_PI;
    const float rpm_right = WHEEL_RIGHT_FORWARD_SIGN*input->wheels.speed_mps[1]/WHEEL_RADIUS_M * 30/(float)M_PI;
    // Convert standing wheel torque including back-EMF compensation.
    out->voltage[WHEEL_LEFT_ID-1] = motor_torque_voltage(WHEEL_LEFT_ID,out->torque[WHEEL_LEFT_ID-1],rpm_left);
    out->voltage[WHEEL_RIGHT_ID-1] = motor_torque_voltage(WHEEL_RIGHT_ID,out->torque[WHEEL_RIGHT_ID-1],rpm_right);
    for (unsigned int i = 0; i < 6; ++i)
        if (!isfinite(out->torque[i]) || !isfinite(out->voltage[i])) return reject(out,"non-finite-output");
    out->reason = "standing-control";
    return true;
}

bool support_test_step(balance_core_t *core, const robot_state_input_t *input, balance_output_t *out)
{
    *out=(balance_output_t){0};
    if (!core->initialized || !core->support_test) return reject(out,"support-test-not-initialized");
    const int64_t now=input->now_us/1000;
    if (now>=core->support_deadline_ms) return reject(out,"support-test-complete");
    if (input->now_us<input->imu.sample_time_us || input->now_us-input->imu.sample_time_us>=30000)
        return reject(out,"support-test-imu-stale");
    for (unsigned int i=0;i<4;++i)
        if (input->joints[i].last_feedback_ms<=0 || now<input->joints[i].last_feedback_ms ||
            now-input->joints[i].last_feedback_ms>=30) return reject(out,"support-test-feedback-stale");
    if (!robot_state_calculate_axes(input,&out->state)) return reject(out,"support-test-invalid-state");
    const float rad=(float)M_PI/180;
    if (!isfinite(input->imu.body_roll_deg) || fabsf(input->imu.body_roll_deg)>15 ||
        fabsf(input->imu.body_pitch_deg)>15) return reject(out,"support-test-body-pose");
    for (unsigned int side=0;side<2;++side)
        if (out->state.leg_length_m[side]<.105f || out->state.leg_length_m[side]>.145f ||
            fabsf(out->state.leg_angle_rad[side]-(float)M_PI_2)>15*rad)
            return reject(out,"support-test-leg-pose");
    const unsigned int ids[2][2]={{2,3},{5,4}};
    for (unsigned int side=0;side<2;++side) {
        PID_CascadeCalc(&core->support_length[side],SPRING_TARGET_LENGTH,
                       out->state.leg_length_m[side],out->state.leg_length_rate_mps[side]);
        out->force[side]=core->support_length[side].output+core->support_feedforward;
        if (out->state.leg_length_m[side]>.12f) out->force[side]-=(out->state.leg_length_m[side]-.12f)*100;
        const float dir=side==0?-1:1;
        const float phi1=(float)M_PI_2+dir*(input->joints[side*2+1].angle-input->reference[side*2+1]);
        const float phi4=(float)M_PI_2+dir*(input->joints[side*2].angle-input->reference[side*2]);
        float torque[2];
        PID_CascadeCalc(&core->support_angle[side],SPRING_TARGET_ANGLE,
                        out->state.leg_angle_rad[side],out->state.leg_rate_rad_s[side]);
        const float local_hip=core->support_angle[side].output;
        out->hip[side]=(side==0?1:-1)*local_hip;
        leg_conv(out->force[side],local_hip,phi1,phi4,torque);
        for (unsigned int joint=0;joint<2;++joint) {
            const unsigned int id=ids[side][joint];
            out->torque[id-1]=dir*torque[joint==0?1:0];
            out->voltage[id-1]=motor_torque_voltage(id,out->torque[id-1],input->joints[side*2+joint].speed_rpm);
            if (!isfinite(out->voltage[id-1])) return reject(out,"support-test-non-finite-output");
        }
    }
    // Existing local leg-angle hold prevents free fore/aft swing during the load test.
    // Wheels/LQR/yaw/roll remain zero; force is commanded, not measured contact.
    out->reason="support-test";
    return true;
}

static void read_input(robot_state_input_t *input)
{
    *input = (robot_state_input_t){0};
    imu_get_state(&input->imu);
    portENTER_CRITICAL(&motor_mux);
    input->joints[0]=motors[1];input->joints[1]=motors[2];
    input->joints[2]=motors[LEG_RIGHT_INDEX];input->joints[3]=motors[LEG_LEFT_INDEX];
    secondary_reference_locked(input->reference);
    input->reference[2]=leg_zero_right;input->reference[3]=leg_zero_left;
    input->calibrated[0]=secondary_calibrated;input->calibrated[1]=leg_calibrated;
    const motor_state_t left=motors[WHEEL_LEFT_ID-1], right=motors[WHEEL_RIGHT_ID-1];
    portEXIT_CRITICAL(&motor_mux);
    // Driver restarts change the accumulated turn count, not the mechanical zero.
    const float turn = 2 * (float)M_PI;
    for (unsigned int i = 0; i < 4; ++i)
        input->reference[i] += roundf((input->joints[i].angle-input->reference[i])/turn)*turn;
    input->wheels.position_m_avg=(WHEEL_LEFT_FORWARD_SIGN*left.angle+WHEEL_RIGHT_FORWARD_SIGN*right.angle)*WHEEL_RADIUS_M*.5f;
    input->wheels.speed_mps[0]=WHEEL_LEFT_FORWARD_SIGN*left.speed_rpm*WHEEL_RADIUS_M*(float)M_PI/30;
    input->wheels.speed_mps[1]=WHEEL_RIGHT_FORWARD_SIGN*right.speed_rpm*WHEEL_RADIUS_M*(float)M_PI/30;
    input->wheels.speed_mps_avg=(input->wheels.speed_mps[0]+input->wheels.speed_mps[1])*.5f;
    input->wheels.last_feedback_ms[0]=left.last_feedback_ms;
    input->wheels.last_feedback_ms[1]=right.last_feedback_ms;
    input->now_us=esp_timer_get_time();
}

static void start_control(bool support_test, float feedforward)
{
    robot_state_input_t input;
    balance_core_t candidate;
    leg_pid_t tuning;
    read_input(&input);
    portENTER_CRITICAL(&motor_mux);
    copy_tuning_locked(&tuning);
    portEXIT_CRITICAL(&motor_mux);
    if (!balance_core_init(&candidate,&input,&tuning.length)) {
        printf("BALANCE rejected: need IMU ready, joint/wheel feedback, loaded leg references and finite kinematics\r\n");
        return;
    }
    robot_state_t pose;
    bool positioned=robot_state_calculate_axes(&input,&pose);
    const int64_t now_ms=input.now_us/1000;
    candidate.support_test=support_test;
    candidate.support_feedforward=feedforward;
    candidate.support_deadline_ms=now_ms+10000;
    for (unsigned int side=0;side<2;++side) {
        candidate.support_angle[side]=tuning.angle;
        PID_Clear(&candidate.support_angle[side].outer);PID_Clear(&candidate.support_angle[side].inner);
    }
    for (unsigned int i=0;i<4;++i)
        positioned &= input.joints[i].last_feedback_ms>0 &&
            now_ms-input.joints[i].last_feedback_ms>=0 && now_ms-input.joints[i].last_feedback_ms<100;
    for (unsigned int i=0;i<2;++i)
        positioned &= input.wheels.last_feedback_ms[i]>0 &&
            now_ms-input.wheels.last_feedback_ms[i]>=0 && now_ms-input.wheels.last_feedback_ms[i]<100;
    for (unsigned int side=0;side<2 && positioned;++side) {
        positioned=fabsf(pose.leg_length_m[side]-SPRING_TARGET_LENGTH)<=BALANCE_START_LENGTH_TOLERANCE_M &&
            fabsf(pose.leg_angle_rad[side]-SPRING_TARGET_ANGLE)<=3*(float)M_PI/180 &&
            fabsf(pose.leg_length_rate_mps[side])<.02f && fabsf(pose.leg_rate_rad_s[side])<.2f;
    }
    if (!positioned) {
        printf("BALANCE waiting: each leg needs 128.2 +/-10 mm, 90 +/-3 deg and low speed; then resend balance\r\n");
        return;
    }
    portENTER_CRITICAL(&motor_mux);
    bool idle=!balance_active && !spring_active && !secondary_active && !pulse_active;
    for (unsigned int i=0;i<8;++i) idle &= motors[i].target_voltage==0;
    const bool holding=!balance_active && !pulse_active &&
        motors[0].target_voltage==0 && motors[5].target_voltage==0 &&
        motors[6].target_voltage==0 && motors[7].target_voltage==0 &&
        spring_balance_handoff_locked(candidate.support_length);
    if (idle || holding) {
        candidate.wheel_body_rate_ratio=tuned_body_rate;
        candidate.wheel_speed_ratio=tuned_speed;
        candidate.support_handoff_start_us=holding && !support_test ? input.now_us : 0;
        controller=candidate;last_output=(balance_output_t){0};
        trace_head=trace_count=0;
        ++session;balance_active=true;spring_autostart_pending=false;
        const float right_voltage=motors[1].target_voltage,left_voltage=motors[2].target_voltage;
        secondary_stop_locked("balance-mode");
        // Retain support until the next 4 ms balance update replaces all six commands.
        if (holding) { motors[1].target_voltage=right_voltage;motors[2].target_voltage=left_voltage; }
        spring_active=false;spring_result="balance-mode";result="starting";
    }
    portEXIT_CRITICAL(&motor_mux);
    if ((idle || holding) && support_test) {
        printf("SUPPORT TEST active: feedforward=%.2f N/leg, 10000 ms; independent leg-length PID, local leg-angle hold ON, wheels/LQR/roll OFF; stop cancels\r\n",feedforward);
        return;
    }
    printf(idle || holding ? "BALANCE active: standing LQR/PID/VMC; contact estimate diagnostic only; joints 6.9 V, wheels 4 V; stop disables\r\n" :
                  "BALANCE rejected: stop current output mode first\r\n");
}

void start_balance(void)
{
    start_control(false,0);
}

void start_support_test(float feedforward)
{
    if (!isfinite(feedforward) || feedforward<0 || feedforward>6) {
        printf("SUPPORT TEST rejected: feedforward must be 0..6 N per leg\r\n");
        return;
    }
    // A new bounded stage can replace the previous one without dropping support.
    portENTER_CRITICAL(&motor_mux);
    const bool updating=balance_active && controller.support_test;
    if (updating) {
        controller.support_feedforward=feedforward;
        controller.support_deadline_ms=esp_timer_get_time()/1000+10000;
        for (unsigned int side=0;side<2;++side) {
            PID_Clear(&controller.support_length[side].outer);
            PID_Clear(&controller.support_length[side].inner);
        }
        ++session;
    }
    portEXIT_CRITICAL(&motor_mux);
    if (updating) {
        printf("SUPPORT TEST stage: feedforward=%.2f N/leg, PID reset, 10000 ms deadline renewed\r\n",feedforward);
        return;
    }
    start_control(true,feedforward);
}

void balance_process(int64_t now_ms)
{
    (void)now_ms;
    portENTER_CRITICAL(&motor_mux);
    const bool active=balance_active;
    const unsigned int generation=session;
    balance_core_t work=controller;
    portEXIT_CRITICAL(&motor_mux);
    if (!active) return;
    robot_state_input_t input;
    balance_output_t output;
    read_input(&input);
    const bool ok=work.support_test ? support_test_step(&work,&input,&output) : balance_core_step(&work,&input,&output);
    output.control_time_us=input.now_us;
    output.imu_age_us=input.imu.sample_time_us>0 ? input.now_us-input.imu.sample_time_us : -1;
    portENTER_CRITICAL(&motor_mux);
    if (balance_active && generation==session) {
        output.control_gap_us=last_output.control_time_us>0 ? input.now_us-last_output.control_time_us : 0;
        result=output.reason;last_output=output;
        if (!ok) balance_active=false;
        else controller=work;
        if (ok && !work.support_test) {
            balance_trace_sample_t *sample=&trace_samples[trace_head];
            sample->time_us=output.control_time_us;
            sample->gap_us=(int32_t)output.control_gap_us;sample->imu_age_us=(int32_t)output.imu_age_us;
            for (unsigned int i=0;i<2;++i)
                sample->wheel_age_us[i]=(int32_t)(input.now_us-input.wheels.last_feedback_ms[i]*1000);
            memcpy(sample->error,output.lqr_error,sizeof(sample->error));
            memcpy(sample->voltage,output.voltage,sizeof(sample->voltage));
            memcpy(sample->hip,output.hip,sizeof(sample->hip));
            sample->torque=output.model_torque;sample->hip_torque=output.model_hip_torque;
            sample->mean_length=(output.state.leg_length_m[0]+output.state.leg_length_m[1])*.5f;
            sample->length_force=output.length_force;sample->roll_force=output.roll_force;
            trace_head=(trace_head+1)%BALANCE_TRACE_CAPACITY;
            if (trace_count<BALANCE_TRACE_CAPACITY) ++trace_count;
        }

        for (unsigned int i=0;i<8;++i) motors[i].target_voltage=ok && i<6 ? output.voltage[i] : 0;
    }
    portEXIT_CRITICAL(&motor_mux);
}

static void print_lqr_sample(balance_output_t output)
{
    printf("LQR_SAMPLE control_us=%lld L_m=%.7f error=%.7f,%.7f,%.7f,%.7f,%.7f,%.7f T=%.7f Tp=%.7f torque_ID1_ID6=%.7f,%.7f V_ID1_ID6=%.3f,%.3f gain_ratio=%.3f wheel_position_ratio=%.3f wheel_body_rate_ratio=%.3f wheel_speed_ratio=%.3f legs_mm=%.3f,%.3f angles_deg=%.3f,%.3f\r\n",
        (long long)output.control_time_us,(output.state.leg_length_m[0]+output.state.leg_length_m[1])*.5f,
        output.lqr_error[0],output.lqr_error[1],output.lqr_error[2],output.lqr_error[3],output.lqr_error[4],output.lqr_error[5],
        output.model_torque,output.model_hip_torque,output.torque[0],output.torque[5],output.voltage[0],output.voltage[5],output.lqr_output_ratio,BALANCE_WHEEL_POSITION_RATIO,output.wheel_body_rate_ratio,output.wheel_speed_ratio,
        output.state.leg_length_m[0]*1000,output.state.leg_length_m[1]*1000,
        output.state.leg_angle_rad[0]*180/(float)M_PI,output.state.leg_angle_rad[1]*180/(float)M_PI);
}

void print_balance_sample(void)
{
    portENTER_CRITICAL(&motor_mux);
    const bool active=balance_active && !controller.support_test;
    const balance_output_t output=last_output;
    portEXIT_CRITICAL(&motor_mux);
    if (active && output.control_time_us>0) print_lqr_sample(output);
    else printf("BALANCE_SAMPLE unavailable\r\n");
}

void print_balance(void)
{
    portENTER_CRITICAL(&motor_mux);
    const bool active=balance_active;
    const char *reason=result;
    const balance_output_t output=last_output;
    const float roll_force=output.roll_force;
    const float roll_request=controller.roll.output;
    const bool support_test=controller.support_test;
    const float feedforward=controller.support_feedforward;
    const float length_pid[2]={controller.support_length[0].output,controller.support_length[1].output};
    const float length_integral[2]={controller.support_length[0].inner.integral,controller.support_length[1].inner.integral};
    const int64_t deadline=controller.support_deadline_ms;
    portEXIT_CRITICAL(&motor_mux);
    printf("BALANCE %s phase=%s ground_est=%u length=%.2f,%.2f mm T=%.5f Tp=%.5f Nm support_23_54=%.3f,%.3f N force_23_54=%.3f,%.3f N roll=%.3f N\r\n",
           active ? "active" : "off",reason,(unsigned int)output.grounded,
           output.state.leg_length_m[0]*1000,output.state.leg_length_m[1]*1000,
           output.model_torque,output.model_hip_torque,
           output.support[0],output.support[1],output.force[0],output.force[1],roll_force);
    if (active && !support_test) printf("BALANCE_TERMS t_ms=%lld length_N=%.4f integral_N=%.4f ff_N=%.3f roll_N=%.4f hip_23_54=%.5f,%.5f V1_6=%.3f,%.3f,%.3f,%.3f,%.3f,%.3f control_us=%lld gap_us=%lld imu_age_us=%lld support_blend=%.3f roll_request_N=%.4f\r\n",
        (long long)(esp_timer_get_time()/1000),output.length_force,output.length_integral,output.feedforward,roll_force,
        output.hip[0],output.hip[1],output.voltage[0],output.voltage[1],output.voltage[2],output.voltage[3],output.voltage[4],output.voltage[5],
        (long long)output.control_time_us,(long long)output.control_gap_us,(long long)output.imu_age_us,output.support_blend,roll_request);
    if (active && !support_test && output.control_time_us>0) print_lqr_sample(output);
    if (support_test) printf("SUPPORT TEST ff=%.3f N length_pid_23_54=%.3f,%.3f N integral_23_54=%.3f,%.3f N; remaining=%lld ms; forces are commands\r\n",
                            feedforward,length_pid[0],length_pid[1],length_integral[0],length_integral[1],
                            (long long)(deadline-esp_timer_get_time()/1000));
    if (active && !support_test) printf("BALANCE_LENGTH independent length_pid_23_54=%.3f,%.3f N integral_23_54=%.3f,%.3f N; preparation histories retained separately\r\n",
        length_pid[0],length_pid[1],length_integral[0],length_integral[1]);
}

bool print_balance_waveform(void)
{
    static int64_t last_ms;
    portENTER_CRITICAL(&motor_mux);
    const bool active=balance_active;
    portEXIT_CRITICAL(&motor_mux);
    if (active && esp_timer_get_time()/1000-last_ms>=50) {
        last_ms=esp_timer_get_time()/1000;
        print_balance();
    }
    return active;
}
