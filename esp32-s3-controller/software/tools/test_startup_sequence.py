"""Automatic VMC support on boot, explicit balance handoff and stop cancellation."""
from pathlib import Path
import subprocess
import tempfile

source=Path(__file__).resolve().parents[1]/'main'
with tempfile.TemporaryDirectory(prefix='startup-sequence-') as directory:
    root=Path(directory);(root/'freertos').mkdir()
    (root/'freertos/FreeRTOS.h').write_text('#pragma once\ntypedef int portMUX_TYPE;\n#define portENTER_CRITICAL(x) ((void)(x))\n#define portEXIT_CRITICAL(x) ((void)(x))\n')
    (root/'esp_err.h').write_text('#pragma once\ntypedef int esp_err_t;\n')
    (root/'esp_timer.h').write_text('#pragma once\n#include <stdint.h>\nint64_t esp_timer_get_time(void);\n')
    (root/'test.c').write_text(r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "balance_control.h"
#include "leg_control_internal.h"
motor_state_t motors[8];portMUX_TYPE motor_mux;
static int64_t now=1000;
static imu_state_t imu={.phase=IMU_READY,.roll_deg=180};
int64_t esp_timer_get_time(void) { return now*1000; }
void imu_get_state(imu_state_t *s) { *s=imu; }
void wheel_get_state(wheel_odometry_t *s) { *s=(wheel_odometry_t){0}; }
static void feedback(void) { for(int i=0;i<6;i++)motors[i].last_feedback_ms=now-1; }
static void tick(void) {
    now+=4;feedback();spring_autostart_process(now);secondary_autostart_process(now);
    assert(project_pose_control_allowed(now));spring_process(now);secondary_process(now);
}
static void pose(float length) {
    float lo=0,hi=1.4f,pos[2];
    for(int i=0;i<40;i++) { float a=(lo+hi)/2;assert(solve_leg(-a,a,0,0,pos));if(pos[0]>length)lo=a;else hi=a; }
    float a=(lo+hi)/2;
    motors[1].angle=LEG23_RIGHT_ZERO+a;motors[2].angle=LEG23_LEFT_ZERO-a;
    motors[4].angle=LEG_RIGHT_ZERO-a;motors[3].angle=LEG_LEFT_ZERO+a;
}

static void check_shared_length(void) {
    leg_pid_t pair[2];init_leg_pid(&pair[0]);init_leg_pid(&pair[1]);
    const float zero[2][2]={{0,0},{0,0}};
    motor_state_t motor[2][2]={{{.angle=-.60f,.speed_rpm=.4f},{.angle=.64f,.speed_rpm=-.2f}},
                              {{.angle=.72f,.speed_rpm=-.1f},{.angle=-.68f,.speed_rpm=.3f}}};
    float output[2][2];
    for(int cycle=0;cycle<20;cycle++) {
        leg_pid_t measured[2]={pair[0],pair[1]};
        for(int side=0;side<2;side++)assert(!spring_voltage_directed(side==0 ? 1 : -1,
            motor[side][0].angle,motor[side][1].angle,0,0,
            motor[side][0].speed_rpm,motor[side][1].speed_rpm,
            SPRING_TARGET_LENGTH,SPRING_TARGET_ANGLE,SPRING_LOAD_FEEDFORWARD,output[side],&measured[side]));
        CascadePID expected=pair[0].length;
        PID_CascadeCalc(&expected,SPRING_TARGET_LENGTH,
            (measured[0].pos[0]+measured[1].pos[0])*.5f,(measured[0].spd[0]+measured[1].spd[0])*.5f);
        assert(!spring_voltage_pair(motor,zero,pair,output));
        assert(memcmp(&pair[0].length,&expected,sizeof(expected))==0);
        assert(memcmp(&pair[1].length,&expected,sizeof(expected))==0);
        for(int side=0;side<2;side++) {
            assert(memcmp(&pair[side].angle,&measured[side].angle,sizeof(CascadePID))==0);
            for(int j=0;j<2;j++)assert(isfinite(output[side][j]) && fabsf(output[side][j])<=6.9f);
        }
    }
    motor[1][0].speed_rpm=NAN;
    CascadePID before=pair[0].length;
    assert(spring_voltage_pair(motor,zero,pair,output));
    assert(memcmp(&before,&pair[0].length,sizeof(before))==0);
    puts("PASS: unequal leg lengths/rates use mean feedback, one integration per cycle, independent angles, invalid pair rejected");
}
int main(void) {
    check_shared_length();
    assert(LEG_AUTOSTART_ENABLED==1);imu.sensor.acceleration_g[2]=-1;
    load_leg_references();pose(.150f);feedback();
    if(LEG_AUTOSTART_ENABLED)arm_spring_autostart();
    // Boot remains pending with no fresh feedback; no wheel or leg output.
    now+=500;spring_autostart_process(now);secondary_autostart_process(now);
    assert(!spring_active && !secondary_active && !balance_active && spring_autostart_pending);
    for(int i=0;i<8;i++)assert(motors[i].target_voltage==0);
    for(int i=0;i<70;i++)tick();
    assert(!spring_active && !secondary_active && !balance_active);
    for(int i=0;i<20;i++)tick();
    assert(spring_active && secondary_active && !balance_active && !spring_autostart_pending);
    // Original gains and bearing load, without enabling wheel drive or balance.
    leg_pid_t tuning;copy_tuning_locked(&tuning);
    assert(tuning.length.outer.kp==15 && tuning.length.outer.ki==.045f && tuning.length.outer.kd==0);
    assert(tuning.length.inner.kp==20 && tuning.length.inner.ki==.1f && tuning.length.inner.kd==30);
    assert(tuning.length.outer.maxIntegral==30 && tuning.length.inner.maxIntegral==30);
    assert(SPRING_LOAD_FEEDFORWARD==4);
    assert(motors[0].target_voltage==0 && motors[5].target_voltage==0);
    start_balance();assert(!balance_active);
    tick();
    assert(spring_active && secondary_active && !balance_active);
    assert(motors[0].target_voltage==0 && motors[5].target_voltage==0);
    start_balance();assert(!balance_active && spring_active && secondary_active);
    puts("PASS: boot waits for fresh/stationary feedback then starts original leg PID + 4 N/leg support; wheels zero; no automatic balance");
    pose(SPRING_TARGET_LENGTH+.0005f);
    for(int i=0;i<150;i++) {tick();assert(!balance_active);}
    motors[4].angle+=.2f;tick();
    start_balance();assert(!balance_active && spring_active && secondary_active);
    pose(SPRING_TARGET_LENGTH+.0005f);
    for(int i=0;i<150;i++)tick();
    motors[1].last_feedback_ms=now-101;
    start_balance();assert(!balance_active);feedback();
    CascadePID history, left_history;
    assert(secondary_length_state_locked(&left_history));
    assert(spring_balance_handoff_locked(&history));
    assert(memcmp(&history,&left_history,sizeof(history))==0);
    secondary_process(now);
    CascadePID unchanged;assert(spring_balance_handoff_locked(&unchanged));
    assert(memcmp(&history,&unchanged,sizeof(history))==0);
    // Only same-unit force PID histories are transferred from VMC support.
    assert(fabsf(history.inner.integral)>0 && fabsf(history.outer.integral)>0);
    robot_state_input_t in={0};in.imu=imu;in.now_us=now*1000;
    in.joints[0]=motors[1];in.joints[1]=motors[2];in.joints[2]=motors[4];in.joints[3]=motors[3];
    in.reference[0]=LEG23_RIGHT_ZERO;in.reference[1]=LEG23_LEFT_ZERO;
    in.reference[2]=LEG_RIGHT_ZERO;in.reference[3]=LEG_LEFT_ZERO;
    in.calibrated[0]=in.calibrated[1]=true;
    in.wheels.last_feedback_ms[0]=in.wheels.last_feedback_ms[1]=now-1;
    balance_core_t expected;balance_output_t output;
    assert(balance_core_init(&expected,&in,&history));expected.length=history;
    expected.support_handoff_start_us=in.now_us;
    assert(balance_core_step(&expected,&in,&output));
    assert(output.support_blend==0 && output.roll_force==0);
    for(int side=0;side<2;side++)assert(fabsf(output.force[side]-output.length_force-4)<.0001f);
    float voltage[6];for(int i=0;i<6;i++)voltage[i]=motors[i].target_voltage;
    start_balance();assert(balance_active && !spring_active && !secondary_active);
    for(int i=0;i<6;i++)assert(motors[i].target_voltage==voltage[i]);
    balance_process(now);
    for(int i=0;i<6;i++)assert(fabsf(motors[i].target_voltage-output.voltage[i])<.0001f);
    puts("PASS: VMC handoff preserves force PID history and joint voltages until the balance update");
    stop_all_motors();for(int i=0;i<6;i++)assert(motors[i].target_voltage==0);
    arm_spring_autostart();stop_all_motors();
    for(int i=0;i<300;i++)tick();
    assert(!spring_active && !secondary_active && !balance_active && !spring_autostart_pending);
    puts("PASS: stop cancels VMC support and balance without automatic reactivation");
    // Explicit start accepts each loaded leg within the requested band.
    const float accepted[]={SPRING_TARGET_LENGTH-.0095f,SPRING_TARGET_LENGTH+.0095f};
    for(unsigned int i=0;i<2;i++) {
        pose(accepted[i]);feedback();start_spring();tick();
        start_balance();assert(balance_active);stop_all_motors();
    }
    const float rejected[]={SPRING_TARGET_LENGTH-.0105f,SPRING_TARGET_LENGTH+.0105f};
    for(unsigned int i=0;i<2;i++) {
        pose(rejected[i]);feedback();start_spring();tick();
        start_balance();assert(!balance_active);stop_all_motors();
    }
    // One leg outside the band cannot be hidden by the average length.
    pose(SPRING_TARGET_LENGTH+.0105f);
    const float outside[2]={motors[4].angle,motors[3].angle};
    pose(SPRING_TARGET_LENGTH);motors[4].angle=outside[0];motors[3].angle=outside[1];
    feedback();start_spring();tick();start_balance();assert(!balance_active);stop_all_motors();
    leg_pid_t check={.feedback_valid=true};
    check.pos[0]=SPRING_TARGET_LENGTH+.009f;check.pos[1]=SPRING_TARGET_ANGLE;
    assert(leg_at_start_pose(&check));
    check.pos[1]+=3.1f*(float)M_PI/180;assert(!leg_at_start_pose(&check));
    check.pos[1]=SPRING_TARGET_ANGLE;check.spd[0]=.021f;assert(!leg_at_start_pose(&check));
    puts("PASS: +/-9.5 mm accepted, +/-10.5 mm and one-leg excess rejected; strict angle/symmetry gates retained");
}
''')
    names=('balance_control.c','robot_state.c','leg_control.c','leg_calibration.c',
           'leg_pulse.c','leg_spring.c','leg_secondary.c','leg_algorithm.c','leg_self_test.c',
           'leg_pos.c','leg_spd.c','leg_conv.c','PID.c','lqr_k.c','motor_output.c')
    exe=root/'test.exe'
    subprocess.run(['gcc','-std=gnu11','-O2','-Wall','-Wextra','-Werror',
                    '-I'+str(root),'-I'+str(source),str(root/'test.c'),
                    *[str(source/n) for n in names],'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
