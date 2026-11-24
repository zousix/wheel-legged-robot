"""Compile the actual controller and original PID/kinematics on a host with gcc."""
from pathlib import Path
import subprocess
import tempfile
import math

software = Path(__file__).resolve().parents[1]
local_source = software / 'main'
with tempfile.TemporaryDirectory(prefix='leg-control-') as directory:
    root = Path(directory)
    (root / 'freertos').mkdir()
    (root / 'freertos/FreeRTOS.h').write_text('''#pragma once
typedef int portMUX_TYPE;
void test_enter(void);
#define portENTER_CRITICAL(x) ((void)(x), test_enter())
#define portEXIT_CRITICAL(x) ((void)(x))
''')
    (root / 'esp_err.h').write_text('#pragma once\ntypedef int esp_err_t;\n')
    (root / 'esp_timer.h').write_text('''#pragma once
#include <stdint.h>
int64_t esp_timer_get_time(void);
''')
    original_source = software.parents[1] / 'esp32-controller' / 'software' / 'src'
    for name in ('leg_spd', 'leg_conv'):
        code = (original_source / (name + '.c')).read_text()
        (root / (name + '_original.c')).write_text(code.replace('void ' + name + '(', 'void original_' + name + '('))
    (root / 'test.c').write_text('''#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "controller_state.h"
#include "imu_service.h"
#include "leg_control.h"
#include "leg_control_internal.h"
#include "leg_pos.h"
#include "leg_spd.h"
#include "leg_conv.h"
void original_leg_spd(float, float, float, float, float[2]);
void original_leg_conv(float, float, float, float, float[2]);
motor_state_t motors[8];
portMUX_TYPE motor_mux;
static int64_t test_now_ms = 1000;
static imu_state_t test_imu;
void imu_get_state(imu_state_t *state) { *state=test_imu; }
int64_t esp_timer_get_time(void) { return test_now_ms * 1000; }
static const char *inflight_command;
static int enter_count;
void test_enter(void) {
    if (inflight_command && ++enter_count == 2) {
        const char *command = inflight_command;
        inflight_command = NULL;
        assert(spring_pid_command(command));
    }
}
static void check_center_crossing(void) {
    int failures_before = 0, compared = 0, checked = 0;
    for (int i=90; i<=1100; ++i) for (int offset=-1; offset<=1; ++offset) {
        float p1 = (float)M_PI_2 + i*0.001f;
        float p4 = (float)M_PI_2 - i*0.001f + offset*0.001f;
        float pos[2], plus[2], minus[2], jac[2][2], spd[2], torque[2], old_spd[2], old_torque[2];
        leg_pos(p1, p4, pos);
        if (pos[0] < 0.085f || pos[0] > 0.156f) continue;
        for (int k=0; k<2; ++k) {
            leg_pos(p1+(k==0 ? 0.001f : 0), p4+(k==1 ? 0.001f : 0), plus);
            leg_pos(p1-(k==0 ? 0.001f : 0), p4-(k==1 ? 0.001f : 0), minus);
            for (int row=0; row<2; ++row) jac[row][k]=(plus[row]-minus[row])/0.002f;
        }
        leg_spd(0.7f,-0.4f,p1,p4,spd);
        leg_conv(-1.3f,0.08f,p1,p4,torque);
        original_leg_spd(0.7f,-0.4f,p1,p4,old_spd);
        original_leg_conv(-1.3f,0.08f,p1,p4,old_torque);
        for (int k=0; k<2; ++k) {
            assert(isfinite(spd[k]) && isfinite(torque[k]));
            assert(fabsf(spd[k]-(jac[k][0]*0.7f-jac[k][1]*0.4f)) < 0.002f);
            assert(fabsf(torque[k]-(-1.3f*jac[0][k]+0.08f*jac[1][k])) < 0.002f);
            if (isfinite(old_spd[k]) && isfinite(old_torque[k])) {
                assert(fabsf(spd[k]-old_spd[k]) < 0.0001f);
                assert(fabsf(torque[k]-old_torque[k]) < 0.0001f);
                ++compared;
            } else ++failures_before;
        }
        ++checked;
    }
    assert(failures_before > 0);
    printf("PASS: %d centered/near-center poses; %d finite original results agree; %d NaN results repaired\\n",
           checked, compared, failures_before);
}
static void check_original_kinematics(void) {
    int checked = 0;
    for (int i=1; i<=10; ++i) for (int j=1; j<=10; ++j) {
        float phi1 = 1.570796327f + 0.1f*i;
        float phi4 = 1.570796327f - 0.1f*j;
        float pos[2], plus[2], minus[2], jac[2][2], spd[2], torque[2];
        leg_pos(phi1, phi4, pos);
        if (pos[0] < 0.085f || pos[0] > 0.156f ||
            fabsf(pos[1]-1.570796327f) > 0.785398164f) continue;
        for (int k=0; k<2; ++k) {
            leg_pos(phi1+(k==0 ? 0.001f : 0), phi4+(k==1 ? 0.001f : 0), plus);
            leg_pos(phi1-(k==0 ? 0.001f : 0), phi4-(k==1 ? 0.001f : 0), minus);
            for (int row=0; row<2; ++row) jac[row][k]=(plus[row]-minus[row])/0.002f;
        }
        leg_spd(0.7f, -0.4f, phi1, phi4, spd);
        leg_conv(-1.3f, 0.08f, phi1, phi4, torque);
        for (int k=0; k<2; ++k) {
            assert(isfinite(spd[k]) && isfinite(torque[k]));
            assert(fabsf(spd[k]-(jac[k][0]*0.7f-jac[k][1]*0.4f)) < 0.002f);
            assert(fabsf(torque[k]-(-1.3f*jac[0][k]+0.08f*jac[1][k])) < 0.002f);
        }
        ++checked;
    }
    assert(checked > 50);
    printf("PASS: original leg_spd/leg_conv versus position derivatives at %d poses\\n", checked);
}
static void check_no_friction_compensation(void) {
    const float p = (float)M_PI/6;
    float pos[2];
    assert(solve_leg(-p, p, 0, 0, pos));
    const float errors[] = {0.0001f, 0.0006f, 0.0012f, -0.0006f, -0.0012f};
    for (size_t i=0; i<5; ++i) {
        leg_pid_t pid;
        init_leg_pid(&pid);
        PID_Init(&pid.length.outer,0,0,0,0,1);
        PID_Init(&pid.length.inner,0,0,0,0,10);
        PID_Init(&pid.angle.outer,0,0,0,0,3);
        PID_Init(&pid.angle.inner,0,0,0,0,1);
        float out[2];
        assert(spring_voltage(-p,p,0,0,0,0,pos[0]+errors[i],pos[1]+0.02f,0,out,&pid) == NULL);
        assert(fabsf(out[0])<0.0001f && fabsf(out[1])<0.0001f);
    }
    puts("PASS: no friction force or torque with disabled PID and zero feedforward");
}
static void check_gravity_and_motor_output(void) {
    const float coefficients[6]={.0101f,.0299f,.0321f,.0317f,.0316f,.0096f};
    const float one_rad_rpm=30/(float)M_PI;
    for (unsigned int id=1; id<=6; ++id) {
        const float emf=(id==1 || id==6) ? .026304f : .22878f;
        assert(fabsf(motor_torque_voltage(id,.001f,0)-.001f/coefficients[id-1])<.00001f);
        assert(fabsf(motor_torque_voltage(id,0,one_rad_rpm)-emf)<.00001f);
        assert(fabsf(motor_torque_voltage(id,0,-one_rad_rpm)+emf)<.00001f);
        assert(motor_torque_voltage(id,0,0)==0);
        const float cap=(id==1 || id==6) ? 4 : 6.9f;
        assert(fabsf(motor_torque_voltage(id,100,0)-cap)<.00001f);
        assert(fabsf(motor_torque_voltage(id,-100,0)+cap)<.00001f);
        assert(isnan(motor_torque_voltage(id,NAN,0)));
        assert(isnan(motor_torque_voltage(id,0,INFINITY)));
    }
    assert(isnan(motor_torque_voltage(0,0,0)));
    assert(isnan(motor_torque_voltage(7,0,0)));
    assert(SPRING_LOAD_FEEDFORWARD==4);
    // Independent finite-difference virtual-work check in raw motor coordinates.
    // At zero PID error, positive gravity feedforward must produce extension
    // torque on both mirrored legs, not the former negative compression force.
    for (int side=0; side<2; ++side) {
        const float dir=side ? -1 : 1;
        const unsigned int ids[2]={side ? 2U : 5U, side ? 3U : 4U};
        for (int angle=20; angle<=50; angle+=10) {
            const float p=angle*(float)M_PI/180;
            const float raw[2]={-dir*p,dir*p};
            float pos[2],out[2];
            assert(solve_leg_directed(dir,raw[0],raw[1],0,0,pos));
            if(pos[0]<.085f) continue;
            assert(spring_voltage_directed(dir,raw[0],raw[1],0,0,0,0,
                pos[0],pos[1],4.0f,out,NULL)==NULL);
            for(int i=0;i<2;i++) {
                float plus[2],minus[2];
                assert(solve_leg_directed(dir,raw[0]+(i==0?.001f:0),raw[1]+(i==1?.001f:0),0,0,plus));
                assert(solve_leg_directed(dir,raw[0]-(i==0?.001f:0),raw[1]-(i==1?.001f:0),0,0,minus));
                const float derivative=(plus[0]-minus[0])/.002f;
                const float actual_torque=out[i]*coefficients[ids[i]-1];
                assert(fabsf(actual_torque-4.0f*derivative)<.0002f);
                assert(actual_torque*derivative>0);
            }
        }
    }
    puts("PASS: per-ID torque conversion, rpm-to-rad/s EMF, final limits and gravity extension on both legs");
}
static void check_original_pose_protection(void) {
    spring_active=true;secondary_active=false;
    motors[LEG_RIGHT_INDEX].target_voltage=1;
    test_imu.body_pitch_deg=50;
    assert(project_pose_control_allowed(1004));
    assert(motors[LEG_RIGHT_INDEX].target_voltage==1);
    test_imu.body_pitch_deg=-50;
    assert(project_pose_control_allowed(1008));
    test_imu.body_pitch_deg=NAN;
    assert(!project_pose_control_allowed(1012));
    for(int i=0;i<8;i++)assert(motors[i].target_voltage==0);
    test_imu.body_pitch_deg=0;
    assert(project_pose_control_allowed(1016));
    stop_all_motors();
    memset(motors,0,sizeof(motors));
    puts("PASS: 45-degree gate/recovery removed; nonfinite attitude still zeros outputs");
}
static void check_flash_references(void) {
    load_leg_references();
    float secondary[2],reference_pos[2],wrapped_pos[2];
    secondary_reference_locked(secondary);
    assert(leg_calibrated && secondary_calibrated);
    assert(fabsf(secondary[0]+2.458f)<.00001f && fabsf(secondary[1]+1.721f)<.00001f);
    assert(fabsf(leg_zero_right-LEG_RIGHT_ZERO)<.00001f && fabsf(leg_zero_left-LEG_LEFT_ZERO)<.00001f);
    for(int side=0;side<2;side++) {
        const float dir=side==0?-1:1;
        const float zr=side==0?secondary[0]:leg_zero_right;
        const float zl=side==0?secondary[1]:leg_zero_left;
        assert(solve_leg_directed(dir,zr,zl,zr,zl,reference_pos));
        assert(solve_leg_directed(dir,zr+2*(float)M_PI,zl-4*(float)M_PI,zr,zl,wrapped_pos));
        assert(fabsf(reference_pos[0]-.150623f)<.00001f);
        assert(fabsf(reference_pos[1]-(float)M_PI_2)<.0001f);
        assert(fabsf(reference_pos[0]-wrapped_pos[0])<.00001f && fabsf(reference_pos[1]-wrapped_pos[1])<.0001f);
    }
    assert(!balance_active && !spring_active && !secondary_active && !pulse_active);
    for(int i=0;i<8;i++)assert(motors[i].target_voltage==0);
    // Runtime calibration remains RAM-only; a reboot reloads the confirmed old values.
    leg_zero_right=123;secondary_load_reference_locked(123,456);
    load_leg_references();secondary_reference_locked(secondary);
    assert(fabsf(leg_zero_right-LEG_RIGHT_ZERO)<.00001f && fabsf(secondary[0]-LEG23_RIGHT_ZERO)<.00001f);
    leg_calibrated=secondary_calibrated=false;
    puts("PASS: confirmed old references reload from firmware Flash, full-turn equivalence, zero output on boot");
}
int main(void) {
    check_flash_references();
    check_original_pose_protection();
    check_gravity_and_motor_output();
    balance_active=true;leg_calibrated=secondary_calibrated=true;
    motors[LEG_RIGHT_INDEX].last_feedback_ms=motors[LEG_LEFT_INDEX].last_feedback_ms=1000;
    assert(!set_motor_voltage(1,.1f));
    assert(!start_pulse(LEG_RIGHT_ID,.5f,50));
    start_spring();assert(!spring_active && !secondary_active);
    const unsigned int old_revision=tuning_revision;
    assert(spring_pid_command("pid lo kp 99"));assert(tuning_revision==old_revision);
    stop_all_motors();assert(!balance_active);
    leg_calibrated=secondary_calibrated=false;memset(motors,0,sizeof(motors));
    puts("PASS: balance excludes legacy/manual/pulse output and stop clears balance ownership");
    // Actual manual path plus the shared final CAN voltage limiter.
    for (unsigned int id=1; id<=8; ++id) {
        const float cap=(id==1 || id==6) ? 4.0f : 6.9f;
        assert(set_motor_voltage(id,100));
        assert(fabsf(motors[id-1].target_voltage-cap)<0.00001f);
        assert(set_motor_voltage(id,-100));
        assert(fabsf(motors[id-1].target_voltage+cap)<0.00001f);
        assert(limit_motor_voltage(id,NAN)==0);
        assert(limit_motor_voltage(id,INFINITY)==0);
        assert(limit_motor_voltage(id,-INFINITY)==0);
        assert(limit_motor_voltage(id,.3f)==.3f);
    }
    assert(!set_motor_voltage(1,NAN));
    assert(!set_motor_voltage(0,1));
    assert(!set_motor_voltage(9,1));
    assert(limit_motor_voltage(0,1)==0 && limit_motor_voltage(9,1)==0);
    stop_all_motors();
    puts("PASS: wheel 4 V / joint 6.9 V limits, both signs, invalid/nonfinite rejection");
    /* Actual outward readings: ID4 +0.390, ID5 -0.340 rad. */
    float measured[2], moved[2], voltages[2];
    assert(solve_leg(5.338f, 1.630f, 5.678f, 1.240f, measured));
    assert(measured[0] > 0.085f && measured[0] < 0.150f);
    assert(fabsf(measured[1]-(float)M_PI_2) < 5*(float)M_PI/180);
    assert(solve_leg(5.337f, 1.631f, 5.678f, 1.240f, moved));
    assert(moved[0] < measured[0]);
    leg_pid_t direction_pid;
    init_leg_pid(&direction_pid);
    assert(spring_voltage(5.338f,1.630f,5.678f,1.240f,-1,1,
        measured[0],measured[1],0,voltages,&direction_pid) == NULL);
    assert(direction_pid.spd[0] < 0);
    assert(spring_voltage(5.678f,1.240f,5.678f,1.240f,0,0,
        SPRING_TARGET_LENGTH,SPRING_TARGET_ANGLE,0,voltages,NULL) == NULL);
    assert(voltages[0] < 0 && voltages[1] > 0);
    puts("PASS: measured outward pose, shortening velocity and branch recovery direction");
    leg_pid_t ramp_pid;init_leg_pid(&ramp_pid);
    const float ramp_target=measured[1]+.2f;
    assert(!spring_voltage(5.338f,1.630f,5.678f,1.240f,0,0,
        measured[0],ramp_target,0,voltages,&ramp_pid));
    assert(fabsf(ramp_pid.angle.outer.error)<.00001f);
    for(unsigned int i=0;i<SPRING_ANGLE_RAMP_CYCLES;i++)
        assert(!spring_voltage(5.338f,1.630f,5.678f,1.240f,0,0,
            measured[0],ramp_target,0,voltages,&ramp_pid));
    assert(fabsf(ramp_pid.angle.outer.error-.2f)<.0001f);
    puts("PASS: preparation leg-angle target ramps from measured pose over 2 seconds");
    check_original_kinematics();
    check_no_friction_compensation();
    check_center_crossing();
    assert(leg_geometry_self_test());
    assert(pulse_self_test());
    assert(spring_self_test());
    stop_all_motors();
    assert(!leg_calibrated);
    start_spring(); /* Missing calibration and feedback: must reject. */
    spring_process(1000);
    for (int i=0; i<8; ++i) assert(motors[i].target_voltage == 0);
    motors[LEG_RIGHT_INDEX].angle = -0.5235988f;
    motors[LEG_LEFT_INDEX].angle = 0.5235988f;
    motors[LEG_RIGHT_INDEX].last_feedback_ms = motors[LEG_LEFT_INDEX].last_feedback_ms = 1000;
    calibrate_leg(true, 0, 0);
    assert(leg_calibrated);
    start_spring();
    spring_process(1000);
    assert(motors[LEG_RIGHT_INDEX].target_voltage > 0.02f && motors[LEG_LEFT_INDEX].target_voltage < -0.02f);
    assert(LEG_RIGHT_ID == 5 && LEG_LEFT_ID == 4);
    for (int i=0; i<8; ++i) if (i != LEG_RIGHT_INDEX && i != LEG_LEFT_INDEX)
        assert(motors[i].target_voltage == 0);
    assert(print_spring_waveform()); /* First cycle starts the angle target at measured pose. */
    const float initial_support_right = motors[LEG_RIGHT_INDEX].target_voltage;
    const float initial_support_left = motors[LEG_LEFT_INDEX].target_voltage;
    motors[LEG_RIGHT_INDEX].last_feedback_ms = motors[LEG_LEFT_INDEX].last_feedback_ms = 2500;
    /* Exercise continued control without tripping loop-late protection. */
    for (int now=1000; now<=2500; now+=4) {
        motors[LEG_RIGHT_INDEX].last_feedback_ms = motors[LEG_LEFT_INDEX].last_feedback_ms = now;
        spring_process(now);
    }
    // A persistent overlong leg must reduce extension; the user's integral
    // gains can reverse net force rather than retaining a positive load.
    assert(motors[LEG_RIGHT_INDEX].target_voltage < initial_support_right);
    assert(motors[LEG_LEFT_INDEX].target_voltage > initial_support_left);
    assert(motors[LEG_RIGHT_INDEX].target_voltage <= 6.9f && motors[LEG_LEFT_INDEX].target_voltage >= -6.9f);
    test_now_ms = 2500;
    assert(print_spring_waveform());
    float before = motors[LEG_RIGHT_INDEX].target_voltage;
    assert(spring_self_test()); /* Must leave active PID history untouched. */
    assert(motors[LEG_RIGHT_INDEX].target_voltage == before);
    spring_process(2532); /* Original controller does not trip on 32 ms feedback gaps. */
    assert(spring_active);
    stop_all_motors();
    test_now_ms = 2532;
    assert(!print_spring_waveform());
    for (int i=0; i<8; ++i) assert(motors[i].target_voltage == 0);
    stop_all_motors();
    motors[LEG_RIGHT_INDEX].last_feedback_ms = motors[LEG_LEFT_INDEX].last_feedback_ms = 0;
    arm_spring_autostart();
    spring_autostart_process(3000);
    assert(spring_autostart_pending && !spring_active);
    leg_calibrated = false;
    motors[LEG_RIGHT_INDEX].last_feedback_ms = motors[LEG_LEFT_INDEX].last_feedback_ms = 3000;
    spring_autostart_process(3000);
    spring_autostart_process(3300);
    assert(!spring_active && !leg_calibrated);
    stop_all_motors();
    assert(!spring_autostart_pending);
    motors[LEG_RIGHT_INDEX].angle = -2.466f - 0.5235988f;
    motors[LEG_LEFT_INDEX].angle = -0.147f + 0.5235988f + 2*(float)M_PI;
    test_now_ms = 3000;
    motors[LEG_RIGHT_INDEX].last_feedback_ms = motors[LEG_LEFT_INDEX].last_feedback_ms = 3000;
    calibrate_leg(true, -2.466f, -0.147f + 2*(float)M_PI);
    motors[LEG_RIGHT_INDEX].speed_rpm = motors[LEG_LEFT_INDEX].speed_rpm = 0;
    arm_spring_autostart();
    for (int now=3000; now<=3300; now+=4) {
        test_now_ms = now;
        motors[LEG_RIGHT_INDEX].last_feedback_ms = motors[LEG_LEFT_INDEX].last_feedback_ms = now;
        spring_autostart_process(now);
        if (now<3300) assert(!spring_active);
    }
    assert(spring_active && leg_calibrated && !spring_autostart_pending);
    assert(fabsf(leg_zero_right + 2.466f) < 0.00001f);
    assert(fabsf(leg_zero_left - (-0.147f + 2*(float)M_PI)) < 0.00001f);
    spring_process(3300);
    assert(motors[LEG_RIGHT_INDEX].target_voltage > 0 && motors[LEG_LEFT_INDEX].target_voltage < 0);
    spring_process(3332);
    assert(spring_active);
    stop_all_motors();
    motors[LEG_RIGHT_INDEX].last_feedback_ms = motors[LEG_LEFT_INDEX].last_feedback_ms = 3400;
    test_now_ms = 3400;
    spring_autostart_process(3400);
    assert(!spring_active && motors[LEG_RIGHT_INDEX].target_voltage == 0 && motors[LEG_LEFT_INDEX].target_voltage == 0);
    stop_all_motors();
    assert(!spring_pid_command("spring"));
    assert(spring_pid_command("pid lo kp 20"));
    assert(spring_pid_command("pid lo ki 0"));
    assert(spring_pid_command("pid lo kd 0"));
    assert(spring_pid_command("pid lo imax 0"));
    assert(spring_pid_command("pid lo max 0.5"));
    assert(spring_pid_command("pid li lpf 0.5"));
    assert(spring_pid_command("pid ao kp 6.25"));
    assert(spring_pid_command("pid ai kp 0.16"));
    start_spring();
    assert(spring_active);
    assert(spring_pid_command("pid lo kp 10")); /* Live update. */
    const char *invalid[] = {"pid lo kp nan", "pid lo kp inf", "pid lo kp -1",
        "pid lo max 0", "pid li lpf 2", "pid xx kp 99", "pid lo xx 99",
        "pid lo kp 99 trailing", "pid lo kp", "pid lo kp 10x"};
    for (size_t i=0; i<sizeof(invalid)/sizeof(invalid[0]); ++i) assert(spring_pid_command(invalid[i]));
    assert(spring_pid_command("pid"));
    test_now_ms = 3404;
    motors[LEG_RIGHT_INDEX].last_feedback_ms = motors[LEG_LEFT_INDEX].last_feedback_ms = 3404;
    spring_process(3404);
    assert(print_spring_waveform());
    float voltage_before_update = motors[LEG_RIGHT_INDEX].target_voltage;
    inflight_command = "pid lo kp 9";
    enter_count = 0;
    test_now_ms = 3408;
    spring_process(3408); /* Simulate a command between snapshot and commit. */
    assert(inflight_command == NULL && motors[LEG_RIGHT_INDEX].target_voltage == voltage_before_update);
    test_now_ms = 3412;
    motors[LEG_RIGHT_INDEX].last_feedback_ms = motors[LEG_LEFT_INDEX].last_feedback_ms = 3412;
    spring_process(3412);
    assert(print_spring_waveform());
    stop_all_motors();
    test_now_ms = 3420;
    motors[LEG_RIGHT_INDEX].last_feedback_ms = motors[LEG_LEFT_INDEX].last_feedback_ms = 3420;
    start_spring();
    spring_process(3420);
    assert(print_spring_waveform()); /* Restart preserves RAM tuning. */
    stop_all_motors();
    /* Original control continues across short feedback gaps. */
    for (int which=0; which<2; ++which) {
        test_now_ms = 6000 + which*100;
        motors[LEG_RIGHT_INDEX].last_feedback_ms = motors[LEG_LEFT_INDEX].last_feedback_ms = test_now_ms;
        start_spring();
        assert(spring_active);
        spring_process(test_now_ms);
        motors[1].last_feedback_ms = motors[2].last_feedback_ms = test_now_ms + 32;
        motors[which == 0 ? LEG_RIGHT_INDEX : LEG_LEFT_INDEX].last_feedback_ms = test_now_ms + 32;
        spring_process(test_now_ms + 32);
        assert(spring_active);
        stop_all_motors();
        for (int i=0; i<8; ++i) assert(motors[i].target_voltage == 0);
    }
    test_now_ms = 6300;
    motors[LEG_RIGHT_INDEX].last_feedback_ms = motors[LEG_LEFT_INDEX].last_feedback_ms = test_now_ms;
    assert(!start_pulse(2, 0.2f, 20) && !start_pulse(3, 0.2f, 20));
    assert(start_pulse(4, 0.2f, 20));
    assert(motors[3].target_voltage == 0.2f && motors[4].target_voltage == 0);
    pulse_process(test_now_ms + 20);
    assert(!pulse_active && motors[3].target_voltage == 0);
    stop_all_motors();
    /* Wheel tests reuse the pulse watchdog, but must monitor ID1/6, not legs. */
    motors[0].angle = -7; motors[5].angle = 5;
    motors[0].speed_rpm = motors[5].speed_rpm = 0;
    motors[0].last_feedback_ms = motors[5].last_feedback_ms = test_now_ms;
    assert(!start_pulse(6,0.51f,80) && !start_pulse(1,-0.3f,101));
    assert(start_pulse(6,0.3f,80));
    assert(motors[5].target_voltage == 0.3f);
    assert(!start_pulse(1,-0.3f,80));
    motors[0].last_feedback_ms = motors[5].last_feedback_ms = test_now_ms+80;
    pulse_process(test_now_ms+80);
    assert(!pulse_active && motors[5].target_voltage == 0);
    test_now_ms += 100;
    motors[0].last_feedback_ms = motors[5].last_feedback_ms = test_now_ms;
    assert(start_pulse(1,-0.3f,80));
    assert(motors[0].target_voltage == -0.3f);
    motors[5].last_feedback_ms = test_now_ms+31;
    pulse_process(test_now_ms+31); /* Stale ID1 must stop both wheels. */
    assert(!pulse_active);
    for (int i=0;i<8;i++) assert(motors[i].target_voltage==0);
    motors[0].last_feedback_ms = motors[5].last_feedback_ms = test_now_ms;
    assert(start_pulse(6,0.3f,80));
    motors[5].angle += PULSE_TRAVEL_LIMIT_RAD + .01f;
    pulse_process(test_now_ms+1);
    assert(!pulse_active && motors[5].target_voltage == 0);
    assert(start_pulse(1,-0.3f,80));
    motors[0].speed_rpm = 81;
    pulse_process(test_now_ms+1);
    assert(!pulse_active && motors[0].target_voltage == 0);
    motors[0].speed_rpm = 0;
    assert(start_pulse(6,0.3f,80));
    stop_all_motors();
    assert(!pulse_active && motors[5].target_voltage == 0);
    assert(pulse_should_stop(100,200,99,99,NAN,0));
    puts("PASS: wheel pulse ID1/6 routing, limits, deadline, pair feedback, travel, speed and stop");
    leg_calibrated = false;
    motors[LEG_RIGHT_INDEX].angle = LEG_RIGHT_ZERO - 0.6f - 2*(float)M_PI;
    motors[LEG_LEFT_INDEX].angle = LEG_LEFT_ZERO + 0.6f + 2*(float)M_PI;
    motors[LEG_RIGHT_INDEX].speed_rpm = motors[LEG_LEFT_INDEX].speed_rpm = 0;
    arm_spring_autostart();
    for (int now=7000; now<=7300; now+=4) {
        test_now_ms = now;
        motors[LEG_RIGHT_INDEX].last_feedback_ms = motors[LEG_LEFT_INDEX].last_feedback_ms = now;
        spring_autostart_process(now);
        if (now<7300) assert(!spring_active);
    }
    assert(spring_active && leg_calibrated && !spring_autostart_pending);
    assert(fabsf(leg_zero_right-(LEG_RIGHT_ZERO-2*(float)M_PI))<0.00001f);
    assert(fabsf(leg_zero_left-(LEG_LEFT_ZERO+2*(float)M_PI))<0.00001f);
    spring_process(7300);
    for (int i=0; i<8; ++i) if (i!=LEG_RIGHT_INDEX && i!=LEG_LEFT_INDEX)
        assert(motors[i].target_voltage == 0);
    stop_all_motors();
    /* Both legs share length feedback with opposite encoder conventions. */
    leg_calibrated = secondary_calibrated = false;
    motors[LEG_RIGHT_INDEX].angle = LEG_RIGHT_ZERO - 0.6f;
    motors[LEG_LEFT_INDEX].angle = LEG_LEFT_ZERO + 0.6f;
    motors[1].angle = LEG23_RIGHT_ZERO + 0.6f;
    motors[2].angle = LEG23_LEFT_ZERO - 0.6f;
    motors[1].speed_rpm = motors[2].speed_rpm = 0;
    arm_spring_autostart();
    for (int now=8000; now<=8300; now+=4) {
        test_now_ms = now;
        for (int i=1; i<=4; ++i) motors[i].last_feedback_ms = now;
        spring_autostart_process(now);
        secondary_autostart_process(now);
    }
    assert(spring_active && secondary_active && secondary_calibrated);
    assert(spring_pid_command("pid lo ki 0"));
    assert(spring_pid_command("pid li ki 0"));
    assert(spring_pid_command("pid li kd 0"));
    assert(spring_pid_command("pid li lpf 1"));
    spring_process(8300);
    secondary_process(8300);
    float primary_before = motors[LEG_RIGHT_INDEX].target_voltage;
    float secondary_before = motors[1].target_voltage;
    motors[LEG_RIGHT_INDEX].angle += 0.03f;
    test_now_ms = 8304;
    for (int i=1; i<=4; ++i) motors[i].last_feedback_ms = test_now_ms;
    spring_process(8304);
    secondary_process(8304);
    assert(fabsf(motors[LEG_RIGHT_INDEX].target_voltage-primary_before) > 0.001f);
    assert(fabsf(motors[1].target_voltage-secondary_before) > 0.001f);
    assert(spring_pid_command("pid lo kp 12"));
    test_now_ms = 8308;
    spring_process(8308);
    secondary_process(8308);
    assert(fabsf(motors[1].target_voltage-secondary_before) > 0.001f);
    test_now_ms = 8336;
    motors[LEG_RIGHT_INDEX].last_feedback_ms = motors[LEG_LEFT_INDEX].last_feedback_ms = 8336;
    motors[1].last_feedback_ms = 8336;
    secondary_process(8336); /* No custom 30 ms stop. */
    assert(spring_active && secondary_active);
    stop_all_motors();
    for (int i=0; i<8; ++i) assert(motors[i].target_voltage == 0);
    for (int i=1; i<=4; ++i) motors[i].last_feedback_ms = 9000;
    test_now_ms = 9000;
    spring_autostart_process(9000);
    secondary_autostart_process(9000);
    assert(!spring_active && !secondary_active);
    arm_spring_autostart();
    for (int now=10000; now<=10300; now+=4) {
        test_now_ms = now;
        for (int i=1; i<=4; ++i) motors[i].last_feedback_ms = now;
        spring_autostart_process(now);
        secondary_autostart_process(now);
    }
    assert(spring_active && secondary_active);
    spring_process(10300);
    secondary_process(10300);
    test_now_ms = 10332;
    motors[1].last_feedback_ms = motors[2].last_feedback_ms = 10332;
    spring_process(10332); /* No custom 30 ms stop. */
    assert(spring_active && secondary_active);
    stop_all_motors();
    for (int i=0; i<8; ++i) assert(motors[i].target_voltage == 0);
    stop_all_motors();
    test_now_ms = 11000;
    motors[1].angle = -2.4f;
    motors[2].angle = -1.5f;
    motors[1].last_feedback_ms = motors[2].last_feedback_ms = 11000;
    motors[1].speed_rpm = 4;
    assert(!secondary_calibrate());
    motors[1].speed_rpm = 0;
    assert(secondary_calibrate() && secondary_calibrated && !secondary_active);
    assert(!secondary_restore_reference(NAN, -1.5f));
    assert(!secondary_restore_reference(-10, -1.5f));
    assert(secondary_restore_reference(-2.41f, -1.51f));
    float secondary_reference[2];
    secondary_reference_locked(secondary_reference);
    assert(fabsf(secondary_reference[0]+2.41f)<0.00001f && fabsf(secondary_reference[1]+1.51f)<0.00001f);
    assert(!secondary_active);
    secondary_print();
    for (int i=0; i<8; ++i) assert(motors[i].target_voltage == 0);
    puts("PASS: ID2/ID3 stationary-only recalibration without enabling motion");
    puts("PASS: dual-leg startup, shared mean-length response, separate angle PID and explicit common stop");
    puts("PASS: ID4/ID5 measured reference autostart across full-turn representations");
    puts("PASS: ID4/ID5 routing, short feedback gaps tolerated, zero outputs on other motors and pulse selection");
    puts("PASS: original startup has no added near-vertical pose gate");
    puts("PASS: project PID, back-EMF units/sign, VMC restoration/braking, zero-output rejection");
}
''')
    executable = root / 'test.exe'
    subprocess.run(['gcc', '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                    '-I'+str(root), '-I'+str(local_source),
                    str(root / 'test.c'),
                    *[str(local_source / name) for name in ('leg_control.c', 'leg_calibration.c',
                        'leg_pulse.c', 'leg_spring.c', 'leg_algorithm.c', 'leg_self_test.c', 'leg_secondary.c', 'motor_output.c')],
                    str(local_source / 'PID.c'), str(local_source / 'leg_pos.c'),
                    str(local_source / 'leg_spd.c'), str(local_source / 'leg_conv.c'),
                    str(root / 'leg_spd_original.c'), str(root / 'leg_conv_original.c'),
                    '-lm', '-o', str(executable)], check=True)
    result = subprocess.run([str(executable)], capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError(result.stdout + result.stderr)
    print(result.stdout, end='')
    frames = [line for line in result.stdout.splitlines() if line.startswith('leg:')]
    assert len(frames) == 5
    for frame, kp in zip(frames[2:], (10, 9, 9)):
        tuned = [float(value) for value in frame.split(':', 1)[1].split(',')]
        assert abs(tuned[2] - kp * (tuned[0]-tuned[1])/1000) < 0.00002
    first_channels = [float(value) for value in frames[0].split(':', 1)[1].split(',')]
    assert abs(first_channels[0] - 128.2) < 0.001 and abs(first_channels[4] - 90) < 0.001
    channels = [float(value) for value in frames[0].split(':', 1)[1].split(',')]
    assert len(channels) == 12 and all(math.isfinite(value) for value in channels)
    assert abs(channels[0] - 128.2) < 0.001 and abs(channels[4] - 90) < 0.001
    nominal_mm = (0.05*math.cos(math.pi/6) + math.sqrt(0.105**2-0.055**2))*1000
    assert abs(channels[1] - nominal_mm) < 0.01 and abs(channels[5] - 90) < 0.01
    assert channels[3] == channels[7] == channels[8] == channels[10] == 0
    assert 0 < channels[9] <= 6.9 and -6.9 <= channels[11] < 0
    print('PASS: FireWater 12-channel order, units, 6.9 V final voltage limits and stop suppression')
