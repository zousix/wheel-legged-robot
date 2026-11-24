"""Compile the complete balance chain; verify original equations, mirrored work and mode ownership."""
from pathlib import Path
import subprocess
import sys
import tempfile

source = Path(__file__).resolve().parents[1] / 'main'
with tempfile.TemporaryDirectory(prefix='balance-control-') as directory:
    root = Path(directory)
    (root / 'freertos').mkdir()
    (root / 'freertos/FreeRTOS.h').write_text('''#pragma once
typedef int portMUX_TYPE;
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
''')
    (root / 'esp_err.h').write_text('#pragma once\ntypedef int esp_err_t;\n')
    (root / 'esp_timer.h').write_text('#pragma once\n#include <stdint.h>\nint64_t esp_timer_get_time(void);\n')
    (root / 'test.c').write_text(r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "balance_control.h"
#include "lqr_k.h"
#include "leg_control_internal.h"
motor_state_t motors[8];portMUX_TYPE motor_mux;
bool leg_calibrated,secondary_calibrated,spring_active,secondary_active,pulse_active,balance_active,spring_autostart_pending;
float leg_zero_right,leg_zero_left;
const char *spring_result,*pulse_result;
static robot_state_input_t fixture;
static int cancel_on_read;
static int rate_on_read;
static bool simulate_handoff;
static CascadePID handoff_length[2];
int64_t esp_timer_get_time(void) { return fixture.now_us; }
void imu_get_state(imu_state_t *s) {
    *s=fixture.imu;
    if(rate_on_read) { rate_on_read=0;assert(balance_rate_command("balance rate 0.7")); }
    if(cancel_on_read) { balance_active=false;for(int i=0;i<8;i++)motors[i].target_voltage=0;cancel_on_read=0; }
}
void wheel_get_state(wheel_odometry_t *s) { *s=fixture.wheels; }
void secondary_reference_locked(float r[2]) { r[0]=fixture.reference[0];r[1]=fixture.reference[1]; }
void secondary_load_reference_locked(float r,float l) { fixture.reference[0]=r;fixture.reference[1]=l;secondary_calibrated=true; }
void secondary_stop_locked(const char *reason) { (void)reason;secondary_active=false;motors[1].target_voltage=motors[2].target_voltage=0; }
void copy_tuning_locked(leg_pid_t *p) { init_leg_pid(p); }
bool spring_balance_handoff_locked(CascadePID p[2]) {
    if (!simulate_handoff) return false;
    p[0]=handoff_length[0];p[1]=handoff_length[1];return true;
}
static void near(float a,float b,float tol) { assert(fabsf(a-b)<tol); }
static robot_state_input_t upright(void) {
    robot_state_input_t in={0};
    in.now_us=2000000;in.imu.phase=IMU_READY;in.imu.roll_deg=180;
    in.imu.sensor.acceleration_g[2]=-1;
    in.calibrated[0]=in.calibrated[1]=true;
    in.wheels.last_feedback_ms[0]=in.wheels.last_feedback_ms[1]=1999;
    in.reference[0]=LEG23_RIGHT_ZERO;in.reference[1]=LEG23_LEFT_ZERO;
    in.reference[2]=LEG_RIGHT_ZERO;in.reference[3]=LEG_LEFT_ZERO;
    float lo=0,hi=1,pos[2];
    for(int i=0;i<40;i++) { float p=(lo+hi)/2;assert(solve_leg(-p,p,0,0,pos));if(pos[0]>SPRING_TARGET_LENGTH)lo=p;else hi=p; }
    const float fold=(lo+hi)/2;
    for(int i=0;i<4;i++)in.joints[i].last_feedback_ms=1999;
    in.joints[0].angle=in.reference[0]+fold;in.joints[1].angle=in.reference[1]-fold;
    in.joints[2].angle=in.reference[2]-fold;in.joints[3].angle=in.reference[3]+fold;
    return in;
}
static void init(balance_core_t *core,robot_state_input_t *in) {
    leg_pid_t pid;init_leg_pid(&pid);assert(balance_core_init(core,in,&pid.length));
}
static void check_virtual_work(const robot_state_input_t *in,const balance_output_t *out) {
    const int ids[2][2]={{2,3},{5,4}};
    for(int side=0;side<2;side++)for(int j=0;j<2;j++) {
        float plus[2],minus[2];const float d=side==0?-1:1;
        float r=in->joints[2*side].angle,l=in->joints[2*side+1].angle;
        assert(solve_leg_directed(d,r+(j==0?.001f:0),l+(j==1?.001f:0),in->reference[2*side],in->reference[2*side+1],plus));
        assert(solve_leg_directed(d,r-(j==0?.001f:0),l-(j==1?.001f:0),in->reference[2*side],in->reference[2*side+1],minus));
        const float dl=(plus[0]-minus[0])/.002f,da=(plus[1]-minus[1])/.002f;
        const float local_hip=(side==0?1:-1)*out->hip[side];
        near(out->torque[ids[side][j]-1],out->force[side]*dl+local_hip*da,.001f);
    }
}
static void check_support_handoff(void) {
    fixture=upright();
    // Distinct histories must survive the real mode transition without averaging.
    fixture.imu.body_roll_deg=10.12f;fixture.imu.roll_deg=188.414f;
    leg_pid_t tuning;init_leg_pid(&tuning);
    handoff_length[0]=handoff_length[1]=tuning.length;
    handoff_length[0].outer.integral=.01f;
    handoff_length[1].outer.integral=-.005f;
    handoff_length[0].inner.integral=1.9296f;
    handoff_length[1].inner.integral=.4f;
    balance_core_t expected;balance_output_t actual,full_output;
    assert(balance_core_init(&expected,&fixture,&tuning.length));
    expected.support_length[0]=handoff_length[0];expected.support_length[1]=handoff_length[1];
    expected.support_handoff_start_us=fixture.now_us;
    leg_calibrated=secondary_calibrated=true;
    leg_zero_right=fixture.reference[2];leg_zero_left=fixture.reference[3];
    for(int i=0;i<8;i++)motors[i]=(motor_state_t){0};
    motors[1]=fixture.joints[0];motors[2]=fixture.joints[1];
    motors[4]=fixture.joints[2];motors[3]=fixture.joints[3];
    motors[0].last_feedback_ms=motors[5].last_feedback_ms=1999;
    for(int i=1;i<5;i++)motors[i].target_voltage=.1f;
    spring_active=secondary_active=simulate_handoff=true;
    balance_active=pulse_active=false;
    start_balance();assert(balance_active);simulate_handoff=false;
    const int64_t start=fixture.now_us;
    const int64_t offsets[]={0,4000,12000,124000,250000,400000,500000,800000};
    float last_blend=-1;
    for(unsigned int i=0;i<sizeof(offsets)/sizeof(offsets[0]);i++) {
        fixture.now_us=start+offsets[i];fixture.imu.sample_time_us=fixture.now_us-1000;
        balance_core_t full=expected;full.support_handoff_start_us=0;
        assert(balance_core_step(&full,&fixture,&full_output));
        assert(balance_core_step(&expected,&fixture,&actual));
        assert(actual.support_blend>=last_blend && actual.support_blend<=1);
        last_blend=actual.support_blend;
        near(actual.length_integral,full_output.length_integral,.000001f);
        assert(expected.support_length[0].inner.integral>1.9f && actual.feedforward==4);
        assert(expected.support_length[1].inner.integral<.5f);
        for(int wheel=0;wheel<=5;wheel+=5)near(actual.voltage[wheel],full_output.voltage[wheel],.000001f);
        for(int side=0;side<2;side++)near(actual.hip[side],full_output.hip[side],.000001f);
        if(offsets[i]==0) {
            near(actual.support_blend,1,.000001f);
            const float soft=(SPRING_TARGET_LENGTH-.12f)*100;
            near(actual.force[0],expected.support_length[0].output+4-actual.roll_force-soft,.0001f);
            near(actual.force[1],expected.support_length[1].output+4+actual.roll_force-soft,.0001f);
            assert(fabsf(actual.roll_force)>1.5f);
        }
        if(offsets[i]==250000)near(actual.support_blend,1,.000001f);
        if(offsets[i]>=BALANCE_SUPPORT_HANDOFF_US) {
            near(actual.support_blend,1,.000001f);
            for(int j=0;j<6;j++)near(actual.voltage[j],full_output.voltage[j],.000001f);
        }
        check_virtual_work(&fixture,&actual);
        balance_process(fixture.now_us/1000);
        for(int j=0;j<6;j++)near(motors[j].target_voltage,actual.voltage[j],.0002f);
    }
    // A second handoff retains each leg's history again.
    balance_active=false;spring_active=secondary_active=simulate_handoff=true;
    for(int i=0;i<8;i++)motors[i].target_voltage=0;
    for(int i=0;i<8;i++)motors[i].last_feedback_ms=fixture.now_us/1000-1;
    fixture.wheels.last_feedback_ms[0]=fixture.wheels.last_feedback_ms[1]=fixture.now_us/1000-1;
    start_balance();assert(balance_active);simulate_handoff=false;
    assert(balance_core_init(&expected,&fixture,&tuning.length));
    expected.support_handoff_start_us=fixture.now_us;
    expected.support_length[0]=handoff_length[0];expected.support_length[1]=handoff_length[1];
    assert(balance_core_step(&expected,&fixture,&actual));
    near(actual.support_blend,1,.000001f);
    balance_process(fixture.now_us/1000);
    for(int j=0;j<6;j++)near(motors[j].target_voltage,actual.voltage[j],.0002f);
    print_balance();
    balance_active=spring_active=secondary_active=false;
    for(int i=0;i<8;i++)motors[i].target_voltage=0;
    puts("PASS: independent PID histories survive mode transition and restart, with roll/4 N support and mirrored VMC intact");
}
int main(void) {
    {
        robot_state_input_t in=upright();in.wheels.speed_mps_avg=.1f;
        balance_core_t c,baseline;balance_output_t o,b;
        init(&baseline,&in);assert(baseline.wheel_speed_ratio==1);
        assert(balance_core_step(&baseline,&in,&b));
        for(int trial=0;trial<3;++trial) {
            init(&c,&in);c.wheel_speed_ratio=.9f+.05f*trial;
            assert(balance_core_step(&c,&in,&o));
            near(o.model_torque,b.model_torque*c.wheel_speed_ratio,.000001f);
            near(o.model_hip_torque,b.model_hip_torque,.000001f);
            for(int side=0;side<2;++side) near(o.force[side],b.force[side],.000001f);
        }
        assert(balance_rate_command("balance speed 0.95"));
        assert(balance_rate_command("balance speed nan"));
        assert(balance_rate_command("balance speed -1"));
        assert(balance_rate_command("balance speed"));
        assert(balance_rate_command("balance speed 1.0"));
        puts("PASS: speed tuning changes wheel speed feedback only, preserves hip/support, rejects nonfinite/negative gains");
    }
    check_support_handoff();
    {
        robot_state_input_t s=upright();balance_core_t c;balance_output_t o;
        init(&c,&s);assert(balance_core_step(&c,&s,&o));
        CascadePID right=c.support_length[1];
        s.joints[0].angle+=.05f;s.joints[1].angle-=.05f;
        assert(balance_core_step(&c,&s,&o));
        assert(c.support_length[0].output>0);
        near(c.support_length[1].inner.integral,right.inner.integral,.0001f);
        near(c.support_length[1].outer.integral,right.outer.integral,.0001f);
        near(c.support_length[1].output,right.output,.0001f);
        assert(o.force[0]>o.force[1]);
        check_virtual_work(&s,&o);
        puts("PASS: normal balance left-only compression preserves right PID history and support command");
    }
    {
        robot_state_input_t s=upright();balance_core_t c;balance_output_t o;
        s.imu.sample_time_us=s.now_us;
        init(&c,&s);c.support_test=true;c.support_feedforward=4;c.support_deadline_ms=3500;
        leg_pid_t angle_tuning;init_leg_pid(&angle_tuning);
        c.support_angle[0]=c.support_angle[1]=angle_tuning.angle;
        assert(support_test_step(&c,&s,&o));
        near(o.force[0],4-(SPRING_TARGET_LENGTH-.12f)*100,.0001f);
        near(o.force[0],o.force[1],.0001f);
        assert(o.voltage[0]==0 && o.voltage[5]==0 && o.model_torque==0 && o.model_hip_torque==0);
        assert(c.roll.output==0 && c.yaw.output==0 && c.leg_difference.output==0);
        check_virtual_work(&s,&o);
        // Compress only the left leg: its PID must react without changing the right force.
        const float right_force=o.force[1];
        s.joints[0].angle+=.05f;s.joints[1].angle-=.05f;
        assert(support_test_step(&c,&s,&o));
        assert(o.state.leg_length_m[0]<SPRING_TARGET_LENGTH);
        assert(c.support_length[0].outer.error>0 && o.force[0]>right_force);
        near(o.force[1],right_force,.0001f);
        near(c.support_length[1].inner.integral,0,.0001f);
        assert(o.voltage[0]==0 && o.voltage[5]==0);
        check_virtual_work(&s,&o);
        puts("PASS: left-only compression changes only left length PID; right force/history stay unchanged");
        // With one short leg and one long leg, the corrections must oppose each other.
        s.joints[2].angle+=.05f;s.joints[3].angle-=.05f;
        assert(support_test_step(&c,&s,&o));
        assert(o.state.leg_length_m[1]>SPRING_TARGET_LENGTH);
        assert(c.support_length[0].output>0 && c.support_length[1].output<0);
        near(c.length.output,0,.0001f);
        check_virtual_work(&s,&o);
        puts("PASS: short/long legs get opposite length corrections; normal balance PID history untouched");
        for(int tick=0;tick<10000;tick++)assert(support_test_step(&c,&s,&o));
        near(c.support_length[0].inner.integral,30,.0001f);
        near(c.support_length[1].inner.integral,-30,.0001f);
        near(c.length.inner.maxIntegral,30,.0001f);
        near(c.length.outer.maxIntegral,30,.0001f);
        assert(fabsf(c.support_length[0].outer.output)<=1 && fabsf(c.support_length[0].output)<=10);
        near(c.length.outer.kp,15,.0001f);near(c.length.outer.ki,.045f,.0001f);
        near(c.length.inner.kp,20,.0001f);near(c.length.inner.ki,.1f,.0001f);
        assert(o.voltage[0]==0 && o.voltage[5]==0);
        for(int i=1;i<5;i++)assert(isfinite(o.voltage[i]) && fabsf(o.voltage[i])<=6.9f);
        puts("PASS: user length tuning 15/.05/0 and 20/.1/30, integral +/-30, speed/force 1/10 and voltage limits hold");
        // A nonzero local leg angle must receive restoring local torque.
        s.joints[0].angle=-2.431f;s.joints[1].angle=-2.529f;
        assert(support_test_step(&c,&s,&o));
        assert(o.state.leg_angle_rad[0]>(float)M_PI_2 && c.support_angle[0].output<0);
        assert(o.voltage[0]==0 && o.voltage[5]==0 && o.model_hip_torque==0);
        check_virtual_work(&s,&o);
        s.now_us=3500000;assert(!support_test_step(&c,&s,&o));
        for(int i=0;i<6;i++)assert(o.voltage[i]==0);
        s=upright();s.imu.sample_time_us=s.now_us;init(&c,&s);
        c.support_test=true;c.support_feedforward=4;c.support_deadline_ms=3500;
        s.joints[0].last_feedback_ms=1970;assert(!support_test_step(&c,&s,&o));
        s.joints[0].last_feedback_ms=1999;s.imu.body_roll_deg=16;
        assert(!support_test_step(&c,&s,&o));
        puts("PASS: static test isolates length PID/force, wheels/LQR/roll stay zero, deadline and stale/pose faults zero outputs");
    }
    robot_state_input_t in=upright();balance_core_t core;balance_output_t out;
    init(&core,&in);assert(balance_core_step(&core,&in,&out));
    assert(out.grounded);
    near(out.model_torque,0,.00001f);near(out.model_hip_torque,0,.00001f);
    near(out.torque[0],0,.00001f);near(out.torque[5],0,.00001f);
    near(out.force[0],SPRING_LOAD_FEEDFORWARD-(SPRING_TARGET_LENGTH-.12f)*100,.0001f);
    check_virtual_work(&in,&out);
    puts("PASS: original length/soft-extension/gravity/contact chain and both mirrored VMC virtual-work identities");
    in=upright();init(&core,&in);in.imu.body_pitch_deg=5;
    assert(balance_core_step(&core,&in,&out));
    near(out.lqr_output_ratio,.3f,.00001f);
    near(out.model_torque,-0.1231481915f*.3f,.00001f);near(out.model_hip_torque,-0.2179436657f*.3f,.00002f);
    near(out.lqr_error[0],5*(float)M_PI/180,.00001f);
    near(out.lqr_error[4],-5*(float)M_PI/180,.00001f);
    // Record the referenced position error, rather than a later standalone STATE snapshot.
    const float torque_before_position=out.model_torque, hip_before_position=out.model_hip_torque;
    float position_gain[12];lqr_k((out.state.leg_length_m[0]+out.state.leg_length_m[1])*.5f,position_gain);
    core.position_reference=.02f;
    assert(balance_core_step(&core,&in,&out));near(out.lqr_error[2],-.02f,.00001f);
    near(out.model_torque-torque_before_position,-.02f*position_gain[4]*.3f*.8f,.000001f);
    near(out.model_hip_torque-hip_before_position,-.02f*position_gain[5]*.3f,.000001f);
    assert(BALANCE_WHEEL_POSITION_RATIO==.8f);
    puts("PASS: B scales only wheel position contribution by 0.8; hip position term unchanged");
    core.position_reference=0;
    assert(balance_core_step(&core,&in,&out));
    near(out.torque[0],0.1231481915f*.3f,.00001f);near(out.torque[5],-0.1231481915f*.3f,.00001f);
    near(out.voltage[0],motor_torque_voltage(1,out.torque[0],0),.00001f);
    near(out.voltage[5],motor_torque_voltage(6,out.torque[5],0),.00001f);
    assert(fabsf(out.voltage[0])<4 && fabsf(out.voltage[5])<4);
    check_virtual_work(&in,&out);
    puts("PASS: MATLAB tilt golden values, original wheel +/-yaw mapping, no invented torque caps, final voltage limits");
    // Mixed joint/body motion: only the combined wheel body-rate term changes.
    const float rates[]={.5f,.75f,.8f,1.0f};
    for(int rate=0;rate<4;rate++)for(int trial=0;trial<3;trial++) {
        in=upright();init(&core,&in);
        core.wheel_body_rate_ratio=rates[rate];
        in.imu.corrected_gyro_dps[1]=(trial-1)*30;
        in.joints[0].speed_rpm=2;in.joints[1].speed_rpm=-1;
        assert(balance_core_step(&core,&in,&out));
        float gain[12], expected_t=0, expected_tp=0;
        lqr_k((out.state.leg_length_m[0]+out.state.leg_length_m[1])*.5f,gain);
        for(int i=0;i<6;i++) {
            expected_t+=.3f*gain[i*2]*out.lqr_error[i]*(i==2 ? .8f : 1);
            expected_tp+=.3f*gain[i*2+1]*out.lqr_error[i];
        }
        expected_t+=.3f*(rates[rate]-1)*(gain[10]-gain[2])*out.lqr_error[5];
        near(out.model_torque,expected_t,.000001f);
        near(out.model_hip_torque,expected_tp,.000001f);
        near(out.wheel_body_rate_ratio,rates[rate],.000001f);
    }
    puts("PASS: live wheel body-rate 0.5/0.75/0.8/1.0; pure joint-rate and all hip terms unchanged");
    in=upright();init(&core,&in);in.imu.yaw_deg=5;
    assert(balance_core_step(&core,&in,&out));assert(out.torque[5]>0 && out.torque[0]>0);
    in=upright();init(&core,&in);in.imu.body_roll_deg=5;in.imu.roll_deg=185;
    in.imu.sensor.acceleration_g[1]=sinf(185*(float)M_PI/180);in.imu.sensor.acceleration_g[2]=cosf(185*(float)M_PI/180);
    assert(balance_core_step(&core,&in,&out));assert(out.force[0]>out.force[1]);
    assert(core.roll.output<0);
    near(out.force[0]-out.force[1],-2*core.roll.output,.00001f);
    check_virtual_work(&in,&out);
    in=upright();init(&core,&in);in.imu.body_roll_deg=-5;in.imu.roll_deg=175;
    in.imu.sensor.acceleration_g[1]=sinf(175*(float)M_PI/180);in.imu.sensor.acceleration_g[2]=cosf(175*(float)M_PI/180);
    assert(balance_core_step(&core,&in,&out));assert(out.force[1]>out.force[0]);
    assert(core.roll.output>0);check_virtual_work(&in,&out);
    puts("PASS: left lean increases ID2/3 support, right lean increases ID5/4 support; standing VMC routes both pairs");
    // A left-only forward swing must pull that leg back and bring the right leg forward.
    in=upright();in.joints[0].angle=-2.431f;in.joints[1].angle=-2.529f;
    init(&core,&in);assert(balance_core_step(&core,&in,&out));
    assert(out.state.s[0]<0 && core.leg_difference.output>0);
    assert(out.hip[0]<out.model_hip_torque && out.hip[1]>out.model_hip_torque);
    check_virtual_work(&in,&out);
    puts("PASS: measured ID2/3 forward swing has hip-behind theta and restoring leg synchronization torques");
    // Reproduce the recorded startup without claiming measured ground force.
    in=upright();init(&core,&in);
    in.imu.body_roll_deg=4.4f;
    in.imu.roll_deg=182.694f;
    in.imu.body_pitch_deg=-.44f;
    in.imu.pitch_deg=-.13f;
    in.imu.sensor.acceleration_g[0]=.0024f;
    in.imu.sensor.acceleration_g[1]=-.0486f;
    in.imu.sensor.acceleration_g[2]=-1.0016f;
    in.joints[0].angle=-1.975f;in.joints[1].angle=-2.370f;
    in.joints[2].angle=5.019f;in.joints[3].angle=1.981f;
    assert(balance_core_step(&core,&in,&out));assert(out.grounded);
    printf("REPLAY startup: support ID23/ID54=%.3f/%.3f N force=%.3f/%.3f N rollPID=%.3f N threshold=3 N; estimated, not measured\n",
           out.support[0],out.support[1],out.force[0],out.force[1],core.roll.output);
    in=upright();init(&core,&in);in.imu.body_roll_deg=4.4f;
    in.imu.roll_deg=184.4f;
    in.imu.sensor.acceleration_g[1]=sinf(in.imu.roll_deg*(float)M_PI/180);
    in.imu.sensor.acceleration_g[2]=cosf(in.imu.roll_deg*(float)M_PI/180);
    for(int i=0;i<500;i++) {
        in.now_us+=4000;assert(balance_core_step(&core,&in,&out));
    }
    assert(out.support[1]<3 && !out.grounded);
    near(core.roll.output,-20*4.4f*(float)M_PI/180,.001f);
    printf("REPLAY static 4.4 deg roll at target: support ID23/ID54=%.3f/%.3f N; low contact estimate remains diagnostic; standing support stays enabled\n",out.support[0],out.support[1]);
    in=upright();init(&core,&in);in.imu.body_pitch_deg=5;
    assert(balance_core_step(&core,&in,&out));
    near(out.torque[0],.1231481915f*.3f,.00001f);near(out.torque[5],-.1231481915f*.3f,.00001f);
    near(out.model_hip_torque,-.2179436657f*.3f,.00002f);
    assert(out.feedforward==4 && strcmp(out.reason,"standing-control")==0);
    in=upright();
    float lo=0,hi=1.5f,pos[2];
    for(int i=0;i<40;i++) {
        float fold=(lo+hi)/2;assert(solve_leg(-fold,fold,0,0,pos));
        if(pos[0]>.09f)lo=fold;else hi=fold;
    }
    for(int side=0;side<2;side++) {
        float dir=side==0?-1:1;
        in.joints[2*side].angle=in.reference[2*side]-dir*(lo+hi)/2;
        in.joints[2*side+1].angle=in.reference[2*side+1]+dir*(lo+hi)/2;
    }
    init(&core,&in);in.wheels.position_m_avg=.2f;
    assert(balance_core_step(&core,&in,&out));assert(out.grounded);near(core.position_reference,0,.000001f);
    puts("PASS: standing gains/support remain active; contact estimate cannot reset position reference");
    // Regression for the captured false-airborne transition: old code changed
    // 128.2 to 120 mm and removed 4 N on the very next control step.
    for(int load=-2;load<=0;load+=2) {
        in=upright();in.imu.body_roll_deg=4.8f;in.imu.roll_deg=184.8f;
        in.imu.sensor.acceleration_g[1]=sinf(in.imu.roll_deg*(float)M_PI/180);
        in.imu.sensor.acceleration_g[2]=cosf(in.imu.roll_deg*(float)M_PI/180);
        init(&core,&in);
        core.support_length[0].inner.integral=core.support_length[1].inner.integral=load;
        const float ref=core.position_reference;
        for(int tick=0;tick<500;tick++) {
            in.now_us+=4000;
            assert(balance_core_step(&core,&in,&out));
            assert(!out.grounded);
            near(core.support_length[0].outer.error,0,.00001f);
            near(core.support_length[1].outer.error,0,.00001f);
            near(out.length_force,(float)load,.0001f);
            near(out.feedforward,4,.00001f);
            near(out.force[0],out.length_force+4-core.roll.output-(SPRING_TARGET_LENGTH-.12f)*100,.0001f);
            near(out.force[1],out.length_force+4+core.roll.output-(SPRING_TARGET_LENGTH-.12f)*100,.0001f);
            near(core.position_reference,ref,.00001f);
            assert(strcmp(out.reason,"standing-control")==0);
        }
    }
    // Vary the diagnostic inertial correction to cross the contact threshold:
    // it must not change any actuator command or reset the standing reference.
    in=upright();in.imu.body_pitch_deg=2;
    balance_core_t ca,cb;balance_output_t oa,ob;init(&ca,&in);cb=ca;
    assert(balance_core_step(&ca,&in,&oa));
    in.imu.sensor.acceleration_g[2]=1; // Synthetic estimator-only threshold crossing.
    assert(balance_core_step(&cb,&in,&ob));
    assert(oa.grounded != ob.grounded);
    for(int i=0;i<6;i++)near(oa.voltage[i],ob.voltage[i],.00001f);
    near(ca.length.outer.integral,cb.length.outer.integral,.00001f);
    near(ca.length.inner.integral,cb.length.inner.integral,.00001f);
    puts("PASS: low/negative contact estimate never retracts to 120 mm, removes feedforward, switches gains, or reanchors position; inertial contact estimate cannot change outputs");
    in=upright();init(&core,&in);
    const float original_reference=core.position_reference;
    for(int sign=-1;sign<=1;sign+=2) {
        in.imu.body_pitch_deg=sign*50;
        assert(balance_core_step(&core,&in,&out));
        assert(strcmp(out.reason,"standing-control")==0);
        assert(fabsf(out.voltage[0])>0 && fabsf(out.voltage[5])>0);
        for(int i=0;i<6;i++)assert(isfinite(out.voltage[i]) && fabsf(out.voltage[i])<=(i==0||i==5?4:6.9f));
    }
    in.imu.body_pitch_deg=0;in.now_us+=4000000;in.wheels.position_m_avg=.3f;
    assert(balance_core_step(&core,&in,&out));near(core.position_reference,original_reference,.000001f);
    puts("PASS: +/-50 degree body/world-leg angles keep control active; no 4-second wait/reanchor; voltage limits retained");
    in=upright();init(&core,&in);
    // Recorded forward-swept right leg: local angle about 6 degrees, body upright.
    in.joints[2].angle=2.397f;in.joints[3].angle=.363f;
    assert(balance_core_step(&core,&in,&out));
    assert(out.state.leg_angle_rad[1]<(float)M_PI_4);
    assert(strcmp(out.reason,"standing-control")==0);
    assert(fabsf(out.voltage[3])+fabsf(out.voltage[4])>.01f);
    puts("PASS: leg-only world tilt beyond 45 degrees no longer pauses output");
    in=upright();for(int i=0;i<4;i++)in.joints[i].angle=in.reference[i];init(&core,&in);
    in.imu.body_pitch_deg=5;
    assert(balance_core_step(&core,&in,&out));
    assert(out.state.leg_length_m[0]>.14f && out.state.leg_length_m[1]>.14f);
    assert(fabsf(out.model_torque)>.01f);
    assert(fabsf(out.voltage[0])>.1f && fabsf(out.voltage[5])>.1f);
    in.imu.phase=IMU_FAILED;assert(!balance_core_step(&core,&in,&out));
    for(int i=0;i<6;i++)assert(out.voltage[i]==0);
    puts("PASS: >140 mm legs keep original LQR/wheel output without a preparation gate; invalid IMU/math never produces voltage");
    fixture=upright();leg_calibrated=secondary_calibrated=true;
    leg_zero_right=fixture.reference[2];leg_zero_left=fixture.reference[3];
    motors[1]=fixture.joints[0];motors[2]=fixture.joints[1];motors[4]=fixture.joints[2];motors[3]=fixture.joints[3];
    motors[0].last_feedback_ms=motors[5].last_feedback_ms=1999;
    start_balance();assert(balance_active);balance_process(2000);
    // At the target, the selected 4 N feedforward exceeds soft-extension subtraction.
    assert(motors[1].target_voltage<0 && motors[2].target_voltage>0);
    assert(motors[4].target_voltage>0 && motors[3].target_voltage<0);
    motors[0].speed_rpm=60;motors[5].speed_rpm=-60;
    motors[0].angle+=.1f;motors[5].angle-=.1f;
    fixture.now_us+=4000;fixture.imu.sample_time_us=fixture.now_us-1500;
    balance_process(2004);
    // Printing later must retain the timestamp/age of the actual control sample.
    fixture.now_us+=50000;print_balance();fixture.now_us-=50000;
    // Model requests and physical forward-positive output must agree with the
    // direct core calculation using both wheels moving forward at one rev/s.
    robot_state_input_t forward=fixture;
    forward.wheels.position_m_avg=(motors[0].angle-motors[5].angle)*WHEEL_RADIUS_M*.5f;
    forward.wheels.speed_mps[0]=forward.wheels.speed_mps[1]=2*(float)M_PI*WHEEL_RADIUS_M;
    forward.wheels.speed_mps_avg=forward.wheels.speed_mps[0];
    balance_core_t expected;balance_output_t expected_out;init(&expected,&fixture);
    assert(balance_core_step(&expected,&fixture,&expected_out));
    assert(balance_core_step(&expected,&forward,&expected_out));
    for(int i=0;i<6;i++)near(motors[i].target_voltage,expected_out.voltage[i],.0002f);
    // Build nonzero length history before updating the live multiplier.
    fixture.joints[0].angle+=.01f;fixture.joints[1].angle-=.01f;
    motors[1]=fixture.joints[0];motors[2]=fixture.joints[1];
    fixture.now_us+=4000;fixture.imu.sample_time_us=fixture.now_us-1500;
    forward=fixture;
    forward.wheels.position_m_avg=(motors[0].angle-motors[5].angle)*WHEEL_RADIUS_M*.5f;
    forward.wheels.speed_mps[0]=forward.wheels.speed_mps[1]=2*(float)M_PI*WHEEL_RADIUS_M;
    forward.wheels.speed_mps_avg=forward.wheels.speed_mps[0];
    assert(balance_core_step(&expected,&forward,&expected_out));
    balance_process(fixture.now_us/1000);
    assert(fabsf(expected.support_length[0].inner.integral)>0);
    float retained[8];for(int i=0;i<8;i++)retained[i]=motors[i].target_voltage;
    assert(!balance_rate_command("pid"));
    const char *invalid_rates[]={"balance rate nan","balance rate inf","balance rate 0.49","balance rate 1.01","balance rate 0.7 extra"};
    for(int i=0;i<5;i++)assert(balance_rate_command(invalid_rates[i]));
    assert(balance_rate_command("balance rate"));
    assert(balance_rate_command("balance rate 0.75"));
    assert(balance_active);
    for(int i=0;i<8;i++)near(motors[i].target_voltage,retained[i],.000001f);
    expected.wheel_body_rate_ratio=.75f;
    fixture.now_us+=4000;fixture.imu.sample_time_us=fixture.now_us-1500;
    forward.now_us=fixture.now_us;forward.imu=fixture.imu;
    assert(balance_core_step(&expected,&forward,&expected_out));
    balance_process(fixture.now_us/1000);
    for(int i=0;i<6;i++)near(motors[i].target_voltage,expected_out.voltage[i],.0002f);
    print_balance(); // This sample must report 0.75, not the compiled default.
    for(int i=0;i<8;i++)retained[i]=motors[i].target_voltage;
    fixture.now_us+=4000;fixture.imu.sample_time_us=fixture.now_us-1500;
    rate_on_read=1;balance_process(fixture.now_us/1000);
    for(int i=0;i<8;i++)near(motors[i].target_voltage,retained[i],.000001f);
    assert(balance_rate_command("balance rate 0.8"));
    puts("PASS: live rate update preserves nonzero PID history/output; invalid values rejected; in-flight old-gain output discarded");
    motors[0].speed_rpm=motors[5].speed_rpm=0;
    print_balance_trace(); // Must reject while the controller still owns output.
    for(int i=0;i<1027;i++) {
        fixture.now_us+=4000;fixture.imu.sample_time_us=fixture.now_us-1500;
        motors[0].last_feedback_ms=motors[5].last_feedback_ms=fixture.now_us/1000-1;
        balance_process(fixture.now_us/1000);
    }
    cancel_on_read=1;balance_process(2004);assert(!balance_active);
    for(int i=0;i<8;i++)assert(motors[i].target_voltage==0);
    motors[0].target_voltage=.1f;print_balance_trace(); // Manual voltage also rejects dump.
    motors[0].target_voltage=0;print_balance_trace();
    puts("PASS: production mode routes all six motors and never re-enables after a concurrent stop");
    in=upright();in.joints[0].last_feedback_ms=0;
    leg_pid_t pid;init_leg_pid(&pid);assert(!balance_core_init(&core,&in,&pid.length));
    puts("PASS: stored references do not bypass initial joint feedback readiness");
    fixture=upright();fixture.imu.sample_time_us=fixture.now_us;
    motors[0].last_feedback_ms=motors[5].last_feedback_ms=1999;
    leg_zero_right=fixture.reference[2];leg_zero_left=fixture.reference[3];
    motors[1]=fixture.joints[0];motors[2]=fixture.joints[1];motors[4]=fixture.joints[2];motors[3]=fixture.joints[3];
    for(int i=0;i<8;i++)motors[i].target_voltage=0;
    balance_active=spring_active=secondary_active=pulse_active=false;
    start_support_test(7);assert(!balance_active);
    start_support_test(4);assert(balance_active);balance_process(2000);
    assert(motors[0].target_voltage==0 && motors[5].target_voltage==0);
    assert(motors[1].target_voltage<0 && motors[2].target_voltage>0);
    start_support_test(5);assert(balance_active);balance_process(2000);
    assert(motors[0].target_voltage==0 && motors[5].target_voltage==0);
    fixture.now_us+=9999000;
    fixture.imu.sample_time_us=fixture.now_us;
    for(int i=0;i<8;i++)motors[i].last_feedback_ms=11999;
    balance_process(11999);assert(balance_active);
    fixture.now_us+=1000;balance_process(12000);assert(!balance_active);
    for(int i=0;i<8;i++)assert(motors[i].target_voltage==0);
    puts("PASS: production static-test entry, invalid force rejection, active before 10 s and automatic 10000 ms all-motor stop");
}
''')
    exe = root / 'test.exe'
    sources = ('balance_control.c','robot_state.c','leg_calibration.c','leg_algorithm.c',
               'leg_pos.c','leg_spd.c','leg_conv.c','PID.c','lqr_k.c','motor_output.c')
    subprocess.run(['gcc','-std=gnu11','-O2','-Wall','-Wextra','-Werror',
                    '-I'+str(root),'-I'+str(source),str(root/'test.c'),
                    *[str(source/name) for name in sources],'-lm','-o',str(exe)],check=True)
    result=subprocess.run([str(exe)],capture_output=True,text=True)
    if result.returncode:
        raise RuntimeError(result.stdout[-3000:]+result.stderr)
    print('\n'.join(line for line in result.stdout.splitlines() if line.strip() and not line.startswith('BT,')))
    trace=[line.split(',') for line in result.stdout.splitlines() if line.startswith('BT,')]
    assert len(trace)==1024 and all(len(row)==25 for row in trace)
    times=[int(row[1]) for row in trace]
    assert all(b-a==4000 for a,b in zip(times,times[1:]))
    assert all(int(row[3])==1500 and int(row[4])==1000 and int(row[5])==1000 for row in trace)
    assert result.stdout.count('BALANCE_TRACE rejected: stop all outputs first')==2
    assert 'BALANCE_TRACE_END count=1024' in result.stdout
    print('PASS: ring wrap retains last 1024 consecutive samples; synchronized ages; no dump during actuation')
    assert 'control_us=2004000 gap_us=4000 imu_age_us=1500' in result.stdout
    assert 'LQR_SAMPLE control_us=2004000' in result.stdout
    assert 'wheel_body_rate_ratio=0.750' in result.stdout
    capture=root/'capture.log'
    capture.write_text(result.stdout,encoding='utf-8')
    subprocess.run([sys.executable,str(Path(__file__).with_name('analyze_balance_trace.py')),str(capture)],check=True)
    subprocess.run([sys.executable,str(Path(__file__).with_name('analyze_lqr_samples.py')),str(capture)],check=True)
