"""Exercise the actual six-state composition and generated leg kinematics on the host."""
from pathlib import Path
import subprocess
import tempfile

source = Path(__file__).resolve().parents[1] / "main"
with tempfile.TemporaryDirectory(prefix="robot-state-") as directory:
    root = Path(directory)
    (root / "freertos").mkdir()
    (root / "freertos/FreeRTOS.h").write_text("""#pragma once
typedef int portMUX_TYPE;
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
""")
    (root / "esp_err.h").write_text("#pragma once\ntypedef int esp_err_t;\n")
    (root / "esp_timer.h").write_text("#pragma once\n#include <stdint.h>\nint64_t esp_timer_get_time(void);\n")
    (root / "test.c").write_text(r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "robot_state.h"
#include "lqr_preview.h"
#include "leg_control_internal.h"
motor_state_t motors[8];
portMUX_TYPE motor_mux;
bool leg_calibrated, secondary_calibrated, spring_active, secondary_active, pulse_active, balance_active;
float leg_zero_right,leg_zero_left;
static robot_state_input_t fixture;
int64_t esp_timer_get_time(void) { return fixture.now_us; }
void imu_get_state(imu_state_t *state) { *state=fixture.imu; }
void wheel_get_state(wheel_odometry_t *state) { *state=fixture.wheels; }
void secondary_reference_locked(float reference[2]) { reference[0]=fixture.reference[0]; reference[1]=fixture.reference[1]; }
void secondary_load_reference_locked(float r,float l) { fixture.reference[0]=r;fixture.reference[1]=l;secondary_calibrated=true; }
static void close_to(float x,float y,float tolerance) { assert(fabsf(x-y)<tolerance); }
static robot_state_input_t upright(void) {
    robot_state_input_t input={0};
    input.now_us=1000000;
    input.imu.phase=IMU_READY; input.imu.sample_time_us=999000; input.imu.roll_deg=180;
    input.wheels.valid=true;
    input.wheels.last_feedback_ms[0]=input.wheels.last_feedback_ms[1]=999;
    input.reference[0]=LEG23_RIGHT_ZERO;input.reference[1]=LEG23_LEFT_ZERO;
    input.reference[2]=LEG_RIGHT_ZERO;input.reference[3]=LEG_LEFT_ZERO;
    input.calibrated[0]=input.calibrated[1]=true;
    for(int i=0;i<4;i++) input.joints[i]=(motor_state_t){.angle=input.reference[i],.last_feedback_ms=999};
    return input;
}
static void check_lqr_preview(void) {
    robot_state_t state={.valid=true,.reason="test",
        .leg_length_m={.1282f,.1282f},.leg_angle_rad={1.57079632679f,1.57079632679f}};
    lqr_preview_t p;
    assert(lqr_preview_calculate(&state,0,0,&p));
    assert(p.valid && !p.wheel_limited);
    close_to(p.model_torque_nm,0,.000001f);close_to(p.model_hip_torque_nm,0,.000001f);
    state.s[2]=.2f;state.s[3]=.1f;
    assert(lqr_preview_calculate(&state,.2f,.1f,&p));
    close_to(p.wheel_forward_raw_nm,0,.000001f);
    state.s[2]=state.s[3]=0;
    // Golden 5-degree body-forward fixture in model axes, compared with MATLAB K.
    const float tilt=5*3.14159265358979323846f/180;
    state.s[0]=tilt;state.s[4]=-tilt;
    assert(lqr_preview_calculate(&state,0,0,&p) && p.wheel_limited);
    close_to(p.wheel_forward_raw_nm,.1231481915f,.000002f);
    close_to(p.model_hip_torque_nm,-.2179436657f,.000002f);
    close_to(p.wheel_forward_limited_nm,.03f,.000001f);
    state.s[0]=-tilt;state.s[4]=tilt;
    assert(lqr_preview_calculate(&state,0,0,&p) && p.wheel_limited);
    close_to(p.wheel_forward_limited_nm,-.03f,.000001f);
    state.s[0]=state.s[4]=0;
    state.valid=false;state.reason="imu-feedback-stale";
    assert(!lqr_preview_calculate(&state,0,0,&p));
    assert(!p.valid && strcmp(p.reason,"imu-feedback-stale")==0 && p.model_torque_nm==0);
    state.valid=true;
    state.leg_length_m[0]=.150f;state.leg_length_m[1]=.1064f;
    assert(!lqr_preview_calculate(&state,0,0,&p)); // Mean is valid, one leg is not.
    state.leg_length_m[0]=.1282f;state.leg_length_m[1]=.1282f;
    state.leg_angle_rad[0]+=50*tilt/5;state.leg_angle_rad[1]+=50*tilt/5;
    assert(!lqr_preview_calculate(&state,0,0,&p)); // Opposite leg tilts cancel in mean.
    state.leg_angle_rad[0]=state.leg_angle_rad[1]=1.57079632679f;
    state.s[4]=.8f;
    assert(!lqr_preview_calculate(&state,0,0,&p));
    state.s[4]=0;state.s[1]=NAN;
    assert(!lqr_preview_calculate(&state,0,0,&p));
    state.s[1]=0;
    assert(!lqr_preview_calculate(&state,NAN,0,&p));
    puts("PASS: LQR preview equilibrium, reference errors, MATLAB tilt golden values, torque limits and invalid/individual-leg pose gates");
}
int main(void) {
    check_lqr_preview();
    robot_state_input_t input=upright();robot_state_t state;
    assert(robot_state_calculate(&input,&state));
    for(int i=0;i<6;i++)close_to(state.s[i],0,0.00001f);
    close_to(state.leg_length_m[0],0.150623f,0.00001f);
    close_to(state.leg_length_m[1],0.150623f,0.00001f);
    const float rad=3.14159265358979323846f/180;
    input.imu.body_pitch_deg=10; input.imu.corrected_gyro_dps[1]=20;
    input.wheels.position_m_avg=.2f;input.wheels.speed_mps_avg=.1f;
    assert(robot_state_calculate(&input,&state));
    close_to(state.s[0],10*rad,0.00001f);close_to(state.s[1],20*rad,0.00001f);
    close_to(state.s[2],.2f,0.00001f);close_to(state.s[3],.1f,0.00001f);
    close_to(state.s[4],-10*rad,0.00001f);close_to(state.s[5],-20*rad,0.00001f);
    // Euler pitch rate must account for roll; copying gyro_y blindly reverses it.
    input.imu.roll_deg=0;
    assert(robot_state_calculate(&input,&state));close_to(state.s[5],20*rad,0.00001f);
    input.imu.roll_deg=90;input.imu.corrected_gyro_dps[2]=10;
    assert(robot_state_calculate(&input,&state));close_to(state.s[5],-10*rad,0.00001f);
    // Independent world-vector construction checks the two mirrored local frames.
    // ID2/3 left forward is -local x; ID5/4 right forward is +local x. Rotate into world
    // with forward body pitch, then measure wheel-to-hip angle from world +Y.
    input=upright();
    input.joints[0].angle=-2.258f;input.joints[1].angle=-1.641f;
    input.joints[2].angle=5.911f;input.joints[3].angle=2.090f;
    input.imu.body_pitch_deg=3;
    assert(robot_state_calculate(&input,&state));
    float world_theta=0;
    for(int pair=0;pair<2;pair++) {
        float angle=state.leg_angle_rad[pair],length=state.leg_length_m[pair];
        float foot_forward=(pair==0?-1:1)*length*cosf(angle);
        float foot_down=length*sinf(angle),pitch=3*rad;
        float hip_x=-foot_forward*cosf(pitch)+foot_down*sinf(pitch);
        float hip_y=foot_forward*sinf(pitch)+foot_down*cosf(pitch);
        world_theta+=atan2f(hip_x,hip_y)/2;
    }
    close_to(state.s[0],world_theta,.00001f);
    // 2026-10-19 left-forward samples, other leg held fixed in this fixture.
    input=upright();input.joints[0].angle=-1.989f;input.joints[1].angle=-2.063f;
    assert(robot_state_calculate(&input,&state));float before_theta=state.s[0];
    float before_angle=state.leg_angle_rad[0];
    input.joints[0].angle=-2.431f;input.joints[1].angle=-2.529f;
    assert(robot_state_calculate(&input,&state));assert(state.s[0]<before_theta);
    assert(state.leg_angle_rad[0]>before_angle);
    // 2026-10-19 right-forward samples, other leg held fixed in this fixture.
    input=upright();input.joints[2].angle=5.767f;input.joints[3].angle=1.237f;
    assert(robot_state_calculate(&input,&state));before_theta=state.s[0];
    before_angle=state.leg_angle_rad[1];
    input.joints[2].angle=4.782f;input.joints[3].angle=1.147f;
    assert(robot_state_calculate(&input,&state));assert(state.s[0]<before_theta);
    assert(state.leg_angle_rad[1]<before_angle);
    // Numerical derivative checks the whole leg-angle and body-rate chain.
    input=upright();input.imu.corrected_gyro_dps[1]=20;
    input.joints[0].speed_rpm=-13;input.joints[1].speed_rpm=-7;
    input.joints[2].speed_rpm=5;input.joints[3].speed_rpm=17;
    assert(robot_state_calculate(&input,&state));
    robot_state_input_t next=input;robot_state_t next_state;const float dt=.001f;
    for(int i=0;i<4;i++)next.joints[i].angle+=input.joints[i].speed_rpm*6*rad*dt;
    next.imu.body_pitch_deg+=20*dt;
    assert(robot_state_calculate(&next,&next_state));
    close_to((next_state.s[0]-state.s[0])/dt,state.s[1],.002f);
    close_to((next_state.s[4]-state.s[4])/dt,state.s[5],.0001f);
    input=upright();input.calibrated[0]=false;
    assert(!robot_state_calculate(&input,&state)&&isnan(state.s[0]));
    input=upright();input.imu.phase=IMU_FAILED;
    assert(!robot_state_calculate(&input,&state));
    input=upright();input.imu.sample_time_us=970000;
    assert(!robot_state_calculate(&input,&state));
    input=upright();input.wheels.valid=false;
    assert(!robot_state_calculate(&input,&state));
    input=upright();input.wheels.last_feedback_ms[0]=970;
    assert(!robot_state_calculate(&input,&state));
    input=upright();input.joints[0].last_feedback_ms=970;
    assert(!robot_state_calculate(&input,&state));
    input=upright();input.joints[2].last_feedback_ms=1001;
    assert(!robot_state_calculate(&input,&state));
    input=upright();input.joints[0].angle=NAN;
    assert(!robot_state_calculate(&input,&state));
    input=upright();input.imu.body_pitch_deg=NAN;
    assert(!robot_state_calculate(&input,&state)&&isnan(state.s[2]));
    // Production snapshot must route both pairs correctly and never change output.
    fixture=upright();
    motors[1]=fixture.joints[0];motors[2]=fixture.joints[1];motors[4]=fixture.joints[2];motors[3]=fixture.joints[3];
    leg_zero_right=fixture.reference[2];leg_zero_left=fixture.reference[3];
    secondary_calibrated=leg_calibrated=true;
    motors[0].target_voltage=.123f;
    robot_state_snapshot(&state);assert(state.valid);assert(motors[0].target_voltage==.123f);
    print_lqr_preview(); // Rest-length fixture is outside fitted range; never writes targets.
    assert(motors[0].target_voltage==.123f);
    puts("PASS: six-state order/SI units, upright/tilt, roll-aware gyro rate, kinematic derivative, missing/stale/NaN inputs, pair routing and no output writes");
}
''')
    exe = root / "test.exe"
    subprocess.run(["gcc", "-std=gnu11", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-I"+str(root), "-I"+str(source), str(root / "test.c"),
                    *[str(source / name) for name in ("robot_state.c", "leg_calibration.c", "leg_pos.c", "leg_spd.c", "lqr_preview.c", "lqr_k.c")],
                    "-lm", "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
