"""Compile production wheel feedback calculation and exercise signs, units and offline gaps."""
from pathlib import Path
import subprocess
import tempfile

source = Path(__file__).resolve().parents[1] / "main"
with tempfile.TemporaryDirectory(prefix="wheel-test-") as directory:
    root = Path(directory)
    (root / "freertos").mkdir()
    (root / "freertos/FreeRTOS.h").write_text("""#pragma once
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
""")
    (root / "esp_timer.h").write_text("#pragma once\n#include <stdint.h>\nint64_t esp_timer_get_time(void);\n")
    (root / "test.c").write_text(r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "wheel_odometry.h"
motor_state_t motors[8];
portMUX_TYPE motor_mux;
static int64_t now_ms = 1000;
int64_t esp_timer_get_time(void) { return now_ms * 1000; }
static void close_to(float actual, float expected) { assert(fabsf(actual-expected) < 0.00001f); }
int main(void) {
    assert(WHEEL_LEFT_ID==1 && WHEEL_RIGHT_ID==6);
    assert(WHEEL_LEFT_FORWARD_SIGN==1 && WHEEL_RIGHT_FORWARD_SIGN==-1);
    wheel_odometry_t state = {0};
    motor_state_t pair[2] = {{.angle=5,.last_feedback_ms=1000}, {.angle=-7,.last_feedback_ms=1000}};
    wheel_odometry_update(&state,pair,1000,false);
    assert(state.valid); close_to(state.position_m_avg,0);
    // One forward turn on both wheels must advance 65*pi mm, not the old 55*pi.
    pair[0].angle += 6.283185307f; pair[1].angle -= 6.283185307f;
    pair[0].speed_rpm = 60; pair[1].speed_rpm = -60;
    wheel_odometry_update(&state,pair,1001,false);
    close_to(state.position_m[0],0.20420352f); close_to(state.position_m[1],0.20420352f);
    close_to(state.speed_mps_avg,0.20420352f);
    // Multiple turns retain cumulative distance; unequal wheel speeds are averaged.
    pair[0].angle += 12.566370614f; pair[0].speed_rpm=120;
    wheel_odometry_update(&state,pair,1002,false);
    close_to(state.position_m[0],3*0.20420352f); close_to(state.speed_mps_avg,1.5f*0.20420352f);
    pair[0].speed_rpm=-60; pair[1].speed_rpm=60;
    wheel_odometry_update(&state,pair,1003,false); close_to(state.speed_mps_avg,-0.20420352f);
    // In-place counter rotation has zero averaged speed.
    pair[0].speed_rpm=60; pair[1].speed_rpm=60;
    wheel_odometry_update(&state,pair,1004,false); close_to(state.speed_mps_avg,0);
    wheel_odometry_update(&state,pair,1100,false);
    assert(!state.valid && !state.reference_valid && isnan(state.position_m_avg));
    pair[0].last_feedback_ms=pair[1].last_feedback_ms=1101;
    wheel_odometry_update(&state,pair,1101,false); assert(!state.valid);
    wheel_odometry_update(&state,pair,1101,true); assert(state.valid); close_to(state.position_m_avg,0);
    // A gap hidden by reconnecting before the next controller tick also invalidates.
    pair[0].last_feedback_ms=pair[1].last_feedback_ms=1300;
    wheel_odometry_update(&state,pair,1300,false); assert(!state.valid);
    pair[0].angle=NAN; wheel_odometry_update(&state,pair,1300,true); assert(!state.valid);
    pair[0].angle=0; pair[1].speed_rpm=INFINITY;
    wheel_odometry_update(&state,pair,1300,true); assert(!state.valid);
    pair[1].speed_rpm=0;
    wheel_odometry_update(&state,pair,1299,true); assert(!state.valid); // future timestamps
    // Exercise real ID-to-array mapping and stationary zero wrapper.
    motors[0]=(motor_state_t){.angle=-11.046f,.speed_rpm=0,.last_feedback_ms=1000};
    motors[5]=(motor_state_t){.angle=6.131f,.speed_rpm=0,.last_feedback_ms=1000};
    wheel_process(); wheel_get_state(&state); assert(state.valid);
    // Recorded physical left-only forward hand turn: ID1 +5.940 rad, ID6 unchanged.
    motors[0].angle+=5.940f;
    wheel_process(); wheel_get_state(&state);
    close_to(state.position_m[0],5.940f*.0325f);close_to(state.position_m[1],0);
    // Recorded right-only forward hand turn: ID6 -6.210 rad, ID1 unchanged.
    motors[5].angle-=6.210f;
    wheel_process(); wheel_get_state(&state);
    close_to(state.position_m[1],6.210f*.0325f);
    close_to(state.position_m_avg,(5.940f+6.210f)*.0325f/2);
    assert(wheel_zero()); wheel_get_state(&state); close_to(state.position_m_avg,0);
    motors[0].speed_rpm=3; assert(!wheel_zero());
    motors[0].speed_rpm=0; now_ms=1200; assert(!wheel_zero());
    puts("PASS: 65mm circumference, physical left ID1/+1, right ID6/-1, rpm conversion, cumulative distance, turns, gaps, invalid inputs, stationary zero");
}
''')
    exe = root / "test.exe"
    subprocess.run(["gcc", "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-I"+str(root), "-I"+str(source), str(root / "test.c"),
                    str(source / "wheel_odometry.c"), "-lm", "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
