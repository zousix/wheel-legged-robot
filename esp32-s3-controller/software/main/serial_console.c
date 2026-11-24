#include <stdio.h>
#include <string.h>
#include <stdatomic.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "controller_state.h"
#include "motor_can.h"
#include "leg_control.h"
#include "leg_control_internal.h"
#include "serial_console.h"
#include "imu_service.h"
#include "wheel_odometry.h"
#include "robot_state.h"
#include "lqr_preview.h"
#include "balance_control.h"

static atomic_bool imu_stream_enabled = true;
static atomic_bool trace_dumping;

void feedback_log_task(void *arg)
{
    int64_t last_text_ms = 0;
    int64_t last_imu_ms = 0;
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
        spring_autostart_process(esp_timer_get_time() / 1000);
        secondary_autostart_process(esp_timer_get_time() / 1000);
        if (atomic_load(&trace_dumping)) continue;
        if (print_balance_waveform() || print_spring_waveform()) continue;
        const int64_t now = esp_timer_get_time() / 1000;
        if (atomic_load(&imu_stream_enabled)) {
            if (now - last_imu_ms >= 20) {
                last_imu_ms = now;
                imu_state_t imu;
                imu_get_state(&imu);
                if (imu.phase == IMU_READY && esp_timer_get_time() - imu.sample_time_us < 100000)
                    printf("imu:%.2f,%.2f,%.2f\r\n", imu.body_roll_deg, imu.body_pitch_deg, imu.yaw_deg);
            }
            continue;
        }
        if (now - last_text_ms < 1000) continue;
        last_text_ms = now;
        for (size_t i = 0; i < 8; ++i) {
            motor_state_t motor;
            portENTER_CRITICAL(&motor_mux);
            motor = motors[i];
            portEXIT_CRITICAL(&motor_mux);
            if (motor.last_feedback_ms != 0 && now - motor.last_feedback_ms < 500) {
                printf("CAN feedback ID=%u angle=%.3f rad speed=%.1f rpm\r\n",
                       (unsigned int)i + 1, motor.angle, motor.speed_rpm);
            }
        }
        print_leg();
    }
}

static void run_command(char *line)
{
    char *end = line + strcspn(line, "\r\n");
    *end = '\0';

    unsigned int id;
    float voltage;
    float other_voltage;
    float reference_right, reference_left;
    unsigned int duration_ms;
    char extra;
    if (spring_pid_command(line)) {
        // PID commands are parsed and acknowledged by the spring controller.
    } else if (balance_rate_command(line)) {
        // Live LQR tuning is acknowledged without changing output ownership.
    } else if (sscanf(line, "pairpulse %u %f %f %u %c", &id, &voltage, &other_voltage, &duration_ms, &extra) == 4) {
        if (start_pair_pulse(id,voltage,other_voltage,duration_ms)) printf("PAIRPULSE started ID=%u V=%.3f,%.3f duration=%u ms\r\n",id,voltage,other_voltage,duration_ms);
        else printf("PAIRPULSE rejected: pair ID2/3 or ID5/4, each <=1 V, 20..80 ms, stopped and fresh stationary feedback required\r\n");
    } else if (sscanf(line, "pulse %u %f %u %c", &id, &voltage, &duration_ms, &extra) == 3) {
        if (start_pulse(id, voltage, duration_ms)) printf("PULSE started ID=%u voltage=%.3f V duration=%u ms\r\n", id, voltage, duration_ms);
        else printf("PULSE rejected: ID1..6 <=1 V; wheels 20..100 ms, legs 20..150 ms; stop all, selected pair online/stationary\r\n");
    } else if (strcmp(line, "pulse") == 0) {
        print_pulse();
    } else if (sscanf(line, "v %u %f %c", &id, &voltage, &extra) == 2) {
        if (set_motor_voltage(id, voltage)) {
            printf("motor %u target=%.3f V\r\n", id, motors[id - 1].target_voltage);
        } else {
            printf("invalid command: use motor ID 1..8 and a finite voltage\r\n");
        }
    } else if (strcmp(line, "stop") == 0) {
        stop_all_motors();
        printf("all motor targets set to 0 V\r\n");
    } else if (strcmp(line, "status") == 0) {
        printf("CONSOLE stack_free_min=%u bytes\r\n", (unsigned int)uxTaskGetStackHighWaterMark(NULL));
        print_status();
        print_balance();
        print_leg();
        print_pulse();
        print_imu();
        print_wheels();
    } else if (strcmp(line, "wheel") == 0) {
        print_wheels();
    } else if (strcmp(line, "state") == 0) {
        print_robot_state();
    } else if (strcmp(line, "lqr") == 0) {
        print_lqr_preview();
    } else if (strcmp(line, "balance") == 0 || strcmp(line, "balance on") == 0) {
        start_balance();
    } else if (sscanf(line, "support %f %c", &voltage, &extra) == 1) {
        start_support_test(voltage);
    } else if (strcmp(line, "balance trace") == 0) {
        atomic_store(&trace_dumping,true);
        print_balance_trace();
        atomic_store(&trace_dumping,false);
    } else if (strcmp(line, "balance sample") == 0) {
        print_balance_sample();
    } else if (strcmp(line, "balance status") == 0) {
        print_balance();
    } else if (strcmp(line, "balance off") == 0) {
        stop_all_motors();
        print_balance();
    } else if (strcmp(line, "wheel zero") == 0) {
        printf("WHEELS zero %s; RAM only\r\n", wheel_zero() ? "complete" : "rejected: need fresh feedback and abs(rpm)<=2");
        print_wheels();
    } else if (strcmp(line, "imu") == 0) {
        print_imu();
    } else if (strcmp(line, "imu restart") == 0) {
        const esp_err_t result = start_imu();
        printf("IMU restart: %s; only stopped/failed IMU can restart; automatic Fusion startup\r\n",
               esp_err_to_name(result));
    } else if (strcmp(line, "imu zero") == 0) {
        if (imu_request_zero()) printf("IMU zero requested: hold the body upright and stationary; RAM only\r\n");
        else printf("IMU zero rejected: IMU must be ready with fresh data\r\n");
    } else if (strcmp(line, "imu on") == 0 || strcmp(line, "imu off") == 0) {
        const bool enabled = strcmp(line, "imu on") == 0;
        atomic_store(&imu_stream_enabled, enabled);
        printf("IMU angles output %s (50 Hz: body roll,pitch,relative yaw in degrees; pauses during leg waveform output)\r\n", enabled ? "on" : "off");
    } else if (strcmp(line, "cal23") == 0) {
        secondary_calibrate();
        secondary_print();
    } else if (sscanf(line, "ref23 %f %f %c", &reference_right, &reference_left, &extra) == 2) {
        secondary_restore_reference(reference_right, reference_left);
        secondary_print();
    } else if (strcmp(line, "cal") == 0) {
        calibrate_leg(false, 0, 0);
        print_leg();
    } else if (sscanf(line, "ref %f %f %c", &reference_right, &reference_left, &extra) == 2) {
        calibrate_leg(true, reference_right, reference_left);
        print_leg();
    } else if (strcmp(line, "spring") == 0) {
        start_spring();
    } else if (strcmp(line, "leg") == 0) {
        print_leg();
    } else if (strcmp(line, "legtest") == 0) {
        printf("LEG geometry self-test: %s\r\n", leg_geometry_self_test() ? "PASS" : "FAIL");
        printf("PULSE limits self-test: %s\r\n", pulse_self_test() ? "PASS" : "FAIL");
        printf("SPRING VMC self-test: %s\r\n", spring_self_test() ? "PASS" : "FAIL");
    } else if (strcmp(line, "help") == 0) {
        printf("v <id> <voltage> | pulse <id> <voltage> <ms> | pulse | stop | status | cal | cal23 | ref <ID5zero> <ID4zero> | spring | leg | legtest | help\r\n");
        printf("pid | pid <lo|li|ao|ai> <kp|ki|kd|imax|max|lpf> <value>; RAM only\r\n");
        printf("imu: IMU963RA status, acceleration, gyro, attitude and raw magnetometer\r\n");
        printf("imu on | imu off: body roll,pitch,relative yaw at 50 Hz | imu zero: stationary upright reference, RAM only\r\n");
        printf("imu restart: retry stopped/failed IMU without rebooting S3 or changing leg references\r\n");
        printf("wheel | wheel zero: forward-positive encoder distance/speed; left ID1/+1, right ID6/-1, tyre diameter 65 mm\r\n");
        printf("state: model axes [theta,dtheta,x,dx,phi,dphi], SI units; theta hip-ahead positive, phi backward positive; no output\r\n");
        printf("lqr: read-only model torque preview, 85..140 mm legs, wheel +/-0.03 Nm limit; no motor output\r\n");
        printf("Boot: automatic leg-length/angle PID + 4 N/leg support; wheels zero until balance command\r\n");
        printf("balance | balance on: start balance; needs each leg 128.2 +/-10 mm, 90 +/-7 deg and low speed\r\n");
        printf("balance is ground-standing control; contact estimate is diagnostic only, no automatic airborne/cushioning switch\r\n");
        printf("Firmware 45-degree pose pause and 4-second automatic recovery are removed\r\n");
        printf("balance trace: dump last 1024 synchronous samples after stop; no motor output\r\n");
        printf("balance rate [0.5..1.0]: inspect/set wheel combined body-rate multiplier, RAM only; clears trace on change\r\n");
        printf("balance status | status: inspect; balance off | stop: stop all and cancel automatic support; spring: restart leg support\r\n");
        printf("support <0..6>: 10000 ms static support test, N per leg; independent length/local angle PID, wheels/LQR/roll off\r\n");
        printf("ref23 <ID2zero> <ID3zero>: restore a known reference in the same motor power cycle; RAM only\r\n");
    } else if (*line != '\0') {
        printf("unknown command; type help\r\n");
    }
}

void serial_command_task(void *arg)
{
    char line[96];
    size_t length = 0;
    bool overflow = false;
    printf("Commands: v <id> <voltage>, stop, status, cal, leg, legtest, pid, help\r\n");
    while (true) {
        const int ch = getchar();
        if (ch == EOF) {
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(20));
        } else if (ch == '\r' || ch == '\n') {
            if (overflow) {
                printf("command rejected: line too long\r\n");
            } else if (length > 0) {
                line[length] = '\0';
                run_command(line);
            }
            length = 0;
            overflow = false;
        } else if (!overflow && length < sizeof(line) - 1) {
            line[length++] = (char)ch;
        } else {
            overflow = true;
        }
    }
}
