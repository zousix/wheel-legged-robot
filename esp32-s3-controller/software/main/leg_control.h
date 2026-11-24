#pragma once

#include <stdbool.h>
#include <stdint.h>

bool pulse_self_test(void);
bool start_pulse(unsigned int id, float voltage, unsigned int duration_ms);
bool start_pair_pulse(unsigned int id, float voltage, float other_voltage, unsigned int duration_ms);
void pulse_process(int64_t now);
void print_pulse(void);
bool spring_self_test(void);
void spring_process(int64_t now);
void start_spring(void);
void arm_spring_autostart(void);
void spring_autostart_process(int64_t now);
bool print_spring_waveform(void);
bool spring_pid_command(const char *line);
bool leg_geometry_self_test(void);
void calibrate_leg(bool restore, float reference_right, float reference_left);
void load_leg_references(void);
void print_leg(void);
bool set_motor_voltage(unsigned int id, float voltage);
void stop_all_motors(void);
