#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"

#define CONTROL_PERIOD_MS 4

typedef struct {
    float target_voltage;
    float angle;
    float speed_rpm;
    int64_t last_feedback_ms;
} motor_state_t;

// Internal shared state. All reads and writes require motor_mux.
extern motor_state_t motors[8];
extern portMUX_TYPE motor_mux;
extern bool leg_calibrated;
