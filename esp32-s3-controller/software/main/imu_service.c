#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdatomic.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_service.h"
#include "imu_attitude.h"
#include "imu_config.h"

static const char *TAG = "imu963ra";
static portMUX_TYPE imu_mux = portMUX_INITIALIZER_UNLOCKED;
static imu_state_t latest;
static float roll_zero = IMU_ROLL_ZERO_DEG, pitch_zero = IMU_PITCH_ZERO_DEG;
static atomic_bool zero_requested;
static atomic_bool task_running;

static void publish(const imu_state_t *state)
{
    portENTER_CRITICAL(&imu_mux);
    latest = *state;
    portEXIT_CRITICAL(&imu_mux);
}

void imu_get_state(imu_state_t *state)
{
    portENTER_CRITICAL(&imu_mux);
    *state = latest;
    state->roll_zero_deg = roll_zero;
    state->pitch_zero_deg = pitch_zero;
    portEXIT_CRITICAL(&imu_mux);
    float body[2];
    imu_body_angles(state->roll_deg, state->pitch_deg, state->roll_zero_deg, state->pitch_zero_deg, body);
    state->body_roll_deg = body[0];
    state->body_pitch_deg = body[1];
}

bool imu_request_zero(void)
{
    imu_state_t state;
    imu_get_state(&state);
    if (state.phase != IMU_READY || esp_timer_get_time() - state.sample_time_us >= 100000) return false;
    atomic_store(&zero_requested, true);
    return true;
}

static void imu_task(void *arg)
{
    (void)arg;
    imu_state_t state = {.phase = IMU_STARTING};
    imu_attitude_t attitude;
    imu_attitude_init(&attitude);
    state.error = imu963ra_init(IMU_SPI_SCK_GPIO, IMU_SPI_MOSI_GPIO,
                               IMU_SPI_MISO_GPIO, IMU_SPI_CS_GPIO);
    state.device_id = imu963ra_device_id();
    state.magnetometer_available = imu963ra_has_magnetometer();
    if (state.error != ESP_OK) goto failed;
    ESP_LOGI(TAG, "WHO_AM_I=0x%02x, magnetometer=%s; automatic Fusion startup",
             state.device_id, state.magnetometer_available ? "OK" : "unavailable");
    // Warm-up runs in a separate task so CAN and console stay responsive.
    vTaskDelay(pdMS_TO_TICKS(2000));
    state.phase = IMU_SETTLING;
    publish(&state);
    const int64_t calibration_start = esp_timer_get_time();
    int64_t last_sample = calibration_start;
    int64_t zero_start = 0;
    unsigned int zero_count = 0;
    float roll_anchor = 0, pitch_anchor = 0, roll_sum = 0, pitch_sum = 0;
    while (true) {
        if (atomic_exchange(&zero_requested, false)) {
            zero_start = esp_timer_get_time();
            zero_count = 0;
            roll_sum = pitch_sum = 0;
        }
        bool fresh;
        state.error = imu963ra_read(&state.sensor, &fresh);
        if (state.error != ESP_OK) goto failed;
        int64_t now = esp_timer_get_time();
        if (fresh) {
            // First sample's age includes sensor warm-up; use nominal dt once.
            float dt = state.samples == 0 ? 1.0f / IMU_SAMPLE_RATE_HZ : (now - last_sample) * 1e-6f;
            // A scheduling gap is not a sensor/maths fault. Do not integrate
            // the latest sample across unobserved time or discard learned bias.
            if (dt > 0.05f) dt = 1.0f / IMU_SAMPLE_RATE_HZ;
            last_sample = now;
            if (!imu_attitude_update(&attitude, state.sensor.acceleration_g, state.sensor.gyro_dps, dt)) {
                state.error = ESP_ERR_INVALID_STATE;
                goto failed;
            }
            state.samples++;
            state.sample_time_us = now;
            if (state.sensor.magnetic_fresh) state.magnetic_time_us = now;
            state.phase = attitude.ready ? IMU_READY : IMU_SETTLING;
            state.roll_deg = attitude.roll_deg;
            state.pitch_deg = attitude.pitch_deg;
            state.yaw_deg = attitude.yaw_deg;
            memcpy(state.gyro_bias_dps, attitude.bias_dps, sizeof(state.gyro_bias_dps));
            memcpy(state.corrected_gyro_dps, attitude.corrected_gyro_dps, sizeof(state.corrected_gyro_dps));
            publish(&state);
            if (zero_start) {
                float norm_squared = 0;
                bool stationary = state.phase == IMU_READY;
                for (unsigned int axis = 0; axis < 3; ++axis) {
                    norm_squared += state.sensor.acceleration_g[axis] * state.sensor.acceleration_g[axis];
                    if (fabsf(state.corrected_gyro_dps[axis]) > 1.0f) stationary = false;
                }
                if (norm_squared < 0.81f || norm_squared > 1.21f) stationary = false;
                if (!zero_count) {
                    roll_anchor = state.roll_deg;
                    pitch_anchor = state.pitch_deg;
                }
                const float roll_delta = imu_wrap_degrees(state.roll_deg - roll_anchor);
                if (fabsf(roll_delta) > 0.3f || fabsf(state.pitch_deg - pitch_anchor) > 0.3f) stationary = false;
                if (!stationary) {
                    zero_count = 0;
                    roll_sum = pitch_sum = 0;
                } else {
                    roll_sum += roll_delta;
                    pitch_sum += state.pitch_deg;
                    if (++zero_count >= IMU_SAMPLE_RATE_HZ) {
                        const float new_roll_zero = imu_wrap_degrees(roll_anchor + roll_sum / zero_count);
                        const float new_pitch_zero = pitch_sum / zero_count;
                        portENTER_CRITICAL(&imu_mux);
                        roll_zero = new_roll_zero;
                        pitch_zero = new_pitch_zero;
                        portEXIT_CRITICAL(&imu_mux);
                        zero_start = 0;
                        printf("IMU zero complete: roll=%.3f pitch=%.3f deg; RAM only\r\n", new_roll_zero, new_pitch_zero);
                    }
                }
                if (zero_start && now - zero_start > 5000000) {
                    zero_start = 0;
                    printf("IMU zero rejected: keep upright and stationary for 1 second; previous reference retained\r\n");
                }
            }
        }
        if (now - last_sample > 100000) {
            state.error = ESP_ERR_TIMEOUT;
            goto failed;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
failed:
    state.phase = IMU_FAILED;
    publish(&state);
    imu963ra_deinit();
    ESP_LOGE(TAG, "IMU stopped: %s, WHO_AM_I=0x%02x", esp_err_to_name(state.error), state.device_id);
    atomic_store(&task_running, false);
    vTaskDelete(NULL);
}

esp_err_t start_imu(void)
{
    imu_state_t state;
    imu_get_state(&state);
    if (state.phase != IMU_NOT_STARTED && state.phase != IMU_FAILED) return ESP_ERR_INVALID_STATE;
    if (IMU_SPI_SCK_GPIO < 0 || IMU_SPI_MOSI_GPIO < 0 || IMU_SPI_MISO_GPIO < 0 || IMU_SPI_CS_GPIO < 0) {
        ESP_LOGW(TAG, "SPI pins are not configured in imu_config.h");
        return ESP_ERR_INVALID_ARG;
    }
    bool expected = false;
    if (!atomic_compare_exchange_strong(&task_running, &expected, true)) return ESP_ERR_INVALID_STATE;
    state = (imu_state_t){.phase = IMU_STARTING};
    atomic_store(&zero_requested, false);
    publish(&state);
    // Sensor acquisition must preempt console formatting (5) and CAN parsing (6).
    if (xTaskCreate(imu_task, "imu963ra", 4096, NULL, 7, NULL) != pdPASS) {
        state.phase = IMU_FAILED;
        state.error = ESP_ERR_NO_MEM;
        publish(&state);
        atomic_store(&task_running, false);
        return state.error;
    }
    return ESP_OK;
}

void print_imu(void)
{
    static const char *names[] = {"not-started", "starting", "settling", "ready", "failed"};
    imu_state_t state;
    imu_get_state(&state);
    int64_t age_ms = state.sample_time_us ? (esp_timer_get_time() - state.sample_time_us) / 1000 : -1;
    printf("IMU %s id=0x%02x error=%s samples=%u age=%lld ms mag=%s\r\n",
           names[state.phase], state.device_id, esp_err_to_name(state.error), state.samples,
           (long long)age_ms, state.magnetometer_available ? "available" : "unavailable");
    if (!state.sample_time_us) return;
    printf("IMU body: left_roll=%.2f forward_pitch=%.2f deg; reference=%.3f,%.3f deg\r\n",
           state.body_roll_deg, state.body_pitch_deg, state.roll_zero_deg, state.pitch_zero_deg);
    printf("IMU sensor axes: roll=%.2f pitch=%.2f yaw=%.2f deg (six-axis, relative yaw)\r\n",
           state.roll_deg, state.pitch_deg, state.yaw_deg);
    printf("IMU acc[g]=%.4f,%.4f,%.4f raw_gyro[dps]=%.3f,%.3f,%.3f\r\n",
           state.sensor.acceleration_g[0], state.sensor.acceleration_g[1], state.sensor.acceleration_g[2],
           state.sensor.gyro_dps[0], state.sensor.gyro_dps[1], state.sensor.gyro_dps[2]);
    printf("IMU corrected_gyro[dps]=%.3f,%.3f,%.3f bias[dps]=%.3f,%.3f,%.3f\r\n",
           state.corrected_gyro_dps[0], state.corrected_gyro_dps[1], state.corrected_gyro_dps[2],
           state.gyro_bias_dps[0], state.gyro_bias_dps[1], state.gyro_bias_dps[2]);
    if (state.magnetic_time_us)
        printf("IMU mag[gauss]=%.4f,%.4f,%.4f hub_age=%lld ms (uncalibrated)\r\n",
               state.sensor.magnetic_gauss[0], state.sensor.magnetic_gauss[1], state.sensor.magnetic_gauss[2],
               (long long)((esp_timer_get_time() - state.magnetic_time_us) / 1000));
}
