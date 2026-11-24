#include <inttypes.h>
#include <stdio.h>
#include "driver/twai.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "controller_state.h"
#include "motor_can.h"
#include "leg_config.h"
#include "leg_control_internal.h"

#define CAN_TX_GPIO GPIO_NUM_4
#define CAN_RX_GPIO GPIO_NUM_5

static const char *TAG = "foc_can";
motor_state_t motors[8];
portMUX_TYPE motor_mux = portMUX_INITIALIZER_UNLOCKED;

static const char *can_state_name(twai_state_t state)
{
    switch (state) {
        case TWAI_STATE_STOPPED: return "stopped";
        case TWAI_STATE_RUNNING: return "running";
        case TWAI_STATE_BUS_OFF: return "bus-off";
        case TWAI_STATE_RECOVERING: return "recovering";
        default: return "unknown";
    }
}

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t read_i16_le(const uint8_t *data)
{
    const uint16_t value = read_u16_le(data);
    return value <= INT16_MAX ? (int16_t)value : (int16_t)((int32_t)value - 65536);
}

static int32_t read_i32_le(const uint8_t *data)
{
    const uint32_t value = (uint32_t)data[0] |
                           ((uint32_t)data[1] << 8) |
                           ((uint32_t)data[2] << 16) |
                           ((uint32_t)data[3] << 24);
    return value <= INT32_MAX ? (int32_t)value : (int32_t)((int64_t)value - 4294967296LL);
}

static esp_err_t send_control_frame(uint32_t id, size_t first_motor)
{
    float voltage[4];
    portENTER_CRITICAL(&motor_mux);
    for (size_t i = 0; i < 4; ++i)
        voltage[i] = limit_motor_voltage((unsigned int)(first_motor + i + 1),
                                        motors[first_motor + i].target_voltage);
    portEXIT_CRITICAL(&motor_mux);

    twai_message_t message = {0};
    message.identifier = id;
    message.data_length_code = 8;
    for (size_t i = 0; i < 4; ++i) {
        const int16_t millivolts = (int16_t)(voltage[i] * 1000.0f);
        const uint16_t raw = (uint16_t)millivolts;
        message.data[i * 2] = (uint8_t)raw;
        message.data[i * 2 + 1] = (uint8_t)(raw >> 8);
    }
    return twai_transmit(&message, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
}

void send_motor_commands(void)
{
    static int64_t last_warning_ms;
    const esp_err_t left = send_control_frame(0x100, 0);
    const esp_err_t right = send_control_frame(0x200, 4);
    const int64_t now = esp_timer_get_time() / 1000;
    if ((left != ESP_OK || right != ESP_OK) && now - last_warning_ms >= 1000) {
        last_warning_ms = now;
        ESP_LOGW(TAG, "CAN transmit failed: 0x100=%s 0x200=%s",
                 esp_err_to_name(left), esp_err_to_name(right));
    }
}

void motor_send_task(void *arg)
{
    (void)arg;
    while (true) {
        send_motor_commands();
        // Match original Motor_SendTask; transmit backpressure cannot block PID/LQR.
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void can_receive_task(void *arg)
{
    twai_message_t message;
    while (true) {
        if (twai_receive(&message, pdMS_TO_TICKS(20)) != ESP_OK) {
            vTaskDelay(1);
            continue;
        }
        if (message.extd || message.rtr || message.data_length_code < 6 ||
            message.identifier < 0x101 || message.identifier > 0x108) {
            continue;
        }

        const size_t index = message.identifier - 0x101;
        const int64_t now = esp_timer_get_time() / 1000;
        const int32_t raw_angle = read_i32_le(message.data);
        const int16_t raw_speed = read_i16_le(&message.data[4]);

        portENTER_CRITICAL(&motor_mux);
        motors[index].angle = raw_angle / 1000.0f;
        motors[index].speed_rpm = raw_speed / 10.0f;
        motors[index].last_feedback_ms = now;
        portEXIT_CRITICAL(&motor_mux);
    }
}

void print_status(void)
{
    twai_status_info_t can;
    if (twai_get_status_info(&can) == ESP_OK) {
        printf("CAN state=%s tx_err=%" PRIu32 " rx_err=%" PRIu32
               " tx_failed=%" PRIu32 " bus_errors=%" PRIu32 " rx_missed=%" PRIu32
               " rx_overrun=%" PRIu32 " rx_queued=%" PRIu32 " tx_queued=%" PRIu32
               " arb_lost=%" PRIu32 "\r\n",
               can_state_name(can.state), can.tx_error_counter, can.rx_error_counter,
               can.tx_failed_count, can.bus_error_count, can.rx_missed_count,
               can.rx_overrun_count, can.msgs_to_rx, can.msgs_to_tx, can.arb_lost_count);
    }
    const int64_t now = esp_timer_get_time() / 1000;
    for (size_t i = 0; i < 8; ++i) {
        motor_state_t motor;
        portENTER_CRITICAL(&motor_mux);
        motor = motors[i];
        portEXIT_CRITICAL(&motor_mux);
        printf("ID=%u target=%.3f V angle=%.3f rad speed=%.1f rpm feedback=%s\r\n",
               (unsigned int)i + 1, motor.target_voltage, motor.angle, motor.speed_rpm,
               motor.last_feedback_ms != 0 && now - motor.last_feedback_ms < 500 ? "online" : "offline");
    }
}

void maintain_can(void)
{
    static int64_t last_check_ms;
    const int64_t now = esp_timer_get_time() / 1000;
    if (now - last_check_ms < 100) return;
    last_check_ms = now;

    twai_status_info_t status;
    if (twai_get_status_info(&status) != ESP_OK) return;
    if (status.state == TWAI_STATE_BUS_OFF) {
        if (twai_initiate_recovery() == ESP_OK) {
            ESP_LOGW(TAG, "CAN bus-off; recovery started");
        }
    } else if (status.state == TWAI_STATE_STOPPED) {
        if (twai_start() == ESP_OK) {
            ESP_LOGI(TAG, "CAN recovered and restarted");
        }
    }
}

void start_can(void)
{
    twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    general.rx_queue_len = 32;
    const twai_timing_config_t timing = TWAI_TIMING_CONFIG_1MBITS();
    const twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&general, &timing, &filter));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "TWAI ready: TX=GPIO%d RX=GPIO%d bitrate=1Mbps", CAN_TX_GPIO, CAN_RX_GPIO);
}
