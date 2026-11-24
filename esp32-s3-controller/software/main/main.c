#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "controller_state.h"
#include "motor_can.h"
#include "leg_control.h"
#include "leg_config.h"
#include "leg_control_internal.h"
#include "serial_console.h"
#include "imu_service.h"
#include "wheel_odometry.h"
#include "balance_control.h"

static const char *TAG = "foc_can";

static void control_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        const int64_t now = esp_timer_get_time() / 1000;
        wheel_process();
        portENTER_CRITICAL(&motor_mux);
        const bool balancing = balance_active;
        portEXIT_CRITICAL(&motor_mux);
        if (balancing) {
            balance_process(now);
        } else if (project_pose_control_allowed(now)) {
            spring_process(now);
            secondary_process(now);
        }
        // Match original Ctrl_Task: absolute 4 ms releases, no CAN/console waits.
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

void app_main(void)
{
    start_can();
    stop_all_motors();
    load_leg_references();
    ESP_ERROR_CHECK(leg_geometry_self_test() ? ESP_OK : ESP_FAIL);
    ESP_ERROR_CHECK(pulse_self_test() ? ESP_OK : ESP_FAIL);
    ESP_ERROR_CHECK(spring_self_test() ? ESP_OK : ESP_FAIL);
    ESP_LOGI(TAG, "Dual-leg control: independent length PID in preparation and balance; per-leg histories retained");
    /* Outputs stay at zero until an explicit preparation or test command. */
    ESP_ERROR_CHECK(xTaskCreate(can_receive_task, "can_receive", 3072, NULL, 6, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    // Balance startup holds controller snapshots while printf formats its reply.
    ESP_ERROR_CHECK(xTaskCreate(serial_command_task, "serial_command", 6144, NULL, 5, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(feedback_log_task, "feedback_log", 3072, NULL, 4, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    // IMU failures remain visible through 'imu'; calibration never blocks CAN.
    (void)start_imu();
    ESP_ERROR_CHECK(xTaskCreate(control_task, "control", 4096, NULL, 6, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(motor_send_task, "motor_send", 3072, NULL, 5, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    // Keep pulse deadlines separate from the fixed-period control calculation.
    vTaskPrioritySet(NULL, 5);

    while (true) {
        const int64_t now = esp_timer_get_time() / 1000;
        pulse_process(now);
        maintain_can();
        vTaskDelay(1);
    }
}
