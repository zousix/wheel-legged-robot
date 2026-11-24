/*
 * Copyright (c) 2025 SEEKFREE 逐飞科技
 * SPDX-License-Identifier: GPL-3.0-or-later
 * ESP-IDF adaptation of the user's zf_device_imu963ra.c (STC32G library).
 * Original register definitions and full copyright notice: imu963ra_regs.h.
 */
#include <string.h>
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu963ra.h"
#include "imu963ra_regs.h"

static spi_device_handle_t device;
static bool bus_owned, mag_available;
static uint8_t device_id;

static esp_err_t read_registers(uint8_t reg, uint8_t *data, size_t size)
{
    if (!device || size == 0 || size > 12) return ESP_ERR_INVALID_ARG;
    uint8_t tx[13] = {reg | IMU963RA_SPI_R}, rx[13];
    spi_transaction_t transfer = {
        .length = (size + 1) * 8, .tx_buffer = tx, .rx_buffer = rx,
    };
    esp_err_t result = spi_device_transmit(device, &transfer);
    if (result == ESP_OK) memcpy(data, rx + 1, size);
    return result;
}

static esp_err_t write_register(uint8_t reg, uint8_t value)
{
    spi_transaction_t transfer = {
        .flags = SPI_TRANS_USE_TXDATA, .length = 16,
        .tx_data = {reg & 0x7f, value},
    };
    return spi_device_transmit(device, &transfer);
}

#define TRY(expression) do { esp_err_t result_ = (expression); if (result_ != ESP_OK) return result_; } while (0)

// Called only with the sensor-hub bank selected. Polls are bounded, unlike the
// old MCU's blocking bus loops. SPI failures and slave NACKs reach the caller.
static esp_err_t hub_wait(uint8_t mask)
{
    for (unsigned int attempt = 0; attempt < 50; ++attempt) {
        uint8_t status;
        TRY(read_registers(IMU963RA_STATUS_MASTER, &status, 1));
        if (status & 0x08) return ESP_FAIL;
        if (status & mask) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t mag_write(uint8_t reg, uint8_t value)
{
    TRY(write_register(IMU963RA_SLV0_CONFIG, 0));
    TRY(write_register(IMU963RA_SLV0_ADD, IMU963RA_MAG_ADDR << 1));
    TRY(write_register(IMU963RA_SLV0_SUBADD, reg));
    TRY(write_register(IMU963RA_DATAWRITE_SLV0, value));
    TRY(write_register(IMU963RA_MASTER_CONFIG, 0x4c));
    return hub_wait(0x80);
}

static esp_err_t mag_init(void)
{
    TRY(write_register(IMU963RA_MASTER_CONFIG, 0x80));
    vTaskDelay(pdMS_TO_TICKS(2));
    TRY(write_register(IMU963RA_MASTER_CONFIG, 0));
    vTaskDelay(pdMS_TO_TICKS(2));
    TRY(mag_write(IMU963RA_MAG_CONTROL2, 0x80));
    vTaskDelay(pdMS_TO_TICKS(2));
    TRY(mag_write(IMU963RA_MAG_CONTROL2, 0));
    TRY(write_register(IMU963RA_SLV0_ADD, (IMU963RA_MAG_ADDR << 1) | 1));
    TRY(write_register(IMU963RA_SLV0_SUBADD, IMU963RA_MAG_CHIP_ID));
    TRY(write_register(IMU963RA_SLV0_CONFIG, 1));
    TRY(write_register(IMU963RA_MASTER_CONFIG, 0x4c));
    TRY(hub_wait(1));
    uint8_t id;
    TRY(read_registers(IMU963RA_SENSOR_HUB_1, &id, 1));
    if (id != 0xff) return ESP_ERR_NOT_FOUND;
    // Original SeekFree defaults: QMC5883L +/-8 gauss, continuous 50 Hz.
    TRY(mag_write(IMU963RA_MAG_CONTROL1, 0x19));
    TRY(mag_write(IMU963RA_MAG_FBR, 1));
    TRY(write_register(IMU963RA_SLV0_ADD, (IMU963RA_MAG_ADDR << 1) | 1));
    TRY(write_register(IMU963RA_SLV0_SUBADD, IMU963RA_MAG_OUTX_L));
    TRY(write_register(IMU963RA_SLV0_CONFIG, 6));
    return write_register(IMU963RA_MASTER_CONFIG, 0x6c);
}

void imu963ra_deinit(void)
{
    if (device) { spi_bus_remove_device(device); device = NULL; }
    if (bus_owned) { spi_bus_free(SPI2_HOST); bus_owned = false; }
    mag_available = false;
}

static esp_err_t configure_sensor(void)
{
    vTaskDelay(pdMS_TO_TICKS(10));
    // An S3 reset can interrupt a hub read while the powered IMU retains its bank.
    TRY(write_register(IMU963RA_FUNC_CFG_ACCESS, 0));
    TRY(read_registers(IMU963RA_WHO_AM_I, &device_id, 1));
    if (device_id != 0x6b) return ESP_ERR_NOT_FOUND;
    TRY(write_register(IMU963RA_CTRL3_C, 1));
    bool reset_complete = false;
    for (unsigned int attempt = 0; attempt < 100; ++attempt) {
        uint8_t reset;
        vTaskDelay(pdMS_TO_TICKS(1));
        TRY(read_registers(IMU963RA_CTRL3_C, &reset, 1));
        if (!(reset & 1)) { reset_complete = true; break; }
    }
    if (!reset_complete) return ESP_ERR_TIMEOUT;
    TRY(write_register(IMU963RA_CTRL3_C, 0x44)); // BDU and address increment
    TRY(write_register(IMU963RA_CTRL1_XL, 0x5c)); // 208 Hz, +/-8 g
    TRY(write_register(IMU963RA_CTRL2_G, 0x5c));  // 208 Hz, +/-2000 degrees/s
    TRY(write_register(IMU963RA_CTRL4_C, 0x06)); // gyro LPF1, disable primary I2C
    TRY(write_register(IMU963RA_CTRL5_C, 0));
    TRY(write_register(IMU963RA_CTRL6_C, 0));
    TRY(write_register(IMU963RA_CTRL7_G, 0));
    // LSM6DSR I3C_disable is bit 1, not bit 0 as in the supplied STC driver.
    TRY(write_register(IMU963RA_CTRL9_XL, 0x02));
    TRY(write_register(IMU963RA_FUNC_CFG_ACCESS, 0x40));
    esp_err_t mag_result = mag_init();
    if (mag_result != ESP_OK) {
        // Restore main bank even when a magnetometer is absent.
        (void)write_register(IMU963RA_MASTER_CONFIG, 0);
    }
    TRY(write_register(IMU963RA_FUNC_CFG_ACCESS, 0));
    mag_available = mag_result == ESP_OK;
    return ESP_OK;
}

esp_err_t imu963ra_init(int sck, int mosi, int miso, int cs)
{
    if (device) return ESP_ERR_INVALID_STATE;
    const int pins[] = {sck, mosi, miso, cs};
    for (unsigned int i = 0; i < 4; ++i) {
        if (pins[i] < 0) return ESP_ERR_INVALID_ARG;
        for (unsigned int j = 0; j < i; ++j)
            if (pins[i] == pins[j]) return ESP_ERR_INVALID_ARG;
    }
    device_id = 0;
    mag_available = false;
    spi_bus_config_t bus = {
        .mosi_io_num = mosi, .miso_io_num = miso, .sclk_io_num = sck,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 32,
    };
    TRY(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED));
    bus_owned = true;
    spi_device_interface_config_t config = {
        .clock_speed_hz = 1000000, .mode = 0, .spics_io_num = cs, .queue_size = 1,
    };
    esp_err_t result = spi_bus_add_device(SPI2_HOST, &config, &device);
    if (result == ESP_OK) result = configure_sensor();
    if (result != ESP_OK) imu963ra_deinit();
    return result;
}

static int16_t read_int16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

esp_err_t imu963ra_read(imu963ra_sample_t *sample, bool *fresh)
{
    if (!device || !sample || !fresh) return ESP_ERR_INVALID_STATE;
    *fresh = false;
    sample->magnetic_fresh = false;
    uint8_t status, raw[12];
    TRY(read_registers(IMU963RA_STATUS_REG, &status, 1));
    if ((status & 3) != 3) return ESP_OK;
    TRY(read_registers(IMU963RA_OUTX_L_G, raw, sizeof(raw)));
    for (unsigned int axis = 0; axis < 3; ++axis) {
        sample->gyro_dps[axis] = read_int16(raw + 2 * axis) * 0.07f;
        sample->acceleration_g[axis] = read_int16(raw + 6 + 2 * axis) * 0.000244f;
    }
    if (mag_available) {
        TRY(write_register(IMU963RA_FUNC_CFG_ACCESS, 0x40));
        esp_err_t result = read_registers(IMU963RA_STATUS_MASTER, &status, 1);
        if (result == ESP_OK && (status & 0x08)) result = ESP_FAIL;
        if (result == ESP_OK && (status & 1)) {
            result = read_registers(IMU963RA_SENSOR_HUB_1, raw, 6);
            if (result == ESP_OK) {
                for (unsigned int axis = 0; axis < 3; ++axis)
                    sample->magnetic_gauss[axis] = read_int16(raw + 2 * axis) / 3000.0f;
                // Fresh hub transfer; QMC itself runs at 50 Hz, not 208 Hz.
                sample->magnetic_fresh = true;
            }
        }
        esp_err_t restore = write_register(IMU963RA_FUNC_CFG_ACCESS, 0);
        if (restore != ESP_OK) return restore;
        if (result != ESP_OK) return result;
    }
    *fresh = true;
    return ESP_OK;
}

uint8_t imu963ra_device_id(void) { return device_id; }
bool imu963ra_has_magnetometer(void) { return mag_available; }
