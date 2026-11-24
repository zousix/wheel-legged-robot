#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    float acceleration_g[3], gyro_dps[3], magnetic_gauss[3];
    bool magnetic_fresh;
} imu963ra_sample_t;

esp_err_t imu963ra_init(int sck, int mosi, int miso, int cs);
esp_err_t imu963ra_read(imu963ra_sample_t *sample, bool *fresh);
uint8_t imu963ra_device_id(void);
bool imu963ra_has_magnetometer(void);
void imu963ra_deinit(void);
