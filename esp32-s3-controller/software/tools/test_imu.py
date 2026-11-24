"""Exercise the production IMU driver with SPI register mocks and Fusion with known motions."""
from pathlib import Path
import subprocess
import tempfile

source = Path(__file__).resolve().parents[1] / "main"
with tempfile.TemporaryDirectory(prefix="s3-imu-test-") as directory:
    root = Path(directory)
    (root / "driver").mkdir()
    (root / "freertos").mkdir()
    (root / "esp_err.h").write_text("""#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG 1
#define ESP_ERR_INVALID_STATE 2
#define ESP_ERR_TIMEOUT 3
#define ESP_ERR_NOT_FOUND 4
""")
    (root / "freertos/FreeRTOS.h").write_text("#pragma once\n#define pdMS_TO_TICKS(x) (x)\n")
    (root / "freertos/task.h").write_text("#pragma once\nvoid vTaskDelay(unsigned int ticks);\n")
    (root / "driver/spi_master.h").write_text("""#pragma once
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
typedef void *spi_device_handle_t;
typedef struct { int mosi_io_num,miso_io_num,sclk_io_num,quadwp_io_num,quadhd_io_num,max_transfer_sz; } spi_bus_config_t;
typedef struct { int clock_speed_hz,mode,spics_io_num,queue_size; } spi_device_interface_config_t;
typedef struct { unsigned int flags; size_t length; const void *tx_buffer; void *rx_buffer; uint8_t tx_data[4]; } spi_transaction_t;
#define SPI_TRANS_USE_TXDATA 1
#define SPI2_HOST 2
#define SPI_DMA_DISABLED 0
esp_err_t spi_bus_initialize(int, const spi_bus_config_t *, int);
esp_err_t spi_bus_add_device(int, const spi_device_interface_config_t *, spi_device_handle_t *);
esp_err_t spi_bus_remove_device(spi_device_handle_t);
esp_err_t spi_bus_free(int);
esp_err_t spi_device_transmit(spi_device_handle_t, spi_transaction_t *);
""")
    (root / "test.c").write_text(r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "driver/spi_master.h"
#include "imu963ra.h"
#include "imu963ra_regs.h"
#include "imu_attitude.h"
#include "imu_config.h"

static unsigned char main_regs[256], hub_regs[256];
static int bank, allocated, devices, missing_mag, fail_data, reset_stuck, calls;
static unsigned int delayed;
void vTaskDelay(unsigned int ticks) { delayed += ticks; }
esp_err_t spi_bus_initialize(int bus, const spi_bus_config_t *cfg, int dma) {
    assert(bus == SPI2_HOST && dma == SPI_DMA_DISABLED);
    assert(cfg->sclk_io_num == 12 && cfg->mosi_io_num == 11 && cfg->miso_io_num == 13);
    assert(!allocated); allocated = 1; return ESP_OK;
}
esp_err_t spi_bus_add_device(int bus, const spi_device_interface_config_t *cfg, spi_device_handle_t *dev) {
    (void)bus; assert(cfg->mode == 0 && cfg->spics_io_num == 10);
    assert(cfg->clock_speed_hz <= 10000000); *dev = (void *)1; devices++; return ESP_OK;
}
esp_err_t spi_bus_remove_device(spi_device_handle_t dev) { (void)dev; devices--; return ESP_OK; }
esp_err_t spi_bus_free(int bus) { (void)bus; allocated = 0; return ESP_OK; }
esp_err_t spi_device_transmit(spi_device_handle_t dev, spi_transaction_t *t) {
    assert(dev && t->length % 8 == 0); calls++;
    const unsigned char *tx = t->flags & SPI_TRANS_USE_TXDATA ? t->tx_data : t->tx_buffer;
    unsigned char reg = tx[0] & 0x7f;
    unsigned char *regs = bank ? hub_regs : main_regs;
    if (!(tx[0] & 0x80)) {
        assert(t->length == 16);
        if (reg == IMU963RA_FUNC_CFG_ACCESS) bank = tx[1] == 0x40;
        else if (!bank && reg == IMU963RA_CTRL3_C && tx[1] == 1)
            regs[reg] = reset_stuck ? 1 : 0;
        else regs[reg] = tx[1];
        if (bank && reg == IMU963RA_MASTER_CONFIG && tx[1] == 0x4c)
            hub_regs[IMU963RA_STATUS_MASTER] = missing_mag ? 8 : 0x81;
        return ESP_OK;
    }
    if ((fail_data == 1 && !bank && reg == IMU963RA_OUTX_L_G) ||
        (fail_data == 2 && bank && reg == IMU963RA_SENSOR_HUB_1)) return ESP_FAIL;
    unsigned char *rx = t->rx_buffer; rx[0] = 0;
    for (unsigned int i = 1; i < t->length/8; ++i) rx[i] = regs[reg + i - 1];
    if (bank && reg == IMU963RA_SENSOR_HUB_1 && hub_regs[IMU963RA_SLV0_SUBADD] == IMU963RA_MAG_CHIP_ID)
        rx[1] = 0xff;
    return ESP_OK;
}
static void reset_mock(void) {
    assert(!devices && !allocated);
    memset(main_regs, 0, sizeof(main_regs)); memset(hub_regs, 0, sizeof(hub_regs));
    bank = missing_mag = fail_data = reset_stuck = calls = 0; delayed = 0;
    main_regs[IMU963RA_WHO_AM_I] = 0x6b; main_regs[IMU963RA_STATUS_REG] = 3;
}
static esp_err_t init(void) {
    return imu963ra_init(IMU_SPI_SCK_GPIO, IMU_SPI_MOSI_GPIO, IMU_SPI_MISO_GPIO, IMU_SPI_CS_GPIO);
}
static void set_word(unsigned char *reg, int value) { reg[0] = value & 255; reg[1] = (value >> 8) & 255; }
static void check_driver(void) {
    reset_mock(); assert(init() == ESP_OK);
    assert(imu963ra_has_magnetometer() && imu963ra_device_id() == 0x6b && bank == 0);
    assert(main_regs[IMU963RA_CTRL1_XL] == 0x5c && main_regs[IMU963RA_CTRL2_G] == 0x5c);
    assert(main_regs[IMU963RA_CTRL9_XL] == 2);
    assert(init() == ESP_ERR_INVALID_STATE);
    set_word(main_regs + IMU963RA_OUTX_L_G, -1000);
    set_word(main_regs + 0x28 + 4, 4098); // LSM6DSR accelerometer Z, independent register address
    set_word(hub_regs + IMU963RA_SENSOR_HUB_1, -3000);
    imu963ra_sample_t sample = {0}; bool fresh;
    assert(imu963ra_read(&sample, &fresh) == ESP_OK && fresh && sample.magnetic_fresh && bank == 0);
    assert(fabsf(sample.gyro_dps[0] + 70) < 0.001f);
    assert(fabsf(sample.acceleration_g[2] - 1) < 0.001f);
    assert(fabsf(sample.magnetic_gauss[0] + 1) < 0.001f);
    main_regs[IMU963RA_STATUS_REG] = 1;
    assert(imu963ra_read(&sample, &fresh) == ESP_OK && !fresh);
    main_regs[IMU963RA_STATUS_REG] = 3;
    // A sensor-hub read error must restore the main bank and not publish data.
    fail_data = 2;
    assert(imu963ra_read(&sample, &fresh) == ESP_FAIL && !fresh && bank == 0);
    fail_data = 1;
    assert(imu963ra_read(&sample, &fresh) == ESP_FAIL && !fresh && bank == 0);
    fail_data = 0;
    imu963ra_deinit(); assert(!allocated && !devices);
    // Host-only reset while the powered sensor retains its hub bank.
    reset_mock(); bank = 1;
    assert(init() == ESP_OK && imu963ra_device_id() == 0x6b && bank == 0);
    imu963ra_deinit();
    reset_mock(); main_regs[IMU963RA_WHO_AM_I] = 0;
    assert(init() == ESP_ERR_NOT_FOUND && !allocated && !devices);
    reset_mock(); reset_stuck = 1;
    assert(init() == ESP_ERR_TIMEOUT && delayed < 1000 && !allocated && !devices);
    reset_mock(); missing_mag = 1;
    assert(init() == ESP_OK && !imu963ra_has_magnetometer() && bank == 0);
    assert(imu963ra_read(&sample, &fresh) == ESP_OK && fresh && !sample.magnetic_fresh);
    imu963ra_deinit();
    assert(imu963ra_init(12,12,13,10) == ESP_ERR_INVALID_ARG && !allocated);
    puts("SPI: units/endian, matching ODR, ID/reset timeout, missing mag and error handling PASS");
}
static void check_attitude(void) {
    float body[2];
    imu_body_angles(178.294f, -0.573f, 178.294f, -0.573f, body);
    assert(fabsf(body[0]) < 0.001f && fabsf(body[1]) < 0.001f);
    imu_body_angles(-173.169f, -18.03f, 178.294f, -0.573f, body);
    assert(body[0] > 8 && body[0] < 9 && body[1] > 17 && body[1] < 18);
    imu_body_angles(170.495f, 19.143f, 178.294f, -0.573f, body);
    assert(body[0] < -7 && body[1] < -19);
    assert(fabsf(imu_wrap_degrees(-359.9f)-0.1f) < 0.001f);
    assert(fabsf(imu_wrap_degrees(359.9f)+0.1f) < 0.001f);
    puts("Body angles: measured forward/back/left/right signs and +/-180 wrap PASS");
    imu_attitude_t state;
    imu_attitude_init(&state);
    const float dt = 1.0f / IMU_SAMPLE_RATE_HZ;
    const float acc[3] = {0,0,1}, bias[3] = {0.5f,-0.25f,0.75f}, moving[3] = {10,0,0};
    const float yaw_motion[3] = {0,0,10};
    for (int i=0; i<1000; i++) assert(imu_attitude_update(&state, acc, yaw_motion, dt));
    assert(state.ready && fabsf(state.yaw_deg)>1);
    assert(imu_attitude_update(&state, acc, moving, dt) && state.ready);
    imu_attitude_init(&state);
    for (int i=0; i<16000; i++) assert(imu_attitude_update(&state, acc, bias, dt));
    assert(state.ready);
    for (int i=0; i<3; i++) assert(fabsf(state.bias_dps[i]-bias[i]) < 0.001f);
    assert(fabsf(state.pitch_deg)<0.1f && fabsf(state.roll_deg)<0.1f);
    FusionAhrsSetHeading(&state.ahrs,0);
    const float yaw_rate[3] = {0.5f,-0.25f,90.75f};
    for (unsigned int i=0; i<IMU_SAMPLE_RATE_HZ; i++) assert(imu_attitude_update(&state, acc, yaw_rate, dt));
    assert(fabsf(state.yaw_deg-90) < 1);
    assert(!imu_attitude_update(&state, acc, bias, 0));
    assert(!imu_attitude_update(&state, acc, bias, 0.1f));
    const float bad[3] = {NAN,0,1}, zero[3] = {0};
    assert(!imu_attitude_update(&state,bad,bias,dt));
    assert(!imu_attitude_update(&state,zero,bias,dt));
    imu_attitude_init(&state);
    const float tilt[3] = {-0.5f,0,0.8660254f};
    for(int i=0;i<2000;i++) assert(imu_attitude_update(&state,tilt,zero,dt));
    assert(fabsf(state.pitch_deg-30) < 0.2f && fabsf(state.roll_deg)<0.1f);
    puts("Fusion: startup during motion without a stationary gate, background bias learning, static tilt, 90deg yaw, invalid data PASS");
}
int main(void) { check_driver(); check_attitude(); return 0; }
''')
    exe = root / "test.exe"
    subprocess.run([
        "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2",
        "-I" + str(root), "-I" + str(source), str(root / "test.c"),
        str(source / "imu963ra.c"), str(source / "imu_attitude.c"),
        str(source / "fusion/FusionAhrs.c"), str(source / "fusion/FusionOffset.c"),
        "-lm", "-o", str(exe),
    ], check=True)
    subprocess.run([str(exe)], check=True)
