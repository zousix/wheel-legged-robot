"""Run the production IMU task against simulated time/sensor data and real Fusion."""
from pathlib import Path
import subprocess
import tempfile

source = Path(__file__).resolve().parents[1] / 'main'
with tempfile.TemporaryDirectory(prefix='imu-service-') as directory:
    root = Path(directory)
    (root / 'freertos').mkdir()
    (root / 'esp_err.h').write_text('''#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG 1
#define ESP_ERR_INVALID_STATE 2
#define ESP_ERR_TIMEOUT 3
#define ESP_ERR_NO_MEM 4
static inline const char *esp_err_to_name(int err) { (void)err; return "mock"; }
''')
    (root / 'esp_timer.h').write_text('#pragma once\n#include <stdint.h>\nint64_t esp_timer_get_time(void);\n')
    (root / 'esp_log.h').write_text('''#pragma once
#define ESP_LOGI(tag,...) ((void)(tag))
#define ESP_LOGW(tag,...) ((void)(tag))
#define ESP_LOGE(tag,...) ((void)(tag))
''')
    (root / 'freertos/FreeRTOS.h').write_text('''#pragma once
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
#define pdMS_TO_TICKS(x) (x)
#define pdPASS 1
''')
    (root / 'freertos/task.h').write_text('''#pragma once
int xTaskCreate(void (*task)(void *), const char *, unsigned int, void *, unsigned int, void *);
void vTaskDelay(unsigned int);
void vTaskDelete(void *);
''')
    (root / 'test.c').write_text(r'''
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "imu_service.h"
#include "imu_attitude.h"
static int64_t now=1000000;
static unsigned int reads, moving_reads=2500, fail_after=3600;
static int fail_creation=1, no_fresh, allocated, ready_seen, ready_during_motion, delayed_ready;
static void (*task_entry)(void *);
static jmp_buf task_exit;
int64_t esp_timer_get_time(void) { return now; }
int xTaskCreate(void (*fn)(void *),const char *name,unsigned int stack,void *arg,unsigned int priority,void *handle) {
    (void)name;(void)stack;(void)arg;(void)handle;
    assert(priority>6); // Acquisition must outrank console/control and CAN parsing.
    if(fail_creation) { fail_creation=0;return 0; }
    task_entry=fn;return 1;
}
void vTaskDelay(unsigned int ms) {
    now+=(int64_t)ms*1000;
    imu_state_t s;imu_get_state(&s);
    if(s.phase==IMU_READY) ready_seen=1;
    if(s.phase==IMU_READY && reads<moving_reads) ready_during_motion=1;
}
void vTaskDelete(void *arg) { (void)arg;longjmp(task_exit,1); }
esp_err_t imu963ra_init(int sck,int mosi,int miso,int cs) {
    assert(sck==12 && mosi==11 && miso==13 && cs==10 && !allocated);
    allocated=1;reads=0;return ESP_OK;
}
esp_err_t imu963ra_read(imu963ra_sample_t *s,bool *fresh) {
    now+=4000;
    if(reads==1500) now+=80000; // One delayed service interval must retain READY.
    if(++reads>fail_after) return ESP_FAIL;
    memset(s,0,sizeof(*s));s->acceleration_g[2]=-1;
    s->gyro_dps[0]=reads<=moving_reads ? 10 : .1f;
    if(reads==1502) { imu_state_t state;imu_get_state(&state);delayed_ready=state.phase==IMU_READY; }
    *fresh=!no_fresh;return ESP_OK;
}
uint8_t imu963ra_device_id(void) { return 0x6b; }
bool imu963ra_has_magnetometer(void) { return true; }
void imu963ra_deinit(void) {
    // phase FAILED is published before cleanup: a restart must not race deinit.
    assert(start_imu()==ESP_ERR_INVALID_STATE);
    allocated=0;
}
static void run_task(void) { if(setjmp(task_exit)==0) task_entry(NULL); }
int main(void) {
    imu_state_t state;
    assert(start_imu()==ESP_ERR_NO_MEM);
    assert(start_imu()==ESP_OK);
    assert(start_imu()==ESP_ERR_INVALID_STATE);
    run_task();imu_get_state(&state);
    assert(ready_during_motion && delayed_ready && ready_seen && reads==3601);
    assert(state.phase==IMU_FAILED && state.error==ESP_FAIL && state.samples==3600 && !allocated);
    puts("PASS: startup reaches READY while moving; an 80 ms scheduling gap preserves Fusion state");
    moving_reads=0;fail_after=1600;ready_seen=0;
    assert(start_imu()==ESP_OK);run_task();imu_get_state(&state);
    assert(ready_seen && state.phase==IMU_FAILED && state.samples==1600 && !allocated);
    puts("PASS: failed-task restart, create-failure retry, duplicate-start and cleanup-race rejection");
    no_fresh=1;
    assert(start_imu()==ESP_OK);run_task();imu_get_state(&state);
    assert(state.phase==IMU_FAILED && state.error==ESP_ERR_TIMEOUT && state.samples==0 && !allocated);
    puts("PASS: actual missing sensor samples still report a transport timeout");
}
''')
    exe = root / 'test.exe'
    subprocess.run(['gcc', '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-I'+str(root), '-I'+str(source), str(root / 'test.c'),
                    *[str(source / name) for name in ('imu_service.c', 'imu_attitude.c',
                        'fusion/FusionAhrs.c', 'fusion/FusionOffset.c')],
                    '-lm', '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
