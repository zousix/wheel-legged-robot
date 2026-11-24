"""Exercise production CAN encoding/decoding after restoring ID4 driver calibration."""
from pathlib import Path
import subprocess
import tempfile

source = Path(__file__).resolve().parents[1] / 'main'
with tempfile.TemporaryDirectory(prefix='motor-can-') as directory:
    root = Path(directory)
    (root / 'driver').mkdir()
    (root / 'freertos').mkdir()
    (root / 'freertos/FreeRTOS.h').write_text('''#pragma once
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
#define pdMS_TO_TICKS(x) (x)
''')
    (root / 'freertos/task.h').write_text('#pragma once\nvoid vTaskDelay(int);\n')
    (root / 'esp_timer.h').write_text('#pragma once\n#include <stdint.h>\nint64_t esp_timer_get_time(void);\n')
    (root / 'esp_err.h').write_text('''#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERROR_CHECK(x) ((void)(x))
static inline const char *esp_err_to_name(int e) {(void)e;return "mock";}
''')
    (root / 'esp_log.h').write_text('''#pragma once
#define ESP_LOGI(tag,...) ((void)(tag))
#define ESP_LOGW(tag,...) ((void)(tag))
''')
    (root / 'driver/twai.h').write_text('''#pragma once
#include <stdint.h>
#include "esp_err.h"
typedef enum {TWAI_STATE_STOPPED,TWAI_STATE_RUNNING,TWAI_STATE_BUS_OFF,TWAI_STATE_RECOVERING} twai_state_t;
typedef struct {uint32_t identifier;int extd,rtr,data_length_code;uint8_t data[8];} twai_message_t;
typedef struct {twai_state_t state;uint32_t tx_error_counter,rx_error_counter,tx_failed_count,bus_error_count,rx_missed_count,rx_overrun_count,msgs_to_rx,msgs_to_tx,arb_lost_count;} twai_status_info_t;
typedef struct {int rx_queue_len;} twai_general_config_t;
typedef int twai_timing_config_t;
typedef int twai_filter_config_t;
#define GPIO_NUM_4 4
#define GPIO_NUM_5 5
#define TWAI_MODE_NORMAL 0
#define TWAI_GENERAL_CONFIG_DEFAULT(a,b,c) ((twai_general_config_t){0})
#define TWAI_TIMING_CONFIG_1MBITS() 0
#define TWAI_FILTER_CONFIG_ACCEPT_ALL() 0
esp_err_t twai_transmit(const twai_message_t *,int);
esp_err_t twai_receive(twai_message_t *,int);
esp_err_t twai_get_status_info(twai_status_info_t *);
esp_err_t twai_initiate_recovery(void);
esp_err_t twai_start(void);
esp_err_t twai_driver_install(const twai_general_config_t *,const twai_timing_config_t *,const twai_filter_config_t *);
''')
    (root / 'test.c').write_text(r'''
#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "motor_can.c"
static twai_message_t sent[2], incoming;
static int sent_count, received;
static jmp_buf done;
void vTaskDelay(int t) {assert(t==2);longjmp(done,1);}
int64_t esp_timer_get_time(void) {return 1000000;}
esp_err_t twai_transmit(const twai_message_t *m,int timeout) {
    assert(timeout==4 && sent_count<2);sent[sent_count++]=*m;return ESP_OK;
}
esp_err_t twai_receive(twai_message_t *m,int timeout) {
    (void)timeout;if(received++)longjmp(done,1);*m=incoming;return ESP_OK;
}
esp_err_t twai_get_status_info(twai_status_info_t *s) {memset(s,0,sizeof(*s));return ESP_OK;}
esp_err_t twai_initiate_recovery(void) {return ESP_OK;}
esp_err_t twai_start(void) {return ESP_OK;}
esp_err_t twai_driver_install(const twai_general_config_t *g,const twai_timing_config_t *t,const twai_filter_config_t *f) {
    (void)g;(void)t;(void)f;return ESP_OK;
}
static void feed(unsigned int id,int32_t angle,int16_t speed) {
    incoming=(twai_message_t){.identifier=0x100+id,.data_length_code=8};
    memcpy(incoming.data,&angle,4);memcpy(incoming.data+4,&speed,2);received=0;
    if(!setjmp(done))can_receive_task(NULL);
}
int main(void) {
    // Recalibrated ID4 already reports the recorded positive controller axis.
    feed(4,1201,0);assert(fabsf(motors[3].angle-1.201f)<.00001f);
    feed(4,1386,123);assert(fabsf(motors[3].angle-1.386f)<.00001f);
    assert(fabsf(motors[3].speed_rpm-12.3f)<.00001f && motors[3].last_feedback_ms==1000);
    for(unsigned int id=1;id<=8;id++)if(id!=4) {
        feed(id,-2345,-123);assert(fabsf(motors[id-1].angle+2.345f)<.00001f);
        assert(fabsf(motors[id-1].speed_rpm+12.3f)<.00001f);
    }
    for(int i=0;i<8;i++)motors[i].target_voltage=(float)(i+1)*.25f;
    send_motor_commands();assert(sent_count==2 && sent[0].identifier==0x100 && sent[1].identifier==0x200);
    for(int i=0;i<8;i++) {
        int16_t wire=read_i16_le(sent[i/4].data+2*(i%4));
        assert(wire==(i+1)*250);
    }
    sent_count=0;motors[3].target_voltage=-20;send_motor_commands();
    assert(read_i16_le(sent[0].data+6)==-6900);
    sent_count=0;for(int i=0;i<8;i++)motors[i].target_voltage=0;send_motor_commands();
    for(int frame=0;frame<2;frame++)for(int b=0;b<8;b++)assert(sent[frame].data[b]==0);
    puts("PASS: recalibrated ID4 angle/rpm and voltage retain their signs; all IDs, limits, zero frames");
    sent_count=0;
    if(!setjmp(done))motor_send_task(NULL);
    assert(sent_count==2);
    puts("PASS: independent sender emits both frames then delays 2 ms");
}
''')
    exe = root / 'test.exe'
    subprocess.run(['gcc', '-std=gnu11', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-Wno-unused-parameter', '-I'+str(root), '-I'+str(source),
                    str(root / 'test.c'), '-lm', '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
