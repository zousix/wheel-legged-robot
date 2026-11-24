"""Exercise the production control task with simulated tick releases; no hardware."""
from pathlib import Path
import subprocess
import tempfile

source = Path(__file__).resolve().parents[1] / 'main'
with tempfile.TemporaryDirectory(prefix='control-schedule-') as directory:
    root = Path(directory)
    (root/'freertos').mkdir()
    (root/'freertos/FreeRTOS.h').write_text('''#pragma once
typedef int portMUX_TYPE;
typedef unsigned int TickType_t;
#define pdMS_TO_TICKS(x) (x)
#define pdPASS 1
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
''')
    (root/'freertos/task.h').write_text('''#pragma once
#include "FreeRTOS.h"
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *,TickType_t);
void vTaskDelay(unsigned int);
void vTaskPrioritySet(void *,unsigned int);
int xTaskCreate(void (*)(void *),const char *,unsigned int,void *,unsigned int,void *);
''')
    (root/'esp_err.h').write_text('''#pragma once
#include <assert.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 1
#define ESP_ERROR_CHECK(x) assert((x)==ESP_OK)
''')
    (root/'esp_log.h').write_text('#pragma once\n#define ESP_LOGI(tag,...) ((void)(tag))\n')
    (root/'esp_timer.h').write_text('#pragma once\n#include <stdint.h>\nint64_t esp_timer_get_time(void);\n')
    (root/'test.c').write_text(r'''
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "main.c"
portMUX_TYPE motor_mux;
bool balance_active;
static unsigned int tick=100, cycles, wheels, spring, secondary, balance;
static jmp_buf finished;
static unsigned int control_created, sender_created;
void start_can(void) {}
void stop_all_motors(void) {}
void load_leg_references(void) {}
bool leg_geometry_self_test(void) {return true;}
bool pulse_self_test(void) {return true;}
bool spring_self_test(void) {return true;}
void arm_spring_autostart(void) {}
esp_err_t start_imu(void) {return ESP_OK;}
void can_receive_task(void *p) {(void)p;}
void serial_command_task(void *p) {(void)p;}
void feedback_log_task(void *p) {(void)p;}
void motor_send_task(void *p) {(void)p;}
void pulse_process(int64_t now) {(void)now;}
void maintain_can(void) {}
void vTaskPrioritySet(void *p,unsigned int priority) {(void)p;(void)priority;}
void vTaskDelay(unsigned int ticks) {assert(ticks==1);longjmp(finished,1);}
int xTaskCreate(void (*fn)(void *),const char *name,unsigned int stack,void *arg,unsigned int priority,void *handle) {
    (void)stack;(void)arg;(void)handle;
    if(!strcmp(name,"control")) {assert(fn==control_task && priority>5);++control_created;}
    if(!strcmp(name,"motor_send")) {assert(fn==motor_send_task);++sender_created;}
    return pdPASS;
}
TickType_t xTaskGetTickCount(void) {return tick;}
int64_t esp_timer_get_time(void) {return (int64_t)tick*1000;}
void wheel_process(void) {++wheels;}
bool project_pose_control_allowed(int64_t now) {assert(now==tick);return cycles!=2;}
void spring_process(int64_t now) {assert(now==tick);++spring;tick++;}
void secondary_process(int64_t now) {assert(now+1==tick);++secondary;}
void balance_process(int64_t now) {assert(now==tick);++balance;tick++;}
void send_motor_commands(void) {assert(!"CAN send must stay outside control task");}
void vTaskDelayUntil(TickType_t *last,TickType_t period) {
    assert(period==4 && *last==100+cycles*4);
    *last+=period;tick=*last;
    ++cycles;balance_active=cycles==1;
    if(cycles==3)longjmp(finished,1);
}
int main(void) {
    if(!setjmp(finished))app_main();
    assert(control_created==1 && sender_created==1);
    if(!setjmp(finished))control_task(NULL);
    assert(wheels==3 && spring==1 && secondary==1 && balance==1);
    puts("PASS: 4 ms absolute control releases, spring/balance ownership and pose gate; no CAN sends in control task");
}
''')
    exe=root/'test.exe'
    subprocess.run(['gcc','-std=gnu11','-O2','-Wall','-Wextra','-Werror',
                    '-I'+str(root),'-I'+str(source),str(root/'test.c'),'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
