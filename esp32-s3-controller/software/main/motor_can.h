#pragma once

void send_motor_commands(void);
void motor_send_task(void *arg);
void can_receive_task(void *arg);
void print_status(void);
void maintain_can(void);
void start_can(void);
