idf_component_register(SRCS "main.c" "motor_can.c" "leg_control.c" "serial_console.c"
                            "leg_calibration.c" "leg_pulse.c" "leg_spring.c"
                            "leg_algorithm.c" "leg_self_test.c" "leg_secondary.c"
                            "imu963ra.c" "imu_attitude.c" "imu_service.c"
                            "wheel_odometry.c"
                            "robot_state.c" "motor_output.c" "lqr_preview.c" "balance_control.c"
                            "fusion/FusionAhrs.c" "fusion/FusionOffset.c"
                            "PID.c" "leg_pos.c" "leg_spd.c" "leg_conv.c" "lqr_k.c"
                       INCLUDE_DIRS "." REQUIRES driver esp_timer)
