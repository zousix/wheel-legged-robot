# 双轮腿机器人控制系统

基于 **ESP32-S3 + FreeRTOS** 与 **STM32 电机驱动** 的双轮腿机器人控制固件，包含六电机 CAN 通信、IMU 姿态解算、腿长 PID、LQR/VMC 平衡控制以及串口调试与数据回放工具。

这是我的双轮腿机器人项目，主要做 ESP32-S3 控制固件、六电机通信、姿态反馈接入、平衡控制集成和实机调参。仓库包含 STM32 驱动源码及配套测试工具。

## 功能与实现

| 模块 | 实现内容 | 代码入口 |
| --- | --- | --- |
| 运动控制 | 融合姿态、关节编码器和轮速反馈，结合腿长 PID、LQR 与五连杆 VMC 完成控制输出 | [balance_control.c](esp32-s3-controller/software/main/balance_control.c)、[leg_algorithm.c](esp32-s3-controller/software/main/leg_algorithm.c) |
| 任务协同 | 分离控制、CAN 接收、指令处理与电机发送任务，使用周期唤醒和临界区同步共享状态 | [main.c](esp32-s3-controller/software/main/main.c) |
| 电机通信 | 电压指令打包、角度/速度反馈解析、反馈时间戳、节点状态及 bus-off 恢复 | [motor_can.c](esp32-s3-controller/software/main/motor_can.c) |
| 电机驱动 | STM32 电压模式 FOC、SVPWM、AS5600 角度采集、电角度零位校准及参数存储 | [FOCMotor.c](stm32-foc/software/USER/FOCMotor.c)、[main.c](stm32-foc/software/Src/main.c) |
| 姿态与反馈 | IMU963RA SPI 采集、Fusion 姿态解算、陀螺仪偏置处理、轮式里程反馈 | [imu_service.c](esp32-s3-controller/software/main/imu_service.c)、[wheel_odometry.c](esp32-s3-controller/software/main/wheel_odometry.c) |
| 调试与测试 | 串口配置与日志、控制轨迹采集、CAN 协议与控制链主机回归测试 | [tools](esp32-s3-controller/software/tools) |

## 控制架构

```text
串口指令 ───────────────────────────────┐
IMU963RA → 姿态解算 ────────────────────┤
CAN反馈 → 关节角度 / 轮速 / 节点状态 ───┤
                                      ↓
                          ESP32-S3 / FreeRTOS
                      腿长PID → LQR / 五连杆VMC
                                      ↓
                              电机输出与限幅
                                      ↓
                            1 Mbps CAN / 六电机
                                      ↓
                       STM32 → 电压模式FOC → SVPWM
```

## 控制参数

| 项目 | 配置 |
| --- | --- |
| 控制任务周期 | 4 ms |
| 电机发送任务周期 | 2 ms |
| CAN 速率 | 1 Mbps |
| 控制轨迹缓冲 | 1024 帧 |

## 实机表现

- 连续稳定站立 300 s。
- 支持前进、后退和旋转。

## 目录

```text
esp32-s3-controller/software/
  main/                 当前 ESP32-S3 控制固件
  tools/                主机测试、日志分析与串口采集脚本
stm32-foc/software/     STM32 FOC 源码、HAL/CMSIS 与 Keil 工程
esp32-controller/      控制算法对照测试源码
matlab/lqr_k.m          LQR 增益对照文件，供一致性测试使用
docs/                  构建说明与验证范围
```
