# 双轮腿机器人控制系统

基于 **ESP32-S3 + FreeRTOS** 与 **STM32 电机驱动** 的双轮腿机器人控制固件，包含六电机 CAN 通信、IMU 姿态解算、腿长 PID、LQR/VMC 平衡控制以及串口调试与数据回放工具。

本仓库整理当前开发工作区的源码，重点展示控制固件与实机调试工作。当前为开发快照：本次主机检查 7 项通过、3 项断言失败，详见 [验证记录](docs/VALIDATION.md)。

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

## 参数与实机结果

| 项目 | 数值或功能 | 依据 |
| --- | --- | --- |
| 控制任务周期 | 4 ms | 固件调度配置；不等同于最坏执行时间测量 |
| 电机发送任务周期 | 2 ms | 独立发送任务配置 |
| CAN 速率 | 1 Mbps | TWAI 配置 |
| 控制轨迹缓冲 | 1024 帧 | 固件轨迹记录实现 |
| 连续站立 | 300 s | 作者提供的实机结果 |
| 基本运动 | 前进、后退、旋转 | 作者提供的实机结果 |

实机结果与主机回归测试分开记录。本次发布整理的是工作区源码快照，300 s 站立及运动演示尚未与本快照的具体提交绑定；不将主机测试通过解释为上述实机性能已经复现。测试范围见 [验证记录](docs/VALIDATION.md)。

## 目录

```text
esp32-s3-controller/software/
  main/                 当前 ESP32-S3 控制固件
  tools/                主机测试、日志分析与串口采集脚本
stm32-foc/software/     STM32 FOC 源码、HAL/CMSIS 与 Keil 工程
esp32-controller/      原控制器参考源码，供算法对照测试使用
matlab/lqr_k.m          LQR 增益对照文件，供一致性测试使用
docs/                  构建说明与验证范围
```

## 构建与测试

ESP32-S3 使用 ESP-IDF；STM32 使用 Keil MDK 工程。构建入口及板级配置要求见 [构建说明](docs/BUILD.md)。

安装 Python 3 与 GCC 后，在仓库根目录运行以下主机测试；这些测试不连接机器人：

```sh
python esp32-s3-controller/software/tools/test_control_schedule.py
python esp32-s3-controller/software/tools/test_motor_can.py
python esp32-s3-controller/software/tools/test_robot_state.py
python esp32-s3-controller/software/tools/test_balance_control.py
python esp32-s3-controller/software/tools/test_lqr_k.py
```

`capture_*.py` 等脚本会访问串口，部分脚本会下发控制指令，不能作为普通主机单元测试直接批量运行。串口采集脚本另需 `pyserial`。

## 来源与许可证

本项目基于 [Skythinker616/foc-wheel-legged-robot](https://github.com/Skythinker616/foc-wheel-legged-robot) 继续开发。仓库重点整理 ESP32-S3 控制链、通信、姿态反馈和测试工具；保留原有 FOC 与控制算法的来源说明，不将继承代码表述为全部原创。详见 [来源说明](CREDITS.md)。

本仓库按 [GNU GPL v3](LICENSE) 发布，第三方文件保留各自的许可证与版权声明。
