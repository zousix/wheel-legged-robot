# 来源与改动范围

原始项目：[Skythinker616/foc-wheel-legged-robot](https://github.com/Skythinker616/foc-wheel-legged-robot)。我在原项目的机械、电路、控制算法和 STM32 FOC 方案上继续开发，原有源码的版权与来源声明均保留。

我主要围绕以下部分进行开发和调试：

- ESP32-S3 / ESP-IDF 控制固件及 FreeRTOS 任务拆分。
- CAN 电机节点接入、反馈状态管理、通信恢复及控制输出。
- IMU963RA 采集、姿态与轮速反馈、平衡控制集成和串口调试工具。
- 主机回归测试及日志分析脚本。
- STM32 驱动接入、编码器反馈与实机标定调试。

原控制器源码与 LQR 增益文件用于对照测试。这个仓库主要存放控制固件，没有收录原项目完整的机械、Android 和 Linux 图传模块。

第三方组件：

- STM32 HAL / CMSIS：保留随源码附带的版权与许可证声明。
- Fusion：保留 [Fusion_LICENSE.md](esp32-s3-controller/software/main/fusion/Fusion_LICENSE.md)。
- FOC / 磁编码器相关移植来源：保留 STM32 源文件内的 SimpleFOC 及移植作者说明。

整体许可证见 [LICENSE](LICENSE)。重新分发时请一并保留这些声明。
