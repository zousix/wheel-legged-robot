# 来源与改动范围

原始项目：[Skythinker616/foc-wheel-legged-robot](https://github.com/Skythinker616/foc-wheel-legged-robot)。原项目包含机械、电路、控制算法和 STM32 FOC 等实现。本仓库是基于该项目的开发整理版本，不代表全套硬件及算法均为独立原创。

本次整理重点保留：

- ESP32-S3 / ESP-IDF 控制固件及 FreeRTOS 任务拆分。
- CAN 电机节点接入、反馈状态管理、通信恢复及控制输出。
- IMU963RA 采集、姿态与轮速反馈、平衡控制集成和串口调试工具。
- 主机回归测试及日志分析脚本。
- 当前工作区的 STM32 驱动修改，保留原代码中的作者说明。

原控制器源码与 LQR 增益文件保留用于对照测试。没有在本整理副本中收录原项目完整的机械、Android 和 Linux 图传模块。

第三方组件：

- STM32 HAL / CMSIS：保留随源码附带的版权与许可证声明。
- Fusion：保留 [Fusion_LICENSE.md](esp32-s3-controller/software/main/fusion/Fusion_LICENSE.md)。
- FOC / 磁编码器相关移植来源：保留 STM32 源文件内的 SimpleFOC 及移植作者说明。

整体许可证见 [LICENSE](LICENSE)。重新分发时请一并保留这些声明。
