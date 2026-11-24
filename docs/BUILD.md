# 构建入口

## ESP32-S3 控制器

在已经安装并激活 ESP-IDF 的终端进入：

```sh
cd esp32-s3-controller/software
idf.py build
```

`sdkconfig` 和 `sdkconfig.defaults` 已配置目标 ESP32-S3，FreeRTOS tick 为 1000 Hz，控制台使用 USB Serial/JTAG。实际编译入口是 `main/CMakeLists.txt`。

当前整理环境没有完成 ESP-IDF 固件构建，未声明可复现的 ESP-IDF 版本。首次构建时应记录本机 `idf.py --version`，并以实际编译结果确认依赖兼容性。

默认 CAN 引脚为 TX GPIO4、RX GPIO5；IMU 引脚与方向定义见 `main/imu_config.h`。腿零位、安装方向与输出限幅保存在 `main/leg_config.h`，需要与实际装配匹配。

## STM32 电机驱动

使用 Keil MDK 打开：

```text
stm32-foc/software/MDK-ARM/C6T6SimpleFoc.uvprojx
```

工程声明的设备包是 `Keil.STM32F1xx_DFP.2.2.0`。驱动采用 STM32F103、DRV8313 与 AS5600，通过 CAN 接收电压指令并反馈编码器数据。该版本是电压模式 FOC，不包含电流采样闭环。

当前 `Src/main.c` 启用的是 **motorID = 5** 的调试入口，并包含串口诊断输出。它不是可对六块板原样重复烧录的统一镜像；每块驱动的 ID、旋转方向、零电角度及标定状态均需分别核对。

本次没有执行固件烧录或电机动作，也没有修改原工作区的固件参数。

## 主机测试

主机算法测试需要 Python 3、可从 PATH 调用的 GCC，以及 GCC 的数学库支持。脚本在临时目录编译生产源码，使用桩函数替代 HAL/RTOS 接口，不依赖实际硬件。

`esp32-controller/software/src` 与 `matlab/lqr_k.m` 是部分对照测试的输入，不能仅复制 ESP32-S3 目录后运行所有测试。
