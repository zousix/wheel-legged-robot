# 测试记录

更新日期：2026-11-24。

## 实机测试与配置

- 4 ms 控制周期、2 ms 发送周期、1 Mbps CAN 与 1024 帧缓冲来自源码配置。
- 实机已完成连续稳定站立 300 s，以及前进、后退和旋转。
- 实机测试对应的固件版本和演示视频尚未归档到仓库，当前提交还需要重新做实机回归。
- 速度、漂移、超调量和运动成功率暂未形成测量记录。

## 主机回归

环境：Windows、Python 3、MinGW GCC。共运行 10 项，7 项通过，3 项未通过。

| 测试脚本 | 结果 |
| --- | --- |
| `test_control_schedule.py` | 通过 |
| `test_motor_can.py` | 通过 |
| `test_robot_state.py` | 通过 |
| `test_lqr_k.py` | 通过 |
| `test_imu.py` | 通过 |
| `test_imu_service.py` | 通过 |
| `test_wheel_odometry.py` | 通过 |
| `test_balance_control.py` | 失败：`out.grounded` 断言触发 |
| `test_startup_sequence.py` | 失败：腿长 PID 状态与预期快照的 `memcmp` 比较不相等 |
| `test_project_control.py` | 失败：测试期望的 ID2/ID3 零位与当前固件值不一致 |

零位测试期望 -2.458 / -1.721 rad，当前固件使用 -2.428 / -1.742 rad，需要对照实际标定记录更新测试。接地判定和启动状态两项还在排查。

主机测试通过模拟接口验证协议及控制逻辑。电机电气性能、实际任务抖动和机器人稳定性需要另外做实机测试，当前版本仍在调试中。

Keil 工程引用的源码路径已检查，未发现缺失文件。

## 构建范围

上述结果来自主机 GCC 测试，ESP-IDF 和 Keil 的完整固件构建尚未验证。构建入口与板级配置见 [BUILD.md](BUILD.md)。
