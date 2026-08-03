# CHASSIS_A FreeRTOS 基础配置

RTOS 代码位于 `user_code/RTOS`。该层只负责调度、队列和同步，不直接访问 HAL 外设，也不实现电机协议与控制算法。

## 内核配置

| 参数 | 配置 |
|---|---:|
| FreeRTOS | 10.6.2，CMSIS-RTOS2 接口 |
| Tick | 1000 Hz |
| 内存管理 | `heap_4`，64 KB |
| 最大优先级 | 56 |
| 队列注册表 | 16 |
| 栈检测 | `configCHECK_FOR_STACK_OVERFLOW = 2` |
| ISR RTOS API 优先级边界 | 5 |

## 任务规划

| 任务 | 优先级 | 栈 | 周期/触发 | 实现文件 |
|---|---:|---:|---|---|
| `bootstrapTask` | AboveNormal | 2048 B | 启动一次 | CubeMX `freertos.c` |
| `controlTask` | Realtime | 2048 B | 1 ms | `task_control.c` |
| `sensorTask` | High1 | 1536 B | 任务标志 | `task_io.c` |
| `canRxTask` | High | 1536 B | CAN 队列 | `task_io.c` |
| `safetyTask` | High | 1024 B | 10 ms | `task_supervision.c` |
| `mechanismTask` | AboveNormal1 | 1536 B | 5 ms | `task_control.c` |
| `navigationTask` | AboveNormal | 3072 B | 10 ms | `task_navigation.c` |
| `communicationTask` | Normal | 2048 B | 通信队列 | `task_io.c` |
| `monitorTask` | Low | 1024 B | 100 ms | `task_supervision.c` |

周期任务统一使用 `osDelayUntil()`。任务直接调用对应 App 功能域接口，不再通过弱函数覆盖组织业务。

## 队列规划

| 队列 | 深度 | 元素 | 队满策略 |
|---|---:|---|---|
| `canRxQueue` | 64 | `app_can_frame_t` | 丢弃新帧并记录计数 |
| `chassisCmdQueue` | 4 | `app_chassis_command_t` | 淘汰最旧命令，保留最新命令 |
| `mechanismCmdQueue` | 4 | `app_mechanism_command_t` | 淘汰最旧命令，保留最新命令 |
| `trajectoryQueue` | 32 | `app_trajectory_point_t` | FIFO，队满返回失败 |
| `actionQueue` | 8 | `app_action_command_t` | FIFO，保持动作顺序 |
| `imuQueue` | 4 | `app_imu_sample_t` | 淘汰最旧样本，保留最新状态 |
| `dt35Queue` | 2 | `app_dt35_sample_t` | 淘汰最旧样本，保留最新距离 |
| `communicationQueue` | 16 | `app_comm_packet_t` | 队满丢弃并记录计数 |
| `telemetryQueue` | 4 | `app_runtime_stats_t` | 淘汰最旧统计快照 |

## 同步和接口

- `systemEvents` 保存初始化、急停、CAN、传感器在线和故障状态。
- `robotStateMutex` 使用优先级继承，保护完整机器人状态快照。
- IMU 与 DT35 入队后通过任务标志唤醒 `sensorTask`，不使用轮询。
- 队列消息定义在 `app_messages.h`，状态快照和事件定义在 `app_state.h`，业务代码不依赖 CMSIS-RTOS。
- `app_rtos.h` 只公开数据提交、状态快照、事件和故障接口；队列与任务句柄均为内部对象。

从 ISR 调用提交接口时，超时参数必须为 `0U`。
