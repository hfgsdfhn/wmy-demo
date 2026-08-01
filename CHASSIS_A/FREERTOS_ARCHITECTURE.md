# CHASSIS_A FreeRTOS 基础架构

## 调度参数

- 内核：FreeRTOS 10.6.2，通过 CMSIS-RTOS2 API 使用。
- 系统节拍：1 kHz。
- 动态内存：`heap_4`，RTOS 堆 64 KB。
- 中断优先级分组：4 位抢占优先级。
- 可调用 RTOS ISR API 的最高优先级：5。三路 FDCAN IT0 均设为 5。
- 已启用栈溢出检测、动态内存分配失败钩子和集中断言记录。

## 任务

| 任务 | CMSIS 优先级 | 栈 | 运行方式 | 当前职责 |
|---|---:|---:|---|---|
| `bootstrapTask` | AboveNormal | 2048 B | 启动后执行一次 | 配置过滤器、启动三路 FDCAN、发布初始化事件后退出 |
| `controlTask` | Realtime | 2048 B | 1 ms 周期 | 获取最新底盘和翻墙机构命令，检查急停状态，调用控制接口 |
| `canRxTask` | High | 1536 B | 队列事件驱动 | 处理 ISR 投递的 CAN 帧并分发给电机驱动 |
| `safetyTask` | High | 1024 B | 10 ms 周期 | 汇总 CAN/电机/通信故障并管理急停事件 |
| `navigationTask` | AboveNormal | 3072 B | 10 ms 周期 | 消费轨迹点；后续承载 B 样条和路径跟踪 |
| `communicationTask` | Normal | 2048 B | 队列事件驱动 | 分发 DT35、另一主控、USB 和 DM-IMU 数据包 |
| `monitorTask` | Low | 1024 B | 100 ms 周期 | 汇总队列丢包、控制超期和系统事件，生成遥测快照 |

周期任务统一使用 `osDelayUntil()`，避免普通 `osDelay()` 造成周期漂移。任何任务都不应使用忙等待。

## 队列和共享对象

| 对象 | 深度 | 元素类型 | 用途 |
|---|---:|---|---|
| `canRxQueue` | 64 | `app_can_frame_t` | 三路 FDCAN ISR 到 CAN 分发任务 |
| `chassisCmdQueue` | 4 | `app_chassis_command_t` | 底盘速度命令，队满时保留较新命令 |
| `mechanismCmdQueue` | 4 | `app_mechanism_command_t` | 四杆角度和后轮速度命令 |
| `trajectoryQueue` | 32 | `app_trajectory_point_t` | B 样条控制点或采样点 |
| `communicationQueue` | 16 | `app_comm_packet_t` | 各通信接收端到协议分发任务 |
| `telemetryQueue` | 4 | `app_runtime_stats_t` | 最新运行统计，队满时淘汰最旧值 |
| `systemEvents` | 24 bit | 事件标志 | 初始化、CAN 就绪、通信在线、急停和故障状态 |
| `robotStateMutex` | 1 | 优先级继承互斥锁 | 保护世界坐标、速度、距离和机构状态快照 |

## 中断边界

FDCAN ISR 只做以下操作：

1. 从硬件 FIFO 取出报文。
2. 复制 ID、总线号、时间戳和最多 8 字节数据。
3. 使用零等待方式投递 `canRxQueue`。
4. 队列满时增加丢包计数，不在 ISR 中阻塞或解析电机协议。

UART DMA、USB CDC 和 RS485 接入后也应遵守相同边界：ISR/回调只切分数据块并投递，协议解析放在 `communicationTask`。

## 后续模块接口

应用层通过以下弱定义函数接入，当前默认实现不会发送电机命令：

- `App_OnCanFrame()`：分发 AK60、RS01 和 M2006 反馈。
- `App_ControlStep()`：执行运动学、PID 和 CAN 控制帧发送。
- `App_NavigationStep()`：执行 B 样条生成和路径跟踪。
- `App_OnCommunicationPacket()`：解析串口、USB 和 IMU 协议。
- `App_SafetyStep()`：增加设备超时、机械限位和急停策略。
- `App_MonitorStep()`：输出调试与运行状态。

## 当前未配置

UART DMA 通道、RS485 `DE/RE` 引脚、USB CDC 中间件和具体电机过滤 ID 暂未配置。这些依赖最终硬件连接和协议，不应在信息不足时写死。
