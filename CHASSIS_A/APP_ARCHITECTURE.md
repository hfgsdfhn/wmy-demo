# CHASSIS_A 应用层架构

## 分层规则

```text
Core        CubeMX 生成的启动、外设初始化和中断入口
BSP         HAL 句柄、DMA、中断回调和原始总线收发
Driver      AK60、RS01、M2006、DM-IMU 等设备协议
Algorithm   PID、运动学、B 样条和路径跟踪纯算法
App         按底盘、翻墙、导航、传感器、通信和安全组织业务
RTOS        任务周期、队列、事件和 App 调度适配
Config      总线分配、设备 ID、周期、栈和控制参数
```

主要依赖方向为 `RTOS -> App -> Driver -> BSP -> HAL`；RTOS 的 I/O 适配器可以直接依赖 BSP 回调接口，但不能访问 HAL 句柄。Algorithm 只能依赖标准 C 类型和数学库；App/Algorithm 禁止包含 HAL、CMSIS-RTOS 或具体外设句柄。

## App 功能域

| 模块 | 设备/状态所有权 | 当前职责 |
|---|---|---|
| `app_init` | 初始化完成状态 | 按顺序启动 BSP、设备实例和各 App 模块 |
| `app_chassis` | 4 个 AK60 实例 | 保存底盘命令并接收反馈，预留 X 形运动学入口 |
| `app_climb` | 4 个 RS01、2 个 M2006 实例 | 保存机构命令和动作，预留翻墙状态机入口 |
| `app_navigation` | 当前轨迹目标 | 接收轨迹点，预留 B 样条与路径跟踪入口 |
| `app_sensors` | 最新 IMU、DT35 样本 | 提供带时间戳的传感器快照 |
| `app_communication` | 各通信源最后数据包 | 统一接收主控、小主机和传感器链路数据 |
| `app_can_router` | 无 | 按总线和 CAN ID 分发电机反馈 |
| `app_safety` | 当前系统事件 | 汇总急停、CAN 和电机故障状态 |
| `app_monitor` | 最新运行统计 | 保存遥测和诊断快照 |

模块内部设备实例为私有静态对象，不提供全局可写上下文。

## 公共数据头文件

| 文件 | 内容 |
|---|---|
| `app_messages.h` | CAN 帧、底盘/机构命令、动作、IMU/DT35 原始样本和通信数据包 |
| `app_state.h` | 融合后的机器人状态、运行统计和系统事件位 |
| `path.h` | 唯一轨迹点类型 `PathPoint`（`x/y/s` 为 mm，`theta` 为 rad，`curvature` 为 1/m，`velocity` 为 m/s） |

队列容量属于 RTOS 配置，不放入公共数据头文件。

## 当前数据流

```text
FDCAN IRQ -> bsp_can -> canRxQueue -> task_io -> app_can_router
                                              -> app_chassis / app_climb

IMU/DT35 parser -> RTOS queue -> task_io -> app_sensors
trajectory input -> trajectoryQueue -> task_navigation -> app_navigation
control command -> latest queue -> task_control -> app_chassis / app_climb
system events -> task_supervision -> app_safety / app_monitor
CAN recovery -> task_supervision -> bsp_board
```

FDCAN1、FDCAN2、FDCAN3 当前分别默认分配给 AK60、RS01、M2006，设备 ID 和总线映射集中在 `board_config.h`。接线确认后只修改配置，不修改任务代码。

本阶段不会主动发送电机控制命令，也没有实现串口协议、USB 协议、B 样条或翻墙状态机。
