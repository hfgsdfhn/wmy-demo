#ifndef APP_MESSAGES_H
#define APP_MESSAGES_H /* 应用消息模块头文件保护宏 */

#include <stdint.h>
#include <stdbool.h>

#include "bsp_can.h"
#include "bsp_uart.h"

#define APP_CAN_DATA_MAX_SIZE             8U  /* CAN 数据段最大字节数 */
#define APP_COMM_PAYLOAD_MAX_SIZE         64U /* 通信负载最大字节数 */

/*
 * 应用层 CAN 总线索引（枚举）
 * 说明：表示系统中可用的 CAN 总线编号。
 */
typedef enum
{
    APP_CAN_BUS_1 = 0,
    APP_CAN_BUS_2,
    APP_CAN_BUS_3,
    APP_CAN_BUS_COUNT
} app_can_bus_t;


/*
 * CAN 帧结构（在应用层使用）
 * 字段：
 *  - timestamp_ms: 帧接收或发送的时间戳，单位毫秒（ms）
 *  - identifier: CAN 标识符（ID）
 *  - bus: 使用的 CAN 总线（app_can_bus_t）
 *  - is_extended_id: 是否为扩展 ID（1 表示扩展帧）
 *  - length: 数据有效字节数（0..APP_CAN_DATA_MAX_SIZE）
 *  - data: 数据负载数组
 */
typedef struct
{
    uint32_t timestamp_ms;            /* 时间戳（ms） */
    uint32_t identifier;              /* CAN 标识符 */
    app_can_bus_t bus;                /* 使用的 CAN 总线 */
    uint8_t is_extended_id;           /* 是否为扩展 ID（非零为扩展） */
    uint8_t length;                   /* 数据长度（字节） */
    uint8_t data[APP_CAN_DATA_MAX_SIZE]; /* 数据负载 */
} app_can_frame_t;

/*
 * 底盘运动命令（由上层/PC/遥控等产生的目标速度和角速度）
 * 字段说明：
 *  - sequence: 命令序列号（用于追踪/帧丢失检测）
 *  - timestamp_ms: 命令的时间戳（ms）
 *  - velocity_x_mps: 车体在 x 方向的目标线速度，单位米/秒（m/s）
 *  - velocity_y_mps: 车体在 y 方向的目标线速度，单位米/秒（m/s）
 *  - yaw_rate_radps: 偏航角速度（绕垂直轴），单位弧度/秒（rad/s）
 *  - valid: 命令是否有效（0 = 无效，非 0 = 有效）
 */
typedef struct
{
    uint32_t sequence;                /* 命令序列号 */
    uint32_t timestamp_ms;            /* 时间戳（ms） */
    float velocity_x_mps;             /* 目标线速度 X（m/s） */
    float velocity_y_mps;             /* 目标线速度 Y（m/s） */
    float yaw_rate_radps;             /* 偏航角速度（rad/s） */
    uint8_t valid;                    /* 有效标志 */
} app_chassis_command_t;

/*
 * 轨迹点（用于路径/规划），坐标单位为毫米，theta 为弧度
 * 字段：
 *  - x, y: 轨迹点坐标（mm）
 *  - theta: 朝向角（rad）
 *  - curvature: 曲率
 *  - s: 路段弧长或路径坐标（按使用约定）
 *  - velocity: 目标速度（按照使用约定，通常为 m/s 或 mm/s——请参照上层调用者）
 */
/* Coordinates are in millimetres; theta is in radians. */
typedef struct
{
    float x;                          /* x（mm） */
    float y;                          /* y（mm） */
    float theta;                      /* 朝向（rad） */
    float curvature;                  /* 曲率 */
    float s;                          /* 路径坐标/弧长 */
    float velocity;                   /* 速度（使用约定） */
} app_trajectory_point_t;

/*
 * 真实世界状态（用于上报给 PC 或记录）
 * 字段说明：
 *  - timestamp_ms: 状态时间戳（ms）
 *  - position_x_m/position_y_m: 位置，单位米（m）
 *  - yaw_deg: 角速度积分得到的航向角，角度（deg）
 *  - velocity_x_mps/velocity_y_mps: 各方向速度，单位 m/s
 *  - yaw_rate_radps: 偏航角速度，单位 rad/s
 *  - dt35_distance_m: 来自 DT35 传感器的距离（数组，单位 m）
 *  - status_flags: 状态标志位（按上层定义）
 */
typedef struct
{
    uint32_t timestamp_ms;            /* 时间戳（ms） */
    float position_x_m;               /* 位置 X（m） */
    float position_y_m;               /* 位置 Y（m） */
    float yaw_deg;                    /* 角速度积分航向（deg） */
    float imu_yaw_deg;                /* IMU 直接航向（deg） */
    float chassis_yaw_deg;            /* 底盘逆解角速度积分航向（deg） */
    float fused_yaw_deg;              /* IMU/底盘融合角速度积分航向（deg） */
    float fused_yaw_rate_radps;       /* 融合偏航角速度（rad/s） */
    float velocity_x_mps;             /* 速度 X（m/s） */
    float velocity_y_mps;             /* 速度 Y（m/s） */
    float yaw_rate_radps;             /* 偏航角速度（rad/s） */
    float dt35_distance_m[2];         /* DT35 传感器距离（m） */
    uint32_t status_flags;            /* 状态标志 */
} app_real_world_t;

/*
 * DT35 采样（超声/测距设备）
 * - timestamp_ms: 采样时间戳（ms）
 * - distance_m: 两个测点的距离，单位 m
 * - valid: 是否有效
 */
typedef struct
{
    uint32_t timestamp_ms;            /* 时间戳（ms） */
    float distance_m[2];              /* 距离数组（m） */
    uint8_t valid;                    /* 是否有效 */
} app_dt35_sample_t;

/*
 * 双 MCU 控制命令（例如遥控器输入序列）
 * 字段说明：摇杆值为 int16，范围和含义由上层协议决定
 */
typedef struct
{
    uint8_t control_mode;             /* 控制模式 */
    uint8_t action_id;                /* 动作 ID */
    uint8_t flags;                    /* 额外标志 */
    int16_t rc_joy_left_x;            /* 左摇杆 X */
    int16_t rc_joy_left_y;            /* 左摇杆 Y */
    int16_t rc_joy_right_x;           /* 右摇杆 X */
    int16_t rc_joy_right_y;           /* 右摇杆 Y */
    int16_t shoulder_left;            /* 左肩键 */
    int16_t shoulder_right;           /* 右肩键 */
    uint32_t timestamp_ms;            /* 最近一次有效控制帧接收时间 */
    uint8_t sequence;                 /* 最近一次有效控制帧序号 */
} app_dual_control_t;

typedef enum
{
    APP_DUAL_CONTROL_UNAVAILABLE = 0,
    APP_DUAL_CONTROL_ACTIVE,
    APP_DUAL_CONTROL_STOP
} app_dual_control_freshness_t;

typedef struct
{
    uint16_t session_id;
    int16_t joy_x;
    int16_t joy_y;
    int16_t joy_right_x;
    int16_t joy_right_y;
    uint32_t keys;
    uint8_t switches;
    uint8_t flags;
    uint8_t fault;
    uint8_t desired_power;
    uint16_t sequence;
    uint32_t timestamp_ms;
    uint32_t lost_frames;
} app_remote_control_t;

typedef enum
{
    APP_REMOTE_CONTROL_UNAVAILABLE = 0,
    APP_REMOTE_CONTROL_ACTIVE,
    APP_REMOTE_CONTROL_STOP
} app_remote_control_freshness_t;

/* RC_STATE switches.bit3 is remote SW23 on PC0 and gates region actions. */
#define APP_REMOTE_SWITCH_PC0_REGION_ENABLE (1U << 3U)

/*
 * 双 MCU 状态（上报）
 */
typedef struct
{
    uint8_t chassis_state;            /* 底盘状态 */
    uint8_t current_action_id;        /* 当前动作 ID */
    uint8_t action_result;            /* 动作结果 */
    uint8_t fault_code;               /* 故障码 */
    uint16_t status_flags;            /* 状态标志 */
    uint32_t timestamp_ms;            /* 时间戳（ms） */
    uint8_t sequence;                 /* 序列号 */
} app_dual_status_t;

/*
 * PC 健康上报结构（用于发送给上位机）
 * 字段：
 *  - timestamp_ms: 时间戳（ms）
 *  - online_mask: 在线掩码（各子系统在线位）
 *  - system_events: 系统事件位域
 *  - can_error_mask: CAN 错误掩码
 *  - pc_command: PC 命令（枚举）
 *  - motor_feedback_mask: 电机反馈有效掩码
 *  - motor_tx_ok_count/failed_count: 发送成功/失败计数
 *  - motor_target_erpm: 目标电机转速（单位：ERPM）
 *  - motor_last_feedback_ms: 上次电机反馈时间（ms）
 */
typedef struct
{
    uint32_t timestamp_ms;            /* 时间戳（ms） */
    uint8_t online_mask;              /* 在线掩码 */
    uint32_t system_events;           /* 系统事件位域 */
    uint32_t can_error_mask;          /* CAN 错误掩码 */
    uint8_t pc_command;               /* PC 命令 */
    uint8_t motor_feedback_mask;      /* 电机反馈掩码 */
    uint32_t motor_tx_ok_count;       /* 发送成功计数 */
    uint32_t motor_tx_failed_count;   /* 发送失败计数 */
    float motor_target_erpm[4];       /* 目标电机转速（ERPM） */
    float motor_measured_speed_mps[4]; /* 实际轮速（m/s），电机 ID 1-4 */
    uint32_t motor_last_feedback_ms;  /* 上次反馈时间（ms） */
} app_pc_health_t;

/*
 * 以下为对外接口函数声明，均在 app_messages.c 中实现。
 * 为每个函数提供简短中文说明（参数和用途）。
 */

/**
 * 初始化消息子系统（CAN/UART parser 状态等）。
 */
void AppMessages_Init(void);

/**
 * 处理接收到的 CAN 帧。
 * 参数：
 *  - can: 指向所用的 CAN 总线句柄
 *  - can_id: CAN 标识符
 *  - is_extended_id: 是否为扩展 ID
 *  - data: 数据指针
 *  - length: 数据长度
 */
void AppMessages_OnCanRx(bsp_can_t *can, uint32_t can_id, bool is_extended_id,
                         const uint8_t *data, uint8_t length);

/**
 * 处理 CAN 错误事件。
 */
void AppMessages_OnCanError(bsp_can_t *can, uint32_t error_flags);

/**
 * 处理来自 IMU（串口）的接收数据。
 */
void AppMessages_OnImuRx(bsp_uart_t *uart, const uint8_t *data, uint16_t len);

/**
 * 处理来自 PC 的串口数据（包含命令、限幅设置等）。
 */

/**
 * 处理来自 DT35 传感器的串口数据。
 */
void AppMessages_OnDt35Rx(bsp_uart_t *uart, const uint8_t *data, uint16_t len);

/**
 * 处理来自另一个 MCU（Dual MCU）的串口数据。
 */
void AppMessages_OnDualMcuRx(bsp_uart_t *uart, const uint8_t *data, uint16_t len);
void AppMessages_OnRemoteControlRx(bsp_uart_t *uart, const uint8_t *data, uint16_t len);

/* The dual-MCU motion owner is selected by APP_MESSAGES_ENABLE_UPPER_CONTROL. */
bool AppMessages_IsUpperControlEnabled(void);

/**
 * 解析 DT35 帧数据到内部结构。
 * 返回：是否解析成功。
 */
bool AppMessages_ParseDt35(const uint8_t *data, uint16_t len, uint32_t timestamp_ms);

/**
 * 解析双 MCU 的控制帧（来自远程控制器等）。
 */
bool AppMessages_ParseDualControl(const uint8_t *frame, uint16_t len);

/**
 * 解析双 MCU 的状态帧。
 */
bool AppMessages_ParseDualStatus(const uint8_t *frame, uint16_t len, uint32_t timestamp_ms);

/**
 * 检查消息超时（例如 PC/IMU/DT35 等超时处理）。
 */
void AppMessages_CheckTimeout(uint32_t timestamp_ms);
uint8_t AppMessages_GetOnlineMask(void);

/**
 * 获取最近解析到的 DT35 采样。
 */
bool AppMessages_GetDt35(app_dt35_sample_t *sample);

/**
 * 获取最近解析到的双 MCU 控制命令。
 */
bool AppMessages_GetDualControl(app_dual_control_t *control);
app_dual_control_freshness_t AppMessages_GetDualControlFreshness(uint32_t timestamp_ms);
bool AppMessages_GetRemoteControl(app_remote_control_t *control);
app_remote_control_freshness_t AppMessages_GetRemoteControlFreshness(uint32_t timestamp_ms);

/**
 * 获取最近解析到的双 MCU 状态。
 */
bool AppMessages_GetDualStatus(app_dual_status_t *status);

/**
 * 构建发送到双 MCU 的控制帧。
 */
uint16_t AppMessages_BuildDualControl(const app_dual_control_t *control,
                                      uint8_t sequence, uint8_t *frame,
                                      uint16_t capacity);

/**
 * 构建双 MCU 的状态帧以发送给对端。
 */
uint16_t AppMessages_BuildDualStatus(const app_dual_status_t *status,
                                     uint8_t sequence, uint8_t *frame,
                                     uint16_t capacity);

/**
 * 发送双 MCU 控制帧（封装并写入 UART）。
 */
bool AppMessages_SendDualControl(bsp_uart_t *uart, const app_dual_control_t *control);
bool AppMessages_SendDualStatus(bsp_uart_t *uart, const app_dual_status_t *status);

#endif /* APP_MESSAGES_H */
