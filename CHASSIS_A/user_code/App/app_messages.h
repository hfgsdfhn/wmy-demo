#ifndef APP_MESSAGES_H
#define APP_MESSAGES_H

#include <stdint.h>

#define APP_CAN_DATA_MAX_SIZE             8U
#define APP_COMM_PAYLOAD_MAX_SIZE         64U

typedef enum
{
    APP_CAN_BUS_1 = 0,
    APP_CAN_BUS_2,
    APP_CAN_BUS_3,
    APP_CAN_BUS_COUNT
} app_can_bus_t;            //三路CAN总线，分别对应CAN1、CAN2、CAN3

typedef enum
{
    APP_COMM_SOURCE_DT35 = 0,
    APP_COMM_SOURCE_MAIN_CONTROLLER,
    APP_COMM_SOURCE_USB,
    APP_COMM_SOURCE_DM_IMU,
    APP_COMM_SOURCE_COUNT
} app_comm_source_t;        //通信源，分别对应DT35、主控、USB、DM_IMU

typedef struct
{
    uint32_t timestamp_ms;
    uint32_t identifier;
    app_can_bus_t bus;
    uint8_t is_extended_id;
    uint8_t length;
    uint8_t data[APP_CAN_DATA_MAX_SIZE];
} app_can_frame_t;          //CAN帧结构体，包含时间戳、ID、总线号、是否扩展ID、数据长度和数据内容

typedef struct
{
    uint32_t sequence;
    uint32_t timestamp_ms;
    float velocity_x_mps;
    float velocity_y_mps;
    float yaw_rate_radps;
    float motor_velocity_x_mps;
    float motor_velocity_y_mps;
    float motor_yaw_rate_radps;
    uint8_t valid;
} app_chassis_command_t;    //底盘控制命令结构体，包含序列号、时间戳、x方向速度、y方向速度、偏航角速度和有效性标志

typedef struct
{
    uint32_t sequence;
    uint32_t timestamp_ms;
    float rod_angle_rad[4];
    float rear_wheel_speed_radps[2];
    uint32_t control_flags;
    uint8_t valid;
} app_mechanism_command_t;  //机构控制命令结构体，包含序列号、时间戳、四个杆角度、两个后轮速度、控制标志和有效性标志

typedef struct
{
    uint32_t sequence;
    float position_x_m;
    float position_y_m;
    float yaw_rad;
    float target_speed_mps;
    uint32_t flags;
} app_trajectory_point_t;   //轨迹点结构体，包含序列号、x位置、y位置、偏航角、目标速度和标志位

typedef struct
{
    uint32_t timestamp_ms;
    app_comm_source_t source;
    uint16_t length;
    uint8_t data[APP_COMM_PAYLOAD_MAX_SIZE];
} app_comm_packet_t;        //通信数据包结构体，包含时间戳、数据源、数据长度和数据内容

typedef struct
{
    uint32_t sequence;
    uint32_t timestamp_ms;
    uint32_t action_id;
    float parameter[4];
} app_action_command_t;     //动作控制命令结构体，包含序列号、时间戳、动作ID和四个参数

#endif /* APP_MESSAGES_H */
