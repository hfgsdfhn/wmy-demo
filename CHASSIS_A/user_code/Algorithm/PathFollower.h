#ifndef PATH_FOLLOWER_H
#define PATH_FOLLOWER_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 世界坐标系约定：x 轴前方、y 轴左方、yaw 逆时针为正；角度单位均为 rad。
 * 路径点按车辆行驶顺序排列，至少需要两个点。
 */
typedef struct
{
    float position_x_m;       /* 世界坐标 x，单位 m */
    float position_y_m;       /* 世界坐标 y，单位 m */
    float yaw_rad;            /* 期望航向，单位 rad；仅 use_path_yaw 为真时使用 */
    float target_speed_mps;   /* 此点期望速度，<= 0 时使用 cruise_speed_mps */
} path_follower_point_t;

/* 底盘定位模块在每个控制周期提供的位姿和速度。 */
typedef struct
{
    float position_x_m;       /* 世界坐标 x，单位 m */
    float position_y_m;       /* 世界坐标 y，单位 m */
    float yaw_rad;            /* 当前航向，单位 rad */
    float linear_speed_mps;   /* 当前线速度模长，用于计算动态前视距离，单位 m/s */
} path_follower_pose_t;

/* 底盘运动学类型。 */
typedef enum
{
    PATH_FOLLOWER_HOLONOMIC = 0,   /* 全向底盘，可输出 vx、vy、wz */
    PATH_FOLLOWER_NONHOLONOMIC     /* 差速/阿克曼底盘，只输出 vx、wz */
} path_follower_mode_t;

/* 路径跟踪器参数，建议由实际场地和底盘能力整定。 */
typedef struct
{
    path_follower_mode_t mode;          /* 底盘运动学类型 */
    float minimum_lookahead_m;          /* 最小前视距离，单位 m */
    float lookahead_time_s;             /* 动态前视时间：L = Lmin + |v| * T，单位 s */
    float cruise_speed_mps;             /* 路径点未给速度时采用的巡航速度，单位 m/s */
    float maximum_speed_mps;            /* 平动速度上限，单位 m/s */
    float maximum_yaw_rate_radps;       /* 角速度上限，单位 rad/s */
    float position_gain;                /* 位置误差比例增益，单位 1/s */
    float heading_gain;                 /* 航向误差比例增益，单位 1/s */
    float maximum_deceleration_mps2;    /* 到终点的制动减速度，单位 m/s^2 */
    float finish_distance_m;            /* 终点位置判定阈值，单位 m */
    float finish_yaw_error_rad;         /* 终点航向判定阈值，单位 rad */
    bool   use_path_yaw;                  /* true：跟随路径点 yaw；false：跟随路径切线 */
} path_follower_config_t;

/* 每个控制周期输出的车体速度指令和调试信息。 */
typedef struct
{
    float velocity_x_mps;       /* 车体 x 轴速度，前方为正，单位 m/s */
    float velocity_y_mps;       /* 车体 y 轴速度，左方为正，单位 m/s */
    float yaw_rate_radps;       /* 车体角速度，逆时针为正，单位 rad/s */
    float target_x_m;           /* 本周期前视目标点的世界 x 坐标，单位 m */
    float target_y_m;           /* 本周期前视目标点的世界 y 坐标，单位 m */
    float target_yaw_rad;       /* 本周期目标航向，单位 rad */
    float cross_track_error_m;  /* 横向偏差，路径左侧为正，单位 m */
    uint16_t target_segment;    /* 前视目标所在路径段编号 */
    bool finished;              /* 已到达终点时为 true，速度输出为零 */
} path_follower_output_t;

/* 路径跟踪器运行状态，调用方应为每条路径保留一个实例。 */
typedef struct
{
    const path_follower_point_t *path;  /* 外部提供的路径点数组，本模块不复制数组 */
    uint16_t point_count;               /* 路径点数量 */
    uint16_t current_segment;           /* 当前最近段，保证跟踪进度不回退 */
    path_follower_config_t config;      /* 当前参数副本 */
    bool initialized;                   /* 初始化成功标志 */
    bool finished;                      /* 终点到达后的锁存标志 */
} path_follower_t;

/**
 * @brief 写入一组适合低速调试的默认参数。
 * @param config 待写入的参数结构体。
 */
void PathFollower_DefaultConfig(path_follower_config_t *config);

/**
 * @brief 绑定路径点并初始化路径跟踪器。
 * @param follower 跟踪器实例。
 * @param path 按行驶顺序排列的路径点数组。
 * @param point_count 路径点数量，至少为 2。
 * @param config 已配置的跟踪参数。
 * @retval true 初始化成功。
 * @retval false 输入指针、点数或参数不合法。
 */
bool PathFollower_Init(path_follower_t *follower,
                       const path_follower_point_t *path,
                       uint16_t point_count,
                       const path_follower_config_t *config);

/**
 * @brief 从路径首段重新开始跟踪，不修改绑定的路径和参数。
 * @param follower 跟踪器实例。
 */
void PathFollower_Reset(path_follower_t *follower);

/**
 * @brief 执行一次路径跟踪计算。
 * @param follower 已初始化的跟踪器实例。
 * @param pose 当前定位位姿。
 * @param output 输出车体速度指令及调试数据。
 * @retval true 本次计算有效。
 * @retval false 输入无效或跟踪器尚未初始化。
 */
bool PathFollower_Step(path_follower_t *follower,
                       const path_follower_pose_t *pose,
                       path_follower_output_t *output);

#endif /* PATH_FOLLOWER_H */
