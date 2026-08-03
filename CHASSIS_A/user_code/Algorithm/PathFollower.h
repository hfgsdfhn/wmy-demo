#ifndef PATH_FOLLOWER_H
#define PATH_FOLLOWER_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 坐标系约定：世界坐标系 X、Y 与 yaw 满足右手系，yaw 逆时针为正。
 * 路径 x、y、s 的单位是 mm；速度相关量均使用 m、s、rad。
 */
typedef struct
{
    float x;          /* 路径 X 坐标，单位 mm */
    float y;          /* 路径 Y 坐标，单位 mm */
    float theta;      /* 路径切线方向角，单位 rad */
    float curvature;  /* 曲率，单位 1/m；当前算法预留，不直接使用 */
    float s;          /* 从起点累计路径长度，单位 mm，必须单调不减 */
    float velocity;   /* 当前点期望切向速度，单位 m/s */
} PathPoint;

/* 独立的法向速度 PID。误差输入单位为 m，输出单位为 m/s。 */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float last_error;
    float output_limit; /* PID 输出绝对值上限，单位 m/s；<= 0 表示不限幅 */
} PID_t;

/* 跟踪器可调参数。 */
typedef struct
{
    float lookahead_distance_mm;   /* 固定前视距离，建议 300~500 mm */
    float max_linear_speed_mps;    /* 平移合速度上限，单位 m/s */
    float max_angular_speed_radps; /* 角速度上限，单位 rad/s */
    float yaw_kp;                  /* 固定 yaw 的比例增益，单位 1/s */
    float finish_distance_mm;      /* 终点距离阈值，单位 mm */
    float finish_speed_mps;        /* 终点路径速度阈值，单位 m/s */
    float dt_s;                    /* 调用周期，单位 s */
    PID_t normal_pid;              /* 法向误差 PID */
} PathTrackerConfig;

/* 初始化跟踪器和默认参数。默认控制周期为 0.01 s，即 100 Hz。 */
void PathTracker_Init(void);

/* 绑定离线路径数组。本模块仅保存指针，路径数组在跟踪期间必须保持有效。 */
void PathTracker_SetPath(const PathPoint *path, uint16_t size);

/* 获取和设置参数。SetConfig 会清除 PID 历史状态。 */
void PathTracker_GetConfig(PathTrackerConfig *config);
bool PathTracker_SetConfig(const PathTrackerConfig *config);

/* 重新执行当前路径时，清除 PID 状态和完成标志。 */
void PathTracker_Reset(void);

/*
 * 每个控制周期调用一次。
 * robot_x、robot_y 单位 mm；robot_yaw、target_yaw 单位 rad；
 * vx_body、vy_body 单位 m/s；omega 单位 rad/s。
 */
void PathTracker_Update(float robot_x, float robot_y,
                        float robot_yaw, float target_yaw,
                        float *vx_body, float *vy_body, float *omega);

/* 查询完成状态以及用于调试的最近点、前视点索引。 */
bool PathTracker_IsFinished(void);
uint16_t PathTracker_GetNearestIndex(void);
uint16_t PathTracker_GetTargetIndex(void);

#endif /* PATH_FOLLOWER_H */
