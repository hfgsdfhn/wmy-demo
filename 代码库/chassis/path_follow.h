#ifndef PATH_FOLLOW_H
#define PATH_FOLLOW_H

#include <stdbool.h>
#include <stdint.h>

/* 路径跟随器输出，线速度单位 m/s，角速度单位 rad/s。 */
typedef struct
{
    float vx;
    float vy;
    float vw;
} path_follow_speed_t;

typedef struct
{
    const void *path;
    uint16_t path_size;
    uint16_t nearest_index;
    float lookahead_mm;
    float normal_gain;
    float normal_speed_limit;
    float yaw_gain;
    float yaw_speed_limit;
    float finish_distance_mm;
    bool finished;
} path_follow_t;

bool PathFollow_Init(path_follow_t *follow,
                     const void *path,
                     uint16_t path_size,
                     float lookahead_mm,
                     float normal_gain,
                     float normal_speed_limit,
                     float yaw_gain,
                     float yaw_speed_limit,
                     float finish_distance_mm);

void PathFollow_Reset(path_follow_t *follow);

bool PathFollow_Calc(path_follow_t *follow,
                     float x_m,       /* 当前世界坐标 x，单位 m。 */
                     float y_m,       /* 当前世界坐标 y，单位 m。 */
                     float yaw_rad,   /* 当前底盘 yaw，单位 rad。 */
                     path_follow_speed_t *speed);

#endif
