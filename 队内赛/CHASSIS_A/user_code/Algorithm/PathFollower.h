#ifndef PATH_FOLLOWER_H
#define PATH_FOLLOWER_H /* 路径跟踪算法头文件保护宏 */

#include <stdbool.h>

/* 初始化 */
void PathFollower_Init(void);

/* 设置路径 */
void PathFollower_SetDefaultPath(void);

/*
 * 更新跟踪
 *
 * robot_x/y : mm
 * robot_yaw  : rad
 *
 * 输出:
 * vx_body : m/s
 * vy_body : m/s
 * omega   : rad/s
 */
void PathFollower_Update(float robot_x,
                         float robot_y,
                         float robot_yaw,
                         float target_yaw,
                         float *vx_body,
                         float *vy_body,
                         float *omega);

/* 是否完成 */
bool PathFollower_IsFinished(void);

#endif /* PATH_FOLLOWER_H */
