#include "PathFollower.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "path.h"

#define PATH_FOLLOWER_PI                    (3.14159265358979323846f) /* 圆周率 */
#define PATH_FOLLOWER_TWO_PI                (2.0f * PATH_FOLLOWER_PI) /* 一周弧度 */
#define PATH_FOLLOWER_LOOKAHEAD_MM          (400.0f) /* 路径前视距离，单位 mm */
#define PATH_FOLLOWER_LATERAL_KP            (2.0f) /* 横向误差比例系数 */
#define PATH_FOLLOWER_YAW_KP                (3.0f) /* 航向误差比例系数 */
#define PATH_FOLLOWER_MAX_LINEAR_SPEED_MPS  (1.0f) /* 最大线速度，单位 m/s */
#define PATH_FOLLOWER_MAX_ANGULAR_SPEED_RADPS (2.0f) /* 最大角速度，单位 rad/s */
#define PATH_FOLLOWER_FINISH_DISTANCE_MM    (50.0f) /* 路径完成距离阈值，单位 mm */

typedef struct
{
    const PathPoint *path;
    uint16_t size;
    bool finished;
} PathFollowerState;

static PathFollowerState g_path_follower;

static float PathFollower_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static float PathFollower_NormalizeAngle(float angle)
{
    while (angle > PATH_FOLLOWER_PI)
    {
        angle -= PATH_FOLLOWER_TWO_PI;
    }
    while (angle < -PATH_FOLLOWER_PI)
    {
        angle += PATH_FOLLOWER_TWO_PI;
    }
    return angle;
}

static void PathFollower_WriteZero(float *vx_body, float *vy_body, float *omega)
{
    if (vx_body != NULL)
    {
        *vx_body = 0.0f;
    }
    if (vy_body != NULL)
    {
        *vy_body = 0.0f;
    }
    if (omega != NULL)
    {
        *omega = 0.0f;
    }
}

static uint16_t PathFollower_FindNearest(float robot_x, float robot_y)
{
    uint16_t index;
    uint16_t nearest = 0U;
    float minimum_distance_squared = 0.0f;

    for (index = 0U; index < g_path_follower.size; index++)
    {
        float dx = robot_x - g_path_follower.path[index].x;
        float dy = robot_y - g_path_follower.path[index].y;
        float distance_squared = dx * dx + dy * dy;

        if ((index == 0U) || (distance_squared < minimum_distance_squared))
        {
            minimum_distance_squared = distance_squared;
            nearest = index;
        }
    }
    return nearest;
}

static uint16_t PathFollower_FindLookahead(uint16_t nearest)
{
    uint16_t index;
    float lookahead_s = g_path_follower.path[nearest].s + PATH_FOLLOWER_LOOKAHEAD_MM;

    for (index = nearest; index < g_path_follower.size; index++)
    {
        if (g_path_follower.path[index].s >= lookahead_s)
        {
            return index;
        }
    }
    return (uint16_t)(g_path_follower.size - 1U);
}

void PathFollower_Init(void)
{
    g_path_follower.path = NULL;
    g_path_follower.size = 0U;
    g_path_follower.finished = false;
}

void PathFollower_SetDefaultPath(void)
{
    g_path_follower.path = path;
    g_path_follower.size = PATH_SIZE;
    g_path_follower.finished = false;
}

/**
 * @brief 路径点跟踪
 *        基于速度跟踪曲线
 * @param robot_x 
 * @param robot_y 
 * @param robot_yaw 
 * @param target_yaw 
 * @param vx_body 
 * @param vy_body 
 * @param omega 
 */
void PathFollower_Update(float robot_x,
                         float robot_y,
                         float robot_yaw,
                         float target_yaw,
                         float *vx_body,
                         float *vy_body,
                         float *omega)
{
    const PathPoint *target;
    const PathPoint *endpoint;
    uint16_t nearest;
    uint16_t lookahead;
    float tangent_x;
    float tangent_y;
    float normal_x;
    float normal_y;
    float lateral_error_m;
    float world_vx;
    float world_vy;
    float speed;
    float cosine;
    float sine;
    float dx;
    float dy;

    PathFollower_WriteZero(vx_body, vy_body, omega);
    if ((vx_body == NULL) || (vy_body == NULL) || (omega == NULL)
        || (g_path_follower.path == NULL) || (g_path_follower.size == 0U)
        || g_path_follower.finished)
    {
        return;
    }

    endpoint = &g_path_follower.path[g_path_follower.size - 1U];
    dx = robot_x - endpoint->x;
    dy = robot_y - endpoint->y;
    if ((dx * dx + dy * dy)
        <= (PATH_FOLLOWER_FINISH_DISTANCE_MM * PATH_FOLLOWER_FINISH_DISTANCE_MM))
    {
        g_path_follower.finished = true;
        return;
    }

    nearest = PathFollower_FindNearest(robot_x, robot_y);
    lookahead = PathFollower_FindLookahead(nearest);
    target = &g_path_follower.path[lookahead];

    tangent_x = cosf(target->theta);
    tangent_y = sinf(target->theta);
    normal_x = -tangent_y;
    normal_y = tangent_x;
    lateral_error_m = ((robot_x - target->x) * normal_x
                     + (robot_y - target->y) * normal_y) * 0.001f;

    world_vx = target->velocity * tangent_x
             - PATH_FOLLOWER_LATERAL_KP * lateral_error_m * normal_x;
    world_vy = target->velocity * tangent_y
             - PATH_FOLLOWER_LATERAL_KP * lateral_error_m * normal_y;
    speed = sqrtf(world_vx * world_vx + world_vy * world_vy);
    if (speed > PATH_FOLLOWER_MAX_LINEAR_SPEED_MPS)
    {
        world_vx *= PATH_FOLLOWER_MAX_LINEAR_SPEED_MPS / speed;
        world_vy *= PATH_FOLLOWER_MAX_LINEAR_SPEED_MPS / speed;
    }

    cosine = cosf(robot_yaw);
    sine = sinf(robot_yaw);
    *vx_body = cosine * world_vx + sine * world_vy;
    *vy_body = -sine * world_vx + cosine * world_vy;
    *omega = PathFollower_Clamp(
        PATH_FOLLOWER_YAW_KP * PathFollower_NormalizeAngle(target_yaw - robot_yaw),
        -PATH_FOLLOWER_MAX_ANGULAR_SPEED_RADPS,
        PATH_FOLLOWER_MAX_ANGULAR_SPEED_RADPS);
}

bool PathFollower_IsFinished(void)
{
    return g_path_follower.finished;
}
