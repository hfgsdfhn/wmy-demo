#include "PathFollower.h"

#include <math.h>
#include <stddef.h>

#define PATH_TRACKER_PI        (3.14159265358979323846f)
#define PATH_TRACKER_TWO_PI    (2.0f * PATH_TRACKER_PI)

typedef struct
{
    const PathPoint *path;
    uint16_t size;
    uint16_t nearest_index;
    uint16_t target_index;
    bool finished;
    PathTrackerConfig config;
} PathTrackerState;

static PathTrackerState g_path_tracker;

static float PathTracker_Clamp(float value, float minimum, float maximum)
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

/* 归一化到 [-pi, pi]，使车体始终选择最短 yaw 旋转方向。 */
static float PathTracker_NormalizeAngle(float angle)
{
    while (angle > PATH_TRACKER_PI)
    {
        angle -= PATH_TRACKER_TWO_PI;
    }
    while (angle < -PATH_TRACKER_PI)
    {
        angle += PATH_TRACKER_TWO_PI;
    }
    return angle;
}

static void PathTracker_WriteZero(float *vx_body, float *vy_body, float *omega)
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

/*
 * 位置误差已转换为 m，因此 PID 输出为 m/s。
 * 对积分项限幅，避免速度饱和时积分继续累积造成冲出路径。
 */
static float PathTracker_PIDUpdate(PID_t *pid, float error, float dt_s)
{
    float derivative;
    float output;
    float integral_limit;

    pid->integral += error * dt_s;
    if (pid->ki != 0.0f)
    {
        integral_limit = (pid->output_limit > 0.0f)
                       ? fabsf(pid->output_limit / pid->ki) : 100.0f;
        pid->integral = PathTracker_Clamp(pid->integral,
                                          -integral_limit, integral_limit);
    }
    derivative = (error - pid->last_error) / dt_s;
    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    pid->last_error = error;

    if (pid->output_limit > 0.0f)
    {
        output = PathTracker_Clamp(output, -pid->output_limit, pid->output_limit);
    }
    return output;
}

/* 遍历每个路径点，按欧氏距离平方找出最近点，避免无意义的 sqrtf 开销。 */
static uint16_t PathTracker_FindNearestIndex(float robot_x, float robot_y)
{
    uint16_t index;
    uint16_t nearest_index = 0U;
    float dx;
    float dy;
    float distance_squared;
    float minimum_distance_squared = 0.0f;

    for (index = 0U; index < g_path_tracker.size; index++)
    {
        dx = robot_x - g_path_tracker.path[index].x;
        dy = robot_y - g_path_tracker.path[index].y;
        distance_squared = dx * dx + dy * dy;
        if ((index == 0U) || (distance_squared < minimum_distance_squared))
        {
            minimum_distance_squared = distance_squared;
            nearest_index = index;
        }
    }
    return nearest_index;
}

/* 使用 s 选取前视目标，而非直接以最近点作为控制目标。 */
static uint16_t PathTracker_FindTargetIndex(uint16_t nearest_index)
{
    uint16_t index;
    float target_s = g_path_tracker.path[nearest_index].s
                   + g_path_tracker.config.lookahead_distance_mm;

    for (index = nearest_index; index < g_path_tracker.size; index++)
    {
        if (g_path_tracker.path[index].s >= target_s)
        {
            return index;
        }
    }
    return (uint16_t)(g_path_tracker.size - 1U);
}

void PathTracker_Init(void)
{
    g_path_tracker.path = NULL;
    g_path_tracker.size = 0U;
    g_path_tracker.nearest_index = 0U;
    g_path_tracker.target_index = 0U;
    g_path_tracker.finished = false;

    /* 适合路径点间隔约 50 mm、100 Hz 初始调试的保守默认值。 */
    g_path_tracker.config.lookahead_distance_mm = 400.0f;
    g_path_tracker.config.max_linear_speed_mps = 1.0f;
    g_path_tracker.config.max_angular_speed_radps = 2.0f;
    g_path_tracker.config.yaw_kp = 3.0f;
    g_path_tracker.config.finish_distance_mm = 50.0f;
    g_path_tracker.config.finish_speed_mps = 0.03f;
    g_path_tracker.config.dt_s = 0.01f;
    g_path_tracker.config.normal_pid.kp = 2.0f;
    g_path_tracker.config.normal_pid.ki = 0.0f;
    g_path_tracker.config.normal_pid.kd = 0.05f;
    g_path_tracker.config.normal_pid.integral = 0.0f;
    g_path_tracker.config.normal_pid.last_error = 0.0f;
    g_path_tracker.config.normal_pid.output_limit = 0.5f;
}

void PathTracker_SetPath(const PathPoint *path, uint16_t size)
{
    g_path_tracker.path = path;
    g_path_tracker.size = (path != NULL) ? size : 0U;
    PathTracker_Reset();
}

void PathTracker_GetConfig(PathTrackerConfig *config)
{
    if (config != NULL)
    {
        *config = g_path_tracker.config;
    }
}

bool PathTracker_SetConfig(const PathTrackerConfig *config)
{
    if ((config == NULL) || (config->lookahead_distance_mm < 0.0f)
        || (config->max_linear_speed_mps <= 0.0f)
        || (config->max_angular_speed_radps <= 0.0f)
        || (config->finish_distance_mm <= 0.0f)
        || (config->finish_speed_mps < 0.0f) || (config->dt_s <= 0.0f))
    {
        return false;
    }

    g_path_tracker.config = *config;
    g_path_tracker.config.normal_pid.integral = 0.0f;
    g_path_tracker.config.normal_pid.last_error = 0.0f;
    return true;
}

void PathTracker_Reset(void)
{
    g_path_tracker.nearest_index = 0U;
    g_path_tracker.target_index = 0U;
    g_path_tracker.finished = false;
    g_path_tracker.config.normal_pid.integral = 0.0f;
    g_path_tracker.config.normal_pid.last_error = 0.0f;
}

void PathTracker_Update(float robot_x, float robot_y,
                        float robot_yaw, float target_yaw,
                        float *vx_body, float *vy_body, float *omega)
{
    const PathPoint *target;
    const PathPoint *last_point;
    float tangent_x;
    float tangent_y;
    float normal_x;
    float normal_y;
    float error_m;
    float normal_speed;
    float velocity_world_x;
    float velocity_world_y;
    float velocity_norm;
    float yaw_error;
    float cosine;
    float sine;
    float endpoint_dx;
    float endpoint_dy;
    float endpoint_distance_mm;

    PathTracker_WriteZero(vx_body, vy_body, omega);
    if ((vx_body == NULL) || (vy_body == NULL) || (omega == NULL)
        || (g_path_tracker.path == NULL) || (g_path_tracker.size == 0U)
        || g_path_tracker.finished)
    {
        return;
    }

    last_point = &g_path_tracker.path[g_path_tracker.size - 1U];
    endpoint_dx = robot_x - last_point->x;
    endpoint_dy = robot_y - last_point->y;
    endpoint_distance_mm = sqrtf(endpoint_dx * endpoint_dx + endpoint_dy * endpoint_dy);

    /* 路径终点期望速度应离线规划为零；同时满足位置和速度条件才结束。 */
    if ((endpoint_distance_mm <= g_path_tracker.config.finish_distance_mm)
        && (fabsf(last_point->velocity) <= g_path_tracker.config.finish_speed_mps))
    {
        g_path_tracker.finished = true;
        return;
    }

    g_path_tracker.nearest_index = PathTracker_FindNearestIndex(robot_x, robot_y);
    g_path_tracker.target_index = PathTracker_FindTargetIndex(g_path_tracker.nearest_index);
    target = &g_path_tracker.path[g_path_tracker.target_index];

    tangent_x = cosf(target->theta);
    tangent_y = sinf(target->theta);
    normal_x = -tangent_y;
    normal_y = tangent_x;

    /* dx、dy 最初为 mm，此处乘 0.001 转换到 m 供法向 PID 使用。 */
    error_m = ((robot_x - target->x) * normal_x
             + (robot_y - target->y) * normal_y) * 0.001f;
    normal_speed = PathTracker_PIDUpdate(&g_path_tracker.config.normal_pid,
                                         error_m, g_path_tracker.config.dt_s);

    /* 按要求：法向修正速度为 -normal_speed * normal。 */
    velocity_world_x = target->velocity * tangent_x - normal_speed * normal_x;
    velocity_world_y = target->velocity * tangent_y - normal_speed * normal_y;

    /* 合速度超限时等比例缩放，保持世界坐标系速度方向不变。 */
    velocity_norm = sqrtf(velocity_world_x * velocity_world_x
                        + velocity_world_y * velocity_world_y);
    if (velocity_norm > g_path_tracker.config.max_linear_speed_mps)
    {
        velocity_world_x *= g_path_tracker.config.max_linear_speed_mps / velocity_norm;
        velocity_world_y *= g_path_tracker.config.max_linear_speed_mps / velocity_norm;
    }

    /* 世界系速度旋转到当前车体系。 */
    cosine = cosf(robot_yaw);
    sine = sinf(robot_yaw);
    *vx_body = cosine * velocity_world_x + sine * velocity_world_y;
    *vy_body = -sine * velocity_world_x + cosine * velocity_world_y;

    /* 固定 target_yaw，与路径 theta 无关。 */
    yaw_error = PathTracker_NormalizeAngle(target_yaw - robot_yaw);
    *omega = PathTracker_Clamp(g_path_tracker.config.yaw_kp * yaw_error,
                               -g_path_tracker.config.max_angular_speed_radps,
                               g_path_tracker.config.max_angular_speed_radps);
}

bool PathTracker_IsFinished(void)
{
    return g_path_tracker.finished;
}

uint16_t PathTracker_GetNearestIndex(void)
{
    return g_path_tracker.nearest_index;
}

uint16_t PathTracker_GetTargetIndex(void)
{
    return g_path_tracker.target_index;
}
