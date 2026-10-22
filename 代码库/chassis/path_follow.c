#include "path_follow.h"
#include <math.h>

/* path.h 中 PathPoint 的字段顺序和单位固定为题目给出的格式。 */
typedef struct
{
    float x;
    float y;
    float theta;
    float curvature;
    float s;
    float velocity;
} path_point_t;

static float PathFollow_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float PathFollow_Clamp(float value, float limit)
{
    limit = PathFollow_Abs(limit);
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static float PathFollow_WrapPi(float angle)
{
    while (angle >= 3.1415926f)
    {
        angle -= 6.2831852f;
    }
    while (angle < -3.1415926f)
    {
        angle += 6.2831852f;
    }
    return angle;
}

/**
 * @brief 路径规划初始化
 * 
 * @param follow    路径规划结构体
 * @param path      路径点
 * @param path_size 路径点大小
 * @param lookahead_mm 前视距离
 * @param normal_gain   法相速度增益
 * @param normal_speed_limit 法向速度上限
 * @param yaw_gain          yaw增益
 * @param yaw_speed_limit   yaw速度上线
 * @param finish_distance_mm 终点停止距离
 * @return true 
 * @return false 
 */
bool PathFollow_Init(path_follow_t *follow,
                     const void *path,
                     uint16_t path_size,
                     float lookahead_mm,
                     float normal_gain,
                     float normal_speed_limit,
                     float yaw_gain,
                     float yaw_speed_limit,
                     float finish_distance_mm)
{
    if ((follow == 0) || (path == 0) || (path_size == 0U) ||
        (lookahead_mm < 0.0f) || (normal_gain < 0.0f) ||
        (normal_speed_limit < 0.0f) || (yaw_gain < 0.0f) ||
        (yaw_speed_limit < 0.0f) || (finish_distance_mm < 0.0f))
    {
        return false;
    }

    follow->path = path;
    follow->path_size = path_size;
    follow->lookahead_mm = lookahead_mm;
    follow->normal_gain = normal_gain;
    follow->normal_speed_limit = normal_speed_limit;
    follow->yaw_gain = yaw_gain;
    follow->yaw_speed_limit = yaw_speed_limit;
    follow->finish_distance_mm = finish_distance_mm;
    PathFollow_Reset(follow);
    return true;
}

void PathFollow_Reset(path_follow_t *follow)
{
    if (follow != 0)
    {
        follow->nearest_index = 0U;
        follow->finished = false;
    }
}

/**
 * @brief 根据软件生成的坐标追踪曲线
 * 
 * @param follow    曲线
 * @param x_m 
 * @param y_m 
 * @param yaw_rad 
 * @param speed 
 * @return true 
 * @return false 
 */
bool PathFollow_Calc(path_follow_t *follow,
                     float x_m,
                     float y_m,
                     float yaw_rad,
                     path_follow_speed_t *speed)
{
    const path_point_t *points;
    const path_point_t *nearest;
    const path_point_t *target;
    uint16_t index;
    uint16_t target_index;
    float nearest_distance;
    float distance;
    float target_s;
    float dx;
    float dy;
    float tangent_x;
    float tangent_y;
    float normal_x;
    float normal_y;
    float normal_error;
    float normal_speed;
    float tangent_speed;
    float yaw_error;

    if ((follow == 0) || (follow->path == 0) ||
        (follow->path_size == 0U) || (speed == 0))
    {
        return false;
    }

    if (follow->finished)
    {
        speed->vx = 0.0f;
        speed->vy = 0.0f;
        speed->vw = 0.0f;
        return true;
    }

    /*------------------------ 搜索最近距离 ------------------------*/
    points = (const path_point_t *)follow->path;
    nearest = &points[follow->nearest_index];
    nearest_distance = 1.0e30f;
    for (index = follow->nearest_index; index < follow->path_size; index++)
    {
        dx = x_m - points[index].x * 0.001f;
        dy = y_m - points[index].y * 0.001f;
        distance = dx * dx + dy * dy;
        if (distance < nearest_distance)
        {
            nearest_distance = distance;
            follow->nearest_index = index;
            nearest = &points[index];
        }
    }

    target_s = nearest->s + follow->lookahead_mm;   //根据前视距离选择目标点
    target_index = follow->nearest_index;
    while ((target_index + 1U < follow->path_size) &&
           (points[target_index + 1U].s < target_s))
    {
        target_index++;
    }
    target = &points[target_index];

    /* 前视目标到达路径末点后停止该段路径追踪。 */
    if (target_index == follow->path_size - 1U)
    {
        follow->finished = true;
        speed->vx = 0.0f;
        speed->vy = 0.0f;
        speed->vw = 0.0f;
        return true;
    }

    tangent_x = cosf(target->theta);
    tangent_y = sinf(target->theta);
    normal_x = -tangent_y;
    normal_y = tangent_x;
    dx = x_m - nearest->x * 0.001f;
    dy = y_m - nearest->y * 0.001f;
    normal_error = dx * normal_x + dy * normal_y;
    normal_speed = PathFollow_Clamp(-follow->normal_gain * normal_error,
                                    follow->normal_speed_limit);
    tangent_speed = target->velocity;

    speed->vx = tangent_speed * tangent_x + normal_speed * normal_x;
    speed->vy = tangent_speed * tangent_y + normal_speed * normal_y;
    yaw_error = PathFollow_WrapPi(-yaw_rad);
    speed->vw = PathFollow_Clamp(follow->yaw_gain * yaw_error,
                                 follow->yaw_speed_limit);
    return true;
}
