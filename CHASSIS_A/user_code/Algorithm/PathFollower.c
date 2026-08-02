#include "PathFollower.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define PATH_FOLLOWER_PI                 3.14f
#define PATH_FOLLOWER_TWO_PI             (2.0f * PATH_FOLLOWER_PI)
#define PATH_FOLLOWER_EPSILON            0.0001f

typedef struct
{
    float x;                     /* 最近投影点世界 x 坐标 */
    float y;                     /* 最近投影点世界 y 坐标 */
    float tangent_x;             /* 投影所在路径段的单位切线 x 分量 */
    float tangent_y;             /* 投影所在路径段的单位切线 y 分量 */
    float progress;              /* 投影点在线段内的比例，范围 [0, 1] */
    float remaining_distance;    /* 从投影点到终点的路径弧长，单位 m */
    uint16_t segment;            /* 投影所在路径段编号 */
} path_follower_projection_t;

/* 将数值限制在区间内。 */
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

/* 将角度归一化到 [-pi, pi]，保证航向误差取最短转向方向。 */
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

/* 计算平面两点间的欧氏距离。 */
static float PathFollower_Distance(float x0, float y0, float x1, float y1)
{
    float dx = x1 - x0;
    float dy = y1 - y0;

    return sqrtf(dx * dx + dy * dy);
}

/* 计算相邻路径点构成线段的长度。 */
static float PathFollower_SegmentLength(const path_follower_point_t *start,
                                        const path_follower_point_t *end)
{
    return PathFollower_Distance(start->position_x_m, start->position_y_m,
                                 end->position_x_m, end->position_y_m);
}

/* 初始化前检查所有会影响除零和限幅的参数。 */
static bool PathFollower_IsConfigValid(const path_follower_config_t *config)
{
    return (config != NULL)
        && (config->mode <= PATH_FOLLOWER_NONHOLONOMIC)
        && (config->minimum_lookahead_m > 0.0f)
        && (config->lookahead_time_s >= 0.0f)
        && (config->cruise_speed_mps > 0.0f)
        && (config->maximum_speed_mps > 0.0f)
        && (config->maximum_yaw_rate_radps > 0.0f)
        && (config->position_gain >= 0.0f)
        && (config->heading_gain >= 0.0f)
        && (config->maximum_deceleration_mps2 > 0.0f)
        && (config->finish_distance_m > 0.0f)
        && (config->finish_yaw_error_rad > 0.0f);
}

/* 从指定线段内的投影位置开始，累加到路径终点的弧长。 */
static float PathFollower_RemainingDistance(const path_follower_t *follower,
                                            uint16_t segment,
                                            float segment_progress)
{
    float remaining;
    uint16_t index;
    float length;

    length = PathFollower_SegmentLength(&follower->path[segment],
                                        &follower->path[segment + 1U]);
    remaining = (1.0f - segment_progress) * length;
    for (index = (uint16_t)(segment + 1U);
         index < (uint16_t)(follower->point_count - 1U); index++)
    {
        remaining += PathFollower_SegmentLength(&follower->path[index],
                                                &follower->path[index + 1U]);
    }
    return remaining;
}

/*
 * 在尚未经过的路径段中寻找离当前底盘最近的正交投影点。
 * current_segment 限制搜索起点，避免路径自交时跟踪进度跳回已走过的段。
 */
static void PathFollower_FindClosestProjection(
    const path_follower_t *follower, const path_follower_pose_t *pose,
    path_follower_projection_t *projection)
{
    uint16_t index;
    const path_follower_point_t *start;
    const path_follower_point_t *end;
    float dx;
    float dy;
    float length_squared;
    float progress;
    float x;
    float y;
    float distance_squared;
    float minimum_distance_squared = 0.0f;
    bool found = false;

    for (index = follower->current_segment;
         index < (uint16_t)(follower->point_count - 1U); index++)
    {
        start = &follower->path[index];
        end = &follower->path[index + 1U];
        dx = end->position_x_m - start->position_x_m;
        dy = end->position_y_m - start->position_y_m;
        length_squared = dx * dx + dy * dy;
        if (length_squared < PATH_FOLLOWER_EPSILON)
        {
            continue;
        }

        progress = ((pose->position_x_m - start->position_x_m) * dx
                    + (pose->position_y_m - start->position_y_m) * dy)
                 / length_squared;
        progress = PathFollower_Clamp(progress, 0.0f, 1.0f);
        x = start->position_x_m + progress * dx;
        y = start->position_y_m + progress * dy;
        distance_squared = (pose->position_x_m - x) * (pose->position_x_m - x)
                         + (pose->position_y_m - y) * (pose->position_y_m - y);

        if (!found || (distance_squared < minimum_distance_squared))
        {
            projection->x = x;
            projection->y = y;
            projection->tangent_x = dx / sqrtf(length_squared);
            projection->tangent_y = dy / sqrtf(length_squared);
            projection->progress = progress;
            projection->segment = index;
            minimum_distance_squared = distance_squared;
            found = true;
        }
    }

    if (!found)
    {
        projection->x = follower->path[follower->point_count - 1U].position_x_m;
        projection->y = follower->path[follower->point_count - 1U].position_y_m;
        projection->tangent_x = 1.0f;
        projection->tangent_y = 0.0f;
        projection->progress = 1.0f;
        projection->segment = (uint16_t)(follower->point_count - 2U);
    }
    projection->remaining_distance = PathFollower_RemainingDistance(
        follower, projection->segment, projection->progress);
}

/*
 * 从最近投影点开始沿路径弧长前进，得到动态前视点。
 * 前视距离超过剩余路径长度时，前视点固定为最终路径点。
 */

static void PathFollower_FindLookaheadPoint(
    const path_follower_t *follower, const path_follower_projection_t *projection,
    float lookahead_distance, path_follower_point_t *target,
    uint16_t *target_segment)
{
    uint16_t segment = projection->segment;
    float progress = projection->progress;
    float available_distance;
    float segment_length;
    float ratio;
    const path_follower_point_t *start;
    const path_follower_point_t *end;

    while (segment < (uint16_t)(follower->point_count - 1U))
    {
        start = &follower->path[segment];
        end = &follower->path[segment + 1U];
        segment_length = PathFollower_SegmentLength(start, end);
        if (segment_length < PATH_FOLLOWER_EPSILON)
        {
            segment++;
            progress = 0.0f;
            continue;
        }

        available_distance = (1.0f - progress) * segment_length;
        if (lookahead_distance <= available_distance)
        {
            ratio = progress + lookahead_distance / segment_length;
            target->position_x_m = start->position_x_m
                                 + ratio * (end->position_x_m - start->position_x_m);
            target->position_y_m = start->position_y_m
                                 + ratio * (end->position_y_m - start->position_y_m);
            target->yaw_rad = PathFollower_NormalizeAngle(start->yaw_rad
                              + ratio * PathFollower_NormalizeAngle(end->yaw_rad
                                                                     - start->yaw_rad));
            target->target_speed_mps = start->target_speed_mps
                                     + ratio * (end->target_speed_mps
                                                - start->target_speed_mps);
            *target_segment = segment;
            return;
        }

        lookahead_distance -= available_distance;
        segment++;
        progress = 0.0f;
    }

    *target = follower->path[follower->point_count - 1U];
    *target_segment = (uint16_t)(follower->point_count - 2U);
}

/* 根据路径点速度和可用制动距离生成速度参考值。 */
static float PathFollower_TargetSpeed(const path_follower_t *follower,
                                      const path_follower_point_t *target,
                                      float remaining_distance)
{
    float speed = target->target_speed_mps;
    float braking_speed;

    if (speed <= 0.0f)
    {
        speed = follower->config.cruise_speed_mps;
    }
    braking_speed = sqrtf(2.0f * follower->config.maximum_deceleration_mps2
                          * remaining_distance);
    speed = PathFollower_Clamp(speed, 0.0f, follower->config.maximum_speed_mps);
    return fminf(speed, braking_speed);
}

/* 写入默认参数，默认模式适用于麦克纳姆等全向底盘。 */
void PathFollower_DefaultConfig(path_follower_config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    config->mode = PATH_FOLLOWER_HOLONOMIC;
    config->minimum_lookahead_m = 0.30f;
    config->lookahead_time_s = 0.30f;
    config->cruise_speed_mps = 1.00f;
    config->maximum_speed_mps = 1.50f;
    config->maximum_yaw_rate_radps = 3.00f;
    config->position_gain = 1.50f;
    config->heading_gain = 3.00f;
    config->maximum_deceleration_mps2 = 1.00f;
    config->finish_distance_m = 0.05f;
    config->finish_yaw_error_rad = 0.10f;
    config->use_path_yaw = false;
}

/* 仅保存路径数组地址；调用方在跟踪结束前不得释放或改写该数组。 */
bool PathFollower_Init(path_follower_t *follower,
                       const path_follower_point_t *path,
                       uint16_t point_count,
                       const path_follower_config_t *config)
{
    if ((follower == NULL) || (path == NULL) || (point_count < 2U)
        || !PathFollower_IsConfigValid(config))
    {
        return false;
    }

    memset(follower, 0, sizeof(*follower));
    follower->path = path;
    follower->point_count = point_count;
    follower->config = *config;
    follower->initialized = true;
    return true;
}

/* 清除路径进度和完成锁存状态，用于重复执行同一路径。 */
void PathFollower_Reset(path_follower_t *follower)
{
    if (follower != NULL)
    {
        follower->current_segment = 0U;
        follower->finished = false;
    }
}

/*
 * 跟踪主流程：
 * 1. 在路径上寻找当前位置投影；
 * 2. 沿路径弧长选取动态前视点；
 * 3. 根据位置、切线和航向误差生成车体速度指令。
 */
bool PathFollower_Step(path_follower_t *follower,
                       const path_follower_pose_t *pose,
                       path_follower_output_t *output)
{
    path_follower_projection_t projection;
    path_follower_point_t target;
    const path_follower_point_t *last_point;
    float lookahead_distance;
    float target_speed;
    float heading;
    float heading_error;
    float error_x;
    float error_y;
    float velocity_world_x;
    float velocity_world_y;
    float speed;
    float cosine;
    float sine;
    float target_body_x;
    float target_body_y;
    float target_distance_squared;
    float target_tangent_x;
    float target_tangent_y;
    uint16_t target_segment;

    if ((follower == NULL) || (pose == NULL) || (output == NULL)
        || !follower->initialized)
    {
        return false;
    }

    memset(output, 0, sizeof(*output));
    last_point = &follower->path[follower->point_count - 1U];
    heading_error = PathFollower_NormalizeAngle(last_point->yaw_rad - pose->yaw_rad);
    /* 到达终点后锁存完成状态，后续调用持续输出零速度。 */
    if (follower->finished || ((PathFollower_Distance(pose->position_x_m,
            pose->position_y_m, last_point->position_x_m, last_point->position_y_m)
            <= follower->config.finish_distance_m)
            && (!follower->config.use_path_yaw
                || (fabsf(heading_error) <= follower->config.finish_yaw_error_rad))))
    {
        follower->finished = true;
        output->target_x_m = last_point->position_x_m;
        output->target_y_m = last_point->position_y_m;
        output->target_yaw_rad = last_point->yaw_rad;
        output->finished = true;
        return true;
    }

    PathFollower_FindClosestProjection(follower, pose, &projection);
    follower->current_segment = projection.segment;
    /* 车速越高前视越远，以减小高速时的转向抖动。 */
    lookahead_distance = follower->config.minimum_lookahead_m
                       + fabsf(pose->linear_speed_mps)
                       * follower->config.lookahead_time_s;
    PathFollower_FindLookaheadPoint(follower, &projection, lookahead_distance,
                                    &target, &target_segment);
    target_speed = PathFollower_TargetSpeed(follower, &target,
                                             projection.remaining_distance);
    /* 默认使用前视段切线作为目标航向，也可改为路径点显式给定的 yaw。 */
    target_tangent_x = follower->path[target_segment + 1U].position_x_m
                     - follower->path[target_segment].position_x_m;
    target_tangent_y = follower->path[target_segment + 1U].position_y_m
                     - follower->path[target_segment].position_y_m;
    if ((target_tangent_x * target_tangent_x
         + target_tangent_y * target_tangent_y) < PATH_FOLLOWER_EPSILON)
    {
        target_tangent_x = projection.tangent_x;
        target_tangent_y = projection.tangent_y;
    }
    heading = follower->config.use_path_yaw ? target.yaw_rad
                                             : atan2f(target_tangent_y,
                                                     target_tangent_x);
    heading_error = PathFollower_NormalizeAngle(heading - pose->yaw_rad);
    cosine = cosf(pose->yaw_rad);
    sine = sinf(pose->yaw_rad);
    error_x = target.position_x_m - pose->position_x_m;
    error_y = target.position_y_m - pose->position_y_m;

    output->target_x_m = target.position_x_m;
    output->target_y_m = target.position_y_m;
    output->target_yaw_rad = heading;
    output->target_segment = target_segment;
    output->cross_track_error_m = projection.tangent_x
                                * (pose->position_y_m - projection.y)
                                - projection.tangent_y
                                * (pose->position_x_m - projection.x);

    if (follower->config.mode == PATH_FOLLOWER_HOLONOMIC)
    {
        /* 切线前馈速度叠加位置 P 反馈，再从世界系转换到车体系。 */
        velocity_world_x = target_speed * projection.tangent_x
                         + follower->config.position_gain * error_x;
        velocity_world_y = target_speed * projection.tangent_y
                         + follower->config.position_gain * error_y;
        speed = sqrtf(velocity_world_x * velocity_world_x
                      + velocity_world_y * velocity_world_y);
        if (speed > follower->config.maximum_speed_mps)
        {
            velocity_world_x *= follower->config.maximum_speed_mps / speed;
            velocity_world_y *= follower->config.maximum_speed_mps / speed;
        }
        output->velocity_x_mps = cosine * velocity_world_x + sine * velocity_world_y;
        output->velocity_y_mps = -sine * velocity_world_x + cosine * velocity_world_y;
        output->yaw_rate_radps = PathFollower_Clamp(
            follower->config.heading_gain * heading_error,
            -follower->config.maximum_yaw_rate_radps,
            follower->config.maximum_yaw_rate_radps);
    }
    else
    {
        /* Pure Pursuit 曲率：kappa = 2 * y_l / L^2，角速度为 v * kappa。 */
        target_body_x = cosine * error_x + sine * error_y;
        target_body_y = -sine * error_x + cosine * error_y;
        target_distance_squared = target_body_x * target_body_x
                                + target_body_y * target_body_y;
        output->velocity_x_mps = target_speed;
        output->velocity_y_mps = 0.0f;
        if (target_distance_squared > PATH_FOLLOWER_EPSILON)
        {
            output->yaw_rate_radps = output->velocity_x_mps
                                   * 2.0f * target_body_y
                                   / target_distance_squared;
        }
        output->yaw_rate_radps += follower->config.heading_gain * heading_error;
        output->yaw_rate_radps = PathFollower_Clamp(
            output->yaw_rate_radps, -follower->config.maximum_yaw_rate_radps,
            follower->config.maximum_yaw_rate_radps);
    }
    return true;
}
