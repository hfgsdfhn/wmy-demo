#include "omni_chassis.h"

#include <math.h>
#include <stddef.h>

/* Set to 1U to interpret OmniChassis_World() vx/vy in the world frame.
 * Set to 0U for body-frame compatibility while retaining the same API. */
#define OMNI_CHASSIS_USE_WORLD_FRAME        1U

/* 底盘长宽尺寸，单位 mm */
#define OMNI_CHASSIS_LENGTH_MM            558.8f /* 底盘长度，单位 mm */
#define OMNI_CHASSIS_WIDTH_MM             558.8f /* 底盘宽度，单位 mm */
#define OMINI_DEADZONE_MPS                     0.00f /* 轮速死区，单位 m/s */
/* 轮子半径，单位 mm */
/* 轮子半径，单位 m */
/* 旋转中心到轮子中心的半径，单位 m */
#define OMNI_CHASSIS_ROTATION_RADIUS_M    \
    ((OMNI_CHASSIS_LENGTH_MM + OMNI_CHASSIS_WIDTH_MM) * 0.0005f /1.41421356237)

static float OmniChassis_ApplyDeadzone(float speed_mps)
{
    return (fabsf(speed_mps) < OMINI_DEADZONE_MPS) ? 0.0f : speed_mps;
}

float OmniChassis_WheelSpeedToYawRate(float wheel_speed_mps)
{
    return wheel_speed_mps / OMNI_CHASSIS_ROTATION_RADIUS_M;
}

void OmniChassis_WorldToBody(float vx_world_mps, float vy_world_mps,
                             float yaw_rad, float *vx_body_mps,
                             float *vy_body_mps)
{
    if ((vx_body_mps == NULL) || (vy_body_mps == NULL))
    {
        return;
    }

#if (OMNI_CHASSIS_USE_WORLD_FRAME != 0U)
    if (!isfinite(yaw_rad))
    {
        *vx_body_mps = 0.0f;
        *vy_body_mps = 0.0f;
        return;
    }

    *vx_body_mps = cosf(yaw_rad) * vx_world_mps
                 + sinf(yaw_rad) * vy_world_mps;
    *vy_body_mps = -sinf(yaw_rad) * vx_world_mps
                 + cosf(yaw_rad) * vy_world_mps;
#else
    (void)yaw_rad;
    *vx_body_mps = vx_world_mps;
    *vy_body_mps = vy_world_mps;
#endif
}

/*
 * 运动学正解：根据底盘整体速度 vx/vy/omega 推算四个轮速。
 * vx_mps：底盘前后速度，单位 m/s
 * vy_mps：底盘左右速度，单位 m/s·
 * omega_radps：底盘角速度，单位 rad/s
 */
void OmniChassis_ForwardKinematics(float vx_mps, float vy_mps,
                                   float omega_radps,
                                   omni_chassis_wheel_speeds_t *wheel_speeds)
{
    float rotation_speed;

    if (wheel_speeds == NULL)
    {
        return;
    }

    /* 旋转时，每个轮子需要补偿的线速度 */
    rotation_speed = OMNI_CHASSIS_ROTATION_RADIUS_M * omega_radps;

    /* 四个轮的目标转速，单位 rpm */
    wheel_speeds->left_front_mps = OmniChassis_ApplyDeadzone(
        vx_mps - vy_mps - rotation_speed);
    wheel_speeds->right_rear_mps = OmniChassis_ApplyDeadzone(
        -(vx_mps - vy_mps + rotation_speed));
    wheel_speeds->right_front_mps = OmniChassis_ApplyDeadzone(
        -(vx_mps + vy_mps + rotation_speed));
    wheel_speeds->left_rear_mps = OmniChassis_ApplyDeadzone(
        vx_mps + vy_mps - rotation_speed);
}

/*
 * 运动学逆解：根据四个轮速恢复底盘整体速度 vx/vy/omega。
 * 输出单位分别为 m/s、m/s、rad/s。
 */
void OmniChassis_InverseKinematics(
    const omni_chassis_wheel_speeds_t *wheel_speeds, float *vx_mps,
    float *vy_mps, float *omega_radps)
{
    float left_front_mps;
    float right_rear_mps;
    float right_front_mps;
    float left_rear_mps;

    if ((wheel_speeds == NULL) || (vx_mps == NULL) || (vy_mps == NULL)
        || (omega_radps == NULL))
    {
        return;
    }

    /* 将轮速从 rpm 转换为 rad/s */
    left_front_mps = OmniChassis_ApplyDeadzone(wheel_speeds->left_front_mps);
    right_rear_mps = OmniChassis_ApplyDeadzone(wheel_speeds->right_rear_mps);
    right_front_mps = OmniChassis_ApplyDeadzone(wheel_speeds->right_front_mps);
    left_rear_mps = OmniChassis_ApplyDeadzone(wheel_speeds->left_rear_mps);

    /* 根据四轮角速度求解底盘平移和转动速度 */
    *vx_mps = (left_front_mps - right_rear_mps - right_front_mps
               + left_rear_mps) * 0.25f;
    *vy_mps = (-left_front_mps + right_rear_mps - right_front_mps
               + left_rear_mps) * 0.25f;
    *omega_radps = -(left_front_mps + right_rear_mps + right_front_mps
                     + left_rear_mps)
                 / (4.0f * OMNI_CHASSIS_ROTATION_RADIUS_M);
}

void OmniChassis_World(float vx_world_mps, float vy_world_mps,
                       float omega_radps, float yaw_rad,
                       omni_chassis_wheel_speeds_t *wheel_speeds)
{
#if (OMNI_CHASSIS_USE_WORLD_FRAME != 0U)
    float vx_body_mps;
    float vy_body_mps;

    /* World -> body rotation.  vx and vy remain independent axes: a yaw
     * controller supplies only omega_radps and never remaps one axis to another. */
    OmniChassis_WorldToBody(vx_world_mps, vy_world_mps, yaw_rad,
                            &vx_body_mps, &vy_body_mps);
    OmniChassis_ForwardKinematics(vx_body_mps, vy_body_mps, omega_radps,
                                  wheel_speeds);
#else
    (void)yaw_rad;
    OmniChassis_ForwardKinematics(vx_world_mps, vy_world_mps, omega_radps,
                                  wheel_speeds);
#endif
}
