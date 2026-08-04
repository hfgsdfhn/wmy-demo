#include "omni_chassis.h"

#include <stddef.h>

/* 底盘长宽尺寸，单位 mm */
#define OMNI_CHASSIS_LENGTH_MM            700.0f
#define OMNI_CHASSIS_WIDTH_MM             700.0f
/* 轮子半径，单位 mm */
#define OMNI_CHASSIS_WHEEL_RADIUS_MM      155.0f
/* 圆周率 */
#define OMNI_CHASSIS_PI                   3.1415926f
/* rad/s 转 rpm 的换算系数 */
#define OMNI_CHASSIS_RPM_PER_RADPS        (60.0f / (2.0f * OMNI_CHASSIS_PI))
/* rpm 转 rad/s 的换算系数 */
#define OMNI_CHASSIS_RADPS_PER_RPM        ((2.0f * OMNI_CHASSIS_PI) / 60.0f)
/* 轮子半径，单位 m */
#define OMNI_CHASSIS_WHEEL_RADIUS_M       (OMNI_CHASSIS_WHEEL_RADIUS_MM * 0.001f)
/* 旋转中心到轮子中心的半径，单位 m */
#define OMNI_CHASSIS_ROTATION_RADIUS_M    \
    ((OMNI_CHASSIS_LENGTH_MM + OMNI_CHASSIS_WIDTH_MM) * 0.0005f)

/*
 * 运动学正解：根据底盘整体速度 vx/vy/omega 推算四个轮速。
 * vx_mps：底盘前后速度，单位 m/s
 * vy_mps：底盘左右速度，单位 m/s
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
    wheel_speeds->left_front_rpm = (vx_mps - vy_mps - rotation_speed)
                                  / OMNI_CHASSIS_WHEEL_RADIUS_M
                                  * OMNI_CHASSIS_RPM_PER_RADPS;
    wheel_speeds->right_rear_rpm = (vx_mps - vy_mps + rotation_speed)
                                 / OMNI_CHASSIS_WHEEL_RADIUS_M
                                 * OMNI_CHASSIS_RPM_PER_RADPS;
    wheel_speeds->right_front_rpm = (vx_mps + vy_mps + rotation_speed)
                                  / OMNI_CHASSIS_WHEEL_RADIUS_M
                                  * OMNI_CHASSIS_RPM_PER_RADPS;
    wheel_speeds->left_rear_rpm = (vx_mps + vy_mps - rotation_speed)
                                / OMNI_CHASSIS_WHEEL_RADIUS_M
                                * OMNI_CHASSIS_RPM_PER_RADPS;
}

/*
 * 运动学逆解：根据四个轮速恢复底盘整体速度 vx/vy/omega。
 * 输出单位分别为 m/s、m/s、rad/s。
 */
void OmniChassis_InverseKinematics(
    const omni_chassis_wheel_speeds_t *wheel_speeds, float *vx_mps,
    float *vy_mps, float *omega_radps)
{
    float left_front_radps;
    float right_rear_radps;
    float right_front_radps;
    float left_rear_radps;

    if ((wheel_speeds == NULL) || (vx_mps == NULL) || (vy_mps == NULL)
        || (omega_radps == NULL))
    {
        return;
    }

    /* 将轮速从 rpm 转换为 rad/s */
    left_front_radps = wheel_speeds->left_front_rpm
                      * OMNI_CHASSIS_RADPS_PER_RPM;
    right_rear_radps = wheel_speeds->right_rear_rpm
                     * OMNI_CHASSIS_RADPS_PER_RPM;
    right_front_radps = wheel_speeds->right_front_rpm
                      * OMNI_CHASSIS_RADPS_PER_RPM;
    left_rear_radps = wheel_speeds->left_rear_rpm
                    * OMNI_CHASSIS_RADPS_PER_RPM;

    /* 根据四轮角速度求解底盘平移和转动速度 */
    *vx_mps = OMNI_CHASSIS_WHEEL_RADIUS_M
            * (left_front_radps + right_rear_radps + right_front_radps
               + left_rear_radps) * 0.25f;
    *vy_mps = OMNI_CHASSIS_WHEEL_RADIUS_M
            * (-left_front_radps - right_rear_radps + right_front_radps
               + left_rear_radps) * 0.25f;
    *omega_radps = OMNI_CHASSIS_WHEEL_RADIUS_M
                 * (-left_front_radps + right_rear_radps + right_front_radps
                    - left_rear_radps)
                 / (4.0f * OMNI_CHASSIS_ROTATION_RADIUS_M);
}
