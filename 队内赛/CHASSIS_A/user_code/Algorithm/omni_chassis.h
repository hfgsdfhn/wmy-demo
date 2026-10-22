#ifndef OMNI_CHASSIS_H
#define OMNI_CHASSIS_H /* 全向底盘运动学头文件保护宏 */

typedef struct
{
    float left_front_mps;
    float right_rear_mps;
    float right_front_mps;
    float left_rear_mps;
} omni_chassis_wheel_speeds_t;

/* vx: forward (+), vy: left (+), omega: counter-clockwise (+). */
void OmniChassis_ForwardKinematics(float vx_mps, float vy_mps,
                                   float omega_radps,
                                   omni_chassis_wheel_speeds_t *wheel_speeds);

void OmniChassis_InverseKinematics(
    const omni_chassis_wheel_speeds_t *wheel_speeds, float *vx_mps,
    float *vy_mps, float *omega_radps);

/* Converts world-frame vx/vy to body-frame vx/vy using yaw_rad. */
void OmniChassis_WorldToBody(float vx_world_mps, float vy_world_mps,
                             float yaw_rad, float *vx_body_mps,
                             float *vy_body_mps);

/* Converts world-frame vx/vy to body-frame wheel speeds using yaw_rad.
 * omega_radps is already a body yaw rate and is passed through unchanged. */
void OmniChassis_World(float vx_world_mps, float vy_world_mps,
                       float omega_radps, float yaw_rad,
                       omni_chassis_wheel_speeds_t *wheel_speeds);

/* Convert a wheel tangential speed to the corresponding pure-yaw rate. */
float OmniChassis_WheelSpeedToYawRate(float wheel_speed_mps);

#endif /* OMNI_CHASSIS_H */
