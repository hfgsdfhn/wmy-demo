#ifndef OMNI_CHASSIS_H
#define OMNI_CHASSIS_H

typedef struct
{
    float left_front_rpm;
    float right_rear_rpm;
    float right_front_rpm;
    float left_rear_rpm;
} omni_chassis_wheel_speeds_t;

/* vx: forward (+), vy: left (+), omega: counter-clockwise (+). */
void OmniChassis_ForwardKinematics(float vx_mps, float vy_mps,
                                   float omega_radps,
                                   omni_chassis_wheel_speeds_t *wheel_speeds);

void OmniChassis_InverseKinematics(
    const omni_chassis_wheel_speeds_t *wheel_speeds, float *vx_mps,
    float *vy_mps, float *omega_radps);

#endif /* OMNI_CHASSIS_H */
