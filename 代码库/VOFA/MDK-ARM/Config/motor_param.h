#ifndef MOTOR_PARAM_H
#define MOTOR_PARAM_H

#define MOTOR_CONTROL_SAMPLE_TIME_MS       1.0f
#define MOTOR_CONTROL_RAD_TO_DEG            57.2957795131f

#define M2006_SPEED_KP                      30.0f
#define M2006_SPEED_KI                      0.0f
#define M2006_SPEED_KD                      25.0f
#define M2006_SPEED_OUTPUT_LIMIT            10000.0f
#define M2006_SPEED_INTEGRAL_LIMIT          5000.0f

#define MOTOR_CONTROL_ANGLE_KP             2.5f
#define MOTOR_CONTROL_ANGLE_KI             0.0f
#define MOTOR_CONTROL_ANGLE_KD             2.0f
#define MOTOR_CONTROL_ANGLE_OUTPUT_LIMIT   5000.0f
#define MOTOR_CONTROL_ANGLE_INTEGRAL_LIMIT 180.0f

#endif
