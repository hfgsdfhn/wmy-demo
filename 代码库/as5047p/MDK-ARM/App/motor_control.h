#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

#include "AS5047P.h"
#include "PID.h"
#include "dji_motor.h"

typedef enum
{
    MOTOR_CONTROL_SPEED = 0,        //速度模式
    MOTOR_CONTROL_POSITION = 1,     //位置模式
    MOTOR_CONTROL_CURRENT = 2,      //电流模式
    MOTOR_CONTROL_ZERO = 3          //回零模式
} motor_control_mode_t;

typedef struct
{
    dji_motor_t *motor;
    as5047p_t *encoder;
    pid_t *speed_pid;
    pid_t *angle_pid;
    float target_angle;
    float target_speed;
    float target_current;
    float output_current;
    uint8_t mode;
} motor_control_t;

void MotorControl_Init(motor_control_t *control, dji_motor_t *motor,
                       as5047p_t *encoder, pid_t *speed_pid, pid_t *angle_pid);
void MotorControl_SetSpeed(motor_control_t *control, float speed_rpm);
void MotorControl_SetAngle(motor_control_t *control, float angle);
void MotorControl_SetCurrent(motor_control_t *control, float current);
void MotorControl_Update(motor_control_t *control);

#endif
