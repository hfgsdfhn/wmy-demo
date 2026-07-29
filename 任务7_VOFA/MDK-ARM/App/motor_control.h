#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

#include "PID.h"
#include "dji_motor.h"

typedef enum
{
    MOTOR_CONTROL_SPEED = 0,    //速度环模式
    MOTOR_CONTROL_POSITION,     //位置环模式
    MOTOR_CONTROL_CURRENT       //电流环模式
} motor_control_mode_t;

typedef struct
{
    dji_motor_t *motor;     //选择大疆电机
    pid_t *speed_pid;       //速度环PID控制器
    pid_t *angle_pid;       //位置环PID控制器
    float target_angle;     //目标角度
    float target_speed;     //目标速度
    float target_current;   //目标电流
    float output_current;   //输出电流
    uint8_t mode;
} motor_control_t;

void MotorControl_Init(motor_control_t *control, dji_motor_t *motor,
                       pid_t *speed_pid, pid_t *angle_pid);
void MotorControl_SetSpeed(motor_control_t *control, float speed_rpm);
void MotorControl_SetAngle(motor_control_t *control, float angle);
void MotorControl_SetCurrent(motor_control_t *control, float current);
void MotorControl_Update(motor_control_t *control);

#endif
