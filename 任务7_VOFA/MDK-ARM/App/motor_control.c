#include "Motor_Control.h"

#include <stddef.h>

//设置电流输出
static void MotorControl_OutputCurrent(motor_control_t *control, float current)
{
    control->output_current = current;
    DjiMotor_SetCurrent(control->motor, (int16_t)current);
}
//计算电机转子角度，将编码器值转化为角度
static float MotorControl_RotorAngle(int32_t total_encoder)
{
    return (float)total_encoder * (360.0f / 8192.0f);
}
/**
 * @brief 初始化电机控制
 * 
 * @param control 
 * @param motor 
 * @param speed_pid 
 * @param angle_pid 
 */
void MotorControl_Init(motor_control_t *control, dji_motor_t *motor,
                       pid_t *speed_pid, pid_t *angle_pid)
{
    if ((control == NULL) || (motor == NULL)
        || (speed_pid == NULL) || (angle_pid == NULL)
        || (motor->type != DJI_MOTOR_TYPE_M2006))
    {
        return;
    }
    
    control->motor = motor;
    control->speed_pid = speed_pid;
    control->angle_pid = angle_pid;
    control->target_angle = 0.0f;
    control->target_speed = 0.0f;
    control->target_current = 0.0f;
    control->output_current = 0.0f;
    control->mode = MOTOR_CONTROL_SPEED;

}
/**
 * @brief 设置电机转速
 * 
 * @param control 
 * @param speed_rpm 
 */
void MotorControl_SetSpeed(motor_control_t *control, float speed_rpm)
{
    if (control == NULL)
    { 
        return;
    }

    control->target_speed = speed_rpm;
    control->mode = MOTOR_CONTROL_SPEED;

    if (control->angle_pid != NULL)
    {
        control->angle_pid->integral = 0.0f;
        control->angle_pid->previous_feedback = 0.0f;
        control->angle_pid->initialized = false;
    }
}

/**
 * @brief 设置电机目标角度
 * 
 * @param control 
 * @param angle 
 */
void MotorControl_SetAngle(motor_control_t *control, float angle)
{
    if (control == NULL)
    {
        return;
    }

    control->target_angle = angle;
    control->mode = MOTOR_CONTROL_POSITION;
}

/**
 * @brief 设置电机目标电流
 * 
 * @param control 
 * @param current 
 */
void MotorControl_SetCurrent(motor_control_t *control, float current)
{
    if (control == NULL)
    {
        return;
    }

    control->target_current = current;
    control->mode = MOTOR_CONTROL_CURRENT;
}

/**
 * @brief 更新电机控制
 * 
 * @param control 
 */
void MotorControl_Update(motor_control_t *control)
{
    float current;
    float position;
    float position_target;
    float speed_target;

    if ((control == NULL) || (control->motor == NULL))
    {
        return;
    }

    if (control->mode == MOTOR_CONTROL_SPEED)
    {
        current = PID_Calc(control->speed_pid, control->target_speed,
                           (float)control->motor->speed_rpm);
        MotorControl_OutputCurrent(control, current);
        return;
    }

    if (control->mode == MOTOR_CONTROL_CURRENT)
    {
        MotorControl_OutputCurrent(control, control->target_current);
        return;
    }

    if (control->mode != MOTOR_CONTROL_POSITION)
    {
        MotorControl_OutputCurrent(control, 0.0f);
        return;
    }

    position = MotorControl_RotorAngle(control->motor->total_encoder);
    position_target = control->target_angle;
    speed_target = PID_Calc(control->angle_pid, position_target, position);
    current = PID_Calc(control->speed_pid, speed_target,
                       (float)control->motor->speed_rpm);
    MotorControl_OutputCurrent(control, current);
}
