#include "Motor_Control.h"
#include "motor_param.h"

#include <stddef.h>

//设置电流输出
static void MotorControl_OutputCurrent(motor_control_t *control, float current)
{
    control->output_current = current;
    DjiMotor_SetCurrent(control->motor, (int16_t)current);
}

//计算电机转子角度，将编码器值转化为角度
/**
 * @brief 初始化电机控制
 * 
 * @param control 
 * @param motor 
 * @param speed_pid 
 * @param angle_pid 
 */
void MotorControl_Init(motor_control_t *control, dji_motor_t *motor,
                       as5047p_t *encoder, pid_t *speed_pid, pid_t *angle_pid)
{
    if ((control == NULL) || (motor == NULL)
        || (encoder == NULL) || (speed_pid == NULL) || (angle_pid == NULL)
        || (motor->type != DJI_MOTOR_TYPE_M2006))
    {
        return;
    }

    control->motor = motor;
    control->encoder = encoder;
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
    PID_Reset(control->angle_pid);
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
    if (control->mode != MOTOR_CONTROL_POSITION)
    {
        PID_Reset(control->angle_pid);
        PID_Reset(control->speed_pid);
    }
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
 * @result 主要计算PID
 * @param control 
 */
void MotorControl_Update(motor_control_t *control)
{
    float current;
    float position;
    float position_target;
    float speed_target;

    if ((control == NULL) || (control->motor == NULL) || (control->encoder == NULL))
    {
        return;
    }

    switch (control->mode)
    {
    case MOTOR_CONTROL_SPEED:                                           //速度环
        current = PID_Calc(control->speed_pid, control->target_speed,
                           (float)control->motor->speed_rpm);
        MotorControl_OutputCurrent(control, current);
        break;

    case MOTOR_CONTROL_CURRENT:                                         //电流环
        MotorControl_OutputCurrent(control, control->target_current);
        break;

    case MOTOR_CONTROL_POSITION:                                        //位置环
        if (!AS5047P_ReadAngle(control->encoder))
        {
            MotorControl_OutputCurrent(control, 0.0f);
            break;
        }

        position = AS5047P_GetAngle(control->encoder) * MOTOR_CONTROL_RAD_TO_DEG;
        position_target = position + AS5047P_WrapAngleError(
            control->target_angle - position);
        speed_target = PID_Calc(control->angle_pid, position_target, position);     //外环计算
        current = PID_Calc(control->speed_pid, speed_target,(float)control->motor->speed_rpm);  //内环计算
        MotorControl_OutputCurrent(control, current);
        break;

    default:
        MotorControl_OutputCurrent(control, 0.0f);
        break;
    }
}
