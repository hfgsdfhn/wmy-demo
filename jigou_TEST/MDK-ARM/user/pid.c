#include "pid.h"

static float PID_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float PID_Clamp(float value, float limit)
{
    limit = PID_Abs(limit);

    if (limit == 0.0f)
    {
        return 0.0f;
    }
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

/**
 * @brief PID初始化函数
 * 
 * @param pid 填入PID结构体指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 * @param sample_time_ms 采样时间，单位毫秒
 * @param output_limit  输出限幅
 * @param integral_limit 积分限幅
 */
bool PID_Init(pid_t *pid, float kp, float ki, float kd,
              float sample_time_ms, float output_limit, float integral_limit)
{
    if ((pid == 0) || (sample_time_ms <= 0.0f))
    {
        return false;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->sample_time_ms = sample_time_ms;
    pid->output_limit = PID_Abs(output_limit);
    pid->integral_limit = PID_Abs(integral_limit);
    pid->integral_separation = 0.0f;
    PID_Reset(pid);
    return true;
}

//积分分离
void PID_SetIntegralSeparation(pid_t *pid, float separation)
{
    if (pid != 0)
    {
        pid->integral_separation = PID_Abs(separation);
    }
}

/**
 * @brief PID重置函数 
 * 
 * @param pid 填入PID结构体指针
 */
void PID_Reset(pid_t *pid)
{
    if (pid != 0)
    {
        pid->integral = 0.0f;         
        pid->previous_feedback = 0.0f;
        pid->initialized = false;
    }
}

/**
 * @brief PID计算函数
 * 
 * @param pid 填入PID结构体指针
 * @param target 目标值
 * @param feedback 反馈值
 * @return float 计算结果
 */
float PID_Calc(pid_t *pid, float target, float feedback)
{
    float error;    //误差值
    float derivative;   //微分值
    float output;      //输出值

    error = target - feedback;
    if ((pid->integral_separation > 0.0f)
        && (PID_Abs(error) > pid->integral_separation))
    {
        pid->integral = 0.0f;
    }
    
    else
    {
        pid->integral += error;      //积分值累加
        pid->integral = PID_Clamp(pid->integral, pid->integral_limit);  //积分限幅
    }

    derivative = 0.0f;

    if (pid->initialized)   //只有上一次返回值有效时才计算微分，防止PID刚启动时传入的反馈值过大导致微分项过大，造成输出抖动。
    {
        derivative = -(feedback - pid->previous_feedback) / pid->sample_time_ms;
    }

    output = pid->kp * error + 
             pid->ki * pid->integral + 
             pid->kd * derivative;
    pid->previous_feedback = feedback;
    pid->initialized = true;     //上一次返回值有效
    return PID_Clamp(output, pid->output_limit);        //限幅并输出
}