/**
 * @file PID.c
 * @author 王梦阳 wmy07823@163.com
 * @brief PID算法实现
 * @version 0.1
 * @date 2026-07-24
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "PID.h"

/**
 * @brief 取绝对值函数
 * 
 * @param value 输入值
 * @return float 返回值
 */
static float PID_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

/**
 * @brief PID限幅函数
 * 
 * @param value 
 * @param limit 
 * @return float 
 */
static float PID_Clamp(float value, float limit)
{
    limit = PID_Abs(limit);

    if (limit == 0.0f)
    {
        return value;
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
 * @param integral_limit 输入限幅
 * @return true 初始化成功
 * @return false 结构体指针为空或采样时间小于等于0
 */
bool PID_Init(pid_t *pid, float kp, float ki, float kd,
              float sample_time_ms, float output_limit, float integral_limit)
{
    if ((pid == NULL) || (sample_time_ms <= 0.0f))
    {
        return false;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->sample_time_ms = sample_time_ms;
    pid->output_limit = PID_Abs(output_limit);
    pid->integral_limit = PID_Abs(integral_limit);
    pid->integral = 0.0f;
    pid->previous_feedback = 0.0f;
    pid->initialized = false;
    return true;
}

/**
 * @brief PID计算函数
 * 
 * @param pid 填入PID结构体指针
 * @param target 目标值
 * @param feedback 反馈值
 * @return float 计算结果
 */
float PID_Calc(pid_t *pid, float target, float real)
{
    float error;    //误差值
    float derivative;   //微分值
    float output;      //输出值

    error = target - real;
    pid->integral += error * pid->sample_time_ms;      //积分值累加
    pid->integral = PID_Clamp(pid->integral, pid->integral_limit);  //积分限幅

    derivative = 0.0f;

    if (pid->initialized)   //只有上一次返回值有效时才计算微分，防止PID刚启动时传入的反馈值过大导致微分项过大，造成输出抖动。
    {
        derivative = -(real - pid->previous_feedback) / pid->sample_time_ms;
    }

    output = pid->kp * error + 
             pid->ki * pid->integral + 
             pid->kd * derivative;
    pid->previous_feedback = real;
    pid->initialized = true;     //上一次返回值有效
    return PID_Clamp(output, pid->output_limit);        //限幅并输出
}
