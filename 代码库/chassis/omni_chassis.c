/**
 * @file omni_chassis.c
 * @author wmy
 * @brief 全向轮底盘解算，逆解算
 * @version 0.1
 * @date 2026-09-05
 * 
 * @copyright 版权所有 (c) 2026
 * 
 */
#include "omni_chassis.h"
#include "Chassis_Task.h"
#include "math.h"

omni_chassis_t chassis = {0};  // 初始化为0

// 正运动学：目标速度 → 目标轮速
void chassis_set_speed(void)
{
    float vx = chassis.command_speed.vx;
    float vy = chassis.command_speed.vy;
    float vw = chassis.command_speed.vw;

    chassis.target_wheel.lf = (sin45 * (vx - vy) + CHASSIS_RADIUS * vw) * MPS_TO_ERPM;
    chassis.target_wheel.rf = (sin45 * (vx + vy) + CHASSIS_RADIUS * vw) * MPS_TO_ERPM;
    chassis.target_wheel.lr = -(sin45 * (vx + vy) - CHASSIS_RADIUS * vw) * MPS_TO_ERPM;
    chassis.target_wheel.rr = -(sin45 * (vx - vy) - CHASSIS_RADIUS * vw) * MPS_TO_ERPM;
}

// 逆运动学：当前轮速 → 当前底盘速度
void chassis_get_speed(void)
{
    float lf = chassis.state.wheel.lf;
    float rf = chassis.state.wheel.rf;
    float lr = -chassis.state.wheel.lr;
    float rr = -chassis.state.wheel.rr;

    chassis.state.speed.vx = -(lf + rf + lr + rr) / (4.0f * sin45);
    chassis.state.speed.vy = -(-lf + rf + lr - rr) / (4.0f * sin45);
    chassis.state.speed.vw = (-lf + rf - lr + rr) / (4.0f * CHASSIS_RADIUS);
}

// 线速度转角速度
float Chassis_WheelSpeedToYawRate(float wheel_speed)
{
    return wheel_speed / CHASSIS_RADIUS;
}

// 车身坐标系 → 世界坐标系
void Chassis_BodyToWorld(void)
{
    float body_vx = chassis.target_speed.vx;
    float body_vy = chassis.target_speed.vy;
    float cos_yaw = cosf(chassis.state.position.yaw);
    float sin_yaw = sinf(chassis.state.position.yaw);

    chassis.target_speed.vx = body_vx * cos_yaw - body_vy * sin_yaw;
    chassis.target_speed.vy = body_vx * sin_yaw + body_vy * cos_yaw;
}
