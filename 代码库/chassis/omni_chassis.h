#ifndef OMNI_CHASSIS_H
#define OMNI_CHASSIS_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#define WHEEL_RADIUS     0.155f
#define CHASSIS_WIDTH    0.558f
#define CHASSIS_LENGTH   0.558f
#define sin45            0.7071f
#define CHASSIS_RADIUS   ((CHASSIS_WIDTH / 2.0f + CHASSIS_LENGTH / 2.0f) * sin45)

typedef struct {
    float vx;
    float vy;
    float vw;
} chassis_speed_t;   // 底盘三轴速度

typedef struct {
    float lf;
    float rf;
    float lr;
    float rr;
} wheel_speed_t;     // 四轮轮速

typedef struct {
    float x;
    float y;
    float yaw;
} chassis_position_t; // 底盘位置

typedef struct {
    wheel_speed_t   wheel;   // 当前真实轮速（从编码器反馈换算）
    chassis_speed_t speed;   // 当前真实底盘速度（由轮速逆解算得）
    chassis_position_t position; // 当前真实位置
} chassis_state_t;

typedef struct {
    chassis_state_t state;        // 当前真实状态
    chassis_speed_t target_speed; // 目标速度（决策层下发）
    wheel_speed_t   target_wheel; // 目标轮速（由目标速度解算得到）
    chassis_speed_t command_speed; // 发送最终应用的速度的速度
    chassis_speed_t last_speed;    // 上一时刻的速度（用于计算加速度）
    chassis_position_t target_position; // 目标位置（由决策层下发）
} omni_chassis_t;

extern omni_chassis_t chassis;

void chassis_set_speed(void);
void chassis_get_speed(void);
void Chassis_BodyToWorld(void);
float Chassis_WheelSpeedToYawRate(float wheel_speed);

#endif
