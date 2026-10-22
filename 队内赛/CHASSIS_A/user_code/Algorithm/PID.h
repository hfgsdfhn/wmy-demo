#ifndef PID_H
#define PID_H /* PID 算法头文件保护宏 */

#include <stdbool.h>

typedef struct
{
    float kp;
    float ki;
    float kd;
    float sample_time_ms;
    float integral;
    float previous_feedback;
    float output_limit;
    float integral_limit;
    float integral_separation;
    bool initialized;
} pid_t;

bool PID_Init(pid_t *pid, float kp, float ki, float kd,
              float sample_time_ms, float output_limit, float integral_limit);
/* 误差超过分离阈值时清除积分；阈值为 0 时关闭积分分离。 */
void PID_SetIntegralSeparation(pid_t *pid, float separation);
void PID_Reset(pid_t *pid);
float PID_Calc(pid_t *pid, float target, float feedback);

#endif
