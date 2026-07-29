#ifndef PID_H
#define PID_H

#include <stdbool.h>
#include <stddef.h>

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
    bool initialized;
} pid_t;

bool PID_Init(pid_t *pid, float kp, float ki, float kd,
              float sample_time_ms, float output_limit, float integral_limit);
float PID_Calc(pid_t *pid, float target, float real);

#endif
