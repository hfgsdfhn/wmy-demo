#ifndef MY_MAIN_H
#define MY_MAIN_H

#include "dji_motor.h"
#include "main.h"
#include "pid.h"

extern pid_t M2006_pid[4];
extern pid_t position_pid[4];
extern dji_motor_t M2006[4];
extern dji_motor_t M3508[4];
extern volatile float m2006_current;
extern bsp_can_t can_dji;

#define lf 0
#define rf 1
#define rr 2
#define lr 3

typedef enum
{
    all_up = 1,
    all_back,
    front_up,
    front_back,
    behind_up,
    behind_back,
    qigang_up,
    qigang_down,
}position_case;

typedef enum
{
    step1 = 1,
    step2,
    step3,
    step4,
    step5,
    step6,
    step7,
    step8,
}up_stpe;

typedef struct
{
    float distance;
    float left_current;
    float right_current;
}leg_t;

extern leg_t front;
extern leg_t behind;
extern position_case my_case;
extern up_stpe up_;

void DJI_OnData(bsp_can_t *can, uint32_t can_id, const uint8_t *data,
                uint8_t length);
void My_Init(void);

void My_Loop(void);

#endif
