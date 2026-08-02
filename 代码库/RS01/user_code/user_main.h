#ifndef USER_MAIN_H
#define USER_MAIN_H

#include <stdbool.h>

#include "rs01_motor.h"

extern rs01_motor_t rs01_motor;

extern volatile float rs01_motion_position;
extern volatile float rs01_motion_speed;
extern volatile float rs01_motion_kp;
extern volatile float rs01_motion_kd;
extern volatile float rs01_motion_torque;

bool UserMain_Init(void);
void UserMain_Loop(void);

#endif
