#ifndef USER_MAIN_H
#define USER_MAIN_H

#include "PID.h"

void user_main_init(void);
void user_main_loop(void);

extern volatile pid_t motor2006_speed_pid;
extern volatile pid_t motor2006_position_pid;

#endif /* USER_MAIN_H */
