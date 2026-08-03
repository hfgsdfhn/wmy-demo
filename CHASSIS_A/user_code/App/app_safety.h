#ifndef APP_SAFETY_H
#define APP_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

bool AppSafety_Init(void);
void AppSafety_Step(uint32_t system_events);
bool AppSafety_OutputAllowed(void);
uint32_t AppSafety_GetEvents(void);

#endif /* APP_SAFETY_H */
