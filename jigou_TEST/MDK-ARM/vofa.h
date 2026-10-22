#ifndef VOFA_H
#define VOFA_H

#include <stdint.h>

void VOFA_Init(void);
void VOFA_ParseByte(uint8_t byte);
void VOFA_CheckTimeout(void);

#endif
