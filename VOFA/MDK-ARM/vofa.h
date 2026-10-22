#ifndef VOFA_H
#define VOFA_H

#include "main.h"
#define CH_COUNT 1   //通道数量

typedef struct vofa vofa_t;
struct vofa
{
    float fdata[CH_COUNT];
    uint8_t tail[4];
    HAL_StatusTypeDef tx_state;
    void (*send)(vofa_t *vofa, UART_HandleTypeDef *huart);
};

void vofa_send(vofa_t *vofa, UART_HandleTypeDef *huart);
extern vofa_t vofa;

#endif
