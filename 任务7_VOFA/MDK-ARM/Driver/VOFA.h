/**
 * @file vofa.h
 * @brief VOFA+ JustFloat protocol driver.
 */
#ifndef VOFA_H
#define VOFA_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_uart.h"

#define VOFA_TX_BUF_SIZE       256U
#define VOFA_FRAME_TAIL_SIZE   4U
#define VOFA_MAX_FLOATS        ((VOFA_TX_BUF_SIZE - VOFA_FRAME_TAIL_SIZE) / sizeof(float))
#define VOFA_MAX_CHANNEL 16U

typedef struct
{
    char name[16];
    float *value;
} vofa_channel_t;
typedef struct
{
    uint32_t send_count;
    uint32_t busy_count;
    uint32_t error_count;
} vofa_stat_t;

typedef struct
{
    bsp_uart_t *uart;
    uint8_t tx_buf[VOFA_TX_BUF_SIZE];

    uint16_t period_ms;
    uint32_t start_ms;
    uint32_t last_send_ms;
    vofa_stat_t stat;

    vofa_channel_t channels[VOFA_MAX_CHANNEL];
    uint8_t channel_count;
} vofa_t;

bool Vofa_Init(vofa_t *vofa, bsp_uart_t *uart);

bool Vofa_SendFloat(vofa_t *vofa, float value);

bool Vofa_SendArray(vofa_t *vofa, float *data, uint16_t num);

void Vofa_SetPeriod(vofa_t *vofa, uint16_t period_ms);

void Vofa_Process(vofa_t *vofa);

const vofa_stat_t *Vofa_GetStat(const vofa_t *vofa);

bool Vofa_Register(vofa_t *vofa, const char *name, float *value);

bool Vofa_SendChannels(vofa_t *vofa);

#endif /* VOFA_H */
