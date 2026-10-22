#include "vofa.h"

#include <string.h>

#include "main.h"
#include "my_main.h"

#define VOFA_CURRENT_COMMAND       0x10U
#define VOFA_CURRENT_TIMEOUT_MS    100U

typedef enum
{
    VOFA_WAIT_HEADER_AA = 0,
    VOFA_WAIT_HEADER_55,
    VOFA_WAIT_COMMAND,
    VOFA_WAIT_CURRENT_DATA
} vofa_state_t;

static vofa_state_t vofa_state;
static uint8_t vofa_current_data[sizeof(float)];
static uint8_t vofa_current_index;
static uint32_t vofa_last_current_tick;

void VOFA_Init(void)
{
    vofa_state = VOFA_WAIT_HEADER_AA;
    vofa_current_index = 0U;
    vofa_last_current_tick = HAL_GetTick();
}

void VOFA_ParseByte(uint8_t byte)
{
    switch (vofa_state)
    {
        case VOFA_WAIT_HEADER_AA:
            if (byte == 0xAAU)
            {
                vofa_state = VOFA_WAIT_HEADER_55;
            }
            break;

        case VOFA_WAIT_HEADER_55:
            if (byte == 0x55U)
            {
                vofa_state = VOFA_WAIT_COMMAND;
            }
            else
            {
                vofa_state = (byte == 0xAAU) ? VOFA_WAIT_HEADER_55 : VOFA_WAIT_HEADER_AA;
            }
            break;

        case VOFA_WAIT_COMMAND:
            if ((byte >= 1U) && (byte <= 8U))
            {
                up_ = (up_stpe)byte;
                vofa_state = VOFA_WAIT_HEADER_AA;
            }
            else if (byte == VOFA_CURRENT_COMMAND)
            {
                vofa_current_index = 0U;
                vofa_state = VOFA_WAIT_CURRENT_DATA;
            }
            else
            {
                vofa_state = VOFA_WAIT_HEADER_AA;
            }
            break;

        case VOFA_WAIT_CURRENT_DATA:
            vofa_current_data[vofa_current_index++] = byte;
            if (vofa_current_index >= sizeof(vofa_current_data))
            {
                memcpy((void *)&m2006_current, vofa_current_data, sizeof(m2006_current));
                vofa_last_current_tick = HAL_GetTick();
                vofa_state = VOFA_WAIT_HEADER_AA;
            }
            break;

        default:
            vofa_state = VOFA_WAIT_HEADER_AA;
            break;
    }
}

void VOFA_CheckTimeout(void)
{
    if ((HAL_GetTick() - vofa_last_current_tick) >= VOFA_CURRENT_TIMEOUT_MS)
    {
        m2006_current = 0.0f;
    }
}
