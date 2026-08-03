#ifndef APP_COMMUNICATION_H
#define APP_COMMUNICATION_H

#include <stdbool.h>

#include "app_messages.h"

bool AppCommunication_Init(void);
void AppCommunication_OnPacket(const app_comm_packet_t *packet);
bool AppCommunication_GetLastPacket(app_comm_source_t source,
                                    app_comm_packet_t *packet);

#endif /* APP_COMMUNICATION_H */
