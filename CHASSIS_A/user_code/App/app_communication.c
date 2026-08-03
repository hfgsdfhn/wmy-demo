#include "app_communication.h"

#include <stddef.h>
#include <string.h>

static app_comm_packet_t app_communication_packets[APP_COMM_SOURCE_COUNT];
static bool app_communication_valid[APP_COMM_SOURCE_COUNT];

bool AppCommunication_Init(void)
{
    memset(app_communication_packets, 0, sizeof(app_communication_packets));
    memset(app_communication_valid, 0, sizeof(app_communication_valid));
    return true;
}

void AppCommunication_OnPacket(const app_comm_packet_t *packet)
{
    if ((packet == NULL) || (packet->source >= APP_COMM_SOURCE_COUNT)
        || (packet->length > APP_COMM_PAYLOAD_MAX_SIZE))
    {
        return;
    }

    app_communication_packets[packet->source] = *packet;
    app_communication_valid[packet->source] = true;
}

bool AppCommunication_GetLastPacket(app_comm_source_t source,
                                    app_comm_packet_t *packet)
{
    if ((source >= APP_COMM_SOURCE_COUNT) || (packet == NULL)
        || !app_communication_valid[source])
    {
        return false;
    }
    *packet = app_communication_packets[source];
    return true;
}
