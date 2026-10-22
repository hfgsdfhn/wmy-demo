#ifndef REMOTE_BUTTONS_H
#define REMOTE_BUTTONS_H

#include "app_messages.h"
#include "app_dt35_position.h"

#define PC7  (1UL << 0U)
#define PA8  (1UL << 3U)
#define PA11 (1UL << 6U)
#define PA12 (1UL << 7U)
#define PB8  (1UL << 13U)
#define PB5  (1UL << 11U)
#define PB6  (1UL << 12U)
#define PB9  (1UL << 8U)
#define PC13 (1UL << 9U)

void AppDt35Buttons_Scan(const app_remote_control_t *control);
bool AppDt35Buttons_TakePendingYawAction(uint8_t *action_id);
bool AppDt35Buttons_TakePendingYawZeroRequest(void);
void AppDt35Buttons_CancelRemote(void);

#endif /* REMOTE_BUTTONS_H */
