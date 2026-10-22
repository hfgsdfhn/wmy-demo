#include <assert.h>
#include <string.h>

#include "app_messages.h"

static uint32_t test_tick_ms;

uint32_t HAL_GetTick(void)
{
    return test_tick_ms;
}

void AppRtos_SetEventsFromISR(uint32_t flags)
{
    (void)flags;
}

bool AppRtos_ClearEvents(uint32_t flags)
{
    (void)flags;
    return true;
}

void AppRtos_CanErrorFromISR(bsp_can_t *can, uint32_t error_flags)
{
    (void)can;
    (void)error_flags;
}

bool AppChassis_OnCanFrame(const app_can_frame_t *frame)
{
    (void)frame;
    return true;
}

bool DM_IMU_ParseFeedback(const uint8_t *data, uint16_t len)
{
    (void)data;
    (void)len;
    return false;
}

bool Bsp_Uart_Send(bsp_uart_t *uart, uint8_t *data, uint16_t len)
{
    (void)uart;
    (void)data;
    (void)len;
    return true;
}

/* Include the implementation so this host test exercises the production parser. */
#include "../user_code/App/app_messages.c"

static void TestControlRoundTrip(void)
{
    app_dual_control_t source = {
        .control_mode = 1U,
        .action_id = 1U,
        .flags = (1U << 0U) | (1U << 2U),
        .rc_joy_left_x = 64,
        .rc_joy_left_y = -255,
        .rc_joy_right_x = 127,
        .rc_joy_right_y = -32,
        .shoulder_left = 64,
        .shoulder_right = 16
    };
    app_dual_control_t parsed;
    uint8_t frame[24];

    assert(AppMessages_BuildDualControl(&source, 0x3CU, frame, sizeof(frame)) == 24U);
    assert(frame[0] == 0x55U && frame[1] == 0xAAU);
    assert(frame[2] == 1U && frame[3] == 0x20U && frame[4] == 0x3CU);
    assert(frame[5] == 16U && frame[9] == 0U);

    AppMessages_Init();
    test_tick_ms = 17U;
    AppMessages_OnDualMcuRx(NULL, frame, 7U);
    AppMessages_OnDualMcuRx(NULL, &frame[7], 17U);
    assert(AppMessages_GetDualControl(&parsed));
    assert(parsed.sequence == 0x3CU);
    assert(parsed.rc_joy_left_x == 64 && parsed.rc_joy_left_y == -255);
    assert(parsed.shoulder_left == 64 && parsed.shoulder_right == 16);
}

static void TestRejectAndResynchronize(void)
{
    app_dual_control_t source = {
        .control_mode = 1U,
        .action_id = 1U,
        .flags = (1U << 0U) | (1U << 2U)
    };
    app_dual_control_t parsed;
    uint8_t frame[24];
    uint8_t damaged[24];

    assert(AppMessages_BuildDualControl(&source, 1U, frame, sizeof(frame)) == 24U);
    memcpy(damaged, frame, sizeof(damaged));
    damaged[10] = 0xFFU;
    damaged[11] = 0x7FU;

    AppMessages_Init();
    AppMessages_OnDualMcuRx(NULL, (const uint8_t *)"\x00\x55\x01", 3U);
    AppMessages_OnDualMcuRx(NULL, damaged, sizeof(damaged));
    assert(!AppMessages_GetDualControl(&parsed));

    frame[22] ^= 0x80U;
    AppMessages_OnDualMcuRx(NULL, frame, sizeof(frame));
    assert(!AppMessages_GetDualControl(&parsed));

    assert(AppMessages_BuildDualControl(&source, 2U, frame, sizeof(frame)) == 24U);
    AppMessages_OnDualMcuRx(NULL, frame, sizeof(frame));
    assert(AppMessages_GetDualControl(&parsed));
    assert(parsed.sequence == 2U);
}

static void TestStatusLayout(void)
{
    app_dual_status_t status = {
        .chassis_state = 3U,
        .current_action_id = 1U,
        .action_result = 1U,
        .fault_code = 0U,
        .status_flags = 0x00A3U
    };
    uint8_t frame[16];

    assert(AppMessages_BuildDualStatus(&status, 0xFFU, frame, sizeof(frame)) == 16U);
    assert(frame[3] == 0x21U && frame[4] == 0xFFU && frame[5] == 8U);
    assert(frame[10] == 0xA3U && frame[11] == 0x00U);
    assert(AppMessages_ParseDualStatus(frame, sizeof(frame), 100U));
}

static void PutU16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8U);
}

static void PutU32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8U);
    p[2] = (uint8_t)(value >> 16U);
    p[3] = (uint8_t)(value >> 24U);
}

static void BuildRemoteFrame(uint8_t *frame, uint16_t session, uint16_t sequence,
                             int16_t joy_x, int16_t joy_y, uint32_t keys)
{
    uint16_t crc;

    memset(frame, 0, 27U);
    frame[0] = 0xA5U;
    frame[1] = 0x5AU;
    frame[2] = 0x01U;
    frame[3] = 0x10U;
    PutU16(&frame[4], sequence);
    frame[6] = 18U;
    PutU16(&frame[7], session);
    PutU16(&frame[9], (uint16_t)joy_x);
    PutU16(&frame[11], (uint16_t)joy_y);
    PutU16(&frame[13], (uint16_t)-64);
    PutU16(&frame[15], 91U);
    PutU32(&frame[17], keys);
    frame[21] = APP_REMOTE_SWITCH_PC0_REGION_ENABLE;
    frame[22] = 0x03U;
    frame[23] = 0U;
    frame[24] = 0x07U;
    crc = crc16(&frame[2], 23U);
    PutU16(&frame[25], crc);
}

static void TestRemoteStateV1(void)
{
    uint8_t frame[27];
    app_remote_control_t parsed;

    AppMessages_Init();
    BuildRemoteFrame(frame, 42U, 0x1234U, 100, -100, (1UL << 19U));
    test_tick_ms = 10U;
    AppMessages_OnRemoteControlRx(NULL, frame, 5U);
    AppMessages_OnRemoteControlRx(NULL, &frame[5], 22U);
    assert(AppMessages_GetRemoteControl(&parsed));
    assert(parsed.session_id == 42U && parsed.sequence == 0x1234U);
    assert(parsed.joy_x == 100 && parsed.joy_y == -100);
    assert(parsed.joy_right_x == -64 && parsed.joy_right_y == 91);
    assert(parsed.keys == (1UL << 19U));
    assert((parsed.switches & APP_REMOTE_SWITCH_PC0_REGION_ENABLE) != 0U);
    assert(AppMessages_GetRemoteControlFreshness(109U) == APP_REMOTE_CONTROL_ACTIVE);
    assert(AppMessages_GetRemoteControlFreshness(110U) == APP_REMOTE_CONTROL_STOP);
    assert(AppMessages_GetRemoteControlFreshness(310U) == APP_REMOTE_CONTROL_UNAVAILABLE);

    BuildRemoteFrame(frame, 43U, 1U, 0, 0, 0U);
    test_tick_ms = 20U;
    AppMessages_OnRemoteControlRx(NULL, frame, sizeof(frame));
    assert(AppMessages_GetRemoteControl(&parsed));
    assert(parsed.session_id == 43U && parsed.lost_frames == 0U);

    BuildRemoteFrame(frame, 43U, 2U, 0, 0, 0U);
    PutU16(&frame[11], 256U); /* Valid CRC but invalid normalized axis. */
    PutU16(&frame[25], crc16(&frame[2], 23U));
    AppMessages_OnRemoteControlRx(NULL, frame, sizeof(frame));
    assert(AppMessages_GetRemoteControl(&parsed));
    assert(parsed.sequence == 1U);

    BuildRemoteFrame(frame, 43U, 3U, 0, 0, (1UL << 20U));
    AppMessages_OnRemoteControlRx(NULL, frame, sizeof(frame));
    assert(AppMessages_GetRemoteControl(&parsed));
    assert(parsed.sequence == 1U);
}

int main(void)
{
    TestControlRoundTrip();
    TestRejectAndResynchronize();
    TestStatusLayout();
    TestRemoteStateV1();
    return 0;
}
