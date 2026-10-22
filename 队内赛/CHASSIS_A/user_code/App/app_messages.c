#include "app_messages.h"

#include <string.h>

#include "DM_Imu.h"
#include "app_chassis.h"
#include "app_init.h"
#include "app_monitor.h"
#include "app_rtos.h"
#include "bsp_can.h"
#include "bsp_uart.h"

static uint16_t crc16(const uint8_t *data, uint16_t len);
static uint16_t u16(const uint8_t *p);
static int16_t i16(const uint8_t *p);

/* IMU */
static uint32_t imu_last_ms;
static bool imu_online;

static uint32_t dt35_last_ms;
static uint32_t dual_last_ms;
static uint32_t dual_control_last_ms;

void AppMessages_OnImuRx(bsp_uart_t *uart, const uint8_t *data, uint16_t len)
{
    (void)uart;

    if (DM_IMU_ParseFeedback(data, len)) {
        uint32_t now = HAL_GetTick();
        imu_last_ms = now;
        imu_online = true;
        AppRtos_SetEventsFromISR(APP_EVENT_IMU_ONLINE);
    }
}

/* DT35 */
#define DT35_SIZE 11U /* 单个 DT35 测距数据帧字节数 */
static uint8_t dt35_frame[DT35_SIZE];
static uint8_t dt35_length;
static app_dt35_sample_t dt35_sample;
static volatile bool dt35_valid;

bool AppMessages_ParseDt35(const uint8_t *data, uint16_t len, uint32_t timestamp_ms)
{
    uint16_t i;

    if (data == NULL) {
        return false;
    }

    for (i = 0U; i < len; ++i) {
        if (dt35_length == 0U) {
            if (data[i] == 0xAAU) {
                dt35_frame[dt35_length++] = data[i];
            }
            continue;
        }

        if ((dt35_length == 1U) && (data[i] != 0x55U)) {
            dt35_length = (data[i] == 0xAAU) ? 1U : 0U;
            if (dt35_length) {
                dt35_frame[0] = data[i];
            }
            continue;
        }

        dt35_frame[dt35_length++] = data[i];

        if (dt35_length == DT35_SIZE) {
            uint8_t sum = 0U;
            float x;
            float y;

            for (uint8_t j = 0U; j < 10U; ++j) {
                sum = (uint8_t)(sum + dt35_frame[j]);
            }

            dt35_length = 0U;

            if (sum != dt35_frame[10]) {
                continue;
            }

            memcpy(&x, &dt35_frame[2], sizeof(x));
            memcpy(&y, &dt35_frame[6], sizeof(y));
            dt35_sample.timestamp_ms = timestamp_ms;
            dt35_sample.distance_m[0] = x * 0.001f;
            dt35_sample.distance_m[1] = y * 0.001f;
            dt35_sample.valid = 1U;
            dt35_valid = true;
            dt35_last_ms = timestamp_ms;
            AppRtos_SetEventsFromISR(APP_EVENT_DT35_ONLINE);
            return true;
        }
    }

    return false;
}

void AppMessages_OnDt35Rx(bsp_uart_t *uart, const uint8_t *data, uint16_t len)
{
    (void)uart;
    (void)AppMessages_ParseDt35(data, len, HAL_GetTick());
}

bool AppMessages_GetDt35(app_dt35_sample_t *sample)
{
    if (!sample || !dt35_valid) {
        return false;
    }

    *sample = dt35_sample;
    return true;
}

/* Power-board RC_STATE passthrough received on USART1. */
#define REMOTE_FRAME_MAX 27U
#define REMOTE_PAYLOAD_SIZE 18U
#define REMOTE_STOP_TIMEOUT_MS 500U
#define REMOTE_OFFLINE_TIMEOUT_MS 500U
#define REMOTE_FLAG_ARMED 0x01U
#define REMOTE_FLAG_FAULT 0x04U

static uint8_t remote_frame[REMOTE_FRAME_MAX];
static uint8_t remote_length;
static uint8_t remote_expected;
static app_remote_control_t remote_control;
static volatile bool remote_control_valid;
static uint32_t remote_control_last_ms;
static uint16_t remote_last_sequence;
static bool remote_last_sequence_valid;

static uint32_t u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8U)
         | ((uint32_t)p[2] << 16U) | ((uint32_t)p[3] << 24U);
}

static bool AppMessages_ParseRemoteControl(const uint8_t *f, uint8_t len)
{
    uint8_t payload_len;
    uint16_t sequence;
    uint16_t session_id;
    int16_t joy_x;
    int16_t joy_y;

    if ((f == NULL) || (len != REMOTE_FRAME_MAX) || (f[0] != 0xA5U) || (f[1] != 0x5AU)
        || (f[2] != 0x01U) || (f[3] != 0x10U)) return false;

    payload_len = f[6];
    if ((payload_len != REMOTE_PAYLOAD_SIZE)
        || (len != (uint8_t)(payload_len + 9U))
        || (u16(&f[len - 2U]) != crc16(&f[2], (uint16_t)(len - 4U)))) return false;

    sequence = u16(&f[4]);
    session_id = u16(&f[7]);
    joy_x = i16(&f[9]);
    joy_y = i16(&f[11]);
    if ((joy_x < -255) || (joy_x > 255) || (joy_y < -255) || (joy_y > 255)) return false;

    if (session_id != remote_control.session_id) {
        remote_control.lost_frames = 0U;
        remote_last_sequence_valid = false;
    }
    if (remote_last_sequence_valid) {
        remote_control.lost_frames += (uint16_t)(sequence - remote_last_sequence - 1U);
    }

    remote_control.session_id = session_id;
    remote_control.joy_x = joy_x;
    remote_control.joy_y = joy_y;
    remote_control.joy_right_x = i16(&f[13]);
    remote_control.joy_right_y = i16(&f[15]);
    if ((remote_control.joy_right_x < -255) || (remote_control.joy_right_x > 255)
        || (remote_control.joy_right_y < -255) || (remote_control.joy_right_y > 255)) return false;
    remote_control.keys = u32(&f[17]);
    remote_control.switches = f[21];
    remote_control.flags = f[22];
    remote_control.fault = f[23];
    remote_control.desired_power = f[24];
    if ((remote_control.keys & 0xFFF00000UL) != 0U
        || (remote_control.switches & 0xC0U) != 0U
        || (remote_control.flags & 0xF8U) != 0U
        || (remote_control.desired_power & 0xF8U) != 0U) return false;
    remote_control.sequence = sequence;
    remote_control.timestamp_ms = HAL_GetTick();
    remote_last_sequence = sequence;
    remote_last_sequence_valid = true;
    remote_control_last_ms = remote_control.timestamp_ms;
    remote_control_valid = true;
    return true;
}

void AppMessages_OnRemoteControlRx(bsp_uart_t *uart, const uint8_t *data, uint16_t len)
{
    (void)uart;
    if (data == NULL) return;
    for (uint16_t i = 0U; i < len; ++i) {
        if (remote_length == 0U) {
            if (data[i] == 0xA5U) remote_frame[remote_length++] = data[i];
            continue;
        }
        if ((remote_length == 1U) && (data[i] != 0x5AU)) {
            remote_length = (data[i] == 0xA5U) ? 1U : 0U;
            if (remote_length != 0U) remote_frame[0] = data[i];
            continue;
        }
        remote_frame[remote_length++] = data[i];
        if (remote_length == 7U) {
            remote_expected = (uint8_t)(remote_frame[6] + 9U);
            if (remote_expected != REMOTE_FRAME_MAX) {
                remote_length = 0U;
                remote_expected = 0U;
            }
        }
        if ((remote_expected != 0U) && (remote_length == remote_expected)) {
            (void)AppMessages_ParseRemoteControl(remote_frame, remote_length);
            remote_length = 0U;
            remote_expected = 0U;
        }
    }
}

bool AppMessages_GetRemoteControl(app_remote_control_t *control)
{
    if ((control == NULL) || !remote_control_valid) return false;
    *control = remote_control;
    return true;
}

app_remote_control_freshness_t AppMessages_GetRemoteControlFreshness(uint32_t now)
{
    uint32_t age;
    if (!remote_control_valid) return APP_REMOTE_CONTROL_UNAVAILABLE;
    age = now - remote_control_last_ms;
    if (age < REMOTE_STOP_TIMEOUT_MS) return APP_REMOTE_CONTROL_ACTIVE;
    if (age < REMOTE_OFFLINE_TIMEOUT_MS) return APP_REMOTE_CONTROL_STOP;
    return APP_REMOTE_CONTROL_UNAVAILABLE;
}

/* Upper/lower MCU */
#define APP_MESSAGES_ENABLE_UPPER_CONTROL 0U
#define DUAL_MAX 24U                        /* 双 MCU 通信帧最大字节数 */
#define DUAL_CONTROL_ID 0x20U               /* 双 MCU 控制帧 ID */
#define DUAL_STATUS_ID 0x21U                /* 双 MCU 状态帧 ID */
#define DUAL_CONTROL_STOP_TIMEOUT_MS 100U   /* 双 MCU 控制失效后的停车超时，单位 ms */
#define DUAL_CONTROL_OFFLINE_TIMEOUT_MS 300U /* 双 MCU 离线超时，单位 ms */
static uint8_t dual_frame[DUAL_MAX];
static uint16_t dual_length;
static uint16_t dual_expected;
static app_dual_control_t dual_control;
static app_dual_status_t dual_status;
static volatile bool dual_control_valid;
static volatile bool dual_status_valid;
static uint8_t dual_sequence;
static uint8_t dual_status_sequence;

bool AppMessages_IsUpperControlEnabled(void)
{
    return APP_MESSAGES_ENABLE_UPPER_CONTROL != 0U;
}

static uint16_t crc16(const uint8_t *data, uint16_t len)
{
    uint16_t c = 0xFFFFU;

    for (uint16_t i = 0U; i < len; i++) {
        c ^= data[i];
        for (uint8_t b = 0U; b < 8U; b++) {
            c = (c & 1U)
                    ? (uint16_t)((c >> 1U) ^ 0xA001U)
                    : (uint16_t)(c >> 1U);
        }
    }

    return c;
}

static uint16_t u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8U);
}

static int16_t i16(const uint8_t *p)
{
    return (int16_t)u16(p);
}

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8U);
}

bool AppMessages_ParseDualControl(const uint8_t *f, uint16_t len)
{
    if (!f || len != 24U || f[0] != 0x55U || f[1] != 0xAAU || f[2] != 1U
        || f[3] != DUAL_CONTROL_ID || f[5] != 16U
        || u16(&f[22]) != crc16(&f[2], 20U)) {
        return false;
    }

    if ((i16(&f[10]) < -255) || (i16(&f[10]) > 255)
        || (i16(&f[12]) < -255) || (i16(&f[12]) > 255)
        || (i16(&f[14]) < -255) || (i16(&f[14]) > 255)
        || (i16(&f[16]) < -255) || (i16(&f[16]) > 255)
        || (i16(&f[18]) < -64) || (i16(&f[18]) > 64)
        || (i16(&f[20]) < -64) || (i16(&f[20]) > 64)) {
        return false;
    }

    if (f[9] != 0U) {
        return false;
    }

    dual_control.control_mode = f[6];
    dual_control.action_id = f[7];
    dual_control.flags = f[8];
    dual_control.rc_joy_left_x = i16(&f[10]);
    dual_control.rc_joy_left_y = i16(&f[12]);
    dual_control.rc_joy_right_x = i16(&f[14]);
    dual_control.rc_joy_right_y = i16(&f[16]);
    dual_control.shoulder_left = i16(&f[18]);
    dual_control.shoulder_right = i16(&f[20]);
    dual_control.timestamp_ms = HAL_GetTick();
    dual_control.sequence = f[4];
    dual_control_valid = true;
    dual_control_last_ms = dual_control.timestamp_ms;
    if (((dual_control.flags & (1U << 1U)) != 0U)
        || (dual_control.control_mode == 4U)) {
        AppRtos_SetEventsFromISR(APP_EVENT_ESTOP);
    }
    AppRtos_SetEventsFromISR(APP_EVENT_MAIN_CTRL_ONLINE
                              | APP_EVENT_DUAL_CONTROL_ONLINE);

    return true;
}

bool AppMessages_ParseDualStatus(const uint8_t *f, uint16_t len, uint32_t timestamp_ms)
{
    if (!f || len != 16U || f[0] != 0x55U || f[1] != 0xAAU || f[2] != 1U
        || f[3] != DUAL_STATUS_ID || f[5] != 8U
        || u16(&f[14]) != crc16(&f[2], 12U)) {
        return false;
    }

    dual_status.chassis_state = f[6];
    dual_status.current_action_id = f[7];
    dual_status.action_result = f[8];
    dual_status.fault_code = f[9];
    dual_status.status_flags = u16(&f[10]);
    dual_status.timestamp_ms = timestamp_ms;
    dual_status.sequence = f[4];
    dual_status_valid = true;
    dual_last_ms = timestamp_ms;
    AppRtos_SetEventsFromISR(APP_EVENT_MAIN_CTRL_ONLINE);

    return true;
}

void AppMessages_OnDualMcuRx(bsp_uart_t *uart, const uint8_t *data, uint16_t len)
{
    (void)uart;

    if (data == NULL) {
        return;
    }

    for (uint16_t i = 0U; i < len; i++) {
        if (dual_length == 0U) {
            if (data[i] == 0x55U) {
                dual_frame[dual_length++] = data[i];
            }
            continue;
        }

        if (dual_length == 1U && data[i] != 0xAAU) {
            dual_length = (data[i] == 0x55U) ? 1U : 0U;
            if (dual_length) {
                dual_frame[0] = data[i];
            }
            continue;
        }

        dual_frame[dual_length++] = data[i];

        if (dual_length == 6U) {
            dual_expected = (uint16_t)(8U + dual_frame[5]);
            if (dual_frame[2] != 1U || dual_expected > DUAL_MAX) {
                dual_length = 0U;
                dual_expected = 0U;
            }
        }

        if (dual_expected && dual_length == dual_expected) {
            if (dual_frame[3] == DUAL_CONTROL_ID) {
                (void)AppMessages_ParseDualControl(dual_frame, dual_length);
            } else if (dual_frame[3] == DUAL_STATUS_ID) {
                (void)AppMessages_ParseDualStatus(dual_frame, dual_length, HAL_GetTick());
            }

            dual_length = 0U;
            dual_expected = 0U;
        } else if (dual_length >= DUAL_MAX) {
            dual_length = 0U;
            dual_expected = 0U;
        }
    }
}

bool AppMessages_GetDualControl(app_dual_control_t *v)
{
    if (!v || !dual_control_valid) {
        return false;
    }

    *v = dual_control;
    return true;
}

app_dual_control_freshness_t AppMessages_GetDualControlFreshness(uint32_t now)
{
    uint32_t age;

    if (!dual_control_valid) {
        return APP_DUAL_CONTROL_UNAVAILABLE;
    }

    age = now - dual_control_last_ms;
    if (age < DUAL_CONTROL_STOP_TIMEOUT_MS) {
        return APP_DUAL_CONTROL_ACTIVE;
    }
    if (age < DUAL_CONTROL_OFFLINE_TIMEOUT_MS) {
        return APP_DUAL_CONTROL_STOP;
    }
    return APP_DUAL_CONTROL_UNAVAILABLE;
}

bool AppMessages_GetDualStatus(app_dual_status_t *v)
{
    if (!v || !dual_status_valid) {
        return false;
    }

    *v = dual_status;
    return true;
}

static uint16_t build(uint8_t id, const uint8_t *p, uint8_t n, uint8_t seq,
                      uint8_t *f, uint16_t cap)
{
    uint16_t total = (uint16_t)(n + 8U);

    if (!f || !p || cap < total) {
        return 0U;
    }

    f[0] = 0x55U;
    f[1] = 0xAAU;
    f[2] = 1U;
    f[3] = id;
    f[4] = seq;
    f[5] = n;
    memcpy(&f[6], p, n);
    put16(&f[6 + n], crc16(&f[2], (uint16_t)(4U + n)));
    return total;
}

uint16_t AppMessages_BuildDualControl(const app_dual_control_t *v, uint8_t seq,
                                      uint8_t *f, uint16_t cap)
{
    uint8_t p[16] = {0};

    if (!v) {
        return 0U;
    }

    p[0] = v->control_mode;
    p[1] = v->action_id;
    p[2] = v->flags;
    put16(&p[4], (uint16_t)v->rc_joy_left_x);
    put16(&p[6], (uint16_t)v->rc_joy_left_y);
    put16(&p[8], (uint16_t)v->rc_joy_right_x);
    put16(&p[10], (uint16_t)v->rc_joy_right_y);
    put16(&p[12], (uint16_t)v->shoulder_left);
    put16(&p[14], (uint16_t)v->shoulder_right);

    return build(DUAL_CONTROL_ID, p, 16U, seq, f, cap);
}

uint16_t AppMessages_BuildDualStatus(const app_dual_status_t *v, uint8_t seq,
                                     uint8_t *f, uint16_t cap)
{
    uint8_t p[8] = {0};

    if (!v) {
        return 0U;
    }

    p[0] = v->chassis_state;
    p[1] = v->current_action_id;
    p[2] = v->action_result;
    p[3] = v->fault_code;
    put16(&p[4], v->status_flags);

    return build(DUAL_STATUS_ID, p, 8U, seq, f, cap);
}

bool AppMessages_SendDualControl(bsp_uart_t *uart, const app_dual_control_t *v)
{
    uint8_t f[24];
    uint16_t n = AppMessages_BuildDualControl(v, dual_sequence++, f, sizeof(f));

    return n && Bsp_Uart_Send(uart, f, n);
}

bool AppMessages_SendDualStatus(bsp_uart_t *uart, const app_dual_status_t *v)
{
    uint8_t f[16];
    uint16_t n = AppMessages_BuildDualStatus(v, dual_status_sequence++, f, sizeof(f));

    return n && Bsp_Uart_Send(uart, f, n);
}

/* CAN callbacks */
void AppMessages_OnCanRx(bsp_can_t *can, uint32_t id, bool ext,
                         const uint8_t *data, uint8_t len)
{
    app_can_frame_t f = {0};

    f.timestamp_ms = HAL_GetTick();
    f.identifier = id;
    f.bus = (app_can_bus_t)can->instance_index;
    f.is_extended_id = ext ? 1U : 0U;
    f.length = len;
    memcpy(f.data, data, len);
    (void)AppChassis_OnCanFrame(&f);
}

void AppMessages_OnCanError(bsp_can_t *can, uint32_t error)
{
    AppRtos_CanErrorFromISR(can, error);
    AppRtos_SetEventsFromISR(APP_EVENT_CAN_FAULT);
}

void AppMessages_Init(void)
{
    memset(&dt35_sample, 0, sizeof(dt35_sample));
    dt35_length = 0U;
    dual_length = 0U;
    dual_expected = 0U;
    dual_control_valid = false;
    dual_status_valid = false;
    memset(&remote_control, 0, sizeof(remote_control));
    remote_length = 0U;
    remote_expected = 0U;
    remote_control_valid = false;
    remote_control_last_ms = 0U;
    remote_last_sequence_valid = false;
    imu_online = false;
    dt35_last_ms = 0U;
    dual_last_ms = 0U;
    dual_control_last_ms = 0U;
    dual_status_sequence = 0U;
}

void AppMessages_CheckTimeout(uint32_t now)
{
    if (imu_online && ((uint32_t)(now - imu_last_ms) >= 500U)) {
        imu_online = false;
        AppRtos_ClearEvents(APP_EVENT_IMU_ONLINE);
    }
    if (dt35_valid && ((uint32_t)(now - dt35_last_ms) >= 500U)) {
        dt35_valid = false;
        AppRtos_ClearEvents(APP_EVENT_DT35_ONLINE);
    }
    if (dual_control_valid
        && ((uint32_t)(now - dual_control_last_ms) >= DUAL_CONTROL_OFFLINE_TIMEOUT_MS)) {
        dual_control_valid = false;
        AppRtos_ClearEvents(APP_EVENT_MAIN_CTRL_ONLINE
                            | APP_EVENT_DUAL_CONTROL_ONLINE);
    }
    if (remote_control_valid
        && ((uint32_t)(now - remote_control_last_ms) >= REMOTE_OFFLINE_TIMEOUT_MS)) {
        remote_control_valid = false;
    }
}

uint8_t AppMessages_GetOnlineMask(void)
{
    uint8_t mask = 0U;

    if (dt35_valid) mask |= 1U << 1;
    if (dual_control_valid) mask |= 1U << 2;
    if (remote_control_valid) mask |= 1U << 4;
    if (remote_control_valid && ((remote_control.flags & REMOTE_FLAG_ARMED) != 0U)) {
        mask |= 1U << 5;
    }
    if (remote_control_valid
        && ((remote_control.flags & REMOTE_FLAG_FAULT) != 0U)) {
        mask |= 1U << 6;
    }
    if (imu_online) mask |= 1U << 3;
    return mask;
}
