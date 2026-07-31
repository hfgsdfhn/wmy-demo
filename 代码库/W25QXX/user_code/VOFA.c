/**
 * @file VOFA.c
 * @author 王梦阳 wmy07823@163.com
 * @brief VOFA+ JustFloat协议驱动实现
 * @version 0.1
 * @date 2026-07-25
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "vofa.h"

#include <string.h>

/* JustFloat 协议帧尾，接收端以此识别帧边界 */
#define VOFA_TAIL_BYTE_0 0x00U
#define VOFA_TAIL_BYTE_1 0x00U
#define VOFA_TAIL_BYTE_2 0x80U
#define VOFA_TAIL_BYTE_3 0x7FU

/**
 * @brief 发送原始浮点数据帧（内部函数，不添加时间戳）
 * @param vofa VOFA 实例指针
 * @param data 浮点数据数组
 * @param num 浮点数据个数
 * @retval true  发送成功
 * @retval false 参数无效或发送失败
 */
static bool Vofa_SendRaw(vofa_t *vofa, const float *data, uint16_t num)
{
    uint16_t data_len;
    uint16_t frame_len;
    bool result;

    /* 参数合法性检查 */
    if ((vofa == NULL) || (vofa->uart == NULL) || (data == NULL) || (num == 0U)
        || (num > VOFA_MAX_FLOATS))
    {
        if (vofa != NULL)
        {
            vofa->stat.error_count++;
        }
        return false;
    }

    /* 组装 JustFloat 帧：数据 + 4 字节帧尾 */
    data_len = (uint16_t)(num * sizeof(float));
    frame_len = (uint16_t)(data_len + VOFA_FRAME_TAIL_SIZE);
    memcpy(vofa->tx_buf, data, data_len);
    vofa->tx_buf[data_len] = VOFA_TAIL_BYTE_0;
    vofa->tx_buf[data_len + 1U] = VOFA_TAIL_BYTE_1;
    vofa->tx_buf[data_len + 2U] = VOFA_TAIL_BYTE_2;
    vofa->tx_buf[data_len + 3U] = VOFA_TAIL_BYTE_3;

    /* 通过 UART 发送整帧，根据结果更新统计计数 */
    result = Bsp_Uart_Send(vofa->uart, vofa->tx_buf, frame_len);
    if (result)
    {
        vofa->stat.send_count++;
    }
    else if ((vofa->uart->tx_mode == BSP_UART_TX_DMA) && (vofa->uart->tx_busy != 0U))
    {
        /* DMA 模式下发送失败但总线忙，属于正常竞争，仅计数不报错 */
        vofa->stat.busy_count++;
    }
    else
    {
        vofa->stat.error_count++;
    }
    return result;
}

/**
 * @brief 初始化 VOFA 实例
 * @param vofa VOFA 实例指针
 * @param uart 绑定的 UART 接口指针
 * @retval true  初始化成功
 * @retval false 参数为空
 */
bool Vofa_Init(vofa_t *vofa, bsp_uart_t *uart)
{
    if ((vofa == NULL) || (uart == NULL))
    {
        return false;
    }

    /* 清零全部状态并绑定 UART */
    memset(vofa, 0, sizeof(*vofa));
    vofa->uart = uart;

    /* 记录启动时刻，用于时间戳计算 */
    vofa->start_ms = HAL_GetTick();
    vofa->last_send_ms = vofa->start_ms;
    return true;
}

/**
 * @brief 发送单个浮点数（自动添加时间戳）
 * @param vofa VOFA 实例指针
 * @param value 要发送的浮点数
 * @retval true  发送成功
 * @retval false 参数无效或发送失败
 */
bool Vofa_SendFloat(vofa_t *vofa, float value)
{
    return Vofa_SendArray(vofa, &value, 1U);
}

/**
 * @brief 发送浮点数组（自动在首通道添加相对时间戳）
 * @param vofa VOFA 实例指针
 * @param data 浮点数据数组指针
 * @param num  数据个数（需小于 VOFA_MAX_FLOATS）
 * @retval true  发送成功
 * @retval false 参数无效或发送失败
 */
bool Vofa_SendArray(vofa_t *vofa, float *data, uint16_t num)
{
    float frame_data[VOFA_MAX_FLOATS];

    /* 参数合法性检查，num >= VOFA_MAX_FLOATS 是因为首通道要占一个位置 */
    if ((vofa == NULL) || (data == NULL) || (num == 0U) || (num >= VOFA_MAX_FLOATS))
    {
        if (vofa != NULL)
        {
            vofa->stat.error_count++;
        }
        return false;
    }

    /* 首通道填入从初始化时刻开始计时的相对时间（秒） */
    frame_data[0] = (float)(HAL_GetTick() - vofa->start_ms) / 1000.0f;
    /* 后续通道拷贝用户数据 */
    memcpy(&frame_data[1], data, (size_t)num * sizeof(float));
    return Vofa_SendRaw(vofa, frame_data, (uint16_t)(num + 1U));
}

/**
 * @brief 设置 VOFA 自动发送周期
 * @param vofa      VOFA 实例指针
 * @param period_ms 发送周期（毫秒），设为 0 则停止自动发送
 */
void Vofa_SetPeriod(vofa_t *vofa, uint16_t period_ms)
{
    if (vofa != NULL)
    {
        vofa->period_ms = period_ms;
        /* 重置计时起点，避免立即触发一次发送 */
        vofa->last_send_ms = HAL_GetTick();
    }
}

/**
 * @brief VOFA 周期任务，需在主循环或定时器中持续调用
 *
 * 根据 period_ms 判断是否到达发送时刻，到期则自动发送全部注册通道的当前值。
 *
 * @param vofa VOFA 实例指针
 */
void Vofa_Process(vofa_t *vofa)
{
    uint32_t now;

    /* 未配置周期或未注册通道时直接返回 */
    if ((vofa == NULL) || (vofa->period_ms == 0U) || (vofa->channel_count == 0U))
    {
        return;
    }

    now = HAL_GetTick();
    /* 检查是否到达下一个发送周期 */
    if ((uint32_t)(now - vofa->last_send_ms) >= vofa->period_ms)
    {
        vofa->last_send_ms = now;
        Vofa_SendChannels(vofa);
    }
}

/**
 * @brief 获取 VOFA 发送统计信息
 * @param vofa VOFA 实例指针
 * @retval 非空  指向统计结构体的指针
 * @retval NULL  参数为空
 */
const vofa_stat_t *Vofa_GetStat(const vofa_t *vofa)
{
    return (vofa == NULL) ? NULL : &vofa->stat;
}

/**
 * @brief 注册一个数据通道，将变量地址绑定到通道名称
 *
 * 调用 Vofa_SendChannels() 时会自动读取该地址的最新值并发送。
 *
 * @param vofa  VOFA 实例指针
 * @param name  通道名称（上位机显示用，最长 15 字符）
 * @param value 指向变量地址的指针（发送时读取该地址的值）
 * @retval true  注册成功
 * @retval false 参数无效或通道已满
 */
bool Vofa_Register(vofa_t *vofa, const char *name, float *value)
{
    vofa_channel_t *channel;

    if ((vofa == NULL) || (name == NULL) || (value == NULL)
        || (vofa->channel_count >= VOFA_MAX_CHANNEL))
    {
        return false;
    }

    /* 写入通道名称并确保字符串结尾 */
    channel = &vofa->channels[vofa->channel_count];
    strncpy(channel->name, name, sizeof(channel->name) - 1U);
    channel->name[sizeof(channel->name) - 1U] = '\0';
    /* 保存变量地址，发送时解引用读取最新值 */
    channel->value = value;
    vofa->channel_count++;
    return true;
}

/**
 * @brief 发送所有已注册通道的当前值（自动添加时间戳）
 * @param vofa VOFA 实例指针
 * @retval true  发送成功
 * @retval false 无已注册通道或发送失败
 */
bool Vofa_SendChannels(vofa_t *vofa)
{
    float values[VOFA_MAX_CHANNEL];
    uint8_t index;

    if ((vofa == NULL) || (vofa->channel_count == 0U))
    {
        return false;
    }

    /* 遍历所有已注册通道，读取各变量当前值 */
    for (index = 0U; index < vofa->channel_count; index++)
    {
        values[index] = *vofa->channels[index].value;
    }

    return Vofa_SendArray(vofa, values, vofa->channel_count);
}
