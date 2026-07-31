#include "mins_imu.h"

#include <string.h>

#define ERROR_K                         0.974f  //转动误差系数

#define MINS_IMU_YAW_OFFSET_DEG         2.84f   //上电固定飘移角

#define MINS_IMU_FRAME_HEAD             0x77U   //帧头
#define MINS_IMU_ADDR_CODE              0x00U   //地址码（不知道地址码就在上位机串口助手看一眼，每一帧的第三位是地址码）
#define MINS_IMU_FRAME_EULER_LENGTH     0x0DU   //回复欧拉角数据帧长度
#define MINS_IMU_RESPONSE_EULER         0x84U   //读取三轴欧拉角 接收
#define MINS_IMU_COMMAND_EULER          0x04U   //读取三轴欧拉角 发送
#define MINS_IMU_COMMAND_OUTPUT_TYPE    0x56U   //设置自动输出数据模式 发送/接收
#define MINS_IMU_COMMAND_OUTPUT_RATE    0x0CU   //设置自动输出频率 发送

/**
 * @brief 数据发送函数
 * 
 * @param imu         imu结构体指针
 * @param command     命令字
 * @param payload     数据域
 * @param payload_len 数据域长度
 */
static void Mins_Imu_SendCommand(mins_imu_t *imu,uint8_t command,const uint8_t *payload,uint8_t payload_len)
{
    uint8_t frame[8];       //存储数据帧
    uint8_t index;          //命令索引
    uint8_t checksum;       //校验和（数据长度+命令字）

    if ((imu == NULL) || (imu->uart == NULL))       //安全检测
    {
        return;
    }

    //数据拼接
    frame[0] = MINS_IMU_FRAME_HEAD;             //帧头
    frame[1] = (uint8_t)(4U + payload_len);     //数据长度
    frame[2] = MINS_IMU_ADDR_CODE;                           //地址码
    frame[3] = command;
    for (index = 0U; index < payload_len; index++)  //数据域的数据拼接
    {
        frame[4U + index] = payload[index];
    }

    //校验和计算
    checksum = 0U;
    for (index = 1U; index < (uint8_t)(4U + payload_len); index++)
    {
        checksum = (uint8_t)(checksum + frame[index]);
    }
    frame[4U + payload_len] = checksum;

    Bsp_Uart_Send(imu->uart, frame, (uint16_t)(5U + payload_len));//发送命令
}

/**
 * @brief 帧校验函数
 * 
 * @param frame         接收到的数据帧
 * @param len           数据帧长度（几个字节）
 * @return uint8_t      如果数据合规，返回校验和
 */
static uint8_t Mins_Imu_CheckFrame(const uint8_t *frame, uint16_t len)
{
    uint16_t index;
    uint8_t checksum;

    if ((len < 5U) || (frame[0] != MINS_IMU_FRAME_HEAD)         //数据帧长度不可能小于5，帧头必须是0x77
        || (len != (uint16_t)(frame[1] + 1U)))
    {
        return 0U;
    }

    checksum = 0U;
    for (index = 1U; index < (len - 1U); index++)
    {
        checksum = (uint8_t)(checksum + frame[index]);
    }
    return checksum == frame[len - 1U];
}

/**
 * @brief 数据帧里的欧拉角数据转化成浮点数
 * 
 * @param bcd 
 * @return float 
 */
static float Mins_Imu_DecodeAngle(const uint8_t *bcd)
{
    float angle;

    angle = (float)((bcd[0] & 0x0FU) * 100U);
    angle += (float)(((bcd[1] >> 4U) & 0x0FU) * 10U);
    angle += (float)(bcd[1] & 0x0FU);
    angle += (float)((bcd[2] >> 4U) & 0x0FU) * 0.1f;
    angle += (float)(bcd[2] & 0x0FU) * 0.01f;
    return ((bcd[0] & 0xF0U) == 0x10U) ? -angle : angle;
}

/**
 * @brief YAW角0°时容易跳变解决
 * 
 * @param yaw_delta 
 * @return float 
 */
static float Mins_Imu_WrapYawDelta(float yaw_delta)
{
    while (yaw_delta >= 180.0f)
    {
        yaw_delta -= 360.0f;
    }
    while (yaw_delta < -180.0f)
    {
        yaw_delta += 360.0f;
    }
    return yaw_delta;
}

/**
 * @brief imu初始化，传入imu结构体指针，选择串口
 * 
 * @param imu   
 * @param uart 
 */
void Mins_Imu_Init(mins_imu_t *imu, bsp_uart_t *uart)
{
    if ((imu == NULL) || (uart == NULL))
    {
        return;
    }

    memset(imu, 0, sizeof(*imu));
    imu->uart = uart;
}

//读取欧拉角（其实三个角都读取了）
void Mins_Imu_RequestEuler(mins_imu_t *imu)
{
    Mins_Imu_SendCommand(imu, MINS_IMU_COMMAND_EULER, NULL, 0U);
}

/**
 * @brief 设置自动发送数据模式（固定发送欧拉角，别的数据没写）
 * 
 * @param imu       imu结构体
 * @param rate      发送频率
 */
void Mins_Imu_SetAutoOutput(mins_imu_t *imu, mins_imu_rate_t rate)
{
    uint8_t value;

    value = 0x00U;
    Mins_Imu_SendCommand(imu, MINS_IMU_COMMAND_OUTPUT_TYPE, &value, 1U);

    value = (uint8_t)rate;
    Mins_Imu_SendCommand(imu, MINS_IMU_COMMAND_OUTPUT_RATE, &value, 1U);
}

/**
 * @brief 回包数据解析
 * 
 * @param imu   imu结构体
 * @param data 
 * @param len 
 */
void Mins_Imu_OnRx(mins_imu_t *imu, const uint8_t *data, uint16_t len)
{
    uint16_t offset;
    uint16_t frame_len;

    if ((imu == NULL) || (data == NULL))
    {
        return;
    }

    for (offset = 0U; offset + 2U <= len; offset++)
    {
        if (data[offset] != MINS_IMU_FRAME_HEAD)        //检查帧头对不对
        {
            continue;
        }

        frame_len = (uint16_t)(data[offset + 1U] + 1U);

        if ((data[offset + 1U] == MINS_IMU_FRAME_EULER_LENGTH)
            && (data[offset + 3U] == MINS_IMU_RESPONSE_EULER)
            && Mins_Imu_CheckFrame(&data[offset], frame_len))
        {
            float raw_yaw;

            //将数据转化为浮点数并传入结构体
            imu->euler.pitch = Mins_Imu_DecodeAngle(&data[offset + 4U]);
            imu->euler.roll = Mins_Imu_DecodeAngle(&data[offset + 7U]);
            raw_yaw = Mins_Imu_DecodeAngle(&data[offset + 10U]);

            if (imu->yaw_last_valid == 0U)      //上电后开始角度累加计数
            {
                imu->yaw_last_raw = raw_yaw;
                imu->yaw_total = 0.0f;
                imu->yaw_last_valid = 1U;
            }
            else
            {

                imu->yaw_total -= Mins_Imu_WrapYawDelta(raw_yaw - imu->yaw_last_raw);   //角度累加（总角度减去变化角度）
                imu->yaw_last_raw = raw_yaw;
            }
            imu->euler.yaw = (imu->yaw_total * ERROR_K) + MINS_IMU_YAW_OFFSET_DEG;      //imu误差修正
            imu->update_count++;                                                        //可能没什么用但是还是留着吧
        }

        offset = (uint16_t)(offset + frame_len - 1U);
    }
}
