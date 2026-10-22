#include "user_main.h"

#include "INA226.h"
#include "bsp_i2c.h"
#include "bsp_uart.h"
#include "i2c.h"
#include "usart.h"
#include <string.h>

#define DT35_OUTPUT_VOLTAGE_MIN_V       0.0f
#define DT35_OUTPUT_VOLTAGE_MAX_V       10.0f

#define DT35_MIN_DISTANCE_MM            50.0f  //最小检测距离  
#define DT35_MAX_DISTANCE_MM            4000.0f  //最大检测距离

#define DT35_SAMPLE_PERIOD_MS           10U     //采样周期
#define DT35_INA226_I2C_ADDRESS_1         0x40U   // DT35-1：底盘左侧
#define DT35_INA226_I2C_ADDRESS_2         0x41U   // DT35-2：底盘后侧

static bsp_i2c_t g_i2c_1;
static bsp_i2c_t g_i2c_2;
static ina226_t g_ina226_1;
static ina226_t g_ina226_2;
static bsp_uart_t g_uart;

float g_voltage_v_x;               // DT35-1（左侧）传感器返回的电压
float g_voltage_v_y;               // DT35-2（后侧）传感器返回的电压
float g_distance_mm_y;             // DT35-2（后侧）测出的距离
float g_distance_mm_x;             // DT35-1（左侧）测出的距离
char a = 1;
static uint32_t g_last_sample_tick;

/**
 * @brief 用于将电压值转化为距离
 * 
 * @param voltage_v 
 * @return float 
 */
static float Dt35_VoltageToDistance(float voltage_v)
{
    float ratio;

    ratio = (voltage_v - DT35_OUTPUT_VOLTAGE_MIN_V)
            / (DT35_OUTPUT_VOLTAGE_MAX_V - DT35_OUTPUT_VOLTAGE_MIN_V);      //求电压到距离这个直线的斜率

    return DT35_MIN_DISTANCE_MM + ratio * (DT35_MAX_DISTANCE_MM - DT35_MIN_DISTANCE_MM);
}

static uint8_t UserMain_CalculateChecksum(const uint8_t *data, uint16_t len)
{
    uint32_t sum = 0U;

    for (uint16_t i = 0U; i < len; i++)
    {
        sum += data[i];
    }

    return (uint8_t)(sum & 0xFFU);
}

static bool UserMain_SendDistanceFrame(float x_mm, float y_mm)
{
    uint8_t frame[11];

    frame[0] = 0xAAU;
    frame[1] = 0x55U;
    memcpy(&frame[2], &x_mm, sizeof(x_mm));
    memcpy(&frame[6], &y_mm, sizeof(y_mm));
    frame[10] = UserMain_CalculateChecksum(frame, 10U);

    return Bsp_Uart_Send(&g_uart, frame, sizeof(frame));
}

void UserMain_Init(void)
{
    BspI2c_Init(&g_i2c_1, &hi2c2);
    BspI2c_Init(&g_i2c_2, &hi2c2);
    INA226_Init(&g_ina226_1, &g_i2c_1, DT35_INA226_I2C_ADDRESS_1);
    INA226_Init(&g_ina226_2, &g_i2c_2, DT35_INA226_I2C_ADDRESS_2);
    Bsp_Uart_Init(&g_uart, &huart2, BSP_UART_TX_BLOCKING,
                  NULL, 0U, 0U, NULL);
}

void UserMain_Process(void)
{
    if ((HAL_GetTick() - g_last_sample_tick) < DT35_SAMPLE_PERIOD_MS)
    {
        return;
    }
    g_last_sample_tick = HAL_GetTick();

   a= INA226_ReadBusVoltage(&g_ina226_1, &g_voltage_v_x);//主控读取INA芯片的电压
    a=INA226_ReadBusVoltage(&g_ina226_2, &g_voltage_v_y);//主控读取INA芯片的电压
    g_distance_mm_x = Dt35_VoltageToDistance(g_voltage_v_x);//电压换算成距离
    g_distance_mm_y = Dt35_VoltageToDistance(g_voltage_v_y);//电压换算成距离
    UserMain_SendDistanceFrame(g_distance_mm_x, g_distance_mm_y);
}
