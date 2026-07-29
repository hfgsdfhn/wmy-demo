#include "user_main.h"

#include "INA226.h"
#include "bsp_i2c.h"
#include "bsp_uart.h"
#include "i2c.h"
#include "usart.h"
#include "VOFA.h"

#define DT35_OUTPUT_VOLTAGE_MIN_V       0.0f
#define DT35_OUTPUT_VOLTAGE_MAX_V       10.0f

#define DT35_MIN_DISTANCE_MM            100.0f  //最小检测距离  
#define DT35_MAX_DISTANCE_MM            300.0f  //最大检测距离

#define DT35_SAMPLE_PERIOD_MS           50U     //采样周期
#define DT35_INA226_I2C_ADDRESS         0x40U   //INA226I2C地址

static bsp_i2c_t g_i2c;
static ina226_t g_ina226;
static bsp_uart_t g_uart;
static vofa_t g_vofa;

static float g_voltage_v;               //传感器返回的电压
static float g_distance_mm;             //传感器测出的距离
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

void UserMain_Init(void)
{
    BspI2c_Init(&g_i2c, &hi2c1);
    INA226_Init(&g_ina226, &g_i2c, DT35_INA226_I2C_ADDRESS);
    Bsp_Uart_Init(&g_uart, &huart2, BSP_UART_TX_BLOCKING,
                  NULL, 0U, 0U, NULL);
    Vofa_Init(&g_vofa, &g_uart);
}

void UserMain_Process(void)
{
    if ((HAL_GetTick() - g_last_sample_tick) < DT35_SAMPLE_PERIOD_MS)
    {
        return;
    }
    g_last_sample_tick = HAL_GetTick();

    INA226_ReadBusVoltage(&g_ina226, &g_voltage_v);//主控读取INA芯片的电压
    g_distance_mm = Dt35_VoltageToDistance(g_voltage_v);//电压换算成距离
    Vofa_SendFloat(&g_vofa, g_distance_mm);         //发送到VOFA（如果后面改成发送到串口，改这个函数就行）
}
