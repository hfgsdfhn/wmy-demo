#include "user_main.h"

#include "main.h"
#include "Bsp_Control_Timer.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "dji_motor.h"
#include "motor_control.h"
#include "motor_param.h"
#include "vofa.h"

/* M2006 编码器线数，用于角度换算 */
#define M2006_ENCODER_COUNTS_PER_REV 8192.0f
/* 单圈角度（度） */
#define M2006_DEGREES_PER_REV        360.0f
/* VOFA 数据发送周期（ms） */
#define VOFA_SEND_PERIOD_MS          10U


extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart1;

/**
 * @brief 调试模式
 * @note  MOTOR_CONTROL_SPEED    = 速度模式
 *        MOTOR_CONTROL_POSITION = 位置模式
 *        MOTOR_CONTROL_CURRENT  = 电流模式
 */
volatile uint8_t debug_mode = MOTOR_CONTROL_SPEED;
/* 调试目标值（速度/角度/电流），由上位机或调试接口修改 */
volatile float debug_target = 0.0f;

/* CAN 驱动实例 */
static bsp_can_t can1;
/* 串口驱动实例 */
static bsp_uart_t bsp_uart1;
/* M2006 电机对象 */
static dji_motor_t motor2006;
/* 电机控制对象 */
static motor_control_t motor2006_control;
/* 速度环 PID */
volatile pid_t motor2006_speed_pid;
/* 位置环 PID */
volatile pid_t motor2006_position_pid;

/* VOFA 调试实例 */
static vofa_t vofa1;
/* VOFA 上报：当前速度 */
static float vofa_speed;
/* VOFA 上报：当前角度 */
static float vofa_angle;

/**
 * @brief  CAN 接收回调，解析 M2006 电机反馈数据
 * @param  can    CAN 驱动实例指针
 * @param  can_id 接收到的 CAN ID
 * @param  data   接收数据缓冲区
 * @param  length 数据长度（字节）
 * @retval None
 */
static void MotorCanRx(bsp_can_t *can, uint32_t can_id,
                       const uint8_t *data, uint8_t length)
{
    (void)can;

    /* 仅处理电机反馈帧（标准 ID + 8 字节数据） */
    if ((can_id == DJI_MOTOR_FEEDBACK_ID_BASE) && (length == 8U))
    {
        DjiMotor_ParseFeedback(&motor2006, data);
    }
}

/**
 * @brief  用户初始化：依次初始化各驱动和算法模块
 * @param  None
 * @retval None
 */
void user_main_init(void)
{
    /* 1. CAN 驱动初始化，绑定接收回调 */
    BspCan_Init(&can1, &hcan1, 0U);
    BspCan_SetRxCallback(&can1, MotorCanRx);
    DjiMotor_Init(&motor2006, &can1, 1U, DJI_MOTOR_TYPE_M2006);

    /* 2. 速度环 PID 初始化 */
    (void)PID_Init((pid_t *)&motor2006_speed_pid, M2006_SPEED_KP,
                   M2006_SPEED_KI,
                   M2006_SPEED_KD, MOTOR_CONTROL_SAMPLE_TIME_MS,
                   M2006_SPEED_OUTPUT_LIMIT, M2006_SPEED_INTEGRAL_LIMIT);
    /* 3. 位置环 PID 初始化 */
    (void)PID_Init((pid_t *)&motor2006_position_pid, MOTOR_CONTROL_ANGLE_KP,
                   MOTOR_CONTROL_ANGLE_KI, MOTOR_CONTROL_ANGLE_KD,
                   MOTOR_CONTROL_SAMPLE_TIME_MS,
                   MOTOR_CONTROL_ANGLE_OUTPUT_LIMIT,
                   MOTOR_CONTROL_ANGLE_INTEGRAL_LIMIT);
    /* 4. 电机控制对象绑定电机与 PID */
    MotorControl_Init(&motor2006_control, &motor2006,
                      (pid_t *)&motor2006_speed_pid,
                      (pid_t *)&motor2006_position_pid);

    /* 5. 串口初始化（阻塞发送模式） */
    (void)Bsp_Uart_Init(&bsp_uart1, &huart1, BSP_UART_TX_BLOCKING,
                        NULL, 0U);
    /* 6. VOFA 调试上位机初始化，注册 speed 和 angle 两个变量 */
    (void)Vofa_Init(&vofa1, &bsp_uart1);
    (void)Vofa_Register(&vofa1, "speed", &vofa_speed);
    (void)Vofa_Register(&vofa1, "angle", &vofa_angle);
    Vofa_SetPeriod(&vofa1, VOFA_SEND_PERIOD_MS);
}

/**
 * @brief  用户主循环：处理模式切换、电机控制更新、调试数据上报
 *
 * 执行顺序：
 * 1. 检测调试模式或目标值是否变化，切换控制模式
 * 2. 按控制周期更新电机控制
 * 3. 更新 VOFA 调试变量并发送
 *
 * @param  None
 * @retval None
 */
void user_main_loop(void)
{
    /* 上一拍的 tick 值，用于控制周期判断 */
    static uint32_t last_tick;
    /* 上一次的调试模式，用于变化检测 */
    static uint8_t last_mode = 0xFFU;
    /* 上一次的调试目标值 */
    static float last_target;
    uint32_t tick = BspControlTimer_Get1msCount();

    /* 检测调试模式或目标值是否发生变化 */
    if ((debug_mode != last_mode) || (debug_target != last_target))
    {
        if (debug_mode == MOTOR_CONTROL_POSITION)
        {
            /* 切换到角度/位置控制模式 */
            MotorControl_SetAngle(&motor2006_control, debug_target);
        }
        else if (debug_mode == MOTOR_CONTROL_CURRENT)
        {
            /* 切换到电流控制模式 */
            MotorControl_SetCurrent(&motor2006_control, debug_target);
        }
        else
        {
            /* 默认：速度控制模式 */
            MotorControl_SetSpeed(&motor2006_control, debug_target);
        }
        last_mode = debug_mode;
        last_target = debug_target;
    }

    /* 按 1ms 控制周期更新 */
    if (tick != last_tick)
    {
        last_tick = tick;
        /* 更新电机控制（PID 计算）并发送 CAN 指令 */
        MotorControl_Update(&motor2006_control);
        DjiMotor_SendGroup(&motor2006, 1U);
    }

    /* 更新 VOFA 调试变量 */
    /* 电机当前转速（RPM） */
    vofa_speed = (float)motor2006.speed_rpm;
    /* 编码器值换算为角度（度） */
    vofa_angle = ((float)motor2006.encoder * M2006_DEGREES_PER_REV)
                 / M2006_ENCODER_COUNTS_PER_REV;
    /* VOFA 周期发送处理 */
    Vofa_Process(&vofa1);
}
