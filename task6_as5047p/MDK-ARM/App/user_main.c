#include "user_main.h"

#include "AS5047P.h"
#include "Bsp_Control_Timer.h"
#include "Bsp_Can.h"
#include "Bsp_Spi.h"
#include "dji_motor.h"
#include "motor_control.h"
#include "motor_param.h"

uint8_t debug_mode = MOTOR_CONTROL_SPEED;
float debug_target = 0.0f;

static bsp_can_t can1;
static bsp_spi_t spi1;
static bsp_tim_t tim7;
static as5047p_t encoder;
static dji_motor_t motor2006;
static motor_control_t motor2006_control;
 
pid_t motor2006_speed_pid;
pid_t motor2006_position_pid;

static void UserMain_MotorCanRx(bsp_can_t *can, uint32_t can_id,            //大奖电机CAN中断
                                const uint8_t *data, uint8_t length)
{
    if ((can_id == DJI_MOTOR_FEEDBACK_ID_BASE) && (length == 8U))
    {
        DjiMotor_ParseFeedback(&motor2006, data);
    }
}

static void PID_1ms_Callback(void)                                         //1msPID计算中断
{
    MotorControl_Update(&motor2006_control);
    DjiMotor_SendGroup(&motor2006, 1U);
}

void UserMain_Init(void *can_handle, void *spi_handle, void *tim_handle)
{
    BspCan_Init(&can1, can_handle, 0U, UserMain_MotorCanRx);               //初始化里完成中断函数创建
    DjiMotor_Init(&motor2006, &can1, 1U, DJI_MOTOR_TYPE_M2006);

    BspSpi_Init(&spi1, spi_handle, SPI1_CS_GPIO_Port, SPI1_CS_Pin);
    AS5047P_Init(&encoder, &spi1);

    //将宏定义的参数传入结构体
    PID_Init(&motor2006_speed_pid, M2006_SPEED_KP, M2006_SPEED_KI,         
                   M2006_SPEED_KD, MOTOR_CONTROL_SAMPLE_TIME_MS,
                   M2006_SPEED_OUTPUT_LIMIT, M2006_SPEED_INTEGRAL_LIMIT);
    PID_Init(&motor2006_position_pid, M2006_ANGLE_KP,
                   M2006_ANGLE_KI, M2006_ANGLE_KD,
                   MOTOR_CONTROL_SAMPLE_TIME_MS, M2006_ANGLE_OUTPUT_LIMIT,
                   M2006_ANGLE_INTEGRAL_LIMIT);
    
    //初始化电机
    MotorControl_Init(&motor2006_control, &motor2006, &encoder,
                      &motor2006_speed_pid, &motor2006_position_pid);

    BspTim_Init(&tim7, tim_handle, 0U, PID_1ms_Callback);                  //初始化里完成中断函数创建
}

void UserMain_Loop(void)
{
    static uint8_t last_mode = 0xFFU;
    static float last_target;

    if ((debug_mode != last_mode) || (debug_target != last_target))
    {
        switch (debug_mode)
        {
        case MOTOR_CONTROL_POSITION:
            MotorControl_SetAngle(&motor2006_control, debug_target);       //位置环
            break;

        case MOTOR_CONTROL_CURRENT:
            MotorControl_SetCurrent(&motor2006_control, debug_target);     //电流环
            break;

        case MOTOR_CONTROL_ZERO:
            MotorControl_SetAngle(&motor2006_control, 0.0f);               //归零环
            break;

        case MOTOR_CONTROL_SPEED:
        default:
            MotorControl_SetSpeed(&motor2006_control, debug_target);       //速度环
            break;
        }

        last_mode = debug_mode;
        last_target = debug_target;
    }
}
