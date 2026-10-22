#include "my_main.h"
#include "vofa.h"

#include <stddef.h>

position_case my_case = 0;
up_stpe up_ = 0;
uint32_t time_loader = 0;
uint8_t cnt = 0;
float target_speed[4];
extern FDCAN_HandleTypeDef hfdcan1;
volatile float m2006_current;
typedef struct
{
    float kp;
    float ki;
    float kd;
    float limit;
    float ki_limit;
}debug_pid;

debug_pid debug_pos;
debug_pid debug_sp;
bool is_debug = 0;
pid_t position_pid[4];
pid_t speed_pid[4];
pid_t m2006_pid[4];
dji_motor_t M2006[4];
dji_motor_t M3508[4];
leg_t front;
leg_t behind;
float qian = 0.0f;
float hou = 0.0f;
float together = 0.0f;
bool flag = 0;
bsp_can_t can_dji;

uint16_t average_balance_current = 0;

static void PID_UpdateParams(pid_t *pid, const debug_pid *debug)
{
    pid->kp = debug->kp;
    pid->ki = debug->ki;
    pid->kd = debug->kd;

    pid->output_limit = debug->limit;
    pid->integral_limit = debug->ki_limit;
}

float limit_abs(float value, float max)
{
    if (value > max)
        return max;
    if (value < -max)
        return -max;

    return value;
}

void My_Init(void)
{
    BspCan_Init(&can_dji, &hfdcan1, 1, DJI_OnData);
    for (int i = 0; i < 4; i++)
    {
        PID_Init(&position_pid[i], 0.5f,0.0f,0.5f,5.0f,2000.0f,0.0f);
        PID_Init(&speed_pid[i],4.5f,0.0f,0.5f,5.0f,12000.0f,0.0f);
        DjiMotor_Init(&M3508[i],&can_dji,i+5,1);
        DjiMotor_Init(&M2006[i],&can_dji,i+1,0);
    }
}
//0 2000 900 1 -2700 2 -2500 3 2900
void My_Loop(void)
{
        if(HAL_GetTick() - time_loader >= 5)
        {
            if(is_debug)
            {
                for (int i = 0; i < 4; i++)
                {
                    PID_UpdateParams(&position_pid[i], &debug_pos);
                    PID_UpdateParams(&speed_pid[i], &debug_sp);
                }
            }
            time_loader = HAL_GetTick();
            cnt++;

            //  //通过比例补偿平衡升降
            // target_speed[lf] = PID_Calc(&position_pid[lf], front.distance, M3508[lf].total_encoder) * 2000/2525;
            // M3508[lf].target_current = (int16_t)(PID_Calc(&speed_pid[lf], target_speed[lf], M3508[lf].speed_rpm) * 2000/2525);
            
            // target_speed[rf] = PID_Calc(&position_pid[rf], -front.distance, M3508[rf].total_encoder) * 2700/2525;
            // M3508[rf].target_current = (int16_t)(PID_Calc(&speed_pid[rf], target_speed[rf], M3508[rf].speed_rpm)) * 2700/2525;
            
            // target_speed[rr] = PID_Calc(&position_pid[rr], -behind.distance, M3508[rr].total_encoder) * 2500/2525;
            // M3508[rr].target_current = (int16_t)(PID_Calc(&speed_pid[rr], target_speed[rr], M3508[rr].speed_rpm)) * 2500/2525;
            
            // target_speed[lr] = PID_Calc(&position_pid[lr], behind.distance, M3508[lr].total_encoder) * 2900/2525;
            // M3508[lr].target_current = (int16_t)(PID_Calc(&speed_pid[lr], target_speed[lr], M3508[lr].speed_rpm)) * 2900/2525;

            //通过重力补偿平衡升降
            float front_height = (M3508[lf].total_encoder - M3508[rf].total_encoder)/2.0f;
            float behind_height = (M3508[lr].total_encoder - M3508[rr].total_encoder)/2.0f;
            target_speed[lf] = PID_Calc(&position_pid[lf], front.distance, M3508[lf].total_encoder);
            target_speed[rf] = PID_Calc(&position_pid[rf], -front.distance, M3508[rf].total_encoder);
            target_speed[rr] = PID_Calc(&position_pid[rr], -behind.distance, M3508[rr].total_encoder);
            target_speed[lr] = PID_Calc(&position_pid[lr], behind.distance, M3508[lr].total_encoder);
            if(front_height > 10000)
            {
                M3508[lf].target_current = (int16_t)PID_Calc(&speed_pid[lf], target_speed[lf], M3508[lf].speed_rpm) + (1800-average_balance_current);
                M3508[rf].target_current = (int16_t)PID_Calc(&speed_pid[rf], target_speed[rf], M3508[rf].speed_rpm) - (2800-average_balance_current);
            } else {
                M3508[lf].target_current = (int16_t)PID_Calc(&speed_pid[lf], target_speed[lf], M3508[lf].speed_rpm);
                M3508[rf].target_current = (int16_t)PID_Calc(&speed_pid[rf], target_speed[rf], M3508[rf].speed_rpm);
            }
            if(behind_height > 10000)
            {
                M3508[rr].target_current = (int16_t)PID_Calc(&speed_pid[rr], target_speed[rr], M3508[rr].speed_rpm) - (2900-average_balance_current);
                M3508[lr].target_current = (int16_t)PID_Calc(&speed_pid[lr], target_speed[lr], M3508[lr].speed_rpm) + (3100-average_balance_current);
            } else {
                M3508[rr].target_current = (int16_t)PID_Calc(&speed_pid[rr], target_speed[rr], M3508[rr].speed_rpm);
                M3508[lr].target_current = (int16_t)PID_Calc(&speed_pid[lr], target_speed[lr], M3508[lr].speed_rpm);
            }
            
            // //四条腿控制调试
            // switch(my_case)
            // {
            //     case all_up:
            //     {
            //         float front_height = (M3508[lf].total_encoder - M3508[rf].total_encoder)/2.0f;
            //         float behind_height = (M3508[lr].total_encoder - M3508[rr].total_encoder)/2.0f;
            //         if(flag == 0)
            //         {
            //         front.distance = together + front_height;
            //         behind.distance = together + behind_height;
            //         flag = 1;
            //         }
            //     }
            //     break;

            //     case all_back:
            //         front.distance = 0.0f;
            //         behind.distance = 0.0f;
            //     break;

            //     case front_up:
            //         front.distance = qian;
            //     break;
                
            //     case front_back:
            //         front.distance = 0.0f;
            //     break;

            //     case behind_up:
            //         behind.distance = hou;
            //     break;

            //     case behind_back:
            //         behind.distance = 0.0f;
            //     break;

            //     case qigang_up:
            //         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,1);
            //         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2,1);
            //     break;

            //     case qigang_down:
            //         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,0);
            //         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2,0);
            //     break;

            //     default:

            //     break;
            // }

            
                            //四条腿控制调试
            switch(up_)
            {
                case step1:
                {
                    front.distance = 130000;    //一块升
                    behind.distance = 130000;
                }
                break;

                case step2:
                    front.distance = 8000.0f;
                break;

                case step3:
                    front.distance = 130000;
                    behind.distance = 260000;
                break;
                
                case step4:
                    front.distance = 8000.0f;
                break;

                case step5:
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,1);
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2,1);
                break;
                
                case step6:
                    behind.distance = 260000;
                break;

                case step7:
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,0);
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2,0);
                break;

                case step8:
                    behind.distance = 130000;
                break;

                default:
                break;
            }
        if(cnt>=4)
        {
            cnt = 0;
                VOFA_CheckTimeout();

                DjiMotor_SetCurrent(&M2006[0], m2006_current);
                DjiMotor_SetCurrent(&M2006[1], -m2006_current);
                DjiMotor_SetCurrent(&M2006[2], -m2006_current);
                DjiMotor_SetCurrent(&M2006[3], m2006_current);
                DjiMotor_SendGroup(M2006,4);
        }

        for (int i = 0; i < 4; i++)
        {
            // if(fabsf(M3508[i].current) > 1000 && fabsf(M3508[i].total_encoder) < 5000 &&fabsf(M3508[i].speed_rpm) < 10)
            // limit_abs(M3508[i].current, 500);
            DjiMotor_SetCurrent(&M3508[i], M3508[i].target_current);
        }
        DjiMotor_SendGroup(M3508,4);
    }

}

void DJI_OnData(bsp_can_t *can, uint32_t can_id, const uint8_t *data,
                uint8_t length)
{
    uint8_t motor_id;

    if ((can != &can_dji) || (data == NULL)
        || (length < BSP_CAN_MAX_DATA_LENGTH)
        || (can_id < DJI_MOTOR_FEEDBACK_ID_BASE)
        || (can_id > (DJI_MOTOR_FEEDBACK_ID_BASE + DJI_MOTOR_MAX_ID - 1U)))
    {
        return;
    }

    motor_id = (uint8_t)(can_id - DJI_MOTOR_FEEDBACK_ID_BASE + 1U);
    if (motor_id <= 4U)
    {
        DjiMotor_ParseFeedback(&M2006[motor_id - 1U], data);
    }
    else
    {
        DjiMotor_ParseFeedback(&M3508[motor_id - 5U], data);
    }
}
