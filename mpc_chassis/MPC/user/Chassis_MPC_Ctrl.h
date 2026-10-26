/**
 * @file 王梦阳
 * @brief 中科大开源的方案调用了qpOASES作为求解器
 *  H = 2(B_TQB_- +R)
 * 
 *  g = f =2B_TQ(A_x0- Xref)
 * 
 *  A   约束矩阵
 * 
 *  lbA、ubA    约束左右边界 lbA <= A*x <= ubA。
 * 
 *  lb、ub      优化变量本身的下界和上界    例如对未来控制输入设置幅值限制。
 * @bug     这里约束我只限制了控制量，所以没有速度限制
 *  NV   若优化未来 N 步、每步 nu 个输入，则通常 NV = N * nu。
 *  NC   即 A 有多少行。
 * @version 0.1
 * @date 2026-10-23
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef CHASSIS_MPC_CTRL_H
#define CHASSIS_MPC_CTRL_H

#include "stdint.h"
#include "stdbool.h"
#include "arm_math.h"

#define mpc_dt 0.01f    //(s)
#define MPC_MAX_N 20    //预测步数
#define NX   6       // 状态维度
#define NU   3       // 控制维度
#define NV   (MPC_MAX_N*NU)  // 优化变量总维度 (30)

typedef struct MPC mpc_t;

typedef struct {

    float mass;          // 车体质量 (kg)
    float inertia_z;     // 转动惯量 (kg·m²)
    float max_force_x;   // 纵向最大合力 (N)
    float max_force_y;   // 横向最大合力 (N)
    float max_torque_z;  // 最大旋转力矩 (N·m)

} MpcPhysicalParams;

typedef enum {
    x, y, yaw, vx, vy, w
} mpc_state_t;
typedef enum
{
    Fx,Fy,Mz
} mpc_ctrl;
struct MPC
{
    mpc_state_t chassis_state;
    
    float x[NX]; //状态量
    float u[NU]; //控制量
    
    float Q[NX];
    float R[NU];
    
     uint8_t nWSR;   //最大工作集重计算次数（number of Working Set Recalculations）  限制求解时间
    
    MpcPhysicalParams RobotParam;
    uint8_t i;  //运算了多少步
};
void MPC_Init();
#endif
